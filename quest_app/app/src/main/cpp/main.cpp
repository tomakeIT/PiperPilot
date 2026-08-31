// PiperQuestTeleop — native Quest OpenXR app that streams controller 6-DoF poses
// and inputs over TCP (newline-delimited JSON, port 8735).
//
// Transport is designed for wired use: the host runs
//     adb forward tcp:8735 tcp:8735
// and connects to 127.0.0.1:8735, so all data flows over the USB cable.
//
// Outbound (one line per rendered frame, ~72/90 Hz):
//   {"t":<predicted display time, s>,"mono":<CLOCK_MONOTONIC, s>,"frame":N,
//    "state":"FOCUSED","space":"anchor"|"stage"|"local",
//    "anchor":{"have":b,"tracked":b},
//    "head":{"pos":[x,y,z],"quat":[x,y,z,w],"valid":b,"tracked":b},
//    "left":{"pos":[...],"quat":[x,y,z,w],"valid":b,"tracked":b,"trigger":f,
//            "squeeze":f,"stick":[x,y],"stick_click":b,"primary":b,
//            "secondary":b,"menu":b},
//    "right":{...same fields...}}
// Quaternions are [x,y,z,w] in the reporting space (y-up, meters). "space"
// says which frame the poses are in this frame: "anchor" = the persistent
// spatial anchor (calibration survives reboots), else the session STAGE
// (origin on the floor at guardian center; yaw moves when the guardian is
// redrawn or relocalization fails) or LOCAL as a last resort.
//
// Inbound commands (newline-delimited JSON):
//   {"cmd":"haptic","hand":"left"|"right"|"both","amp":0.0-1.0,"ms":<int>}
//   {"cmd":"color","r":f,"g":f,"b":f,"a":f}  // tint over passthrough
//   {"cmd":"panel","cam":0|1,"x":f,"y":f,"z":f,"width":f,"yaw":deg}
//   {"cmd":"anchor_save"}   // pin the CURRENT stage frame as a persistent
//                           // spatial anchor (send once teleop axes are
//                           // calibrated; poses then arrive in that frame
//                           // forever, across reboots)
//   {"cmd":"anchor_clear"}  // erase the anchor, back to stage reporting
//   {"cmd":"calibrate_x","hand":"left"|"right"}
//                           // two-point X-axis calibration: trigger-click at
//                           // point 1, move along the desired robot +X,
//                           // trigger-click at point 2. The horizontal
//                           // P1->P2 line becomes robot +X (up stays
//                           // gravity-up) and is saved as the persistent
//                           // anchor. While calibrating, hand state in the
//                           // outbound stream is masked (valid:false, no
//                           // buttons) so the host cannot clutch/record;
//                           // "calib":1|2 says which click is awaited.
//   {"cmd":"calibrate_cancel"}  // abort calibration (also: menu button)
//
// Camera panels: a second TCP server on port 8736 receives MJPEG frames
// (host: piper_teleop/quest_video.py) and shows them as floating quad
// layers over passthrough. Framing (little endian):
//   [u32 magic 'PCV1'][u8 cam][u8 pad*3][u32 len][len bytes JPEG]
// Video is strictly lower priority than the pose stream: separate port,
// separate thread, decode off the render thread, latest-frame-only.

#include <android/log.h>
#include <android_native_app_glue.h>
#include <jni.h>

#include <EGL/egl.h>
#include <GLES3/gl3.h>

#define XR_USE_PLATFORM_ANDROID
#define XR_USE_GRAPHICS_API_OPENGL_ES
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#include "stb_image.h"

#define TAG "QuestTeleop"
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define ALOGW(...) __android_log_print(ANDROID_LOG_WARN, TAG, __VA_ARGS__)
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

static const char* kAppVersion = "0.4.0";
static const int kProtocolVersion = 1;
static const uint16_t kPort = 8735;
static const uint16_t kVideoPort = 8736;
// Panels 0..2: camera views (left to right). Panel 3: status HUD strip.
static const int kMaxCams = 4;
static const int kStatusPanel = kMaxCams - 1;
static const uint32_t kVideoMagic = 0x31564350;  // 'PCV1' little endian

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

static bool XrOk(XrResult res, const char* what) {
    if (XR_SUCCEEDED(res)) return true;
    ALOGE("%s failed: %d", what, res);
    return false;
}

#define OXR(call)                              \
    do {                                       \
        XrResult _res = (call);                \
        if (!XrOk(_res, #call)) return false;  \
    } while (0)

static double NowMonotonic() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// Minimal value extraction from a flat one-line JSON command. Good enough for
// the fixed, trusted command set sent by our own host tools.
static bool JsonFindString(const char* line, const char* key, char* out, size_t outLen) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(line, pat);
    if (!p) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p) return false;
    p++;
    while (*p == ' ') p++;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < outLen) out[i++] = *p++;
    out[i] = '\0';
    return true;
}

static bool JsonFindDouble(const char* line, const char* key, double* out) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(line, pat);
    if (!p) return false;
    p = strchr(p + strlen(pat), ':');
    if (!p) return false;
    return sscanf(p + 1, "%lf", out) == 1;
}

// ---------------------------------------------------------------------------
// TCP server: accepts any number of clients, broadcasts frame lines, collects
// inbound command lines. All sockets non-blocking; accept runs on its own
// thread, send/recv happen on the render thread (sub-millisecond at 90 Hz).
// ---------------------------------------------------------------------------

class TeleopServer {
public:
    bool Start(const std::string& helloLine) {
        helloLine_ = helloLine;
        listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (listenFd_ < 0) {
            ALOGE("socket() failed: %s", strerror(errno));
            return false;
        }
        int one = 1;
        setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(kPort);
        if (bind(listenFd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            ALOGE("bind(%d) failed: %s", kPort, strerror(errno));
            return false;
        }
        if (listen(listenFd_, 4) < 0) {
            ALOGE("listen failed: %s", strerror(errno));
            return false;
        }
        running_ = true;
        acceptThread_ = std::thread([this] { AcceptLoop(); });
        ALOGI("TCP server listening on 0.0.0.0:%d", kPort);
        return true;
    }

    void Stop() {
        running_ = false;
        if (listenFd_ >= 0) {
            shutdown(listenFd_, SHUT_RDWR);
            close(listenFd_);
            listenFd_ = -1;
        }
        if (acceptThread_.joinable()) acceptThread_.join();
        std::lock_guard<std::mutex> lock(mu_);
        for (auto& c : clients_) close(c.fd);
        clients_.clear();
    }

    size_t ClientCount() {
        std::lock_guard<std::mutex> lock(mu_);
        return clients_.size();
    }

    void Broadcast(const char* data, size_t len) {
        std::lock_guard<std::mutex> lock(mu_);
        for (auto it = clients_.begin(); it != clients_.end();) {
            // Finish any partially-sent line first: a short write must never
            // splice with a later line (would corrupt the newline-JSON
            // protocol). While a tail is pending, whole new lines are skipped
            // (live-stream semantics).
            if (!it->outbuf.empty()) {
                ssize_t n = send(it->fd, it->outbuf.data(), it->outbuf.size(),
                                 MSG_NOSIGNAL | MSG_DONTWAIT);
                if (n > 0) {
                    it->outbuf.erase(0, (size_t)n);
                } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    ALOGI("client fd=%d dropped (%s)", it->fd, strerror(errno));
                    close(it->fd);
                    it = clients_.erase(it);
                    continue;
                }
            }
            if (!it->outbuf.empty()) {
                if (++it->stalledFrames > 1800) {  // ~20 s stalled -> drop
                    ALOGI("client fd=%d stalled too long, dropping", it->fd);
                    close(it->fd);
                    it = clients_.erase(it);
                    continue;
                }
                ++it;
                continue;  // skip this frame line for the backed-up client
            }

            ssize_t n = send(it->fd, data, len, MSG_NOSIGNAL | MSG_DONTWAIT);
            if (n < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    ALOGI("client fd=%d dropped (%s)", it->fd, strerror(errno));
                    close(it->fd);
                    it = clients_.erase(it);
                    continue;
                }
                if (++it->stalledFrames > 1800) {
                    ALOGI("client fd=%d stalled too long, dropping", it->fd);
                    close(it->fd);
                    it = clients_.erase(it);
                    continue;
                }
            } else if ((size_t)n < len) {
                it->outbuf.assign(data + n, len - (size_t)n);  // finish later
                it->stalledFrames = 0;
            } else {
                it->stalledFrames = 0;
            }
            ++it;
        }
    }

    // Drain complete inbound lines from all clients.
    void PollCommands(std::vector<std::string>* out) {
        std::lock_guard<std::mutex> lock(mu_);
        char buf[512];
        for (auto it = clients_.begin(); it != clients_.end();) {
            bool dead = false;
            for (;;) {
                ssize_t n = recv(it->fd, buf, sizeof(buf), MSG_DONTWAIT);
                if (n > 0) {
                    it->inbuf.append(buf, (size_t)n);
                    if (it->inbuf.size() > 65536) it->inbuf.clear();  // runaway guard
                } else if (n == 0) {
                    dead = true;  // orderly shutdown
                    break;
                } else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) dead = true;
                    break;
                }
            }
            size_t pos;
            while ((pos = it->inbuf.find('\n')) != std::string::npos) {
                out->push_back(it->inbuf.substr(0, pos));
                it->inbuf.erase(0, pos + 1);
            }
            if (dead) {
                ALOGI("client fd=%d disconnected", it->fd);
                close(it->fd);
                it = clients_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    struct Client {
        int fd;
        std::string inbuf;
        std::string outbuf;  // unsent tail of a partially-written line
        int stalledFrames = 0;
    };

    void AcceptLoop() {
        while (running_) {
            sockaddr_in peer{};
            socklen_t len = sizeof(peer);
            int fd = accept(listenFd_, (sockaddr*)&peer, &len);
            if (fd < 0) {
                if (running_) ALOGW("accept failed: %s", strerror(errno));
                continue;
            }
            int one = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
            // Hello line (blocking send would be fine here; best effort).
            send(fd, helloLine_.data(), helloLine_.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
            std::lock_guard<std::mutex> lock(mu_);
            clients_.push_back(Client{fd});
            ALOGI("client connected fd=%d (%s)", fd, inet_ntoa(peer.sin_addr));
        }
    }

    int listenFd_ = -1;
    std::atomic<bool> running_{false};
    std::thread acceptThread_;
    std::mutex mu_;
    std::vector<Client> clients_;
    std::string helloLine_;
};

// ---------------------------------------------------------------------------
// Camera video receiver: MJPEG frames over TCP, decoded off the render
// thread; the render loop only uploads the newest decoded RGBA image.
// ---------------------------------------------------------------------------

class VideoServer {
public:
    struct Decoded {
        std::vector<uint8_t> rgba;
        int w = 0, h = 0;
        uint64_t seq = 0;       // bumps on every decoded frame
        double recvMono = 0.0;  // for staleness-based hiding
    };

    bool Start() {
        listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (listenFd_ < 0) return false;
        int one = 1;
        setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(kVideoPort);
        if (bind(listenFd_, (sockaddr*)&addr, sizeof(addr)) < 0 ||
            listen(listenFd_, 1) < 0) {
            ALOGE("video bind/listen failed: %s", strerror(errno));
            close(listenFd_);
            listenFd_ = -1;
            return false;
        }
        // GLES textures have a bottom-left origin; flip JPEG rows at decode
        // so panels show upright.
        stbi_set_flip_vertically_on_load(1);
        running_ = true;
        thread_ = std::thread([this] { Loop(); });
        ALOGI("video server listening on 0.0.0.0:%d", kVideoPort);
        return true;
    }

    void Stop() {
        running_ = false;
        if (listenFd_ >= 0) {
            shutdown(listenFd_, SHUT_RDWR);
            close(listenFd_);
            listenFd_ = -1;
        }
        {
            std::lock_guard<std::mutex> lock(fdMu_);
            if (clientFd_ >= 0) shutdown(clientFd_, SHUT_RDWR);
        }
        if (thread_.joinable()) thread_.join();
    }

    // Takes the latest frame for `cam` if newer than *lastSeq. Zero-copy:
    // pixel buffers are SWAPPED, never copied (the decode thread reuses the
    // returned buffer's capacity on the next frame).
    bool Latest(int cam, uint64_t* lastSeq, Decoded* out) {
        if (cam < 0 || cam >= kMaxCams) return false;
        std::lock_guard<std::mutex> lock(mu_);
        Decoded& d = frames_[cam];
        if (d.seq == 0 || d.seq == *lastSeq) return false;
        *lastSeq = d.seq;
        out->rgba.swap(d.rgba);
        out->w = d.w;
        out->h = d.h;
        out->seq = d.seq;
        out->recvMono = d.recvMono;
        return true;
    }

    double LastRecvMono(int cam) {
        std::lock_guard<std::mutex> lock(mu_);
        return (cam >= 0 && cam < kMaxCams) ? frames_[cam].recvMono : 0.0;
    }

private:
    bool ReadAll(int fd, uint8_t* dst, size_t n) {
        size_t got = 0;
        while (got < n && running_) {
            ssize_t r = recv(fd, dst + got, n - got, 0);
            if (r <= 0) return false;
            got += (size_t)r;
        }
        return got == n;
    }

    void Loop() {
        std::vector<uint8_t> payload;
        while (running_) {
            int fd = accept(listenFd_, nullptr, nullptr);
            if (fd < 0) continue;
            int one = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            {
                std::lock_guard<std::mutex> lock(fdMu_);
                clientFd_ = fd;
            }
            ALOGI("video client connected");
            uint8_t header[12];
            while (running_ && ReadAll(fd, header, sizeof(header))) {
                uint32_t magic, len;
                memcpy(&magic, header, 4);
                uint8_t cam = header[4];
                memcpy(&len, header + 8, 4);
                if (magic != kVideoMagic || cam >= kMaxCams || len == 0 ||
                    len > 4u * 1024 * 1024) {
                    ALOGW("video framing error (magic=%08x cam=%u len=%u)", magic, cam, len);
                    break;
                }
                payload.resize(len);
                if (!ReadAll(fd, payload.data(), len)) break;

                int w = 0, h = 0, comp = 0;
                unsigned char* px = stbi_load_from_memory(
                    payload.data(), (int)len, &w, &h, &comp, 4);
                if (px == nullptr) {
                    ALOGW("JPEG decode failed (%u bytes)", len);
                    continue;
                }
                {
                    std::lock_guard<std::mutex> lock(mu_);
                    Decoded& d = frames_[cam];
                    d.rgba.assign(px, px + (size_t)w * h * 4);
                    d.w = w;
                    d.h = h;
                    d.seq++;
                    d.recvMono = NowMonotonic();
                }
                stbi_image_free(px);
            }
            close(fd);
            {
                std::lock_guard<std::mutex> lock(fdMu_);
                clientFd_ = -1;
            }
            ALOGI("video client disconnected");
        }
    }

    int listenFd_ = -1;
    int clientFd_ = -1;
    std::mutex fdMu_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex mu_;
    Decoded frames_[kMaxCams];
};

// ---------------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------------

enum Side { LEFT = 0, RIGHT = 1, SIDE_COUNT = 2 };

struct Swapchain {
    XrSwapchain handle = XR_NULL_HANDLE;
    int32_t width = 0;
    int32_t height = 0;
    std::vector<XrSwapchainImageOpenGLESKHR> images;
};

struct AppState {
    android_app* app = nullptr;
    bool resumed = false;

    // EGL
    EGLDisplay eglDisplay = EGL_NO_DISPLAY;
    EGLConfig eglConfig = nullptr;
    EGLContext eglContext = EGL_NO_CONTEXT;
    EGLSurface eglSurface = EGL_NO_SURFACE;  // tiny pbuffer

    // OpenXR core
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId systemId = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace baseSpace = XR_NULL_HANDLE;   // STAGE if available, else LOCAL
    XrSpace viewSpace = XR_NULL_HANDLE;   // head pose
    bool usingStage = false;
    XrSessionState sessionState = XR_SESSION_STATE_UNKNOWN;
    bool sessionRunning = false;
    bool exitRequested = false;

    // Rendering. With passthrough active the projection layer is a mostly
    // transparent tint on top of the camera feed (alpha = 0 -> invisible).
    std::vector<XrViewConfigurationView> viewConfigs;
    Swapchain swapchains[2];
    GLuint fbo = 0;
    float clearColor[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // Passthrough (XR_FB_passthrough)
    bool hasPassthroughExt = false;
    bool passthroughActive = false;
    XrPassthroughFB passthrough = XR_NULL_HANDLE;
    XrPassthroughLayerFB passthroughLayer = XR_NULL_HANDLE;

    // Floating camera panels (quad layers fed by VideoServer).
    struct CamPanel {
        XrSwapchain swapchain = XR_NULL_HANDLE;
        int32_t w = 0, h = 0;
        std::vector<XrSwapchainImageOpenGLESKHR> images;
        uint64_t lastSeq = 0;
        bool hasContent = false;
        XrPosef pose;      // in base space
        float width = 0.55f;  // meters; height follows aspect
        VideoServer::Decoded scratch;  // recycled pixel buffer (swap w/ server)
    } camPanels[kMaxCams];
    VideoServer videoServer;
    // Panels re-place themselves in front of the user's head: at startup and
    // whenever the system recenter gesture fires (long-press the Meta button
    // -> XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING), or on the host
    // command {"cmd":"panels_front"}.
    bool repositionPanels = true;
    XrPosef lastHead;
    bool lastHeadValid = false;

    // Input
    XrActionSet actionSet = XR_NULL_HANDLE;
    XrPath handPaths[SIDE_COUNT];
    XrAction gripPoseAction = XR_NULL_HANDLE;
    XrAction triggerAction = XR_NULL_HANDLE;
    XrAction squeezeAction = XR_NULL_HANDLE;
    XrAction stickAction = XR_NULL_HANDLE;
    XrAction stickClickAction = XR_NULL_HANDLE;
    XrAction primaryAction = XR_NULL_HANDLE;    // X (left) / A (right)
    XrAction secondaryAction = XR_NULL_HANDLE;  // Y (left) / B (right)
    XrAction menuAction = XR_NULL_HANDLE;       // left menu button
    XrAction hapticAction = XR_NULL_HANDLE;
    XrSpace gripSpaces[SIDE_COUNT] = {XR_NULL_HANDLE, XR_NULL_HANDLE};

    // Persistent spatial anchor (XR_FB_spatial_entity + storage + query).
    // Pins the calibrated world frame to the physical room: once saved, poses
    // are reported relative to the anchor, so the host's XR->robot yaw
    // calibration survives reboots and guardian redraws. All spatial-entity
    // operations are async: call -> completion event, tracked by anchorPhase.
    bool hasAnchorExt = false;
    PFN_xrCreateSpatialAnchorFB pfnCreateSpatialAnchor = nullptr;
    PFN_xrSetSpaceComponentStatusFB pfnSetSpaceComponentStatus = nullptr;
    PFN_xrSaveSpaceFB pfnSaveSpace = nullptr;
    PFN_xrEraseSpaceFB pfnEraseSpace = nullptr;
    PFN_xrQuerySpacesFB pfnQuerySpaces = nullptr;
    PFN_xrRetrieveSpaceQueryResultsFB pfnRetrieveSpaceQueryResults = nullptr;
    enum class AnchorPhase {
        Idle,           // no operation in flight
        Creating,       // xrCreateSpatialAnchorFB pending
        NewStorable,    // enabling STORABLE on a freshly created anchor
        Saving,         // xrSaveSpaceFB pending
        Querying,       // xrQuerySpacesFB pending (load stored anchor)
        LoadLocatable,  // enabling LOCATABLE on a loaded anchor
        LoadStorable,   // enabling STORABLE on a loaded anchor
    };
    AnchorPhase anchorPhase = AnchorPhase::Idle;
    XrAsyncRequestIdFB anchorRequest = 0;  // request id of the in-flight op
    XrSpace anchorSpace = XR_NULL_HANDLE;  // active reporting base when tracked
    XrSpace pendingAnchorSpace = XR_NULL_HANDLE;  // created/loaded, not active yet
    XrUuidEXT anchorUuid{};                // valid iff anchorUuidValid
    XrUuidEXT pendingUuid{};
    bool anchorUuidValid = false;
    bool anchorActive = false;             // anchorSpace resolved and usable
    XrSpace eraseSpace = XR_NULL_HANDLE;   // replaced anchor being erased
    XrAsyncRequestIdFB eraseRequest = 0;
    std::string anchorUuidPath;            // <internalDataPath>/anchor_uuid
    double nextAnchorQueryRetry = 0.0;     // relocalization can succeed late
    XrTime lastDisplayTime = 0;

    // Two-point X-axis calibration (host cmd "calibrate_x"): trigger-click
    // at point 1, move along the desired robot +X, trigger-click at point 2;
    // the horizontal P1->P2 line becomes the anchor's -Z (= robot +X after
    // the host's fixed remap) and is saved as the persistent anchor. While
    // active, outbound hand state is masked so the host cannot clutch, move
    // the gripper, or toggle recording from calibration clicks.
    int calibStep = 0;              // 0=off, 1=await point 1, 2=await point 2
    int calibSide = RIGHT;
    XrVector3f calibP1{};
    bool calibTriggerHigh = false;  // trigger hysteresis for click detection
    double calibDeadline = 0.0;
    float calibSavedColor[4] = {0, 0, 0, 0};
    XrPosef calibPreviewPose;       // live arrow P1 -> hand, in base space
    bool calibPreviewValid = false;

    // Faint translucent arrow quad showing robot +X: live preview while
    // calibrating, pinned in anchor space once the anchor is active.
    XrSwapchain axisSwapchain = XR_NULL_HANDLE;
    int32_t axisW = 0, axisH = 0;

    // Optional refresh-rate extension
    bool hasRefreshRateExt = false;

    uint64_t frameCount = 0;
    TeleopServer server;
};

static const char* SessionStateName(XrSessionState s) {
    switch (s) {
        case XR_SESSION_STATE_IDLE: return "IDLE";
        case XR_SESSION_STATE_READY: return "READY";
        case XR_SESSION_STATE_SYNCHRONIZED: return "SYNCHRONIZED";
        case XR_SESSION_STATE_VISIBLE: return "VISIBLE";
        case XR_SESSION_STATE_FOCUSED: return "FOCUSED";
        case XR_SESSION_STATE_STOPPING: return "STOPPING";
        case XR_SESSION_STATE_LOSS_PENDING: return "LOSS_PENDING";
        case XR_SESSION_STATE_EXITING: return "EXITING";
        default: return "UNKNOWN";
    }
}

// ---------------------------------------------------------------------------
// Persistent spatial anchor
//
// STAGE has no absolute yaw: it follows the guardian, which silently rotates
// whenever the boundary is redrawn or relocalization fails after a reboot —
// breaking the host's XR->robot yaw calibration. The fix: once the operator
// has teleop axes calibrated, {"cmd":"anchor_save"} creates a spatial anchor
// at the CURRENT stage origin (identity pose) and persists it to headset
// storage; the uuid is cached in internalDataPath. On every later run the
// anchor is loaded back and, while locatable, all poses are reported
// relative to it — the calibrated frame is pinned to the physical room.
// While the anchor is missing (room not recognized yet) reporting falls
// back to stage and the query retries every 10 s.
// ---------------------------------------------------------------------------

static void ApplyHaptic(AppState& s, int side, double amp, double ms);

static void LoadAnchorFunctions(AppState& s) {
    if (!s.hasAnchorExt) return;
    bool ok = true;
    auto load = [&](const char* name, PFN_xrVoidFunction* fn) {
        if (XR_FAILED(xrGetInstanceProcAddr(s.instance, name, fn)) || *fn == nullptr) {
            ALOGE("anchor: %s unavailable", name);
            ok = false;
        }
    };
    load("xrCreateSpatialAnchorFB", (PFN_xrVoidFunction*)&s.pfnCreateSpatialAnchor);
    load("xrSetSpaceComponentStatusFB", (PFN_xrVoidFunction*)&s.pfnSetSpaceComponentStatus);
    load("xrSaveSpaceFB", (PFN_xrVoidFunction*)&s.pfnSaveSpace);
    load("xrEraseSpaceFB", (PFN_xrVoidFunction*)&s.pfnEraseSpace);
    load("xrQuerySpacesFB", (PFN_xrVoidFunction*)&s.pfnQuerySpaces);
    load("xrRetrieveSpaceQueryResultsFB",
         (PFN_xrVoidFunction*)&s.pfnRetrieveSpaceQueryResults);
    s.hasAnchorExt = ok;
}

static std::string UuidToHex(const XrUuidEXT& u) {
    char buf[XR_UUID_SIZE_EXT * 2 + 1];
    for (int i = 0; i < XR_UUID_SIZE_EXT; i++)
        snprintf(buf + i * 2, 3, "%02x", u.data[i]);
    return std::string(buf);
}

static bool HexToUuid(const char* hex, XrUuidEXT* out) {
    if (strlen(hex) < XR_UUID_SIZE_EXT * 2) return false;
    for (int i = 0; i < XR_UUID_SIZE_EXT; i++) {
        unsigned v = 0;
        if (sscanf(hex + i * 2, "%2x", &v) != 1) return false;
        out->data[i] = (uint8_t)v;
    }
    return true;
}

static void LoadAnchorUuidFile(AppState& s) {
    s.anchorUuidValid = false;
    if (s.anchorUuidPath.empty()) return;
    FILE* f = fopen(s.anchorUuidPath.c_str(), "r");
    if (f == nullptr) return;
    char buf[64] = {0};
    fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (HexToUuid(buf, &s.anchorUuid)) {
        s.anchorUuidValid = true;
        ALOGI("anchor: cached uuid %s", buf);
    }
}

static void WriteAnchorUuidFile(AppState& s) {
    if (s.anchorUuidPath.empty()) return;
    FILE* f = fopen(s.anchorUuidPath.c_str(), "w");
    if (f == nullptr) {
        ALOGE("anchor: cannot write %s", s.anchorUuidPath.c_str());
        return;
    }
    fputs(UuidToHex(s.anchorUuid).c_str(), f);
    fclose(f);
}

// Returns +1 if the component was already enabled, 0 if a request is now
// pending (completion event will fire), -1 on error.
static int TryEnableComponent(AppState& s, XrSpace space, XrSpaceComponentTypeFB type) {
    XrSpaceComponentStatusSetInfoFB info{XR_TYPE_SPACE_COMPONENT_STATUS_SET_INFO_FB};
    info.componentType = type;
    info.enabled = XR_TRUE;
    info.timeout = 0;
    XrResult res = s.pfnSetSpaceComponentStatus(space, &info, &s.anchorRequest);
    if (res == XR_ERROR_SPACE_COMPONENT_STATUS_ALREADY_SET_FB) return 1;
    if (XR_FAILED(res)) {
        ALOGE("anchor: xrSetSpaceComponentStatusFB(%d) failed: %d", (int)type, res);
        return -1;
    }
    return 0;
}

static void AnchorAbort(AppState& s, const char* why) {
    ALOGW("anchor: %s", why);
    if (s.pendingAnchorSpace != XR_NULL_HANDLE) {
        xrDestroySpace(s.pendingAnchorSpace);
        s.pendingAnchorSpace = XR_NULL_HANDLE;
    }
    s.anchorPhase = AppState::AnchorPhase::Idle;
    s.anchorRequest = 0;
}

// Best-effort storage erase of a replaced/cleared anchor so orphans don't
// accumulate; the space handle is destroyed when the erase completes.
static void EraseAnchorSpace(AppState& s, XrSpace space) {
    if (space == XR_NULL_HANDLE) return;
    if (s.eraseSpace == XR_NULL_HANDLE) {
        XrSpaceEraseInfoFB info{XR_TYPE_SPACE_ERASE_INFO_FB};
        info.space = space;
        info.location = XR_SPACE_STORAGE_LOCATION_LOCAL_FB;
        if (XR_SUCCEEDED(s.pfnEraseSpace(s.session, &info, &s.eraseRequest))) {
            s.eraseSpace = space;
            return;
        }
    }
    xrDestroySpace(space);  // an erase is already running (or failed): just free
}

static void ActivateAnchor(AppState& s, const char* how) {
    if (s.anchorSpace != XR_NULL_HANDLE && s.anchorSpace != s.pendingAnchorSpace)
        EraseAnchorSpace(s, s.anchorSpace);  // re-calibration replaced it
    s.anchorSpace = s.pendingAnchorSpace;
    s.pendingAnchorSpace = XR_NULL_HANDLE;
    s.anchorUuid = s.pendingUuid;
    s.anchorUuidValid = true;
    s.anchorActive = true;
    s.anchorPhase = AppState::AnchorPhase::Idle;
    s.anchorRequest = 0;
    ALOGI("anchor: active (%s, uuid %s)", how, UuidToHex(s.anchorUuid).c_str());
}

static void StartAnchorPersist(AppState& s) {
    XrSpaceSaveInfoFB info{XR_TYPE_SPACE_SAVE_INFO_FB};
    info.space = s.pendingAnchorSpace;
    info.location = XR_SPACE_STORAGE_LOCATION_LOCAL_FB;
    info.persistenceMode = XR_SPACE_PERSISTENCE_MODE_INDEFINITE_FB;
    if (XR_FAILED(s.pfnSaveSpace(s.session, &info, &s.anchorRequest))) {
        AnchorAbort(s, "xrSaveSpaceFB failed");
        return;
    }
    s.anchorPhase = AppState::AnchorPhase::Saving;
}

static bool StartAnchorCreatePose(AppState& s, const XrPosef& poseInBase) {
    if (!s.hasAnchorExt) {
        ALOGW("anchor: spatial entity extensions unavailable");
        return false;
    }
    if (s.anchorPhase != AppState::AnchorPhase::Idle) {
        ALOGW("anchor: operation in flight; ignoring create");
        return false;
    }
    if (s.lastDisplayTime == 0) {
        ALOGW("anchor: no frame time yet; ignoring create");
        return false;
    }
    XrSpatialAnchorCreateInfoFB info{XR_TYPE_SPATIAL_ANCHOR_CREATE_INFO_FB};
    info.space = s.baseSpace;
    info.poseInSpace = poseInBase;
    info.time = s.lastDisplayTime;
    if (XR_FAILED(s.pfnCreateSpatialAnchor(s.session, &info, &s.anchorRequest))) {
        ALOGE("anchor: xrCreateSpatialAnchorFB failed");
        return false;
    }
    s.anchorPhase = AppState::AnchorPhase::Creating;
    return true;
}

// {"cmd":"anchor_save"}: pin the current stage frame as-is. Poses keep the
// same numeric values at the moment of the switch (anchor == stage right
// now), so the host's live yaw_offset_deg calibration carries over.
static void StartAnchorCreate(AppState& s) {
    XrPosef identity{};
    identity.orientation.w = 1.0f;
    if (StartAnchorCreatePose(s, identity))
        ALOGI("anchor: creating at current %s origin", s.usingStage ? "stage" : "local");
}

// -- Two-point X-axis calibration -------------------------------------------

static void CalibStart(AppState& s, int side) {
    if (!s.hasAnchorExt) {
        ALOGW("calib: spatial anchors unavailable; cannot calibrate");
        return;
    }
    if (s.calibStep == 0)
        memcpy(s.calibSavedColor, s.clearColor, sizeof(s.clearColor));
    s.calibStep = 1;
    s.calibSide = side;
    s.calibTriggerHigh = true;  // require a release before the first click
    s.calibPreviewValid = false;
    s.calibDeadline = NowMonotonic() + 120.0;
    // Soft blue tint = "calibrating" (restored on finish/cancel).
    s.clearColor[0] = 0.05f;
    s.clearColor[1] = 0.15f;
    s.clearColor[2] = 0.45f;
    s.clearColor[3] = s.passthroughActive ? 0.25f : 1.0f;
    ApplyHaptic(s, side, 0.6, 200);
    ALOGI("calib: armed (%s hand) — trigger at point 1, then at point 2 along robot +X",
          side == LEFT ? "left" : "right");
}

static void CalibEnd(AppState& s, const char* why, bool ok) {
    memcpy(s.clearColor, s.calibSavedColor, sizeof(s.clearColor));
    s.calibStep = 0;
    s.calibPreviewValid = false;
    if (!ok) ApplyHaptic(s, s.calibSide, 1.0, 60);
    ALOGI("calib: %s", why);
}

static void StartAnchorQuery(AppState& s) {
    XrSpaceStorageLocationFilterInfoFB locFilter{
        XR_TYPE_SPACE_STORAGE_LOCATION_FILTER_INFO_FB};
    locFilter.location = XR_SPACE_STORAGE_LOCATION_LOCAL_FB;
    XrSpaceUuidFilterInfoFB uuidFilter{XR_TYPE_SPACE_UUID_FILTER_INFO_FB};
    uuidFilter.next = &locFilter;
    uuidFilter.uuidCount = 1;
    uuidFilter.uuids = &s.anchorUuid;
    XrSpaceQueryInfoFB query{XR_TYPE_SPACE_QUERY_INFO_FB};
    query.queryAction = XR_SPACE_QUERY_ACTION_LOAD_FB;
    query.maxResultCount = 1;
    query.timeout = 0;
    query.filter = (const XrSpaceFilterInfoBaseHeaderFB*)&uuidFilter;
    XrResult res = s.pfnQuerySpaces(
        s.session, (const XrSpaceQueryInfoBaseHeaderFB*)&query, &s.anchorRequest);
    if (XR_FAILED(res)) {
        ALOGW("anchor: xrQuerySpacesFB failed: %d (retry in 10 s)", res);
        s.nextAnchorQueryRetry = NowMonotonic() + 10.0;
        return;
    }
    s.anchorPhase = AppState::AnchorPhase::Querying;
}

// {"cmd":"anchor_clear"}: forget the anchor and go back to stage reporting.
static void AnchorClear(AppState& s) {
    if (!s.anchorUuidPath.empty()) unlink(s.anchorUuidPath.c_str());
    s.anchorUuidValid = false;
    s.anchorActive = false;
    if (s.pendingAnchorSpace != XR_NULL_HANDLE) {  // abandon any op in flight
        xrDestroySpace(s.pendingAnchorSpace);
        s.pendingAnchorSpace = XR_NULL_HANDLE;
    }
    s.anchorPhase = AppState::AnchorPhase::Idle;
    s.anchorRequest = 0;
    if (s.anchorSpace != XR_NULL_HANDLE) {
        EraseAnchorSpace(s, s.anchorSpace);
        s.anchorSpace = XR_NULL_HANDLE;
    }
    ALOGI("anchor: cleared (reporting in %s)", s.usingStage ? "stage" : "local");
}

// ---------------------------------------------------------------------------
// EGL bootstrap (context only; we render into swapchain images via FBO)
// ---------------------------------------------------------------------------

static bool InitEGL(AppState& s) {
    s.eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (s.eglDisplay == EGL_NO_DISPLAY) {
        ALOGE("eglGetDisplay failed");
        return false;
    }
    if (!eglInitialize(s.eglDisplay, nullptr, nullptr)) {
        ALOGE("eglInitialize failed");
        return false;
    }
    const EGLint configAttribs[] = {
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 0,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_NONE};
    EGLint numConfigs = 0;
    if (!eglChooseConfig(s.eglDisplay, configAttribs, &s.eglConfig, 1, &numConfigs) ||
        numConfigs == 0) {
        ALOGE("eglChooseConfig failed");
        return false;
    }
    const EGLint pbufferAttribs[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    s.eglSurface = eglCreatePbufferSurface(s.eglDisplay, s.eglConfig, pbufferAttribs);
    const EGLint contextAttribs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
    s.eglContext = eglCreateContext(s.eglDisplay, s.eglConfig, EGL_NO_CONTEXT, contextAttribs);
    if (s.eglContext == EGL_NO_CONTEXT) {
        ALOGE("eglCreateContext failed");
        return false;
    }
    if (!eglMakeCurrent(s.eglDisplay, s.eglSurface, s.eglSurface, s.eglContext)) {
        ALOGE("eglMakeCurrent failed");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// OpenXR bootstrap
// ---------------------------------------------------------------------------

static bool InitLoader(android_app* app) {
    PFN_xrInitializeLoaderKHR initializeLoader = nullptr;
    XrResult res = xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                                         (PFN_xrVoidFunction*)&initializeLoader);
    if (XR_FAILED(res) || initializeLoader == nullptr) {
        ALOGE("xrInitializeLoaderKHR unavailable");
        return false;
    }
    XrLoaderInitInfoAndroidKHR loaderInfo{XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
    loaderInfo.applicationVM = app->activity->vm;
    loaderInfo.applicationContext = app->activity->clazz;
    return XrOk(initializeLoader((const XrLoaderInitInfoBaseHeaderKHR*)&loaderInfo),
                "xrInitializeLoaderKHR");
}

static bool CreateInstance(AppState& s) {
    // Probe optional extensions.
    uint32_t extCount = 0;
    xrEnumerateInstanceExtensionProperties(nullptr, 0, &extCount, nullptr);
    std::vector<XrExtensionProperties> exts(extCount, {XR_TYPE_EXTENSION_PROPERTIES});
    xrEnumerateInstanceExtensionProperties(nullptr, extCount, &extCount, exts.data());
    bool extSpatial = false, extStorage = false, extQuery = false;
    for (const auto& e : exts) {
        if (strcmp(e.extensionName, "XR_FB_display_refresh_rate") == 0)
            s.hasRefreshRateExt = true;
        if (strcmp(e.extensionName, XR_FB_PASSTHROUGH_EXTENSION_NAME) == 0)
            s.hasPassthroughExt = true;
        if (strcmp(e.extensionName, XR_FB_SPATIAL_ENTITY_EXTENSION_NAME) == 0)
            extSpatial = true;
        if (strcmp(e.extensionName, XR_FB_SPATIAL_ENTITY_STORAGE_EXTENSION_NAME) == 0)
            extStorage = true;
        if (strcmp(e.extensionName, XR_FB_SPATIAL_ENTITY_QUERY_EXTENSION_NAME) == 0)
            extQuery = true;
    }
    s.hasAnchorExt = extSpatial && extStorage && extQuery;

    std::vector<const char*> enabled = {
        XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,
        XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME,
    };
    if (s.hasRefreshRateExt) enabled.push_back("XR_FB_display_refresh_rate");
    if (s.hasPassthroughExt) enabled.push_back(XR_FB_PASSTHROUGH_EXTENSION_NAME);
    if (s.hasAnchorExt) {
        enabled.push_back(XR_FB_SPATIAL_ENTITY_EXTENSION_NAME);
        enabled.push_back(XR_FB_SPATIAL_ENTITY_STORAGE_EXTENSION_NAME);
        enabled.push_back(XR_FB_SPATIAL_ENTITY_QUERY_EXTENSION_NAME);
    }

    XrInstanceCreateInfoAndroidKHR androidInfo{XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
    androidInfo.applicationVM = s.app->activity->vm;
    androidInfo.applicationActivity = s.app->activity->clazz;

    XrInstanceCreateInfo ci{XR_TYPE_INSTANCE_CREATE_INFO};
    ci.next = &androidInfo;
    snprintf(ci.applicationInfo.applicationName, XR_MAX_APPLICATION_NAME_SIZE,
             "PiperQuestTeleop");
    ci.applicationInfo.applicationVersion = 1;
    snprintf(ci.applicationInfo.engineName, XR_MAX_ENGINE_NAME_SIZE, "none");
    ci.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    ci.enabledExtensionCount = (uint32_t)enabled.size();
    ci.enabledExtensionNames = enabled.data();
    OXR(xrCreateInstance(&ci, &s.instance));

    XrInstanceProperties props{XR_TYPE_INSTANCE_PROPERTIES};
    if (XR_SUCCEEDED(xrGetInstanceProperties(s.instance, &props)))
        ALOGI("OpenXR runtime: %s", props.runtimeName);

    LoadAnchorFunctions(s);
    ALOGI("spatial anchors: %s", s.hasAnchorExt ? "supported" : "UNAVAILABLE");

    XrSystemGetInfo sysInfo{XR_TYPE_SYSTEM_GET_INFO};
    sysInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    OXR(xrGetSystem(s.instance, &sysInfo, &s.systemId));
    return true;
}

static bool CreateSession(AppState& s) {
    // Mandatory before session creation for the GLES plugin.
    PFN_xrGetOpenGLESGraphicsRequirementsKHR getReqs = nullptr;
    OXR(xrGetInstanceProcAddr(s.instance, "xrGetOpenGLESGraphicsRequirementsKHR",
                              (PFN_xrVoidFunction*)&getReqs));
    XrGraphicsRequirementsOpenGLESKHR reqs{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
    OXR(getReqs(s.instance, s.systemId, &reqs));

    XrGraphicsBindingOpenGLESAndroidKHR binding{XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
    binding.display = s.eglDisplay;
    binding.config = s.eglConfig;
    binding.context = s.eglContext;

    XrSessionCreateInfo ci{XR_TYPE_SESSION_CREATE_INFO};
    ci.next = &binding;
    ci.systemId = s.systemId;
    OXR(xrCreateSession(s.instance, &ci, &s.session));

    // Reference space: prefer STAGE (floor-level, stable across sessions).
    uint32_t spaceCount = 0;
    xrEnumerateReferenceSpaces(s.session, 0, &spaceCount, nullptr);
    std::vector<XrReferenceSpaceType> spaces(spaceCount);
    xrEnumerateReferenceSpaces(s.session, spaceCount, &spaceCount, spaces.data());
    s.usingStage = false;
    for (auto sp : spaces)
        if (sp == XR_REFERENCE_SPACE_TYPE_STAGE) s.usingStage = true;

    XrReferenceSpaceCreateInfo rsci{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rsci.referenceSpaceType = s.usingStage ? XR_REFERENCE_SPACE_TYPE_STAGE
                                           : XR_REFERENCE_SPACE_TYPE_LOCAL;
    rsci.poseInReferenceSpace.orientation.w = 1.0f;
    OXR(xrCreateReferenceSpace(s.session, &rsci, &s.baseSpace));
    ALOGI("Base space: %s", s.usingStage ? "STAGE" : "LOCAL");

    XrReferenceSpaceCreateInfo vsci{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    vsci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    vsci.poseInReferenceSpace.orientation.w = 1.0f;
    OXR(xrCreateReferenceSpace(s.session, &vsci, &s.viewSpace));
    return true;
}

static void InitPassthrough(AppState& s) {
    if (!s.hasPassthroughExt) {
        ALOGW("XR_FB_passthrough not available; opaque background");
        return;
    }
    PFN_xrCreatePassthroughFB createPassthrough = nullptr;
    PFN_xrCreatePassthroughLayerFB createLayer = nullptr;
    xrGetInstanceProcAddr(s.instance, "xrCreatePassthroughFB",
                          (PFN_xrVoidFunction*)&createPassthrough);
    xrGetInstanceProcAddr(s.instance, "xrCreatePassthroughLayerFB",
                          (PFN_xrVoidFunction*)&createLayer);
    if (!createPassthrough || !createLayer) return;

    XrPassthroughCreateInfoFB pci{XR_TYPE_PASSTHROUGH_CREATE_INFO_FB};
    pci.flags = XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB;
    if (XR_FAILED(createPassthrough(s.session, &pci, &s.passthrough))) {
        ALOGW("xrCreatePassthroughFB failed; opaque background");
        return;
    }
    XrPassthroughLayerCreateInfoFB lci{XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB};
    lci.passthrough = s.passthrough;
    lci.purpose = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB;
    lci.flags = XR_PASSTHROUGH_IS_RUNNING_AT_CREATION_BIT_FB;
    if (XR_FAILED(createLayer(s.session, &lci, &s.passthroughLayer))) {
        ALOGW("xrCreatePassthroughLayerFB failed; opaque background");
        return;
    }
    s.passthroughActive = true;
    ALOGI("passthrough active");
}

static bool CreateSwapchains(AppState& s) {
    uint32_t viewCount = 0;
    OXR(xrEnumerateViewConfigurationViews(s.instance, s.systemId,
                                          XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0,
                                          &viewCount, nullptr));
    s.viewConfigs.resize(viewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
    OXR(xrEnumerateViewConfigurationViews(s.instance, s.systemId,
                                          XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, viewCount,
                                          &viewCount, s.viewConfigs.data()));
    if (viewCount != 2) {
        ALOGE("expected 2 views, got %u", viewCount);
        return false;
    }

    uint32_t formatCount = 0;
    xrEnumerateSwapchainFormats(s.session, 0, &formatCount, nullptr);
    std::vector<int64_t> formats(formatCount);
    xrEnumerateSwapchainFormats(s.session, formatCount, &formatCount, formats.data());
    int64_t chosen = formats.empty() ? GL_SRGB8_ALPHA8 : formats[0];
    for (int64_t f : formats)
        if (f == GL_SRGB8_ALPHA8) chosen = f;

    for (int eye = 0; eye < 2; eye++) {
        // Content is a solid color; quarter resolution saves power.
        int32_t w = (int32_t)s.viewConfigs[eye].recommendedImageRectWidth / 2;
        int32_t h = (int32_t)s.viewConfigs[eye].recommendedImageRectHeight / 2;
        XrSwapchainCreateInfo sci{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        sci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
        sci.format = chosen;
        sci.sampleCount = 1;
        sci.width = w;
        sci.height = h;
        sci.faceCount = 1;
        sci.arraySize = 1;
        sci.mipCount = 1;
        OXR(xrCreateSwapchain(s.session, &sci, &s.swapchains[eye].handle));
        s.swapchains[eye].width = w;
        s.swapchains[eye].height = h;

        uint32_t imgCount = 0;
        xrEnumerateSwapchainImages(s.swapchains[eye].handle, 0, &imgCount, nullptr);
        s.swapchains[eye].images.resize(imgCount, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR});
        OXR(xrEnumerateSwapchainImages(s.swapchains[eye].handle, imgCount, &imgCount,
                                       (XrSwapchainImageBaseHeader*)s.swapchains[eye].images.data()));
    }
    glGenFramebuffers(1, &s.fbo);
    return true;
}

static bool InitActions(AppState& s) {
    OXR(xrStringToPath(s.instance, "/user/hand/left", &s.handPaths[LEFT]));
    OXR(xrStringToPath(s.instance, "/user/hand/right", &s.handPaths[RIGHT]));

    XrActionSetCreateInfo asci{XR_TYPE_ACTION_SET_CREATE_INFO};
    snprintf(asci.actionSetName, XR_MAX_ACTION_SET_NAME_SIZE, "teleop");
    snprintf(asci.localizedActionSetName, XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE, "Teleop");
    OXR(xrCreateActionSet(s.instance, &asci, &s.actionSet));

    auto makeAction = [&](const char* name, XrActionType type, XrAction* out) -> bool {
        XrActionCreateInfo aci{XR_TYPE_ACTION_CREATE_INFO};
        aci.actionType = type;
        snprintf(aci.actionName, XR_MAX_ACTION_NAME_SIZE, "%s", name);
        snprintf(aci.localizedActionName, XR_MAX_LOCALIZED_ACTION_NAME_SIZE, "%s", name);
        aci.countSubactionPaths = SIDE_COUNT;
        aci.subactionPaths = s.handPaths;
        return XrOk(xrCreateAction(s.actionSet, &aci, out), name);
    };

    if (!makeAction("grip_pose", XR_ACTION_TYPE_POSE_INPUT, &s.gripPoseAction)) return false;
    if (!makeAction("trigger", XR_ACTION_TYPE_FLOAT_INPUT, &s.triggerAction)) return false;
    if (!makeAction("squeeze", XR_ACTION_TYPE_FLOAT_INPUT, &s.squeezeAction)) return false;
    if (!makeAction("thumbstick", XR_ACTION_TYPE_VECTOR2F_INPUT, &s.stickAction)) return false;
    if (!makeAction("thumbstick_click", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.stickClickAction))
        return false;
    if (!makeAction("button_primary", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.primaryAction))
        return false;
    if (!makeAction("button_secondary", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.secondaryAction))
        return false;
    if (!makeAction("menu", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.menuAction)) return false;
    if (!makeAction("haptic", XR_ACTION_TYPE_VIBRATION_OUTPUT, &s.hapticAction)) return false;

    auto path = [&](const char* p) {
        XrPath xp = XR_NULL_PATH;
        xrStringToPath(s.instance, p, &xp);
        return xp;
    };

    std::vector<XrActionSuggestedBinding> bindings = {
        {s.gripPoseAction, path("/user/hand/left/input/grip/pose")},
        {s.gripPoseAction, path("/user/hand/right/input/grip/pose")},
        {s.triggerAction, path("/user/hand/left/input/trigger/value")},
        {s.triggerAction, path("/user/hand/right/input/trigger/value")},
        {s.squeezeAction, path("/user/hand/left/input/squeeze/value")},
        {s.squeezeAction, path("/user/hand/right/input/squeeze/value")},
        {s.stickAction, path("/user/hand/left/input/thumbstick")},
        {s.stickAction, path("/user/hand/right/input/thumbstick")},
        {s.stickClickAction, path("/user/hand/left/input/thumbstick/click")},
        {s.stickClickAction, path("/user/hand/right/input/thumbstick/click")},
        {s.primaryAction, path("/user/hand/left/input/x/click")},
        {s.primaryAction, path("/user/hand/right/input/a/click")},
        {s.secondaryAction, path("/user/hand/left/input/y/click")},
        {s.secondaryAction, path("/user/hand/right/input/b/click")},
        {s.menuAction, path("/user/hand/left/input/menu/click")},
        {s.hapticAction, path("/user/hand/left/output/haptic")},
        {s.hapticAction, path("/user/hand/right/output/haptic")},
    };

    XrInteractionProfileSuggestedBinding sugg{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    sugg.interactionProfile = path("/interaction_profiles/oculus/touch_controller");
    sugg.suggestedBindings = bindings.data();
    sugg.countSuggestedBindings = (uint32_t)bindings.size();
    OXR(xrSuggestInteractionProfileBindings(s.instance, &sugg));

    XrSessionActionSetsAttachInfo attach{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    attach.countActionSets = 1;
    attach.actionSets = &s.actionSet;
    OXR(xrAttachSessionActionSets(s.session, &attach));

    for (int side = 0; side < SIDE_COUNT; side++) {
        XrActionSpaceCreateInfo asci2{XR_TYPE_ACTION_SPACE_CREATE_INFO};
        asci2.action = s.gripPoseAction;
        asci2.subactionPath = s.handPaths[side];
        asci2.poseInActionSpace.orientation.w = 1.0f;
        OXR(xrCreateActionSpace(s.session, &asci2, &s.gripSpaces[side]));
    }
    return true;
}

static void TryRequestRefreshRate(AppState& s, float hz) {
    if (!s.hasRefreshRateExt) return;
    PFN_xrRequestDisplayRefreshRateFB requestRate = nullptr;
    xrGetInstanceProcAddr(s.instance, "xrRequestDisplayRefreshRateFB",
                          (PFN_xrVoidFunction*)&requestRate);
    if (requestRate && XR_SUCCEEDED(requestRate(s.session, hz)))
        ALOGI("requested display refresh rate %.0f Hz", hz);
}

// ---------------------------------------------------------------------------
// Event handling
// ---------------------------------------------------------------------------

static void HandleSessionStateChange(AppState& s, const XrEventDataSessionStateChanged& ev) {
    s.sessionState = ev.state;
    ALOGI("session state -> %s", SessionStateName(ev.state));
    switch (ev.state) {
        case XR_SESSION_STATE_READY: {
            XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO};
            bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            if (XrOk(xrBeginSession(s.session, &bi), "xrBeginSession")) {
                s.sessionRunning = true;
                TryRequestRefreshRate(s, 90.0f);
            }
            break;
        }
        case XR_SESSION_STATE_STOPPING:
            xrEndSession(s.session);
            s.sessionRunning = false;
            break;
        case XR_SESSION_STATE_EXITING:
        case XR_SESSION_STATE_LOSS_PENDING:
            s.sessionRunning = false;
            s.exitRequested = true;
            break;
        default:
            break;
    }
}

static void PollXrEvents(AppState& s) {
    for (;;) {
        XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
        XrResult res = xrPollEvent(s.instance, &ev);
        if (res != XR_SUCCESS) break;
        switch (ev.type) {
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED:
                HandleSessionStateChange(s, *(const XrEventDataSessionStateChanged*)&ev);
                break;
            case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING:
                s.exitRequested = true;
                break;
            case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
                // Fired by the system recenter gesture (long-press Meta).
                // STAGE space itself is unaffected, so teleop poses stay
                // stable — we only use this as a "bring panels to me" signal.
                ALOGI("recenter gesture -> repositioning camera panels");
                s.repositionPanels = true;
                break;
            case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: {
                XrInteractionProfileState st{XR_TYPE_INTERACTION_PROFILE_STATE};
                if (XR_SUCCEEDED(xrGetCurrentInteractionProfile(s.session, s.handPaths[RIGHT], &st)) &&
                    st.interactionProfile != XR_NULL_PATH) {
                    char buf[XR_MAX_PATH_LENGTH];
                    uint32_t len = 0;
                    xrPathToString(s.instance, st.interactionProfile, sizeof(buf), &len, buf);
                    ALOGI("interaction profile (right): %s", buf);
                }
                break;
            }
            case XR_TYPE_EVENT_DATA_SPATIAL_ANCHOR_CREATE_COMPLETE_FB: {
                const auto& e = *(const XrEventDataSpatialAnchorCreateCompleteFB*)&ev;
                if (s.anchorPhase != AppState::AnchorPhase::Creating ||
                    e.requestId != s.anchorRequest) {
                    // Stale/abandoned request: free the space the runtime made.
                    if (XR_SUCCEEDED(e.result) && e.space != XR_NULL_HANDLE)
                        xrDestroySpace(e.space);
                    break;
                }
                if (XR_FAILED(e.result)) {
                    ALOGE("anchor: create failed: %d", e.result);
                    AnchorAbort(s, "create failed");
                    break;
                }
                s.pendingAnchorSpace = e.space;
                s.pendingUuid = e.uuid;
                int r = TryEnableComponent(s, e.space, XR_SPACE_COMPONENT_TYPE_STORABLE_FB);
                if (r > 0) StartAnchorPersist(s);
                else if (r == 0) s.anchorPhase = AppState::AnchorPhase::NewStorable;
                else AnchorAbort(s, "enable storable failed");
                break;
            }
            case XR_TYPE_EVENT_DATA_SPACE_SET_STATUS_COMPLETE_FB: {
                const auto& e = *(const XrEventDataSpaceSetStatusCompleteFB*)&ev;
                if (e.requestId != s.anchorRequest) break;
                if (XR_FAILED(e.result)) {
                    ALOGE("anchor: component enable failed: %d", e.result);
                    if (s.anchorPhase == AppState::AnchorPhase::LoadLocatable ||
                        s.anchorPhase == AppState::AnchorPhase::LoadStorable)
                        s.nextAnchorQueryRetry = NowMonotonic() + 10.0;
                    AnchorAbort(s, "component enable failed");
                    break;
                }
                switch (s.anchorPhase) {
                    case AppState::AnchorPhase::NewStorable:
                        StartAnchorPersist(s);
                        break;
                    case AppState::AnchorPhase::LoadLocatable: {
                        int r = TryEnableComponent(s, s.pendingAnchorSpace,
                                                   XR_SPACE_COMPONENT_TYPE_STORABLE_FB);
                        if (r > 0) ActivateAnchor(s, "restored");
                        else if (r == 0) s.anchorPhase = AppState::AnchorPhase::LoadStorable;
                        else AnchorAbort(s, "enable storable failed");
                        break;
                    }
                    case AppState::AnchorPhase::LoadStorable:
                        ActivateAnchor(s, "restored");
                        break;
                    default:
                        break;
                }
                break;
            }
            case XR_TYPE_EVENT_DATA_SPACE_SAVE_COMPLETE_FB: {
                const auto& e = *(const XrEventDataSpaceSaveCompleteFB*)&ev;
                if (e.requestId != s.anchorRequest ||
                    s.anchorPhase != AppState::AnchorPhase::Saving)
                    break;
                if (XR_FAILED(e.result)) {
                    ALOGE("anchor: save failed: %d", e.result);
                    AnchorAbort(s, "save failed");
                    break;
                }
                ActivateAnchor(s, "saved");
                WriteAnchorUuidFile(s);
                ApplyHaptic(s, LEFT, 0.6, 150);   // confirm to the operator
                ApplyHaptic(s, RIGHT, 0.6, 150);
                break;
            }
            case XR_TYPE_EVENT_DATA_SPACE_QUERY_RESULTS_AVAILABLE_FB: {
                const auto& e = *(const XrEventDataSpaceQueryResultsAvailableFB*)&ev;
                if (e.requestId != s.anchorRequest ||
                    s.anchorPhase != AppState::AnchorPhase::Querying)
                    break;
                XrSpaceQueryResultsFB results{XR_TYPE_SPACE_QUERY_RESULTS_FB};
                if (XR_FAILED(s.pfnRetrieveSpaceQueryResults(s.session, e.requestId,
                                                             &results)) ||
                    results.resultCountOutput == 0)
                    break;  // QUERY_COMPLETE will reset the phase and retry
                std::vector<XrSpaceQueryResultFB> res(results.resultCountOutput);
                results.resultCapacityInput = (uint32_t)res.size();
                results.results = res.data();
                if (XR_FAILED(s.pfnRetrieveSpaceQueryResults(s.session, e.requestId,
                                                             &results)) ||
                    results.resultCountOutput == 0 || res[0].space == XR_NULL_HANDLE)
                    break;
                s.pendingAnchorSpace = res[0].space;
                s.pendingUuid = res[0].uuid;
                int r = TryEnableComponent(s, res[0].space,
                                           XR_SPACE_COMPONENT_TYPE_LOCATABLE_FB);
                if (r > 0) {
                    r = TryEnableComponent(s, res[0].space,
                                           XR_SPACE_COMPONENT_TYPE_STORABLE_FB);
                    if (r > 0) ActivateAnchor(s, "restored");
                    else if (r == 0) s.anchorPhase = AppState::AnchorPhase::LoadStorable;
                    else AnchorAbort(s, "enable storable failed");
                } else if (r == 0) {
                    s.anchorPhase = AppState::AnchorPhase::LoadLocatable;
                } else {
                    AnchorAbort(s, "enable locatable failed");
                }
                break;
            }
            case XR_TYPE_EVENT_DATA_SPACE_QUERY_COMPLETE_FB: {
                const auto& e = *(const XrEventDataSpaceQueryCompleteFB*)&ev;
                // Only meaningful if the query delivered no usable result
                // (otherwise anchorRequest has already moved on).
                if (e.requestId != s.anchorRequest ||
                    s.anchorPhase != AppState::AnchorPhase::Querying)
                    break;
                s.anchorPhase = AppState::AnchorPhase::Idle;
                s.anchorRequest = 0;
                s.nextAnchorQueryRetry = NowMonotonic() + 10.0;
                ALOGW("anchor: stored anchor not found (result %d) — room not "
                      "recognized yet? retrying in 10 s", e.result);
                break;
            }
            case XR_TYPE_EVENT_DATA_SPACE_ERASE_COMPLETE_FB: {
                const auto& e = *(const XrEventDataSpaceEraseCompleteFB*)&ev;
                if (e.requestId != s.eraseRequest) break;
                if (XR_FAILED(e.result))
                    ALOGW("anchor: erase of replaced anchor failed: %d", e.result);
                if (s.eraseSpace != XR_NULL_HANDLE) xrDestroySpace(s.eraseSpace);
                s.eraseSpace = XR_NULL_HANDLE;
                s.eraseRequest = 0;
                break;
            }
            default:
                break;
        }
    }
}

// ---------------------------------------------------------------------------
// Inbound commands
// ---------------------------------------------------------------------------

static void ApplyHaptic(AppState& s, int side, double amp, double ms) {
    XrHapticVibration vib{XR_TYPE_HAPTIC_VIBRATION};
    vib.amplitude = (float)fmin(fmax(amp, 0.0), 1.0);
    vib.duration = (XrDuration)(ms * 1e6);  // ms -> ns
    vib.frequency = XR_FREQUENCY_UNSPECIFIED;
    XrHapticActionInfo info{XR_TYPE_HAPTIC_ACTION_INFO};
    info.action = s.hapticAction;
    info.subactionPath = s.handPaths[side];
    xrApplyHapticFeedback(s.session, &info, (const XrHapticBaseHeader*)&vib);
}

static void ProcessCommands(AppState& s) {
    std::vector<std::string> lines;
    s.server.PollCommands(&lines);
    for (const auto& lineStr : lines) {
        const char* line = lineStr.c_str();
        char cmd[32];
        if (!JsonFindString(line, "cmd", cmd, sizeof(cmd))) continue;
        if (strcmp(cmd, "haptic") == 0) {
            char hand[16] = "both";
            JsonFindString(line, "hand", hand, sizeof(hand));
            double amp = 0.5, ms = 100.0;
            JsonFindDouble(line, "amp", &amp);
            JsonFindDouble(line, "ms", &ms);
            if (strcmp(hand, "left") == 0 || strcmp(hand, "both") == 0)
                ApplyHaptic(s, LEFT, amp, ms);
            if (strcmp(hand, "right") == 0 || strcmp(hand, "both") == 0)
                ApplyHaptic(s, RIGHT, amp, ms);
        } else if (strcmp(cmd, "panels_front") == 0) {
            s.repositionPanels = true;
        } else if (strcmp(cmd, "anchor_save") == 0) {
            StartAnchorCreate(s);
        } else if (strcmp(cmd, "anchor_clear") == 0) {
            AnchorClear(s);
        } else if (strcmp(cmd, "calibrate_x") == 0) {
            char hand[16] = "right";
            JsonFindString(line, "hand", hand, sizeof(hand));
            CalibStart(s, strcmp(hand, "left") == 0 ? LEFT : RIGHT);
        } else if (strcmp(cmd, "calibrate_cancel") == 0) {
            if (s.calibStep > 0) CalibEnd(s, "cancelled (host)", false);
        } else if (strcmp(cmd, "panel") == 0) {
            double cam = 0;
            JsonFindDouble(line, "cam", &cam);
            int i = (int)cam;
            if (i >= 0 && i < kMaxCams) {
                AppState::CamPanel& p = s.camPanels[i];
                double x = p.pose.position.x, y = p.pose.position.y,
                       z = p.pose.position.z, w = p.width, yawDeg = 0.0;
                JsonFindDouble(line, "x", &x);
                JsonFindDouble(line, "y", &y);
                JsonFindDouble(line, "z", &z);
                JsonFindDouble(line, "width", &w);
                JsonFindDouble(line, "yaw", &yawDeg);
                p.pose.position = {(float)x, (float)y, (float)z};
                double half = yawDeg * (M_PI / 180.0) / 2.0;  // about +y (up)
                p.pose.orientation = {0.0f, (float)sin(half), 0.0f, (float)cos(half)};
                p.width = (float)fmin(fmax(w, 0.1), 3.0);
            }
        } else if (strcmp(cmd, "color") == 0) {
            double r = 0.0, g = 0.0, b = 0.0, a = 0.35;
            JsonFindDouble(line, "r", &r);
            JsonFindDouble(line, "g", &g);
            JsonFindDouble(line, "b", &b);
            JsonFindDouble(line, "a", &a);  // tint opacity over passthrough
            s.clearColor[0] = (float)r;
            s.clearColor[1] = (float)g;
            s.clearColor[2] = (float)b;
            s.clearColor[3] = (float)fmin(fmax(a, 0.0), 1.0);
        }
    }
}

// ---------------------------------------------------------------------------
// Per-frame: sync inputs, locate poses, build + broadcast JSON
// ---------------------------------------------------------------------------

struct LocatedPose {
    XrPosef pose{};
    bool valid = false;
    bool tracked = false;
};

static LocatedPose Locate(XrSpace space, XrSpace base, XrTime t) {
    LocatedPose out;
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
    if (XR_FAILED(xrLocateSpace(space, base, t, &loc))) return out;
    const XrSpaceLocationFlags validBits =
        XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    const XrSpaceLocationFlags trackedBits =
        XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    out.valid = (loc.locationFlags & validBits) == validBits;
    out.tracked = (loc.locationFlags & trackedBits) == trackedBits;
    if (out.valid) out.pose = loc.pose;
    return out;
}

// -- X-axis arrow + calibration clicks --------------------------------------

// Arrow quad footprint (meters). Lies flat (normal up); +Y of the quad's
// texture is the arrow direction.
static const float kAxisArrowLen = 0.6f;
static const float kAxisArrowWidth = 0.15f;
// R_x(-90 deg): lays the quad flat with its +Y (arrow) along world -Z.
static const XrQuaternionf kAxisFlatQuat{-0.70710678f, 0.0f, 0.0f, 0.70710678f};

static XrQuaternionf QuatMul(const XrQuaternionf& a, const XrQuaternionf& b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// Arrow pose for a horizontal direction (dx,dz) starting at p1: centered
// half a length along the direction, flat, pointing along it.
static XrPosef AxisArrowPose(const XrVector3f& p1, float dx, float dz) {
    float phi = atan2f(-dx, -dz);  // yaw that maps local -Z onto (dx,dz)
    XrQuaternionf yawQ{0.0f, sinf(phi * 0.5f), 0.0f, cosf(phi * 0.5f)};
    XrPosef pose;
    pose.orientation = QuatMul(yawQ, kAxisFlatQuat);
    pose.position = {p1.x + dx * (kAxisArrowLen * 0.5f), p1.y,
                     p1.z + dz * (kAxisArrowLen * 0.5f)};
    return pose;
}

// Per-frame calibration input (calibrating hand only, poses in BASE space —
// the anchor must be created in the session's stage frame).
static void CalibHandleInput(AppState& s, const LocatedPose& grip, float trigger,
                             bool menuPressed) {
    if (menuPressed) {
        CalibEnd(s, "cancelled (menu button)", false);
        return;
    }
    // Live preview arrow while hunting for point 2.
    if (s.calibStep == 2 && grip.valid) {
        float dx = grip.pose.position.x - s.calibP1.x;
        float dz = grip.pose.position.z - s.calibP1.z;
        float n = sqrtf(dx * dx + dz * dz);
        if (n > 0.05f) {
            s.calibPreviewPose = AxisArrowPose(s.calibP1, dx / n, dz / n);
            s.calibPreviewValid = true;
        } else {
            s.calibPreviewValid = false;
        }
    }
    bool high = trigger > (s.calibTriggerHigh ? 0.3f : 0.7f);
    bool clicked = high && !s.calibTriggerHigh;
    s.calibTriggerHigh = high;
    if (!clicked || !grip.valid) return;

    if (s.calibStep == 1) {
        s.calibP1 = grip.pose.position;
        s.calibStep = 2;
        ApplyHaptic(s, s.calibSide, 0.6, 100);
        ALOGI("calib: point 1 latched (%.2f, %.2f, %.2f)",
              s.calibP1.x, s.calibP1.y, s.calibP1.z);
        return;
    }

    // Point 2: the horizontal P1->P2 line becomes robot +X.
    float dx = grip.pose.position.x - s.calibP1.x;
    float dz = grip.pose.position.z - s.calibP1.z;
    float n = sqrtf(dx * dx + dz * dz);
    if (n < 0.15f) {  // too short to define a direction reliably
        ALOGW("calib: points %.2f m apart horizontally; move further and re-click", n);
        ApplyHaptic(s, s.calibSide, 1.0, 40);
        return;
    }
    // Anchor -Z = P1->P2 direction; the host's fixed remap (robot_x = -xr_z)
    // then makes that direction robot +X. Up stays gravity-up.
    XrPosef pose{};
    float yaw = atan2f(-dx / n, -dz / n);
    pose.orientation.y = sinf(yaw * 0.5f);
    pose.orientation.w = cosf(yaw * 0.5f);
    pose.position = s.calibP1;
    if (StartAnchorCreatePose(s, pose)) {
        ALOGI("calib: X axis set (yaw %.1f deg in %s); saving anchor",
              yaw * 180.0 / M_PI, s.usingStage ? "stage" : "local");
        CalibEnd(s, "done — waiting for anchor save", true);
    } else {
        CalibEnd(s, "anchor create refused; try again", false);
    }
}

static int AppendPoseJson(char* buf, size_t cap, const LocatedPose& p) {
    return snprintf(buf, cap,
                    "\"pos\":[%.6f,%.6f,%.6f],\"quat\":[%.6f,%.6f,%.6f,%.6f],"
                    "\"valid\":%s,\"tracked\":%s",
                    p.pose.position.x, p.pose.position.y, p.pose.position.z,
                    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z,
                    p.pose.orientation.w, p.valid ? "true" : "false",
                    p.tracked ? "true" : "false");
}

static void SyncAndBroadcast(AppState& s, XrTime displayTime) {
    XrActiveActionSet active{s.actionSet, XR_NULL_PATH};
    XrActionsSyncInfo syncInfo{XR_TYPE_ACTIONS_SYNC_INFO};
    syncInfo.countActiveActionSets = 1;
    syncInfo.activeActionSets = &active;
    xrSyncActions(s.session, &syncInfo);  // XR_SESSION_NOT_FOCUSED is fine

    LocatedPose headBase = Locate(s.viewSpace, s.baseSpace, displayTime);
    s.lastHead = headBase.pose;  // panel placement stays in base space
    s.lastHeadValid = headBase.valid;

    // Reporting base: the persistent anchor while it is locatable, else the
    // session stage/local space. "space" tells the host which one it got.
    XrSpace reportBase = s.baseSpace;
    const char* spaceName = s.usingStage ? "stage" : "local";
    bool anchorTracked = false;
    if (s.anchorActive) {
        LocatedPose ap = Locate(s.anchorSpace, s.baseSpace, displayTime);
        if (ap.valid) {
            reportBase = s.anchorSpace;
            spaceName = "anchor";
            anchorTracked = ap.tracked;
        }
    }
    LocatedPose head = (reportBase == s.baseSpace)
                           ? headBase
                           : Locate(s.viewSpace, reportBase, displayTime);

    char buf[2048];
    size_t off = 0;
    off += snprintf(buf + off, sizeof(buf) - off,
                    "{\"t\":%.6f,\"mono\":%.6f,\"frame\":%llu,\"state\":\"%s\","
                    "\"space\":\"%s\","
                    "\"anchor\":{\"have\":%s,\"tracked\":%s},\"calib\":%d,\"head\":{",
                    (double)displayTime * 1e-9, NowMonotonic(),
                    (unsigned long long)s.frameCount, SessionStateName(s.sessionState),
                    spaceName, s.anchorUuidValid ? "true" : "false",
                    anchorTracked ? "true" : "false", s.calibStep);
    off += AppendPoseJson(buf + off, sizeof(buf) - off, head);
    off += snprintf(buf + off, sizeof(buf) - off, "}");

    static const char* kSideNames[SIDE_COUNT] = {"left", "right"};
    for (int side = 0; side < SIDE_COUNT; side++) {
        LocatedPose grip = Locate(s.gripSpaces[side], reportBase, displayTime);

        auto getFloat = [&](XrAction a) -> float {
            XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
            gi.action = a;
            gi.subactionPath = s.handPaths[side];
            XrActionStateFloat st{XR_TYPE_ACTION_STATE_FLOAT};
            if (XR_FAILED(xrGetActionStateFloat(s.session, &gi, &st)) || !st.isActive)
                return 0.0f;
            return st.currentState;
        };
        auto getBool = [&](XrAction a) -> bool {
            XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
            gi.action = a;
            gi.subactionPath = s.handPaths[side];
            XrActionStateBoolean st{XR_TYPE_ACTION_STATE_BOOLEAN};
            if (XR_FAILED(xrGetActionStateBoolean(s.session, &gi, &st)) || !st.isActive)
                return false;
            return st.currentState == XR_TRUE;
        };

        XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
        gi.action = s.stickAction;
        gi.subactionPath = s.handPaths[side];
        XrActionStateVector2f stick{XR_TYPE_ACTION_STATE_VECTOR2F};
        xrGetActionStateVector2f(s.session, &gi, &stick);
        float sx = stick.isActive ? stick.currentState.x : 0.0f;
        float sy = stick.isActive ? stick.currentState.y : 0.0f;

        float trigger = getFloat(s.triggerAction);
        float squeeze = getFloat(s.squeezeAction);
        bool stickClick = getBool(s.stickClickAction);
        bool primary = getBool(s.primaryAction);
        bool secondary = getBool(s.secondaryAction);
        bool menu = getBool(s.menuAction);

        if (s.calibStep > 0) {
            if (side == s.calibSide) {
                // Calibration points must live in BASE space (the anchor is
                // created there), not in a currently-active anchor frame.
                LocatedPose gripBase = Locate(s.gripSpaces[side], s.baseSpace, displayTime);
                CalibHandleInput(s, gripBase, trigger, menu);
            }
            // Mask live inputs while calibrating: the host must not clutch,
            // drive the gripper, or toggle recording from calibration clicks.
            grip = LocatedPose{};
            grip.pose.orientation.w = 1.0f;
            trigger = squeeze = sx = sy = 0.0f;
            stickClick = primary = secondary = menu = false;
        }

        off += snprintf(buf + off, sizeof(buf) - off, ",\"%s\":{", kSideNames[side]);
        off += AppendPoseJson(buf + off, sizeof(buf) - off, grip);
        off += snprintf(buf + off, sizeof(buf) - off,
                        ",\"trigger\":%.4f,\"squeeze\":%.4f,\"stick\":[%.4f,%.4f],"
                        "\"stick_click\":%s,\"primary\":%s,\"secondary\":%s,\"menu\":%s}",
                        trigger, squeeze, sx, sy,
                        stickClick ? "true" : "false",
                        primary ? "true" : "false",
                        secondary ? "true" : "false",
                        menu ? "true" : "false");
    }
    off += snprintf(buf + off, sizeof(buf) - off, "}\n");
    if (off >= sizeof(buf) - 1) {
        ALOGW("frame JSON truncated (%zu bytes)", off);
        return;
    }
    s.server.Broadcast(buf, off);
}

// ---------------------------------------------------------------------------
// Rendering (tint layer + optional passthrough + camera panels)
// ---------------------------------------------------------------------------

// Place both panels side by side, 0.9 m in front of the head at eye height,
// facing the user (yaw-only: panels stay upright).
static void PlacePanelsInFront(AppState& s) {
    if (!s.lastHeadValid) return;
    const XrQuaternionf& q = s.lastHead.orientation;
    // Gaze = head -Z axis, from the quaternion's third column (negated).
    float gx = -(2.0f * (q.x * q.z + q.w * q.y));
    float gz = -(1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    float n = sqrtf(gx * gx + gz * gz);
    if (n < 0.2f) return;  // looking straight up/down: keep previous placement
    gx /= n;
    gz /= n;
    float rx = -gz, rz = gx;  // right = gaze x up (y-up space)
    float cx = s.lastHead.position.x + gx * 0.9f;
    float cy = s.lastHead.position.y;
    float cz = s.lastHead.position.z + gz * 0.9f;
    // Panel front (+z of its pose) must point back toward the user (-gaze).
    float phi = atan2f(-gx, -gz);
    XrQuaternionf rot{0.0f, sinf(phi / 2.0f), 0.0f, cosf(phi / 2.0f)};
    static const float kCamOffsets[] = {-0.62f, 0.0f, 0.62f};
    for (int i = 0; i < kMaxCams; i++) {
        // Cameras in a row (config order, left to right); HUD centered below.
        float off = (i < kStatusPanel) ? kCamOffsets[i] : 0.0f;
        float dy = (i == kStatusPanel) ? -0.38f : 0.0f;
        s.camPanels[i].pose.position = {cx + rx * off, cy + dy, cz + rz * off};
        s.camPanels[i].pose.orientation = rot;
    }
    s.repositionPanels = false;
    ALOGI("panels placed in front of head (%.2f, %.2f, %.2f)", cx, cy, cz);
}

// Upload the newest decoded camera frames into their quad swapchains.
// Runs on the render thread (GL context current); ~1 ms per 640x480 upload.
static void UpdateCamPanels(AppState& s) {
    double now = NowMonotonic();
    for (int i = 0; i < kMaxCams; i++) {
        AppState::CamPanel& p = s.camPanels[i];
        VideoServer::Decoded& d = p.scratch;
        if (s.videoServer.Latest(i, &p.lastSeq, &d)) {
            if (p.swapchain == XR_NULL_HANDLE || p.w != d.w || p.h != d.h) {
                if (p.swapchain != XR_NULL_HANDLE) {
                    xrDestroySwapchain(p.swapchain);
                    p.swapchain = XR_NULL_HANDLE;
                    p.images.clear();
                    p.hasContent = false;
                }
                XrSwapchainCreateInfo sci{XR_TYPE_SWAPCHAIN_CREATE_INFO};
                sci.usageFlags =
                    XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
                sci.format = GL_SRGB8_ALPHA8;
                sci.sampleCount = 1;
                sci.width = d.w;
                sci.height = d.h;
                sci.faceCount = 1;
                sci.arraySize = 1;
                sci.mipCount = 1;
                if (XR_FAILED(xrCreateSwapchain(s.session, &sci, &p.swapchain))) {
                    ALOGE("cam panel %d: swapchain create failed", i);
                    continue;
                }
                p.w = d.w;
                p.h = d.h;
                uint32_t n = 0;
                xrEnumerateSwapchainImages(p.swapchain, 0, &n, nullptr);
                p.images.resize(n, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR});
                xrEnumerateSwapchainImages(p.swapchain, n, &n,
                                           (XrSwapchainImageBaseHeader*)p.images.data());
                ALOGI("cam panel %d: %dx%d swapchain (%u images)", i, d.w, d.h, n);
            }
            uint32_t idx = 0;
            XrSwapchainImageAcquireInfo acq{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
            if (XR_SUCCEEDED(xrAcquireSwapchainImage(p.swapchain, &acq, &idx))) {
                XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
                wi.timeout = XR_INFINITE_DURATION;
                xrWaitSwapchainImage(p.swapchain, &wi);
                glBindTexture(GL_TEXTURE_2D, p.images[idx].image);
                glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, p.w, p.h, GL_RGBA,
                                GL_UNSIGNED_BYTE, d.rgba.data());
                glBindTexture(GL_TEXTURE_2D, 0);
                XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                xrReleaseSwapchainImage(p.swapchain, &rel);
                p.hasContent = true;
            }
        }
        // Hide panels whose stream stopped (host closed / stalled).
        if (p.hasContent && now - s.videoServer.LastRecvMono(i) > 3.0)
            p.hasContent = false;
    }
}

// Lazily build the one-time static texture for the +X arrow quad: a faint
// translucent arrow (tip toward the quad's +Y), blended over passthrough.
static void EnsureAxisQuad(AppState& s) {
    if (s.axisSwapchain != XR_NULL_HANDLE) return;
    const int W = 96, H = 384;
    XrSwapchainCreateInfo sci{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    sci.createFlags = XR_SWAPCHAIN_CREATE_STATIC_IMAGE_BIT;
    sci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    sci.format = GL_SRGB8_ALPHA8;
    sci.sampleCount = 1;
    sci.width = W;
    sci.height = H;
    sci.faceCount = 1;
    sci.arraySize = 1;
    sci.mipCount = 1;
    if (XR_FAILED(xrCreateSwapchain(s.session, &sci, &s.axisSwapchain))) {
        ALOGE("axis arrow: swapchain create failed");
        s.axisSwapchain = XR_NULL_HANDLE;
        return;
    }
    s.axisW = W;
    s.axisH = H;
    uint32_t n = 0;
    xrEnumerateSwapchainImages(s.axisSwapchain, 0, &n, nullptr);
    std::vector<XrSwapchainImageOpenGLESKHR> images(n, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR});
    xrEnumerateSwapchainImages(s.axisSwapchain, n, &n,
                               (XrSwapchainImageBaseHeader*)images.data());

    // GL row 0 displays at the quad's -Y edge, so the tip goes at high rows.
    std::vector<uint8_t> px((size_t)W * H * 4, 0);
    const int cx = W / 2;
    const int headBase = (int)(H * 0.62f), tipY = H - 8;
    for (int y = 8; y < tipY; y++) {
        int half;
        if (y < headBase) {
            half = W / 9;  // shaft
        } else {           // head tapers to the tip
            float f = (float)(tipY - y) / (float)(tipY - headBase);
            half = (int)(W * 0.42f * f);
        }
        for (int x = cx - half; x <= cx + half; x++) {
            if (x < 0 || x >= W) continue;
            uint8_t* p = &px[((size_t)y * W + x) * 4];
            p[0] = 140;
            p[1] = 220;
            p[2] = 255;
            p[3] = 96;  // faint — must not obstruct the view
        }
    }

    uint32_t idx = 0;
    XrSwapchainImageAcquireInfo acq{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    if (XR_SUCCEEDED(xrAcquireSwapchainImage(s.axisSwapchain, &acq, &idx))) {
        XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
        wi.timeout = XR_INFINITE_DURATION;
        xrWaitSwapchainImage(s.axisSwapchain, &wi);
        glBindTexture(GL_TEXTURE_2D, images[idx].image);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
        glBindTexture(GL_TEXTURE_2D, 0);
        XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        xrReleaseSwapchainImage(s.axisSwapchain, &rel);
    }
    ALOGI("axis arrow quad ready");
}

static bool RenderFrame(AppState& s) {
    XrFrameWaitInfo waitInfo{XR_TYPE_FRAME_WAIT_INFO};
    XrFrameState frameState{XR_TYPE_FRAME_STATE};
    OXR(xrWaitFrame(s.session, &waitInfo, &frameState));

    XrFrameBeginInfo beginInfo{XR_TYPE_FRAME_BEGIN_INFO};
    OXR(xrBeginFrame(s.session, &beginInfo));

    s.frameCount++;
    s.lastDisplayTime = frameState.predictedDisplayTime;
    ProcessCommands(s);
    // Keep trying to load the cached anchor while it is unresolved:
    // relocalization often kicks in seconds after boot (or once the user
    // looks around the room).
    if (s.hasAnchorExt && s.anchorUuidValid && !s.anchorActive &&
        s.anchorPhase == AppState::AnchorPhase::Idle &&
        NowMonotonic() >= s.nextAnchorQueryRetry)
        StartAnchorQuery(s);
    if (s.calibStep > 0 && NowMonotonic() > s.calibDeadline)
        CalibEnd(s, "timed out", false);
    SyncAndBroadcast(s, frameState.predictedDisplayTime);

    XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    XrCompositionLayerPassthroughFB ptLayer{XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB};
    XrCompositionLayerQuad quadLayers[kMaxCams];
    XrCompositionLayerQuad axisLayer{XR_TYPE_COMPOSITION_LAYER_QUAD};
    XrCompositionLayerProjectionView projViews[2] = {
        {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},
        {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
    const XrCompositionLayerBaseHeader* layers[3 + kMaxCams] = {};

    XrFrameEndInfo endInfo{XR_TYPE_FRAME_END_INFO};
    endInfo.displayTime = frameState.predictedDisplayTime;
    endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    endInfo.layerCount = 0;

    if (frameState.shouldRender) {
        XrViewLocateInfo vli{XR_TYPE_VIEW_LOCATE_INFO};
        vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        vli.displayTime = frameState.predictedDisplayTime;
        vli.space = s.baseSpace;
        XrViewState viewState{XR_TYPE_VIEW_STATE};
        XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
        uint32_t viewCount = 0;
        if (XR_SUCCEEDED(xrLocateViews(s.session, &vli, &viewState, 2, &viewCount, views)) &&
            viewCount == 2) {
            for (int eye = 0; eye < 2; eye++) {
                Swapchain& sc = s.swapchains[eye];
                uint32_t imgIndex = 0;
                XrSwapchainImageAcquireInfo acq{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
                xrAcquireSwapchainImage(sc.handle, &acq, &imgIndex);
                XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
                wi.timeout = XR_INFINITE_DURATION;
                xrWaitSwapchainImage(sc.handle, &wi);

                glBindFramebuffer(GL_FRAMEBUFFER, s.fbo);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                                       sc.images[imgIndex].image, 0);
                glViewport(0, 0, sc.width, sc.height);
                float alpha = s.passthroughActive ? s.clearColor[3] : 1.0f;
                glClearColor(s.clearColor[0], s.clearColor[1], s.clearColor[2], alpha);
                glClear(GL_COLOR_BUFFER_BIT);
                glBindFramebuffer(GL_FRAMEBUFFER, 0);

                XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                xrReleaseSwapchainImage(sc.handle, &rel);

                projViews[eye].pose = views[eye].pose;
                projViews[eye].fov = views[eye].fov;
                projViews[eye].subImage.swapchain = sc.handle;
                projViews[eye].subImage.imageRect.offset = {0, 0};
                projViews[eye].subImage.imageRect.extent = {sc.width, sc.height};
            }
            layer.space = s.baseSpace;
            layer.viewCount = 2;
            layer.views = projViews;
            if (s.passthroughActive) {
                // Passthrough (camera feed) underneath, tint layer on top.
                ptLayer.layerHandle = s.passthroughLayer;
                layer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
                                   XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
                layers[endInfo.layerCount++] = (const XrCompositionLayerBaseHeader*)&ptLayer;
            }
            layers[endInfo.layerCount++] = (const XrCompositionLayerBaseHeader*)&layer;

            if (s.repositionPanels) PlacePanelsInFront(s);
            UpdateCamPanels(s);
            for (int i = 0; i < kMaxCams; i++) {
                AppState::CamPanel& p = s.camPanels[i];
                if (!p.hasContent || p.swapchain == XR_NULL_HANDLE) continue;
                quadLayers[i] = {XR_TYPE_COMPOSITION_LAYER_QUAD};
                quadLayers[i].space = s.baseSpace;
                quadLayers[i].eyeVisibility = XR_EYE_VISIBILITY_BOTH;
                quadLayers[i].subImage.swapchain = p.swapchain;
                quadLayers[i].subImage.imageRect.offset = {0, 0};
                quadLayers[i].subImage.imageRect.extent = {p.w, p.h};
                quadLayers[i].pose = p.pose;
                quadLayers[i].size = {p.width, p.width * (float)p.h / (float)p.w};
                layers[endInfo.layerCount++] = (const XrCompositionLayerBaseHeader*)&quadLayers[i];
            }

            // Faint robot-+X arrow on the floor plane: live preview while
            // hunting calibration point 2, else pinned in anchor space (it
            // re-appears at the same physical spot every session).
            bool showAxis = false;
            if (s.calibStep == 2 && s.calibPreviewValid) {
                axisLayer.space = s.baseSpace;
                axisLayer.pose = s.calibPreviewPose;
                showAxis = true;
            } else if (s.calibStep == 0 && s.anchorActive) {
                axisLayer.space = s.anchorSpace;
                XrVector3f origin{0.0f, 0.0f, 0.0f};
                axisLayer.pose = AxisArrowPose(origin, 0.0f, -1.0f);  // -Z = robot +X
                showAxis = true;
            }
            if (showAxis) {
                EnsureAxisQuad(s);
                if (s.axisSwapchain != XR_NULL_HANDLE) {
                    axisLayer.layerFlags =
                        XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
                        XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
                    axisLayer.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
                    axisLayer.subImage.swapchain = s.axisSwapchain;
                    axisLayer.subImage.imageRect.offset = {0, 0};
                    axisLayer.subImage.imageRect.extent = {s.axisW, s.axisH};
                    axisLayer.size = {kAxisArrowWidth, kAxisArrowLen};
                    layers[endInfo.layerCount++] =
                        (const XrCompositionLayerBaseHeader*)&axisLayer;
                }
            }
            endInfo.layers = layers;
        }
    }
    OXR(xrEndFrame(s.session, &endInfo));
    return true;
}

// ---------------------------------------------------------------------------
// Android glue
// ---------------------------------------------------------------------------

static void OnAppCmd(android_app* app, int32_t cmd) {
    AppState* s = (AppState*)app->userData;
    switch (cmd) {
        case APP_CMD_RESUME:
            s->resumed = true;
            break;
        case APP_CMD_PAUSE:
            s->resumed = false;
            break;
        default:
            break;
    }
}

void android_main(android_app* app) {
    AppState s;
    s.app = app;
    app->userData = &s;
    app->onAppCmd = OnAppCmd;

    if (app->activity->internalDataPath != nullptr)
        s.anchorUuidPath = std::string(app->activity->internalDataPath) + "/anchor_uuid";
    LoadAnchorUuidFile(s);

    JNIEnv* env = nullptr;
    app->activity->vm->AttachCurrentThread(&env, nullptr);

    bool ok = InitLoader(app) && InitEGL(s) && CreateInstance(s) && CreateSession(s) &&
              CreateSwapchains(s) && InitActions(s);
    if (ok) InitPassthrough(s);  // optional; falls back to opaque background
    if (!ok) {
        ALOGE("initialization failed; exiting");
        ANativeActivity_finish(app->activity);
    } else {
        char hello[512];
        snprintf(hello, sizeof(hello),
                 "{\"hello\":{\"app\":\"piper-quest-teleop\",\"version\":\"%s\","
                 "\"protocol\":%d,\"port\":%d,\"video_port\":%d,\"anchors\":%s}}\n",
                 kAppVersion, kProtocolVersion, kPort, kVideoPort,
                 s.hasAnchorExt ? "true" : "false");
        s.server.Start(hello);
        s.videoServer.Start();
        // Fallback placement (PlacePanelsInFront overrides once the head is
        // tracked): cameras in a row, status strip below center.
        for (int i = 0; i < kMaxCams; i++) {
            s.camPanels[i].pose.orientation = {0, 0, 0, 1};
            float x = (i < kStatusPanel) ? -0.62f + 0.62f * i : 0.0f;
            float y = (i == kStatusPanel) ? 0.87f : 1.25f;
            s.camPanels[i].pose.position = {x, y, -0.85f};
        }
        s.camPanels[kStatusPanel].width = 0.5f;  // status strip is narrower
    }

    double lastHeartbeat = 0.0;
    while (!app->destroyRequested && !s.exitRequested) {
        // Drain Android lifecycle events. Block briefly when idle.
        for (;;) {
            int events = 0;
            android_poll_source* source = nullptr;
            int timeoutMs = (s.sessionRunning || app->destroyRequested) ? 0 : 100;
            int id = ALooper_pollOnce(timeoutMs, nullptr, &events, (void**)&source);
            if (id < 0) break;  // timeout or error -> continue main loop
            if (source) source->process(app, source);
            if (app->destroyRequested) break;
        }
        if (app->destroyRequested || !ok) break;

        PollXrEvents(s);

        if (s.sessionRunning) {
            if (!RenderFrame(s)) break;
        } else {
            // Not rendering: keep clients informed at ~5 Hz.
            double now = NowMonotonic();
            if (now - lastHeartbeat > 0.2) {
                lastHeartbeat = now;
                ProcessCommands(s);
                char hb[192];
                int n = snprintf(hb, sizeof(hb),
                                 "{\"heartbeat\":true,\"mono\":%.6f,\"state\":\"%s\"}\n", now,
                                 SessionStateName(s.sessionState));
                s.server.Broadcast(hb, (size_t)n);
            }
        }
    }

    // If the runtime asked us to exit (dash "quit"), tell Android to finish
    // the activity and drain remaining lifecycle events.
    if (s.exitRequested && !app->destroyRequested) {
        ANativeActivity_finish(app->activity);
        while (!app->destroyRequested) {
            int events = 0;
            android_poll_source* source = nullptr;
            if (ALooper_pollOnce(100, nullptr, &events, (void**)&source) < 0) continue;
            if (source) source->process(app, source);
        }
    }

    ALOGI("shutting down");
    s.server.Stop();
    s.videoServer.Stop();
    for (int i = 0; i < kMaxCams; i++)
        if (s.camPanels[i].swapchain != XR_NULL_HANDLE)
            xrDestroySwapchain(s.camPanels[i].swapchain);
    if (s.axisSwapchain != XR_NULL_HANDLE) xrDestroySwapchain(s.axisSwapchain);
    if (s.passthroughLayer != XR_NULL_HANDLE || s.passthrough != XR_NULL_HANDLE) {
        PFN_xrDestroyPassthroughLayerFB destroyLayer = nullptr;
        PFN_xrDestroyPassthroughFB destroyPt = nullptr;
        xrGetInstanceProcAddr(s.instance, "xrDestroyPassthroughLayerFB",
                              (PFN_xrVoidFunction*)&destroyLayer);
        xrGetInstanceProcAddr(s.instance, "xrDestroyPassthroughFB",
                              (PFN_xrVoidFunction*)&destroyPt);
        if (destroyLayer && s.passthroughLayer != XR_NULL_HANDLE)
            destroyLayer(s.passthroughLayer);
        if (destroyPt && s.passthrough != XR_NULL_HANDLE) destroyPt(s.passthrough);
    }
    for (int side = 0; side < SIDE_COUNT; side++)
        if (s.gripSpaces[side] != XR_NULL_HANDLE) xrDestroySpace(s.gripSpaces[side]);
    if (s.anchorSpace != XR_NULL_HANDLE) xrDestroySpace(s.anchorSpace);
    if (s.pendingAnchorSpace != XR_NULL_HANDLE) xrDestroySpace(s.pendingAnchorSpace);
    if (s.eraseSpace != XR_NULL_HANDLE) xrDestroySpace(s.eraseSpace);
    if (s.actionSet != XR_NULL_HANDLE) xrDestroyActionSet(s.actionSet);
    for (int eye = 0; eye < 2; eye++)
        if (s.swapchains[eye].handle != XR_NULL_HANDLE) xrDestroySwapchain(s.swapchains[eye].handle);
    if (s.fbo) glDeleteFramebuffers(1, &s.fbo);
    if (s.baseSpace != XR_NULL_HANDLE) xrDestroySpace(s.baseSpace);
    if (s.viewSpace != XR_NULL_HANDLE) xrDestroySpace(s.viewSpace);
    if (s.session != XR_NULL_HANDLE) xrDestroySession(s.session);
    if (s.instance != XR_NULL_HANDLE) xrDestroyInstance(s.instance);
    if (s.eglDisplay != EGL_NO_DISPLAY) {
        eglMakeCurrent(s.eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (s.eglContext != EGL_NO_CONTEXT) eglDestroyContext(s.eglDisplay, s.eglContext);
        if (s.eglSurface != EGL_NO_SURFACE) eglDestroySurface(s.eglDisplay, s.eglSurface);
        eglTerminate(s.eglDisplay);
    }
    app->activity->vm->DetachCurrentThread();
}
