# Security Policy

## Reporting a vulnerability

Please use the repository's private GitHub Security Advisory reporting flow.
Do not open a public issue for vulnerabilities, leaked credentials, or unsafe
motion behavior that could put operators or hardware at immediate risk.

Include the affected version or commit, reproduction steps, impact, and any
suggested mitigation. Remove camera images, device serials, tokens, private
network addresses, and collected datasets before attaching logs.

## Deployment notes

The dashboard and robot HTTP server do not provide authentication. They bind
to loopback by default and should remain behind a trusted local boundary. Do
not expose them directly to an untrusted LAN or the public internet.

This software is not a safety-rated controller. Always retain independent
hardware emergency-stop and workspace protections.
