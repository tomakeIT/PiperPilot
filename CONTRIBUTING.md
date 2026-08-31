# Contributing

Thanks for helping improve PiperPilot.

## Development setup

Use Python 3.10 or 3.11 on Linux. For a simulation/test environment:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[test]"
python -m pytest
```

The full hardware bootstrap is documented in
`docs/getting-started/installation.md`. Hardware-specific tests should be
optional or skipped when the required device or SDK is unavailable.

## Pull requests

- Keep changes focused and include tests for behavior changes.
- Run `python -m pytest` and `python -m compileall -q piper_teleop`.
- Update the README or MkDocs pages when CLI flags, configuration, protocols,
  or dataset schemas change.
- Do not commit datasets, camera serials, credentials, private endpoints,
  customer names, or machine-specific paths.
- Test motion changes in simulation first. State clearly which robot model,
  firmware, and control mode were used for any real-hardware validation.

By contributing, you agree that your contribution is licensed under the MIT
License in this repository.
