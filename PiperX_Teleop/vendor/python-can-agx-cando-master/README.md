# python-can-agx-cando

Local `python-can` plugin backed by `cando.dll`.

## Scope

This plugin exists to provide a Windows `python-can` backend for projects that need to access the target CAN module through `cando.dll`.

## Goals

- Do not patch `python-can`.
- Auto-select `x64` or `x32` DLL based on the running Python process architecture.
- Support Python `3.6+`.

## Interface name

After installation, use:

```python
import can

bus = can.Bus(interface="agx_cando", channel=0, bitrate=1_000_000)
```

## DLL lookup order

The plugin resolves `cando.dll` in this order:

1. package-local `agx_cando/bin/x64/cando.dll` for 64-bit Python
2. package-local `agx_cando/bin/x32/cando.dll` for 32-bit Python

## Install

```powershell
git clone https://github.com/agilexrobotics/python-can-agx-cando.git
cd python-can-agx-cando
pip3 install .
```

Current distribution mode is source-based installation from GitHub or a checked-out local repository.

Non-Windows source installs are blocked during packaging.

## Development

Run the original Python smoke tests (no hardware or test DLL required):

```powershell
python -m unittest discover -s tests -t .
```

For native implementation, DLL builds and receive policies, see [the native documentation](native/README.md).

Clean build artifacts and Python caches:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\clean-build-cache.ps1
```

## Notes

- This package expects a Windows environment.
- It targets classic CAN frames only.
