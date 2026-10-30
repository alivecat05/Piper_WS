import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description="Build only the native receive DLLs with MSVC.")
    parser.add_argument("--vcvars", required=True, help="Path to Visual Studio VC/Auxiliary/Build/vcvarsall.bat")
    parser.add_argument("--arch", choices=["all", "x64", "x32"], default="all")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    vcvars = Path(args.vcvars).resolve()
    if not vcvars.is_file():
        parser.error("vcvarsall.bat not found")
    for arch in (["x64", "x32"] if args.arch == "all" else [args.arch]):
        target = root / "agx_cando" / "bin" / arch / "agx_receive.dll"
        with tempfile.TemporaryDirectory(prefix="agx-receive-" + arch + "-") as directory:
            build = Path(directory)
            artifact = build / "agx_receive.dll"
            command = ('call "%s" %s >nul && cl /nologo /std:c++14 /EHsc /O2 /W4 /WX /MT /LD '
                       '"%s" /link /DEF:"%s" /OUT:"%s" /IMPLIB:"%s"' %
                       (vcvars, "x64" if arch == "x64" else "x86", root / "native/agx_receive.cpp",
                        root / "native/agx_receive.def", artifact, build / "agx_receive.lib"))
            subprocess.run(command, shell=True, cwd=str(build), check=True)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(artifact), str(target))
        print("Built %s" % target, flush=True)


if __name__ == "__main__":
    main()
