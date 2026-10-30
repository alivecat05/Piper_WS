import sys
from pathlib import Path
from setuptools import find_packages, setup


if sys.platform != "win32":
    raise RuntimeError(
        "python-can-agx-cando can only be built or installed on Windows."
    )


ROOT = Path(__file__).resolve().parent
README = (ROOT / "README.md").read_text(encoding="utf-8")


setup(
    name="python-can-agx-cando",
    version="0.1.0",
    description="python-can plugin backed by cando.dll via ctypes",
    author="科幻木影",
    long_description=README,
    long_description_content_type="text/markdown",
    license="MIT",
    url="https://github.com/agilexrobotics/python-can-agx-cando",
    python_requires=">=3.6",
    packages=find_packages(include=["agx_cando", "agx_cando.*"]),
    install_requires=["python-can>=3.3"],
    keywords="can python-can windows ctypes cando",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Operating System :: Microsoft :: Windows",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: System :: Hardware :: Hardware Drivers",
    ],
    entry_points={
        "can.interface": [
            "agx_cando = agx_cando.bus:AgxCandoBus",
        ]
    },
    project_urls={
        "Homepage": "https://github.com/agilexrobotics/python-can-agx-cando",
    },
    package_data={
        "agx_cando": [
            "bin/x32/*.dll",
            "bin/x64/*.dll",
        ]
    },
    include_package_data=True,
)
