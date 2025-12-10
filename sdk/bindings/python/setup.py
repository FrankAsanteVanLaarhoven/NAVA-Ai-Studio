from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension, build_ext
from pybind11 import get_cmake_dir
import pybind11

ext_modules = [
    Pybind11Extension(
        "nava_sdk",
        [
            "src/nava_sdk.cpp",
        ],
        include_dirs=[
            "../native/src",
            pybind11.get_include(),
        ],
        language='c++',
    ),
]

setup(
    name="nava-sdk",
    version="0.1.0",
    author="NAVΛ Team",
    author_email="team@nava.studio",
    description="NAVΛ SDK Python bindings",
    long_description=open("README.md").read() if os.path.exists("README.md") else "",
    long_description_content_type="text/markdown",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.10",
    install_requires=[
        "pybind11>=2.10.0",
    ],
    package_data={
        "": ["../../assets/logo.svg", "../../assets/icon-512.png"],
    },
    include_package_data=True,
)

