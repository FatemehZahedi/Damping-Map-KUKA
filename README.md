This repository houses code to run a 6 DOF KUKA LBRiiwa robot with various controller such as
- admittance control
- variable damping control
- position control
using the KUKA Fast Robot Interface C++ SDK

The repository structure is as follows
Binaries - contains projects which compile to executables to run KUKA based experiments
Libraries - contains supporting libraries such as KUKA Fast Robot Interface C++ SDK, Trigno Emg Client interfaces, UDP Servers

********************************************************************************

Requirements
- Conan C/C++ package manager
- CMake 3.8+
- C++11 compatible compiler

Both of these should be readily available via a linux distro package manager such as rpm, apt, etc...


********************************************************************************
Build instructions:

Open terminal at top of repo, and repeat the following commands
```
mkdir build
cd build
conan install ..
cmake ..
cmake --build .
```
