
JcnsFranka
============
JcnsFranka is a high level wrapper for LibORL, a motion library for Franka Emika
robots. LibORL in its turn is a wrapper around libfranka and provides simple
functionality for motion in cartesian space.

Requirements
------------
* LibORL [https://forge.frm2.tum.de/review/jcns/mirror/liborl].
  * Eigen3
  * Boost
  * GoogleTest
* libfranka [https://forge.frm2.tum.de/review/jcns/mirror/libfranka]
  * Eigen3
  * Poco

Installation
------------
Build or install Eigen3, Boost and Poco development packages.
```bash
apt install libpoco-dev libeigen3-dev libboost-dev libgtest-dev
```

Build and install release version of libfranka locally.
```bash
git clone --recursive https://forge.frm2.tum.de/review/jcns/mirror/libfranka
cd libfranka
git checkout 0.9.2
git submodule update
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
cmake --build . --parallel 4
cpack -G DEB
dpkg -i libfranka*
```

Build and install release version of LibORL locally.
```bash
git clone "https://forge.frm2.tum.de/review/jcns/mirror/liborl"
cd liborl
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
cmake --build . --parallel 4
cpack -G DEB
dpkg -i liborl*
```

Build and install release version of JcnsFranka library.
```bash
git clone "https://forge.frm2.tum.de/review/jcns/tango/franka"
cd franka
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel 4
cmake --install .
```

Usage
-----
```cmake
find_package(jcnsfranka REQUIRED)
```

Demo
----
Demo is an example of JcnsFranka library build in a Qt application.
To run demo Qt6.3.1 would be necessary. To build demo
```bash
cd demo
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/path/to/Qt/6.3.1/gcc_64
cmake --build . --parallel 4
./franka_demo
```

PyJcnsFranka
------------
PyJcnsFranka is a [python wrapper](pyjcnsfranka/readme.md) for JcnsFranka C++ library.
