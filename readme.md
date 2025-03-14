
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
git clone --recurse-submodules https://forge.frm2.tum.de/review/plugins/gitiles/jcns/mirror/libfranka
cd libfranka
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
cmake --build . --parallel $(nproc)
cmake --install .
```

Build and install release version of LibORL locally.
```bash
git clone https://forge.frm2.tum.de/review/plugins/gitiles/jcns/mirror/liborl
cd liborl
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
cmake --build . --parallel $(nproc)
cmake --install .
```

Build and install release version of JcnsFranka library.
```bash
git clone https://forge.frm2.tum.de/review/plugins/gitiles/jcns/tango/franka
cd franka
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel $(nproc)
cmake --install .
```

Usage
-----
```cmake
find_package(jcnsfranka REQUIRED)
```

Demo
----
jcnsfranka-demo is built against Qt5/6 and installed with libjcnsfranka.
```bash
jcnsfranka-demo
```

PyJcnsFranka
------------
PyJcnsFranka is a [python wrapper](pyjcnsfranka/readme.md) for JcnsFranka C++ library.
