Pyjcnsfranka
============
Python wrapper for JcnsFranka C++ library.

Requirements
------------
None.

Installation
------------
The Python library is installed by CMake together with the
C++ library.

Usage
-----
```python
from pyjcnsfranka.robot import FrankaRobot

robot = FrankaRobot('192.168.201.2')
robot.go_home()
```

```python
from pyjcnsfranka.robot import comtest

comtest('192.168.201.2')
```