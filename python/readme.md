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


robot_ip = '192.168.1.2'
realtime_config = False
robot = FrankaRobot(robot_ip, realtime_config)
robot.reference()
```

```python
from pyjcnsfranka.robot import comtest


robot_ip = '192.168.1.2'
realtime_config = True
limit_rate = True
cutoff_frequency = 1000
comtest(robot_ip, realtime_config, limit_rate, cutoff_frequency)
```