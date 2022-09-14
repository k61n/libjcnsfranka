Pyjcnsfranka
============
Python wrapper for JcnsFranka C++ library.

Requirements
------------
Setuptools
```bash
apt install python3-setuptools
```

Installation
------------
```bash
python3 setup.py install
```

Usage
-----
```python
from pyjcnsfranka.main import Pyjcnsfranka

robot = Pyjcnsfranka('192.168.201.2')
robot.communication_test()
```
