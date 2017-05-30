# PyRIDE remote simple
A pure Python remote interface to PyRIDE servers.

It connects via telnet to the server and queries dynamically for the available methods.

# Usage example
Import `PyRIDERemoteInstance` with 
```python
from pyride_remote_simple import PyRIDERemoteInstance
```.

Create a `PyRIDERemoteInstance("robot_hostname_or_ip")` object, if using an interactive python client (`ipython` recommended) you'll get autocompletion and documentation on the available methods. Callbacks are not implemented (yet?).

```python
from pyride_remote_simple import PyRIDERemoteInstance
# Connect to a robot hostname (or IP), defaults to port 27005
PyPR2 = PyRIDERemoteInstance('pr2')
PyPR2.say('hello')
PyPR2.moveHeadTo(0., 0.)
PyPR2.getBatteryStatus()
PyPR2.getArmJointPositions(True)
PyPR2.send_raw_command("PyPR2.say('hello')")
PyPR2.send_raw_command("dir()")
# Expected output:
# ['PyPR2', '__builtins__', '__doc__', '__name__', '__package__']

# For a Nao
PyNao = PyRIDERemoteInstance('nao')
PyNao.say('hello')
PyNao.send_raw_command("dir()")

# For a REEM
PyREEM = PyRIDERemoteInstance('10.68.0.1')
PyREEM.getArmJointPositions(True)
# Expected output:
# {'arm_left_7_joint': 2.739275735495535e-05, 'arm_left_5_joint': -6.391643382822915e-05, 'arm_left_6_joint': 2.739275735495535e-05, 'arm_left_4_joint': 0.6112902034476808, 'arm_left_1_joint': -0.40065491868780706, 'arm_left_2_joint': 0.2500431928699627, 'arm_left_3_joint': -0.10061569672681259}
# super hacky way of getting the robot description abusing send_raw_command
robot_description = PyREEM.send_raw_command("import rospy;rospy.init_node('hackytest', anonymous=True);robot_desc=rospy.get_param('/robot_description');rospy.signal_shutdown('End hacky test');print(robot_desc)")
```

**Note**: You can send raw python code commands by using the `send_raw_command()` method (make sure to print the return value you need so it gets converted back to a Python object you can manipulate).

# Installation
You can:
* Install from PyPI:
```bash
sudo pip install pyride_remote_simple
```

* Use the `setup.py` script with:
```bash
sudo python setup.py install
```

* You could just copy and paste the classes from [pyride_remote_simple/pyride_remote_simple.py](pyride_remote_simple/pyride_remote_simple.py) **PyRIDECommander** and **PyRIDERemoteInstance** and directly use it if you are very lazy.


