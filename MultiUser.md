# Multi Users #

ROS4MAT can be used in a multi client environment. For instance, one may imagine a robot controlled by a workstation while multiple student laptops record events and data.

In previous tutorials, we have seen that the subscribe call takes several parameters. However, it is possible to call the subscribe function with no parameters other than the sensor name. This will trigger a **silent subscription**, where the sensor's current parameters are not modified: if the sensor is not currently up, then no data will be returned until another user starts it. If the sensor is currently running, then the parameters set by the last user who did a non-silent subscribe will be used for all users. For instance :

**Computer 1**
```
ros4mat('connect', 'my_robot_ip');
ros4mat('subscribe', 'camera', 30, '640x480', 30);
% ...
```

**Computer 2**
```
ros4mat('connect', 'my_robot_ip');
ros4mat('subscribe', 'camera');
```

The computer 2 will then get data from the camera set at 30 fps, 640x480, and a limit of 30 images in the buffer.