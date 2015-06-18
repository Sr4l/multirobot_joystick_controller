multirobot_joystick_controller
==============================

This repository holds the multirobot_joystick_controller packages for
[ROS][1] [Turtlebots][2]. For more information see `package.xml` file.
For license see `LICENSE` and the source code.

This code is not really configurable. You may have to change the
"Config Values" in head of the Python File and maybe even more in the
code.

usage
---------

1. Select a Robot with the **X**, **Y**, **A**, **B** buttons.
2. Press and hold the dead-men key (left shoulder button).
3. Drive the robot, left analog stick linear velocity, right analog
stick for rotation

config values
-------------

- **REFRESH_RATE** - refresh rate of the service, can by low, does not effect message receiving
- **ROBOT_MAX_VELOCITY** - the maximum velocity
- **ROBOT_ROTATION_SCALE** - the robot rotation scale factor, 1.0 => max robot rotation 2*pi rad/s
- **DEADMAN_BUTTON** - button id of the dead-man button
- **TWISTMSG_TOPIC** - topic name for the Twist velocity messages
- **JOYSTICK_TOPIC** - topic name of the ROS joystick node
- **ROBOTS_LIST** - list with robots (namespace names)

[1]: http://www.ros.org
[2]: http://wiki.ros.org/Robots/TurtleBot
