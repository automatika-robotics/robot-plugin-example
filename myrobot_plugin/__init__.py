"""Example Sugarcoat robot plugin package.

A robot plugin is a subclass of :class:`ros_sugar.robot.RobotPlugin`. Pass an
instance to the ``Launcher``::

    from ros_sugar.launch import Launcher
    from myrobot_plugin import MyRobotPlugin

    launcher = Launcher(robot_plugin=MyRobotPlugin())
    launcher.add_pkg(components=[...])
    launcher.bringup()
"""

from .plugin import MyRobotPlugin

__all__ = ["MyRobotPlugin"]
