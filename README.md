# Valkyrie Footstep Planner and Executor
Services for planning and executing walks on NASA's Valkyrie robot.

This package contains a node that provides services for planning and executing footsteps on NASA's Valkyrie robot.  The package depends on [IHMC's `controller_msgs`](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/val-develop/ihmc-interfaces/src/main/messages/ros1/controller_msgs/msg).  The server node also stores footstep plans internally so that nodes using these services do not have to.

For an example of a node that uses these services, please see the `SemanticFrameControllerNode` in the [`val_dynacore` package](https://github.com/esheetz/val_dynacore).
