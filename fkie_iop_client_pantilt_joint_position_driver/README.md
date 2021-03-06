This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_pantilt_joint_position_driver:_ PanTiltJointPositionDriverClient

Lets control a PanTiltJointPositionDriver service.

#### Parameter:

_use_posestamped (bool_ Default: "false")

> Adds support for geometry_msgs::PoseStamped message.

_tf_frame_pantilt (str_ Default: "")

> only if ```use_posestamped``` is true. Transfrom the resceived pose into given tf frame before send to IOP.

#### Publisher:

> None

#### Subscriber:

_cmd_pos_joints (sensor_msgs::JointState)_

> Position commands for specified joints.

_cmd_pos_pan (std_msgs::Float64)_
_cmd_pos_pan32 (std_msgs::Float32)_

> Pan position.

_cmd_pos_tilt (std_msgs::Float64)_
_cmd_pos_tilt32 (std_msgs::Float32)_

> Tils position.

_cmd_pos_pantilt (geometry_msgs::PoseStamped)_

> only if ```use_posestamped``` is true.

_cmd_pos_pantilt (geometry_msgs::PoseStamped)_

> only if ```use_posestamped``` is true.



## _fkie_iop_client_pantilt_joint_position_driver:_ PanTiltJointPositionSensorClient

Receives positions from PanTiltJointPositionSensor service and publishes those to ROS.

#### Parameter:

_use_posestamped (bool_ Default: "false")

> Adds support for geometry_msgs::PoseStamped message.

_tf_frame_pantilt (str_ Default: "")

> only if ```use_posestamped``` is true. Sets the given frame_id in ROS message header before send to ROS.


#### Publisher:

_pos_joints (sensor_msgs::JointState)_

> Position for specified joints.

_pos_pan (std_msgs::Float64)_
_pos_pan32 (std_msgs::Float32)_

> Pan position.

_pos_tilt (std_msgs::Float64)_
_pos_tilt32 (std_msgs::Float32)_

> Tils position.

_pos_pantilt (geometry_msgs::PoseStamped)_

> only if ```use_posestamped``` is true.

#### Subscriber:

> None