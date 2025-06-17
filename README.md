This repository is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/noetic/README.md).

This code is currently highly experimental!

[![noetic](https://github.com/fkie/iop_jaus_manipulator_clients/actions/workflows/main.yaml/badge.svg)](https://github.com/fkie/iop_jaus_manipulator_clients/actions/workflows/main.yaml)

The repository contains clients designed to control services on IOP complient robot. All client services are based on ```SlaveHandlerInterface``` and use funtionality of [Slave](https://github.com/fkie/iop_core/blob/noetic/fkie_iop_ocu_slavelib/README.md).  

### List of service plugins in this repository:

[fkie_iop_client_manipulator_joint_position_sensor: ManipulatorJointPositionSensorClient](fkie_iop_client_manipulator_joint_position_sensor/README.md)  
[fkie_iop_client_manipulator_specification: ManipulatorSpecificationServiceClient](fkie_iop_client_manipulator_specification/README.md)  
[fkie_iop_client_pantilt_joint_position_driver: PanTiltJointPositionDriverClient](fkie_iop_client_pantilt_joint_position_driver/README.md)  
[fkie_iop_client_pantilt_joint_position_driver: PanTiltJointPositionSensorClient](fkie_iop_client_pantilt_joint_position_driver/README.md#fkie_iop_client_pantilt_joint_position_driver-pantilttointpositionsensorclient)  
[fkie_iop_client_pantilt_specification_service: PanTiltMotionProfileServiceClient](fkie_iop_client_pantilt_specification_service/README.md)  
[fkie_iop_client_pantilt_specification_service: PanTiltSpecificationServiceClient](fkie_iop_client_pantilt_specification_service/README.md#fkie_iop_client_pantilt_specification_service-pantiltspecificationserviceclient)  
[fkie_iop_client_primitive_endeffector: PrimitiveEndEffectorClient](fkie_iop_client_primitive_endeffector/README.md)  
[fkie_iop_client_primitive_manipulator: PrimitiveManipulatorClient](fkie_iop_client_primitive_manipulator/README.md)  
[fkie_iop_client_primitive_pantilt: PrimitivePanTiltClient](fkie_iop_client_primitive_pantilt/README.md)  

