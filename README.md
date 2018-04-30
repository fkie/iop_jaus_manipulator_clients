This repository is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).

This code is currently highly experimental!

Build status of latest version:

[![Build Status](https://travis-ci.org/fkie/iop_jaus_manipulator_clients.svg?branch=master)](https://travis-ci.org/fkie/iop_jaus_manipulator_clients)

The repository contains clients designed to control services on IOP complient robot. All client services are based on ```SlaveHandlerInterface``` and use funtionality of [Slave](https://github.com/fkie/iop_core/blob/master/iop_ocu_slavelib_fkie/README.md).  

### List of service plugins in this repository:

[iop_client_manipulator_joint_position_sensor_fkie: ManipulatorJointPositionSensorClient](iop_client_manipulator_joint_position_sensor_fkie/README.md)  
[iop_client_manipulator_specification_fkie: ManipulatorSpecificationServiceClient](iop_client_manipulator_specification_fkie/README.md)  
[iop_client_pantilt_joint_position_driver_fkie: PanTiltJointPositionDriverClient](iop_client_pantilt_joint_position_driver_fkie/README.md)  
[iop_client_pantilt_joint_position_driver_fkie: PanTiltJointPositionSensorClient](iop_client_pantilt_joint_position_driver_fkie/README.md#iop_client_pantilt_joint_position_driver_fkie-pantilttointpositionsensorclient)  
[iop_client_pantilt_specification_service_fkie: PanTiltMotionProfileServiceClient](iop_client_pantilt_specification_service_fkie/README.md)  
[iop_client_pantilt_specification_service_fkie: PanTiltSpecificationServiceClient](iop_client_pantilt_specification_service_fkie/README.md#iop_client_pantilt_specification_service_fkie-pantiltspecificationserviceclient)  
[iop_client_primitive_endeffector_fkie: PrimitiveEndEffectorClient](iop_client_primitive_endeffector_fkie/README.md)  
[iop_client_primitive_manipulator_fkie: PrimitiveManipulatorClient](iop_client_primitive_manipulator_fkie/README.md)  
[iop_client_primitive_pantilt_fkie: PrimitivePanTiltClient](iop_client_primitive_pantilt_fkie/README.md)  

