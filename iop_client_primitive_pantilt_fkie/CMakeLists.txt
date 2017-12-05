cmake_minimum_required(VERSION 2.8.3)
project(iop_client_primitive_pantilt_fkie)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                iop_client_pantilt_specification_service_fkie
                iop_ocu_slavelib_fkie
                moveit_core
                moveit_ros_planning
                roscpp
                sensor_msgs
                )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

iop_init(COMPONENT_ID 0)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        iop_client_pantilt_specification_service_fkie
        iop_ocu_slavelib_fkie
        moveit_core
        moveit_ros_planning
        sensor_msgs
    DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
iop_code_generator(
# IDLS: the service definition files located in iop_builder_fkie/jsidl
# OWN_IDLS: the service definition files located in PROJECT_DIR/jsidl
# OVERRIDES: files which override the generated files while build. The generated files are copied to the same location with .gen extension
# EXTERN_SERVICES: the generated folder name of a service which is used from other library. This folder must be deleted while build.
  IDLS
    urn.jaus.jss.core-v1.1/AccessControlClient.xml
    urn.jaus.jss.core-v1.1/EventsClient.xml
    urn.jaus.jss.core-v1.1/ManagementClient.xml
    urn.jaus.jss.core-v1.1/Transport.xml
    urn.jaus.jss.manipulator-v2.0/PanTiltSpecificationServiceClient.xml
    urn.jaus.jss.manipulator-v2.0/PrimitivePanTiltClient.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_manipulator_PrimitivePanTiltClient/PrimitivePanTiltClient_ReceiveFSM.h
    src/urn_jaus_jss_manipulator_PrimitivePanTiltClient/PrimitivePanTiltClient_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_AccessControlClient
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_ManagementClient
    urn_jaus_jss_core_Transport
    urn_jaus_jss_manipulator_PanTiltSpecificationServiceClient
  GENERATED_SOURCES cppfiles
)

include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_library(${PROJECT_NAME}
            src/PrimitivePanTiltClientPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES ./plugin_iop.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

