# RMF Compliant Lift
This project builds on the [previous ROS2 project](https://github.com/CobWebsRoy/ROS2_Smart_Lift/tree/main) to allow a legacy elevator to communicate via RMF topics, allowing for scalability and integration into larger RMF infrastructure. This add-on kit only allows for the most basic functionalities of the elevator, namely calling for and instructing the elevator to access different floors. Other functionalities like determining the direction of elevator travel is not in the scope of this project, but can be easily implemented using other sensors like an accelerometer.

### Hardware and Software
- Identical to the [previous ROS2 project](https://github.com/CobWebsRoy/ROS2_Smart_Lift/tree/main#hardware)

### Interfaces
Below describes the communications between the lift adapter and a simulated fleet adapter. This project covers only the modules in blue.
![[RMF Lift Comms](/images/RMF Lift Comms.png)](https://github.com/CobWebsRoy/RMF_Compliant_Lift/blob/main/images/RMF%20Lift%20Comms.png)

## How to use this:
Firstly, it is assumed that you have ROS2 Humble desktop running on Ubuntu 22.04 on a Raspberry Pi. You will have to place the two packages ```lift_adapter_template``` and ```rmf_lift_msgs``` into a ROS2 workspace ```rmf_ws/src``` and build the packages. You will also have to edit the ```config.yaml``` file accordingly with the name of the lift. Be sure to install all necessary dependencies using pip. Remember to source the installation. To start the application, there are a few ROS2 nodes that have to be run: Most of the code is written in the ```lift_adapter_template``` package, which will be in the ```src``` folder in the ROS2 workspace. The ```image_tools``` package should already be built into the ROS2 Humble desktop installation.

The ```lift_adapter_template``` node acts as the brains of the application and controls all the actuator commands, subscribes to LiftRequests and publishes the LiftState.
```
ros2 run lift_adapter_template lift_adapter_template -n test_lift -c src/lift_adapter_template/config.yaml
```
The ```qr_decoder``` node analyzes the images from the Raspberry Pi camera and decodes the QR codes in the image indicating the floor that the elevator is currently on.
```
ros2 run lift_controller_v2 qr_decoder
```
The ```cam2image``` node publishes the images taken by the Raspberry Pi camera for decoding.
```
ros2 run image_tools cam2image
```

## To request the service of an elevator:
You can use the ROS2 command line interface with the following message to request the service of an elevator:
```
ros2 topic pub -1 /lift_requests rmf_lift_msgs/LiftRequest "{lift_name: 'test_lift', session_id: 'test', request_type: 1, destination_floor: '1', door_state: 2}"
```
The ```destination_floor``` indicates the floor that the robot is calling the elevator to go to and the ```request_type``` indicates calling the services of the elevator (1) or releasing the elevator (0).
