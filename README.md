# Team Pictures:
TODO we need to take them today

# Robot Pictures:
<p>
  <img src="Pictures/robotPicture.png" height="200">
  <img src="Pictures/2.png" height="200">
  <img src="Pictures/3.png" height="200">
  <img src="Pictures/4.png" height="200">
  <img src="Pictures/5.png" height="200">
  <img src="Pictures/6.png" height="200">
  <img src="Pictures/7.png" height="200">
</p>

# Mechanical Parts:

Drive Base: Modified LaTrax Dessert Prerunner base

Drive Motor: ARRMA MEGA 380 BRUSHED MOTOR

Drive Servo: Traxxas Metal Gear 2265



Odometry: SparkFun Optical Tracking Odometry Sensor - PAA5160E1 (Qwiic)

Lidar: LDRobot LD19 Lidar

Battery: Ovonic air lipo 3s 3000 mAh 11.1v

Camera: Limelight 3A

Main Computer: Jetson Orion Nano 

Addtional Computer: Tieensy 4.1

ESC: HOBBYWING QUICRUN WP 1080 G2 Brushed 2-3s ESC 

# Assebmly and Manufacturing:

## Base

The first layer of this autonomous vehicle is fabricated out of a modified LaTrax Desert Prerunner base. We chose this specific base because of its durability and the fact that it has a pre-built drivetrain, which makes it easier and quicker to fix issues and find specific parts needed to construct the autonomous vehicle. The bottom plate is made out of durable plastic material, making it stronger than a three-dimensional print, and since it comes out of a stock version of the LaTrax Desert Prerunner, it is easy to replace. The suspension on the front and rear axles is also stock, and the connection between the base and suspension is modular, making it easy to fix issues that require disassembly of the bottom plate. In the middle of the vehicle also runs a solid, metal axle, thus further reinforcing it.

To modify the drivetrain we relpaced three major components:
 1.  For attaching a stronger motor to the base, we removed the top gearbox housing and 3d printed a new one to fit the new, more powerful motor.
 2.  For replacing the servo we simply got one with the same dimensions and replaced it with no need for any new mounts
 3.  To be able to mount new compontents we replaced the blue plate with a new wooden laser cut mount

We also conected the OTOS sensor to the bottom of the car for odometry readings.

## First Layer

This layer consists of a laser-cut, wooden base plate that holds mounts for electronic components. Mounts for the battery, the on/off switch,and the Power Distribution Board are 3D printed using PLA and screwed into the base plate. The Lidar is simply mounted by srewing it in directly to the wooden plate. It also supports four metal GoBilda axles that connect the top plate. We choose wood for the material because it is easy and fast to laser cut, and it is less expensive so we have lots of spare material in case the top and middle plates break.

## Second Layer

The top plate is much like the middle plate; except for the shape and the electronics it holds. It is also made of wood and is laser-cut and attaches 3 diffrent components:
-   The Jetson Nano is directy screwed into the board suported by standoffs.
-   And we connected the teensy perpendicular to the wooden plate.
-   And the 3D printed camera mount is screwed into other holes on the Nano board.


# Wiring Diagram:

<img width="1545" height="667" alt="image" src="https://github.com/user-attachments/assets/ea402b43-c923-4fc1-b1a3-8da9963ca78f" />


# Control Structure:

The software we mainly used to connect our hardware components is ros2. We mainly used ros2 because it is a library of software that can be used in robot applications to build or simplify robot application. The way it works is each hardware component is represented by a node which has publishers and subscribers for topics. Topics in ros2 are data streams that allow data to be sent or receive from multiple hardware components or nodes. In order to send data to a topic, you must have a publisher which can only be defined by the node. This is the same for subscribers which receive data from a topic. Subscribers are also defined by the node. An example of this being applied is the Teensy 4.1 node subscribing to the topics motor_speed and servo_angle with the Motor node publishing to the topic motor_speed in order to send the speed the motor should be moving at. The speed, in this example, is sent to the Teensy node because the teensy controls the Motor signals. In the next pages, we will describe in more details the structure of each hardware component in ros2. 

# Low Level Controls (Servo & Motor): 

The Teensy 4.1 node is what we use as the node to control the Motor and Servo. The Teensy 4.1 subscribes to the 2 topics motor_speed and servo_angle to get the speed and angle needed to control the Motor and Servo. This node is mainly used as a low-level controller and has no calculations in order to control the Motor and Servo. The node simply sends the instructions it gets directly to the signals of the Motor and Servo. However, since the Teensy 4.1 is a micro controller. We use a specialized version of ros that is still compatiable with ros2 called micro-ros. Micro-ros is designed for micro controllers which typically has limited memory and processing resources unlike a computer that can handle ros2. The way we apply Micro-Ros is we use it as an extension of the ros2 system, but it is still a separate version which is why you have to run an additional component called the Micro-Ros Agent. So to recap before we go further, we have 2 components that extend ros2: the Micro-Ros which to clarify we will call it Micro-Ros Cilent which is stored in the Micro controller and provides API that can be used to connect to the ros2 system. Then we have the Micro-Ros Agent which bridges Micro-Ros Cilent to Ros2 so it can communicate. TODO (Sagnik) explain Micro-Ros Agent more in context. 

# High Level Controls (Sensors):

TODO Sagnik add explanation of high level nodes and topics and how it works

# Simulation 

One of the ways we helped to speed up the programming process was by working on the obstacle and open-loop navigation code using simulations. For our simulation, we used RViz and Nav2 to help build the scans even without the robot and test our code even if the robot wasn't fully ready. Doing this let us work on the logic and practice using the sensors.

# Camera 

TODO Sagnik & Gregory Write what the camera is trained on and how it was done like 6-8 scentences keep consise
