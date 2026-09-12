# Contents
- Mobility and Mechanical Design
- Power and Sensor Architecture
- Software Architecture and Obstacle Strategy
- Systems Thinking and Engineering Decisions

## Car Base Iterations

### Version 1.0: 3D-Printed Base

Our first car used a fully 3D-printed body as its main structure. We planned to mount a much larger motor and servo, with all our sensors on top. This did not work because the sensors would not fit, and the body was too bulky and inconvenient.

<img height="300" alt="Version 1.0 chassis design" src="https://github.com/user-attachments/assets/555a4b37-9475-4010-af31-b2745bed91af" /><img height="300" alt="Version 1.0 additional view" src="https://github.com/user-attachments/assets/19933b24-d5e7-40bb-89eb-9bfe73578e52" />

### Version 1.1: Adding a Sensor Platform

We kept the main structure and rearranged parts to make it smaller, stronger, and fit better. We added a flat top layer to mount the sensors, which helped most of them fit. However, the base was still too high for the LiDAR to fit and scan the walls effectively.

<img height="300" alt="Version 1.1 chassis with sensor platform" src="https://github.com/user-attachments/assets/907cc0f6-ba47-4c27-bd72-e1abde57032f" />

<img height="300" alt="Version 1.1 additional view" src="https://github.com/user-attachments/assets/c4b09b67-02a9-49fe-9cfa-a2dd3fa77241" />

### Version 2.1: Switching to a Premade Base

After testing our 3D-printed base, we switched to the LaTrax Desert Prerunner base. We removed the casing and original structural plate but kept the original servo and motor at first.

We later decided to replace the motor with one recommended by the MIT course and the servo with one recommended by a local RC car expert.

<img height="300" alt="Version 2.1 premade chassis" src="https://github.com/user-attachments/assets/eeeb84d8-b88b-4afd-9ec9-7f5d06e5da54" />

### Version 2.2: Replacing the Motor and Servo

We replaced the motor with an ARRMA MEGA 380 brushed motor and added a new mount to hold it. We also replaced the servo with a Traxxas 2265 metal-gear servo and added an electronic speed controller (ESC) to control the motor and supply it with power.

<img height="300" alt="Version 2.2 updated components and chassis" src="https://github.com/user-attachments/assets/8e530a42-62d5-4974-b7af-0bd89ea8e1bd" />

## Design Tradeoffs and Changes

We first tried fixing the layout instead of replacing the whole printed base. The flat top layer helped most sensors fit, but it did not solve the LiDAR height problem.

Switching to a premade base still required changes, including removing the casing and structural plate and adding a motor mount. Our main concern was not just fitting the parts, but placing the LiDAR where it could scan the walls effectively.

## Choosing Components

| Component | What We Used | Why We Used It |
|---|---|---|
| Chassis | LaTrax Desert Prerunner base | Our printed bases were too bulky and had problems with sensor space and LiDAR placement. |
| Motor | ARRMA MEGA 380 brushed motor | The MIT course recommended it, it was easy to get, and we already had it. |
| Motor mount | New mount for the replacement motor | To hold the new motor in place. |
| Servo | Traxxas 2265 metal-gear servo | A local RC car expert recommended it, it was easy to find, and we already had it. |
| Electronic speed controller (ESC) | HOBBYWING QUICRUN WP 1080 G2 Brushed 2–3S | To control the motor and supply it with power. |

## Steering and Drivetrain

### Steering
The steering servo moves a set of linkages that change the angle of the front wheels to steer the car.

### Drivetrain
The motor drives all four wheels, with a long driveshaft connecting the front and rear drivetrain. The motor uses a larger gear to drive a smaller gear, increasing speed but reducing torque at that connection.

## Mechanical Stability

### Motor Mount
The motor mount screws into the chassis to hold the motor in place.

### Component Mounts
The battery, on/off switch, and Power Distribution Board use 3D-printed PLA mounts that screw into the wooden plate.

### Plate Supports
Four metal GoBilda standoffs connect the two levels and support the top plate. 

## Power Distribution

Power goes from the battery through a fuse and switch to the PDB, which splits it into three branches:

```text
Battery → Fuse → Switch → PDB
                          ├── ESC → Motor
                          ├── Step-Down Converter → Servo
                          └── Orin
```

The ESC controls the motor, the step-down converter lowers the voltage for the servo, and the last branch powers the Orin.

Our battery is an 11.1V 3S LiPo, which is fine for the motor and the Orin but would immediately fry the servo, since the Traxxas 2265 is rated for around 6V. The Pololu S13V30F5 step-down regulator drops the 11.1V down to a clean 5V, which is safe for the servo and is also the voltage most of our smaller electronics expect. This way one battery powers the whole car, and each device gets the voltage it actually needs.

Only power connections are shown, not signal wires.

## Sensors

| Sensor | Model | What We Use It For | Why We Chose It | Where It Is Mounted | Why This Position |
|---|---|---|---|---|---|
| LiDAR | LDRobot LD19 | Build a 2D map using light reflected off walls and obstacles | We wanted to measure the distance to walls around the car, not just directly in front of it. | Screwed to the Level 1 plate | Keeps the LiDAR below the top of the walls so it can scan them. |
| Camera | GoBilda Limelight 3A | Detect corners and obstacle colors | It gives us information about colors in front of and slightly to the sides of the car, rather than only directly below it. | Elevated above the car and mounted at an angle | Allows the camera to see the far front wall from any position. |
| Odometry | SparkFun Optical Tracking Odometry Sensor | Track the robot’s position on the field | It is small, easy to mount, and fits under the car base. | Under the LaTrax base, in the center of the robot | Places the sensor at the correct height and position for accurate readings. |

## Sensor Consideration 

### LiDAR

Before choosing our current LiDAR, we considered two other models.

#### RPLIDAR A1

We first considered the RPLIDAR A1 because we already owned it and had heard it was reliable.

<img height="300" alt="RPLIDAR A1" src="https://github.com/user-attachments/assets/75b91a00-f88c-422a-8899-ab1ab9f1e55a" />

We decided against it because it was too large, and its external motor took up extra space.

#### RPLIDAR A3

We then switched to the RPLIDAR A3 because we expected it to be more reliable. Its internal motor also gave it a more compact shape that was easier to mount.

<img height="300" alt="RPLIDAR A3" src="https://github.com/user-attachments/assets/5e8c832e-ab1f-47f7-9cb7-f78bb5fc027f" />

However, it was still too large to fit on our robot. It was also heavy and expensive, so we switched to our current LiDAR, the LDRobot LD19.

## Odometry Calibration

Our SparkFun OTOS sensor does not report distance and rotation perfectly, so we calibrate it before use. We drove the car along a known straight line and compared the reported distance to the real distance to get a linear scalar of 0.99554497057, then rotated the car through a known angle and did the same to get an angular scalar of 0.99995448187. We also run `calibrate_otos.py`, which records the sensor for 15 seconds while the car is still and measures the standard deviation of its readings, since the OTOS reports small nonzero values even when nothing is moving. Those standard deviations become the covariances we give the EKF, so it knows how much to trust the sensor.

## Scan Filter

Our LiDAR sometimes returns bad readings, so we run every scan through a filter before using it. Any point closer than 10 cm is dropped because it is almost always a reflection off the car itself, and any point farther than 4 m is dropped because the LiDAR returns a max-range value when it does not hit anything. A speckle filter then removes isolated points that differ from their neighbors by more than 0.5 m, which cleans up random spikes. This leaves us with a scan that only contains real walls and obstacles.

## Training the Limelight

We run object detection on the Limelight to find the red and green pillars, the orange and blue corner lines, and the black outer wall. To train it we took pictures of the real field from many angles, distances, and lighting conditions, then drew bounding boxes around each object and labeled it with the correct class. The Limelight web interface trained a neural detector on this data, which we then deployed to the camera. The pillar detections tell us which side to pass an obstacle on, while the corner lines and black wall give us fixed features we can use to localize the car on the field.

## Open Loop Logic

For our open loop, we use a state machine with 5 states, each one leading to another:

(Init) Our initial state allows the robot to get set and ensures all the sensors are ready and the topics have information before we begin to use them in our calculations. Once finished, the state is set to Lane Follow.

(Lane Follow) Our lane follow state keeps the robot speed set on cmd_vel to go forward until the sensors detect a corner, which sets the state to Corner, unless the number of corners is 12, meaning that the robot has finished 3 full laps, which makes the state Park.

(Corner) Our corner state keeps the robot turning until the sensors detect that the robot is in between the 2 walls. Once it is, the state is set to Lane Follow.

(Park) Our parking state keeps the robot going forward until it is halfway in the tunnel, where it started, and then changes the state to Stop.

(Stop) Our stop state keeps the robot from moving and does not activate a new state.

<img width="1545" height="793" alt="image" src="https://github.com/user-attachments/assets/0e56dc89-2bd5-4246-ad83-ee905e9cc6f5" />

### Obstacle Loop Logic
For our obstacle loop, we use a state machine with 6 states, each one leading to another:

(Init) Our initial state allows the robot to get set and ensures all the sensors are ready and the topics have information before we begin to use them in our calculations. Once finished, the state is set to Lane Follow.

(Lane Follow) Our lane follow state keeps the robot speed set on cmd_vel to go forward until the sensors detect a corner or an obstacle, which sets the state to obstacle or corner depending on detection, unless the number of corners is 12, meaning that the robot has finished 3 full laps, which makes the state Park.

(Corner) Our corner state keeps the robot turning until the sensors detect that the robot is in between the 2 walls. Once it is, the state is set to Lane Follow.

(Obstacle) Our obstacle state keeps the robot turns either left or right of obstacle depending on the color detection of the camera, Once it finishes, the state sets back to Lane Follow.

(Park) Our parking state keeps the robot going forward until it is halfway in the tunnel, where it started, and then changes the state to Stop.
