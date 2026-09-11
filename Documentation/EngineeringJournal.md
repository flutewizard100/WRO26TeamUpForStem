# Contents
- Mobility and Mechanical Design
- Power and Sensor Architecture
- Software Architecture and Obstacle Strategy
- Systems Thinking and Engineering Decisions

# Mobility and Mechanical Design

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

# Power and Sensor Architecture 

## Power Distribution

Power goes from the battery through a fuse and switch to the PDB, which splits it into three branches:

```text
Battery → Fuse → Switch → PDB
                          ├── ESC → Motor
                          ├── Step-Down Converter → Servo
                          └── Orin
```

The ESC controls the motor, the step-down converter lowers the voltage for the servo, and the last branch powers the Orin.

Only power connections are shown, not signal wires.

## Sensors

| Sensor | Model | What We Use It For | Why We Chose It | Where It Is Mounted | Why This Position |
|---|---|---|---|---|---|
| LiDAR | LDRobot LD19 | Build a 2D map using light reflected off walls and obstacles | We wanted to measure the distance to walls around the car, not just directly in front of it. | Screwed to the Level 1 plate | Keeps the LiDAR below the top of the walls so it can scan them. |
| Camera | GoBilda Limelight 3A | Detect corners and obstacle colors | It gives us information about colors in front of and slightly to the sides of the car, rather than only directly below it. | Elevated above the car and mounted at an angle | Allows the camera to see the far front wall from any position. |
| Odometry | SparkFun Optical Tracking Odometry Sensor | Track the robot’s position on the field | It is small, easy to mount, and fits under the car base. | Under the LaTrax base, in the center of the robot | Places the sensor at the correct height and position for accurate readings. |

## Sensor Consideration 

### LiDAR

