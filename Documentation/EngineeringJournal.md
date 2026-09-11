# Contents
- Mobility and Mechanical Design
- Power and Sensor Architecture
- Software Architecture and Obstacle Strategy
- Systems Thinking and Engineering Decisions

# Mobility and Mechanical Design

## Car Base Iterations

### Version 1.0:

In our first attempt at making our own car, we used a fully 3D-printed body as our main structure. We planned to mount a much larger motor and servo, with all of our sensors on top. This plan unfortunately did not work because our sensors would not fit, and the body was too bulky and inconvenient.

<img height="300" alt="Version 1.0 chassis design" src="https://github.com/user-attachments/assets/555a4b37-9475-4010-af31-b2745bed91af" /><img height="300" alt="Version 1.0 additional view" src="https://github.com/user-attachments/assets/19933b24-d5e7-40bb-89eb-9bfe73578e52" />

### Version 1.1:

After our first version, we attempted to keep our main structure and move around some parts to make it smaller, stronger, and fit better. To do this, we added a flat top layer to mount the sensors. This helped solve some of the space issues for most of the sensors, but the base was still too high for the LiDAR to fit and scan the walls effectively.

<img height="300" alt="Version 1.1 chassis with sensor platform" src="https://github.com/user-attachments/assets/907cc0f6-ba47-4c27-bd72-e1abde57032f" />

<img height="300" alt="Version 1.1 additional view" src="https://github.com/user-attachments/assets/c4b09b67-02a9-49fe-9cfa-a2dd3fa77241" />

### Version 2.1:

After our trials with a 3D-printed base, we decided to switch to a premade base and make several major changes. We used the LaTrax Desert Prerunner base and removed the casing and the original structural plate, keeping the original servo and motor at first.

We later decided to replace the motor with one recommended by the MIT course and the servo with one recommended by a local RC car expert.

<img height="300" alt="Version 2.1 premade chassis" src="https://github.com/user-attachments/assets/eeeb84d8-b88b-4afd-9ec9-7f5d06e5da54" />

### Version 2.2:

We replaced the motor with an ARRMA MEGA 380 brushed motor and added a new mount to hold it. We also replaced the servo with a Traxxas 2265 metal-gear servo and added an electronic speed controller (ESC) so we could control the motor and supply it with power.

<img height="300" alt="Version 2.2 updated components and chassis" src="https://github.com/user-attachments/assets/8e530a42-62d5-4974-b7af-0bd89ea8e1bd" />

## Choosing Components

We chose the ARRMA MEGA 380 brushed motor because the MIT course recommended it and we already had it. The Traxxas 2265 metal-gear servo was recommended by a local RC car expert, and we already had that available too.

| Component | What We Used | Why We Used It |
|---|---|---|
| Chassis | LaTrax Desert Prerunner base | Our printed bases were too bulky and had problems with sensor space and LiDAR placement. |
| Motor | ARRMA MEGA 380 brushed motor | The MIT course recommended it, and it was easy to get. |
| Motor mount | New mount for the replacement motor | To hold the new motor in place. |
| Servo | Traxxas 2265 metal-gear servo | A local RC car expert recommended it, and it was easy to find. |
| Electronic speed controller (ESC) | HOBBYWING QUICRUN WP 1080 G2 Brushed 2–3S | To control the motor and supply it with power. |

## Design Tradeoffs and Changes

At first, we tried to keep our 3D-printed structure and fix the layout instead of changing the whole base. Adding the flat top layer helped most of the sensors fit, but it did not fix the height problem for the LiDAR.

We then switched to a premade base instead of continuing with the printed one. We still had to make changes to it, including removing the casing and original structural plate and adding a mount for the replacement motor.

The main problem with our printed versions was not just fitting all the parts. We also needed the LiDAR to sit where it could scan the walls effectively.
