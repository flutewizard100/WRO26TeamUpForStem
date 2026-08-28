

# Contents:
### 1: Base and drivetrain
### 1.1: base structure
### 1.2: drivetrain systems
### 2: Electronics
### 2.1: motor and ESC
### 2.2: servo, steering, and steering controls
### 2.3: LiDAR and camera
### 2.4: wiring diagram
### 3: Software and coding
### 3.1: Code   
### 3.2: software
### 3.3: electronic boards and other components
### 4: Engineering solutions
  
# 1:
1.1: The base

Layer one

#### The first layer of this autonomous vehicle is fabricated out of a modified LaTrax Desert Prerunner base. We chose this specific base because of its durability and the fact that it has a pre-built drivetrain makes it easier and quicker to fix issues and find specific parts needed to construct the autonomous vehicle. The bottom plate is made out of durable plastic material, making it stronger than a three-dimensional print and since it comes out of a stock version of the LaTrax Desert Prerunner, it is easy to replace. the suspension on the front and rear axles is also stock, and the connection between the base and suspension is modular, making it easy to fix issues that require disassembly of the bottom plate. In the middle of the vehicle also runs a solid, metal axle, thus further reinforcing it. 
<img width="600" height="430" alt="latrax" src="https://github.com/user-attachments/assets/6b16b517-49bf-43fd-9922-35f5db57dfde" />

base without modification


<img width="5712" height="4284" alt="HEIF Image(3)" src="https://github.com/user-attachments/assets/0c926a87-ac81-46b2-ad50-93e53219569e" /> 
base prototype with modification and top plates


#### For attaching a stronger motor to the base, we removed the top gearbox housing and 3d printed a new one to fit the new, more powerful motor.
<img width="1200" height="840" alt="httpstraxxas comsitesdefaultfilesimagesproducts7590x" src="https://github.com/user-attachments/assets/7dc44785-77eb-4c4e-9500-2db8c35ddcf7" />
this image shows the piece that was replaced by this 3d print:
<img width="850" height="506" alt="Screenshot 2026-08-22 151803" src="https://github.com/user-attachments/assets/1a554cab-41ee-4c50-9005-22be83098bb8" />
<img width="696" height="713" alt="1Screenshot 2026-08-22 152641" src="https://github.com/user-attachments/assets/929d69d5-c47b-4549-9a1c-7592e7f38e95" /> 

#### We also removed the front bumper from the LaTrax Desert Prerunner in order to attach the Limelight camera mount, which was 3d printed like many of the mounts in this autonomous vehicle.
<img width="257" height="274" alt="Screenshot 2026-08-26 135833" src="https://github.com/user-attachments/assets/7b1241c4-f3d6-4c44-8b40-b53d0bf5ab01" /><img width="925" height="218" alt="333scrnsht" src="https://github.com/user-attachments/assets/daf5569f-b419-4df0-b714-9524a6a3aefe" />
<img width="976" height="256" alt="222scrnsht" src="https://github.com/user-attachments/assets/2795a3a1-2d3c-48c6-ae20-a284915af7fe" />

these pictures show the limelight mount 3d print.




Layer two & Three
#### The second layer is much simpler than the first: It consists of a laser-cut, wooden middle plate that holds electronic components like the battery, the on/off switch, and the LiDAR.
It also supports four metal GoBilda axles that hold the top plate. The reason for the material to be wood is that it is easy and fast to laser cut, and that it is abundant so that we have lots of spare material in case the top and middle plates break. 
The top plate is much like the middle plate; except for the shape and the electronics it holds. It is also made of wood and is laser-cut. It holds the Jetson Nano, Arduino Teensy, and LiDAR control board. 


Prototype one:
#### The first prototype has a 3d printed base:<img width="1001" height="543" alt="444scrnsht" src="https://github.com/user-attachments/assets/555a4b37-9475-4010-af31-b2745bed91af" /><img width="1095" height="671" alt="scrnsht555 1" src="https://github.com/user-attachments/assets/19933b24-d5e7-40bb-89eb-9bfe73578e52" />

#### but in the base you see here, there are only mounts for the Jetson Nano and the Arduino Teensy, but no mount for the motor or battery. So, to fix those issues, we 3d printed another base and moved the board mounts to the top plate:
<img width="1287" height="612" alt="666scrnsht" src="https://github.com/user-attachments/assets/685644a8-46de-48ba-9f4c-13c11f2e182a" />
<img width="939" height="607" alt="777 7 screenshots for documentation" src="https://github.com/user-attachments/assets/907cc0f6-ba47-4c27-bd72-e1abde57032f" />

here is the new prototype for the base:
<img width="1133" height="780" alt="888 8 screenshot for documentation" src="https://github.com/user-attachments/assets/c4b09b67-02a9-49fe-9cfa-a2dd3fa77241" />

