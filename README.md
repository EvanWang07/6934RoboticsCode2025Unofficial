![](TeamLogo6934.png)

# Description: 
The current[^1] 2024-2025 robotics code for FRC team 6934.

## Changelog:  
- Updated the teleoperated swerve code for FRC 2025 <sup>1/19/25</sup>
- Updated the Kraken swerve offsets ~~(hopefully, for the final time)~~ <sup>1/19/25</sup>
- Updated project from WIPlib version 2025.1.1 to 2025.2.1 <sup>1/22/25</sup>
- Reformatted the README file <sup>1/22/25</sup>
- Added experimental pose component estimation methods <sup>1/22/25</sup>
- Updated the LimelightHelpers file for FRC 2025 <sup>1/23/25</sup>
- Vision now works and properly ignores slow mode drive <sup>1/23/25</sup>
- ~~Added an experimental vision auto positioning on-the-fly path generator (BUGGED) <sup>1/26/25</sup>~~
- Added an experimental code for the mailbox/end-effector <sup>2/1/25</sup>
- Added an experimental subsystem for the elevator <sup>2/1/25</sup>
- First successful vision auto-position code created <sup>2/5/25</sup>
- Added an experimental manual and automatic command for the elevator <sup>2/10/25</sup>
- Final candidate code #1 for mailbox/end-effector <sup>2/10/25</sup>
- First successful automatic command for the elevator <sup>2/13/25</sup>
- Updated controller mapping to include automatic scoring buttons <sup>2/16/25</sup>
- Added a test autonomous code for the elevator and end-effector <sup>2/16/25</sup>
- Moved the auto-position code to a dedicated command file <sup>2/24/25</sup>
- Implemented MegaTag 2 for Limelight <sup>3/1/25</sup>
- Revamped auto-position code to use on-the-fly path generation again <sup>3/1/25</sup>
- Added an experimental pathfinding + on-the-fly path code for driving the robot to the coral station <sup>3/4/25</sup>
- Changed pathfinding controls to involve holding, instead of pressing, the respective button <sup>3/6/25</sup>
- Increased elevator PID, pathfinding, and on-the-fly path speeds <sup>3/9/25</sup>
- Added documentation for every method within VisionInfo.java and BasicOperations.java <sup>6/1/25</sup>
- Removed unnecessary methods and variables relating to the climber code <sup>6/1/25</sup>
- Added documentation for every method within SwerveModule.java and some explanations in Swerve.java <sup>6/4/25</sup>
- Added documentation for basic hardware info <sup>6/6/25</sup>

## Issues and Potential Errors:   
- Robot auto-position is still slightly off
- PID elevator sometimes does not respect its bounds and moves to the wrong position

## To-Do List:  
- [ ] Write documentation for the remainder of the code
- [ ] Fix a PID elevator issue causing it to not respect its bounds and move to the wrong position in certain scenarios
- [ ] Clean up the overuse of unit conversions in the elevator PID code

### Unused Code  
- None for now. 

### Notes  
- All code should be <ins>peer-reviewed</ins> and <ins>tested</inv>. 

## Credits  
- Source of Original Code: https://github.com/dirtbikerxz/BaseTalonFXSwerve  
- Modified Code Created By: Evan Wang, Lukas Evans, Will Redekopp, Derek Chang
- Robot "IT" Person: Ethan Jiang
- Robot Created By: FRC Robotics Team 6934 (ACHS Scorpions)  

## Falcon Swerve Chassis Configs  
> [!WARNING]
> The Falcon swerve chassis is currently out of order. Please do not change the below values until it is deemed functional again.

| Name/Component | Offset (Degrees) |
| :--- | :---: |
| Swerve Module 0 | -108.45 |
| Swerve Module 1 | 11.865 + 180 |
| Swerve Module 2 | -31.7268 |
| Swerve Module 3 | N/A (Replaced) |
| CANivore Name | "Canivor<3" |

## Kraken Swerve Chassis Configs  

| Name/Component | Offset (Degrees) |
| :--- | :---: |
| Swerve Module 0 | -47.37312 + 180 |
| Swerve Module 1 | -39.63852 |
| Swerve Module 2 | -128.320 + 180 |
| Swerve Module 3 | 108.54504 |
| CANivore Name | "Second Canivor<3" |

## Control Bindings  
### Drive Controller (**PORT 0**):   
- Field-Centric Driving: *MOVE* Left Joystick (x & y)  
- Robot-Centric Driving: *HOLD* Left Bumper + *MOVE* Left Joystick (x & y)  
- Rotating: *MOVE* Right Joystick (x)  
- Reset Gyro (Field-Centric Driving ONLY): *PRESS* Y-Button  
- Toggle Slow Mode: *PRESS* Right Button
- Auto-Position (LEFT REEF): *PRESS* X-Button
- Auto-Position (CENTER REEF): *PRESS* A-Button
- Auto-Position (RIGHT REEF): *PRESS* B-Button
- Auto-Position (LEFT STATION): *HOLD* Back-Button
- Auto-Position (RIGHT STATION): *HOLD* Start-Button

### Weapons Controller (**PORT 1**):
- Manually Lift/Lower Elevator: *MOVE* Left Joystick (y)
- Manually Operate End-Effector (Intake/Score): *MOVE* Right Joystick (y)
- Climb: *MOVE* Left Trigger
- Automatically Lift/Lower Elevator to Bottom: *PRESS* A-Button
- Automatically Lift/Lower Elevator & Score at Reef Level 1: *PRESS* B-Button (*REQUIRES* Coral)
- Automatically Lift/Lower Elevator & Score at Reef Level 2: *PRESS* X-Button (*REQUIRES* Coral)
- Automatically Lift/Lower Elevator & Score at Reef Level 3: *PRESS* Y-Button (*REQUIRES* Coral)
- Intake Coral: *HOLD* Left Bumper
- Manually Score Coral: *HOLD* Right Bumper

## General Hardware Information
![image_2025-06-06_143659713](https://github.com/user-attachments/assets/4d4adca3-e970-4c08-bf01-6268d5ed9d7c)
### Swerve Specifications:
- MODEL: SDS MK4i Swerve w/ L2 gearing
- AZIMUTH/ANGLE MOTORS: Kraken X60
- DRIVE MOTORS: Kraken X60
- ABSOLUTE ENCODERS: CTRE CANcoders
- GYROSCOPE: Pigeon 2.0
### Elevator:
- MODEL: WCP GreyT Cascade Elevator (Two-stage)
- MOTORS: Kraken X60 x2
### Mailbox:
- MOTOR: NEO Vortex
- CORAL DETECTION: IR Beambreak sensor
### Vision System:
- CAMERA: Limelight 3G
### Other:
- ADDITIONAL CAN CAPACITY - CANivore

[^1]: Last updated 6/6/25 by Dere Chang.

