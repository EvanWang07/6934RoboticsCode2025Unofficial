![](TeamLogo6934.png)

# Description: 
The current[^1] 2024-2025 robotics code for FRC team 6934.

## Changelog:  
- Updated the teleoperated swerve code for FRC 2025  
- Updated the Kraken swerve offsets ~~(hopefully, for the final time)~~
- Reformatted the README file.

## Issues and Potential Errors:  
- New slow-mode toggle may cause issues with Limelight autoalign functionality.  
- Limelight is currently still unable to detect its targets.  

## To-Do List:  
- [ ] Fix Limelight and start on robot pose.  
- [ ] Further adapt the code to the new slow-mode toggle.
- [ ] \(Optional\) - Create new control bindings (Throttle-based and Southpaw)

### Unused Code  
- None for now. 

### Notes  
- All code should be <ins>peer-reviewed</ins> and <ins>tested</inv>. 

## Credits  
- Source of Original Code: https://github.com/dirtbikerxz/BaseTalonFXSwerve  
- Modified Code Created By: Evan Wang, Lukas Evans, Derek Chang  
- Robot Created By: FRC Robotics Team 6934 (ACHS Scorpions)  

## Falcon Swerve Chassis Configs  
> [!WARNING]
> The Falcon swerve chassis is currently out of order. Please do not change the below values until it is deemed functional again.

| Name/Component | Offset *(in degrees)* or Name |
| :--- | :---: |
| Swerve Module 0 | -108.45 |
| Swerve Module 1 | 11.865 + 180 |
| Swerve Module 2 | -31.7268 |
| Swerve Module 3 | N/A (Replaced) |
| CANivore Name | "Canivor<3" |

## Kraken Swerve Chassis Configs  

| Name/Component | Offset *(in degrees)* or Name |
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
- Toggle Slow Mode OFF: *PRESS* Start-Button  <br> 
- Toggle Slow Mode ON: *PRESS* Back-Button  <br> 
- Auto-Align: *HOLD* A-Button  <br>   

[^1]: Last updated 1/22/25 by Derek Chang.
