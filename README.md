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

## Issues and Potential Errors:   
- None for now!

## To-Do List:  
- [ ] Work on robot pose.  
- [ ] \(Optional\) - Create new control bindings (Throttle-based and Southpaw) (Evan: This is unlikely to happen)

### Unused Code  
- None for now. 

### Notes  
- All code should be <ins>peer-reviewed</ins> and <ins>tested</inv>. 

## Credits  
- Source of Original Code: https://github.com/dirtbikerxz/BaseTalonFXSwerve  
- Modified Code Created By: Evan Wang, Lukas Evans, Derek Chang  
- Robot "IT" Person: Ethan Jiang
- Robot Created By: FRC Robotics Team 6934 (ACHS Scorpions)  

## Falcon Swerve Chassis Configs  
> [!WARNING]
> The Falcon swerve chassis is currently out of order. Please do not change the below values until it is deemed functional again. (Evan: Unlikely to happen in the near future)

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
- Toggle Slow Mode OFF: *PRESS* Start-Button  <br> 
- Toggle Slow Mode ON: *PRESS* Back-Button  <br> 
- Auto-Align: *HOLD* A-Button  <br>   

[^1]: Last updated 1/23/25 by Evan Wang.
