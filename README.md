**DESCRIPTION**: The current 2024-2025 robotics code for FRC team 6934  <br>   

**CHANGELOG**:  <br> 
&ensp;&ensp;1. Updated the teleoperated swerve code for FRC 2025  <br> 
&ensp;&ensp;2. Updated the code to account for a shift in location of the robot's "front"  <br> 
&ensp;&ensp;3. Fixed (hopefully) autonomous swerve by updating PathPlannerLib  <br>   
**ISSUES & POTENTIAL ERRORS**:  <br> 
&ensp;&ensp;1. Swerve start-up algorithm continues to fail for Module 0 (FL)  <br> 
&ensp;&ensp;2. New slow-mode toggle may cause issues with limelight autoalign  <br> 
&ensp;&ensp;3. Limelight sometimes loses detection  <br>   
**TODO LIST**:  <br> 
&ensp;&ensp;1. Fix autonomous swerve / Update autonomous swerve for 2025  <br> 
&ensp;&ensp;2. Further adapt the code to the new slow-mode toggle  <br>   
**UNUSED CODE**:  <br> 
&ensp;&ensp;1. None for now!  <br>   
**NOTES**:  <br> 
&ensp;&ensp;1. **ALL** code *ABSOLUTELY* needs to be peer-reviewed **AND** tested!!!  <br>   
**CREDITS**:  <br> 
&ensp;&ensp;1. Source of Original Code: https://github.com/dirtbikerxz/BaseTalonFXSwerve  <br> 
&ensp;&ensp;2. Modified Code Created By: Evan Wang (and soon, others!)  <br> 
&ensp;&ensp;3. Robot Created By: FRC Robotics Team 6934 (ACHS Scorpions)  <br>  

**FALCON SWERVE DRIVE CONFIGURATION**:  <br> 
&ensp;&ensp;1. Swerve Module 0 Offset: -108.45 degrees  <br> 
&ensp;&ensp;2. Swerve Module 1 Offset: 11.865 + 180 degrees  <br> 
&ensp;&ensp;3. Swerve Module 2 Offset: -31.7268 degrees  <br> 
&ensp;&ensp;4. Swerve Module 3 Offset: N/A (Replaced)  <br> 
&ensp;&ensp;5. CANivore Name: "Canivor<3"  <br>  

**KRAKEN SWERVE DRIVE CONFIGURATION**:  <br> 
&ensp;&ensp;1. Swerve Module 0 Offset: 79.101 - 30 degrees  <br> 
&ensp;&ensp;2. Swerve Module 1 Offset: 26.718 degrees  <br> 
&ensp;&ensp;3. Swerve Module 2 Offset: 97.490 + 180 - 10 degrees  <br> 
&ensp;&ensp;4. Swerve Module 3 Offset: 103.281 + 180 degrees  <br> 
&ensp;&ensp;5. CANivore Name: "Second Canivor<3"  <br>  

**DRIVER'S GUIDE**:  <br> 
&ensp;&ensp;1. **DRIVE CONTROLLER** (**PORT 0**):  <br> 
&ensp;&ensp;&ensp;&ensp;* Field-Centric Driving: *MOVE* Left Joystick (x & y)  <br> 
&ensp;&ensp;&ensp;&ensp;* Robot-Centric Driving: *HOLD* Left Bumper + *MOVE* Left Joystick (x & y)  <br> 
&ensp;&ensp;&ensp;&ensp;* Rotating: *MOVE* Right Joystick (x)  <br> 
&ensp;&ensp;&ensp;&ensp;* Reset Gyro (Field-Centric Driving ONLY): *PRESS* Y-Button  <br> 
&ensp;&ensp;&ensp;&ensp;* Toggle Slow Mode OFF: *PRESS* Start-Button  <br> 
&ensp;&ensp;&ensp;&ensp;* Toggle Slow Mode ON: *PRESS* Back-Button  <br> 
&ensp;&ensp;&ensp;&ensp;* Auto-Align: *HOLD* A-Button  <br>   

**LAST UPDATED**: 1/13/25