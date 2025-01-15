package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.QuickTuning;

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(QuickTuning.driveControllerID);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value; // Translation = Y
    private final int strafeAxis = XboxController.Axis.kLeftX.value; // Strafe = X
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton useVision = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton restartSwerve = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton speedUpRobot = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton slowDownRobot = new JoystickButton(driver, XboxController.Button.kBack.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();


    /* Robot Container */
    public RobotContainer() {

        // NamedCommands.registerCommand("Intake Note", new Command(Parameters).withTimeout(Seconds));

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(), 
                () -> useVision.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /* Button Bindings */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        speedUpRobot.onTrue(new InstantCommand(() -> s_Swerve.setSpeedMultiplier(1)));
        slowDownRobot.onTrue(new InstantCommand(() -> s_Swerve.setSpeedMultiplier(QuickTuning.driveSlowModeMultiplier)));
        // restartSwerve.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
    }

    /* Autonomous Code */
    public Command getAutonomousCommand() {
        s_Swerve.setSpeedMultiplier(1);
        return new PathPlannerAuto("Basic Autonomous");
    }
    //lebrom is the goat
}
// Test (learning Git)
//Eyouel moment