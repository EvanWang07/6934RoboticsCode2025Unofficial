package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.NamedCommands; // Will be useful...later
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.QuickTuning;

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(QuickTuning.driveControllerID);
    private final Joystick weapons = new Joystick(QuickTuning.weaponControllerID);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value; // Translation = Y
    private final int strafeAxis = XboxController.Axis.kLeftX.value; // Strafe = X
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Weapon Controls */
    private final int elevatorAxis = XboxController.Axis.kLeftY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton useVision = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton useAutoPosition = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton speedUpRobot = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton slowDownRobot = new JoystickButton(driver, XboxController.Button.kBack.value);

    /* Weapon Buttons */
    private final JoystickButton sendElevatorBottom = new JoystickButton(weapons, XboxController.Button.kA.value);
    private final JoystickButton sendElevatorLevelOne = new JoystickButton(weapons, XboxController.Button.kB.value);
    private final JoystickButton sendElevatorLevelTwo = new JoystickButton(weapons, XboxController.Button.kX.value);
    private final JoystickButton sendElevatorLevelThree = new JoystickButton(weapons, XboxController.Button.kY.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator e_Elevator = new Elevator();

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
                () -> useVision.getAsBoolean(),
                () -> useAutoPosition.getAsBoolean()
            )
        );

        e_Elevator.setDefaultCommand(
            new TeleopElevator(
                e_Elevator,
                () -> -weapons.getRawAxis(elevatorAxis)
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
        
        /* Weapons Buttons */
        sendElevatorBottom.whileTrue(new AutoElevator(e_Elevator, Units.degreesToRotations(Constants.Elevator.elevatorLowerBound / Constants.Elevator.elevatorGearRatio)));
        sendElevatorLevelOne.whileTrue(new AutoElevator(e_Elevator, Constants.Elevator.levelOneHeightInRotations));
        sendElevatorLevelTwo.whileTrue(new AutoElevator(e_Elevator, Constants.Elevator.levelTwoHeightInRotations));
        sendElevatorLevelThree.whileTrue(new AutoElevator(e_Elevator, Constants.Elevator.levelThreeHeightInRotations));
        
    }

    /* Autonomous Code */
    public Command getAutonomousCommand() {
        s_Swerve.setSpeedMultiplier(1);
        return new PathPlannerAuto("Basic Autonomous");
    }
}