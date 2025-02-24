package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.NamedCommands;
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
    private final JoystickButton intakeCoral = new JoystickButton(weapons, XboxController.Button.kLeftBumper.value);
    private final JoystickButton scoreCoral = new JoystickButton(weapons, XboxController.Button.kRightBumper.value);

    private final JoystickButton sendElevatorIntake = new JoystickButton(weapons, XboxController.Button.kA.value);
    private final JoystickButton scoreLevelOne = new JoystickButton(weapons, XboxController.Button.kB.value);
    private final JoystickButton scoreLevelTwo = new JoystickButton(weapons, XboxController.Button.kX.value);
    private final JoystickButton scoreLevelThree = new JoystickButton(weapons, XboxController.Button.kY.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator e_Elevator = new Elevator();
    private final Mailbox m_Mailbox = new Mailbox();

    /* Robot Container */
    public RobotContainer() {

        /* PathPlanner Registered Commands */
        NamedCommands.registerCommand("Intake Coral", new Intake(m_Mailbox, true).withTimeout(2));
        NamedCommands.registerCommand("Score Coral", new Intake(m_Mailbox, false).withTimeout(2));
        NamedCommands.registerCommand("Hold Elevator Steady", new HoldElevatorSteady(e_Elevator).withTimeout(2));
        NamedCommands.registerCommand("Elevator to Intake", new AutoElevator(e_Elevator, Constants.Elevator.intakeHeightInRotations).withTimeout(3));
        NamedCommands.registerCommand("Elevator to L1", new AutoElevator(e_Elevator, Constants.Elevator.levelOneHeightInRotations).withTimeout(3));
        NamedCommands.registerCommand("Elevator to L2", new AutoElevator(e_Elevator, Constants.Elevator.levelTwoHeightInRotations).withTimeout(3));
        NamedCommands.registerCommand("Elevator to L3", new AutoElevator(e_Elevator, Constants.Elevator.levelThreeHeightInRotations).withTimeout(3));

        /* Teleop Swerve Drive */
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

        /* Teleop Elevator */
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

        useAutoPosition.whileTrue(new AutoPosition(s_Swerve));
        
        /* Weapons Buttons */
        intakeCoral.whileTrue(new Intake(m_Mailbox, true));
        scoreCoral.whileTrue(new Intake(m_Mailbox, false));

        sendElevatorIntake.onTrue(new AutoElevator(e_Elevator, Constants.Elevator.intakeHeightInRotations).withTimeout(3));
        scoreLevelOne.onTrue(Commands.runOnce(() -> {
            if (m_Mailbox.coralIsDetected()) {
                new AutoElevator(e_Elevator, Constants.Elevator.levelOneHeightInRotations)
                .andThen(new ParallelRaceGroup(new HoldElevatorSteady(e_Elevator).withTimeout(2), 
                                               new Intake(m_Mailbox, false).withTimeout(2)))
                .andThen(new AutoElevator(e_Elevator, Constants.Elevator.intakeHeightInRotations).withTimeout(3))
                .schedule();
            }
        }, m_Mailbox, e_Elevator));
        scoreLevelTwo.onTrue(Commands.runOnce(() -> {
            if (m_Mailbox.coralIsDetected()) {
                new AutoElevator(e_Elevator, Constants.Elevator.levelTwoHeightInRotations)
                .andThen(new ParallelRaceGroup(new HoldElevatorSteady(e_Elevator).withTimeout(2), 
                                               new Intake(m_Mailbox, false).withTimeout(2)))
                .andThen(new AutoElevator(e_Elevator, Constants.Elevator.intakeHeightInRotations).withTimeout(3))
                .schedule();
            }
        }, m_Mailbox, e_Elevator));
        scoreLevelThree.onTrue(Commands.runOnce(() -> {
            if (m_Mailbox.coralIsDetected()) {
                new AutoElevator(e_Elevator, Constants.Elevator.levelThreeHeightInRotations)
                .andThen(new ParallelRaceGroup(new HoldElevatorSteady(e_Elevator).withTimeout(2), 
                                               new Intake(m_Mailbox, false).withTimeout(2)))
                .andThen(new AutoElevator(e_Elevator, Constants.Elevator.intakeHeightInRotations).withTimeout(3))
                .schedule();
            }
        }, m_Mailbox, e_Elevator));
    }

    /* Autonomous Code */
    public Command getAutonomousCommand() {
        s_Swerve.setSpeedMultiplier(1);
        return new PathPlannerAuto("Basic Autonomous");
    }
}