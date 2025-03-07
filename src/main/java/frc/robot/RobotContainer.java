package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
// import frc.robot.Constants.GameField;
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
    private final int mailboxAxis = XboxController.Axis.kRightY.value;

    private final int climberTrigger = XboxController.Axis.kLeftTrigger.value;

    /* Driver Buttons */
    private final JoystickButton useLeftStationAutoPosition = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton useRightStationAutoPosition = new JoystickButton(driver, XboxController.Button.kStart.value);

    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton toggleSlowMode = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton useCenterReefAutoPosition = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton useRightReefAutoPosition = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton useLeftReefAutoPosition = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

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
    private final Climb c_Climb = new Climb();

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
        NamedCommands.registerCommand("Drive to Left Branch And Score", Commands.runOnce(() -> {
            if (VisionInfo.willTarget()) {
                int detectedTagID = VisionInfo.getTargetID();

                Pose2d currentPose = s_Swerve.getSwervePoseEstimation();
                Pose2d startPose = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
                Pose2d endPose = VisionInfo.getRobotLeftGoal(detectedTagID);

                if (endPose != null) {
                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

                    PathPlannerPath pathToLeftGoal = new PathPlannerPath(
                        waypoints, 
                        new PathConstraints(
                        1.5, 1.25, 
                        Units.degreesToRadians(540), Units.degreesToRadians(720)
                        ),
                        null, // Ideal starting state can be null for on-the-fly paths
                        new GoalEndState(0.0, endPose.getRotation())
                    );

                    pathToLeftGoal.preventFlipping = true;

                    new ParallelDeadlineGroup(AutoBuilder.followPath(pathToLeftGoal), 
                                              new HoldElevatorSteady(e_Elevator).withTimeout(5))
                                                .andThen(new ParallelRaceGroup(new HoldElevatorSteady(e_Elevator).withTimeout(2), 
                                                                               new Intake(m_Mailbox, false).withTimeout(2))).schedule();
                }
            }
        }).withTimeout(10));
        NamedCommands.registerCommand("Drive to Right Branch And Score", Commands.runOnce(() -> {
            if (VisionInfo.willTarget()) {
                int detectedTagID = VisionInfo.getTargetID();

                Pose2d currentPose = s_Swerve.getSwervePoseEstimation();
                Pose2d startPose = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
                Pose2d endPose = VisionInfo.getRobotRightGoal(detectedTagID);

                if (endPose != null) {
                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

                    PathPlannerPath pathToRightGoal = new PathPlannerPath(
                        waypoints, 
                        new PathConstraints(
                        1.5, 1.25, 
                        Units.degreesToRadians(540), Units.degreesToRadians(720)
                        ),
                        null, // Ideal starting state can be null for on-the-fly paths
                        new GoalEndState(0.0, endPose.getRotation())
                    );

                    pathToRightGoal.preventFlipping = true;

                    new ParallelDeadlineGroup(AutoBuilder.followPath(pathToRightGoal), 
                                              new HoldElevatorSteady(e_Elevator).withTimeout(5))
                                                .andThen(new ParallelRaceGroup(new HoldElevatorSteady(e_Elevator).withTimeout(2), 
                                                                               new Intake(m_Mailbox, false).withTimeout(2))).schedule();
                }
            }
        }).withTimeout(10));

        /* Teleop Swerve Drive */
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        /* Teleop Elevator */
        e_Elevator.setDefaultCommand(
            new TeleopElevator(
                e_Elevator,
                () -> -weapons.getRawAxis(elevatorAxis)
            )
        );

        m_Mailbox.setDefaultCommand(
            new TeleopMailbox(
                m_Mailbox,
                () -> -weapons.getRawAxis(mailboxAxis)
            )
        );

        c_Climb.setDefaultCommand(
            new TeleopClimb(
                c_Climb, 
                () -> weapons.getRawAxis(climberTrigger)
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /* Button Bindings */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        toggleSlowMode.onTrue(Commands.runOnce(() -> {
            boolean isSlow = (s_Swerve.getSpeedMultiplier() == QuickTuning.driveSlowModeMultiplier);
            if (isSlow) {
                s_Swerve.setSpeedMultiplier(1);
            } else {
                s_Swerve.setSpeedMultiplier(QuickTuning.driveSlowModeMultiplier);
            }
        }));

        useLeftStationAutoPosition.whileTrue(s_Swerve.pathfindToLeftStation());
        useRightStationAutoPosition.whileTrue(s_Swerve.pathfindToRightStation());

        useLeftReefAutoPosition.onTrue(Commands.runOnce(() -> {
            if (VisionInfo.willTarget()) {
                int detectedTagID = VisionInfo.getTargetID();

                Pose2d currentPose = s_Swerve.getSwervePoseEstimation();
                Pose2d startPose = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
                Pose2d endPose = VisionInfo.getRobotLeftGoal(detectedTagID);

                if (endPose != null) {
                    double originalSpeed = s_Swerve.getSpeedMultiplier();
                    s_Swerve.setSpeedMultiplier(1);

                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

                    PathPlannerPath pathToLeftGoal = new PathPlannerPath(
                        waypoints, 
                        new PathConstraints(
                        1.5, 1.25, 
                        Units.degreesToRadians(540), Units.degreesToRadians(720)
                        ),
                        null, // Ideal starting state can be null for on-the-fly paths
                        new GoalEndState(0.0, endPose.getRotation())
                    );

                    pathToLeftGoal.preventFlipping = true;

                    AutoBuilder.followPath(pathToLeftGoal).andThen(new InstantCommand(() -> s_Swerve.setSpeedMultiplier(originalSpeed))).schedule();
                }
            }
        }));
        useCenterReefAutoPosition.onTrue(Commands.runOnce(() -> {
            if (VisionInfo.willTarget()) {
                int detectedTagID = VisionInfo.getTargetID();

                Pose2d currentPose = s_Swerve.getSwervePoseEstimation();
                Pose2d startPose = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
                Pose2d endPose = VisionInfo.getRobotCenterGoal(detectedTagID);

                if (endPose != null) {
                    double originalSpeed = s_Swerve.getSpeedMultiplier();
                    s_Swerve.setSpeedMultiplier(1);

                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

                    PathPlannerPath pathToCenterGoal = new PathPlannerPath(
                        waypoints, 
                        new PathConstraints(
                        1.5, 1.25, 
                        Units.degreesToRadians(540), Units.degreesToRadians(720)
                        ),
                        null, // Ideal starting state can be null for on-the-fly paths
                        new GoalEndState(0.0, endPose.getRotation())
                    );

                    pathToCenterGoal.preventFlipping = true;

                    AutoBuilder.followPath(pathToCenterGoal).andThen(new InstantCommand(() -> s_Swerve.setSpeedMultiplier(originalSpeed))).schedule();
                }
            }
        }));
        useRightReefAutoPosition.onTrue(Commands.runOnce(() -> {
            if (VisionInfo.willTarget()) {
                int detectedTagID = VisionInfo.getTargetID();

                Pose2d currentPose = s_Swerve.getSwervePoseEstimation();
                Pose2d startPose = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
                Pose2d endPose = VisionInfo.getRobotRightGoal(detectedTagID);

                if (endPose != null) {
                    double originalSpeed = s_Swerve.getSpeedMultiplier();
                    s_Swerve.setSpeedMultiplier(1);

                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

                    PathPlannerPath pathToRightGoal = new PathPlannerPath(
                        waypoints, 
                        new PathConstraints(
                        1.5, 1.25, 
                        Units.degreesToRadians(540), Units.degreesToRadians(720)
                        ),
                        null, // Ideal starting state can be null for on-the-fly paths
                        new GoalEndState(0.0, endPose.getRotation())
                    );

                    pathToRightGoal.preventFlipping = true;

                    AutoBuilder.followPath(pathToRightGoal).andThen(new InstantCommand(() -> s_Swerve.setSpeedMultiplier(originalSpeed))).schedule();
                }
            }
        }));
        
        /* Weapons Buttons */
        intakeCoral.whileTrue(new Intake(m_Mailbox, true));
        scoreCoral.whileTrue(new Intake(m_Mailbox, false));

        sendElevatorIntake.onTrue(new AutoElevator(e_Elevator, Constants.Elevator.intakeHeightInRotations).withTimeout(3));
        scoreLevelOne.onTrue(Commands.runOnce(() -> {
            if (m_Mailbox.coralIsDetected()) {
                new AutoElevator(e_Elevator, Constants.Elevator.levelOneHeightInRotations)
                .andThen(new ParallelRaceGroup(new HoldElevatorSteady(e_Elevator).withTimeout(2), 
                                               new Intake(m_Mailbox, false).withTimeout(2)))
                .schedule();
            }
        }, m_Mailbox, e_Elevator));
        scoreLevelTwo.onTrue(Commands.runOnce(() -> {
            if (m_Mailbox.coralIsDetected()) {
                new AutoElevator(e_Elevator, Constants.Elevator.levelTwoHeightInRotations)
                .andThen(new ParallelRaceGroup(new HoldElevatorSteady(e_Elevator).withTimeout(2), 
                                               new Intake(m_Mailbox, false).withTimeout(2)))
                .schedule();
            }
        }, m_Mailbox, e_Elevator));
        scoreLevelThree.onTrue(Commands.runOnce(() -> {
            if (m_Mailbox.coralIsDetected()) {
                new AutoElevator(e_Elevator, Constants.Elevator.levelThreeHeightInRotations)
                .andThen(new ParallelRaceGroup(new HoldElevatorSteady(e_Elevator).withTimeout(2), 
                                               new Intake(m_Mailbox, false).withTimeout(2)))
                .schedule();
            }
        }, m_Mailbox, e_Elevator));
    }

    /* Autonomous Code */
    public Command getAutonomousCommand() {
        s_Swerve.setSpeedMultiplier(1);
        
        var alliance = DriverStation.getAlliance();
        Pose2d startingPose = Constants.QuickTuning.selectedStartingPose;
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            startingPose = BasicOperations.transformBlueToRedAlliancePose(Constants.QuickTuning.selectedStartingPose);
        }
        s_Swerve.setSwervePoseEstimate(startingPose);

        return new PathPlannerAuto("Efficient LWalltItLCStJ");
    }
}