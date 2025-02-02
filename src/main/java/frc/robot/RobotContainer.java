package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands; // Will be useful...later
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.QuickTuning;
import frc.robot.Constants.Vision;

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
    private final JoystickButton useAutoPosition = new JoystickButton(driver, XboxController.Button.kX.value);
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
                () -> useVision.getAsBoolean(),
                () -> useAutoPosition.getAsBoolean()
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
        // useAutoPosition.onTrue(new PoseAlign(s_Swerve).withTimeout(2));
        /* (Not used for now)
        useAutoPosition.onTrue(new VisionAlign(s_Swerve).withTimeout(3).andThen(Commands.runOnce(() -> { // VisionAlign might not be needed
            s_Swerve.setSpeedMultiplier(0.35);
            s_Swerve.setPose(new Pose2d());
            double targetPose = VisionInfo.getPoseTheta();
            double targetLocation = VisionInfo.getDistance(Vision.reefAprilTagHeights);
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( // Here, the poses are meant for direction, not holonomic rotation
                new Pose2d(targetLocation / 2, 0, new Rotation2d()),
                new Pose2d(targetLocation, 0, new Rotation2d())
            );

            PathConstraints constraints = new PathConstraints(1, 1, Math.PI * 0.5, Math.PI);

            PathPlannerPath visionDrivePath = new PathPlannerPath(
                waypoints,
                constraints,
                null, // Not relevant for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(targetPose)) // Use holonomic rotation here 
            );

            visionDrivePath.preventFlipping = true;

            AutoBuilder.followPath(visionDrivePath).schedule();
        })));
        */
        
        
    }

    /* Autonomous Code */
    public Command getAutonomousCommand() {
        s_Swerve.setSpeedMultiplier(1);
        return new PathPlannerAuto("Basic Autonomous");
    }
}