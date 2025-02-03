package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.VisionInfo;
import frc.robot.Constants.*;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier visionSup;
    private BooleanSupplier autoPositionSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier visionSup, BooleanSupplier autoPositionSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup; // Y
        this.strafeSup = strafeSup; // X
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.visionSup = visionSup;
        this.autoPositionSup = autoPositionSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), QuickTuning.driveStickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), QuickTuning.driveStickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), QuickTuning.driveStickDeadband);
        boolean isFieldCentric = !robotCentricSup.getAsBoolean();
        boolean activateAutoPosition = autoPositionSup.getAsBoolean();
        boolean seekTarget = activateAutoPosition ? false : visionSup.getAsBoolean(); // Auto Position should override Seek Target
        

        /* Vision Target Seeking (If Active) */
        if (seekTarget) {
            // LimelightHelpers.SetRobotOrientation(Vision.limelightName, s_Swerve.getGyroYaw().getDegrees(), 0, 0, 0, 0, 0);
            if (VisionInfo.willTarget()) { // Divide by the speed multiplier to ensure consistent homing speeds no matter the speed setting
                translationVal = -VisionInfo.getForwardCorrectionOutput() / s_Swerve.getSpeedMultiplier();
                strafeVal = 0;
                rotationVal = -VisionInfo.getRotationalCorrectionOutput() / s_Swerve.getSpeedMultiplier();
                isFieldCentric = false; // Limelight needs to use robot-centric swerve
            } else {
                translationVal = 0;
                strafeVal = 0;
                rotationVal = Vision.targetSearchOutput / s_Swerve.getSpeedMultiplier();
                isFieldCentric = false;
            }
        }

        /* Vision Auto Position (If Active) */
        if (activateAutoPosition) {
            if (VisionInfo.willTarget()) { // Divide by the speed multiplier to ensure consistent homing speeds no matter the speed setting
                translationVal = -VisionInfo.getForwardCorrectionOutput() / s_Swerve.getSpeedMultiplier(); 
                strafeVal = VisionInfo.getHorizontalCorrectionOutput() / s_Swerve.getSpeedMultiplier(); 
                rotationVal = VisionInfo.getPoseCorrectionOutput() / s_Swerve.getSpeedMultiplier();
                isFieldCentric = false;
            } else {
                translationVal = 0;
                strafeVal = 0;
                rotationVal = Vision.targetSearchOutput / s_Swerve.getSpeedMultiplier();
                isFieldCentric = false;
            }
        }

        VisionInfo.updateSummaryValues(); // Puts relevant vision values into SmartDashboard

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            isFieldCentric, 
            true
        );
    }
}