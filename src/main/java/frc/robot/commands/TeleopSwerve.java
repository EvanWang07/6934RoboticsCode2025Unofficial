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

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier visionSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.visionSup = visionSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), QuickTuning.driveStickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), QuickTuning.driveStickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), QuickTuning.driveStickDeadband);
        boolean isFieldCentric = !robotCentricSup.getAsBoolean();
        boolean activateVision = visionSup.getAsBoolean();

        /* Vision (if active) */
        if (activateVision) {
            if (VisionInfo.willTarget()) {
                translationVal = VisionInfo.getTranslationalCorrectionOutput();
                strafeVal = 0;
                rotationVal = -VisionInfo.getRotationalCorrectionOutput();
                isFieldCentric = false; // Limelight needs to use robot-centric swerve
            } else {
                translationVal = 0;
                strafeVal = 0;
                rotationVal = Vision.targetSearchOutput;
                isFieldCentric = false;
            }
            System.out.println("TX: " + VisionInfo.getTX(false) + ", TY: " + VisionInfo.getTY(false) + ", TV: " + VisionInfo.willTarget());
        }

        VisionInfo.updateSummaryValues();

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            isFieldCentric, 
            true
        );
    }
}