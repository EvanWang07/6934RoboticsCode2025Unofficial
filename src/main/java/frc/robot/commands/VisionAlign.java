package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.VisionInfo;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionAlign extends Command {
    private Swerve s_Swerve;

    public VisionAlign(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        double rotationVal = -VisionInfo.getRotationalCorrectionOutput() / s_Swerve.getSpeedMultiplier(); // Rotational gain

        /* Drive */
        s_Swerve.drive(
            new Translation2d(0, 0), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
    }

    @Override
    public boolean isFinished() {
        if (VisionInfo.isHorizontallyAligned()) { // Check alignment
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive( // Stops the swerve drive
            new Translation2d(0, 0), 
            0, 
            false, 
            true
        );
    }
}
