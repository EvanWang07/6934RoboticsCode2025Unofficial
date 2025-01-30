package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.VisionInfo;
import frc.robot.subsystems.Swerve;

public class PoseAlign extends Command {
    private Swerve s_Swerve;

    public PoseAlign(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        double rotationVal = VisionInfo.getPoseCorrectionOutput() / s_Swerve.getSpeedMultiplier();

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
        if (VisionInfo.isZeroPose()) { // Check alignment
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        /* Drive */
        s_Swerve.drive( // Stops the swerve drive
            new Translation2d(0, 0), 
            0, 
            false, 
            true
        );
    }
}
