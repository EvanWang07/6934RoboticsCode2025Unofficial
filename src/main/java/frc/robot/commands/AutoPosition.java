package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.VisionInfo;
import frc.robot.subsystems.Swerve;

public class AutoPosition extends Command {
    private Swerve s_Swerve;
    private ProfiledPIDController horizontalPIDController;
    private ProfiledPIDController forwardPIDController;
    private ProfiledPIDController rotationPIDController;

    public AutoPosition(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        horizontalPIDController = new ProfiledPIDController(3, 
                                                            Vision.TXkI, 
                                                            Vision.TXkD, 
                                                            new TrapezoidProfile.Constraints(Vision.TXMaxSpeed, Vision.TXMaxAcceleration));
        forwardPIDController = new ProfiledPIDController(3, 
                                                         Vision.TYkI, 
                                                         Vision.TYkD, 
                                                         new TrapezoidProfile.Constraints(Vision.TYMaxSpeed, Vision.TYMaxAcceleration));
        rotationPIDController = new ProfiledPIDController(4, 
                                                          Vision.posekI, 
                                                          Vision.posekD, 
                                                          new TrapezoidProfile.Constraints(Vision.poseMaxSpeed, Vision.poseMaxAcceleration));
    }

    @Override
    public void initialize() {
        horizontalPIDController.setTolerance(Units.degreesToRotations(Vision.TXTolerance));
        horizontalPIDController.reset(Units.degreesToRotations(VisionInfo.getTX(false)));
        forwardPIDController.setTolerance(Units.degreesToRotations(Vision.TYTolerance));
        forwardPIDController.reset(Units.degreesToRotations(VisionInfo.getTY(false)));
        rotationPIDController.setTolerance(Units.degreesToRotations(Vision.poseTolerance));
        rotationPIDController.reset(Units.degreesToRotations(VisionInfo.getPoseTheta()));
    }

    @Override
    public void execute() {
        double horizontalOutput = 0;
        double forwardOutput = 0;
        double rotationalOutput = 0;
        if (VisionInfo.willTarget()) {
            horizontalOutput = horizontalPIDController.calculate(Units.degreesToRotations(VisionInfo.getTX(false)), 0) / s_Swerve.getSpeedMultiplier();
            forwardOutput = forwardPIDController.calculate(Units.degreesToRotations(VisionInfo.getTY(false)), 0) / s_Swerve.getSpeedMultiplier();
            rotationalOutput = -rotationPIDController.calculate(Units.degreesToRotations(VisionInfo.getPoseTheta()), 0) / s_Swerve.getSpeedMultiplier();
            if (Math.abs(rotationPIDController.getPositionError()) > Units.degreesToRotations(10)) {
                forwardOutput = 0;
                horizontalOutput = 0;
                System.out.println(rotationPIDController.getPositionError() + " " + Units.degreesToRotations(10));
            }
        }
        
        s_Swerve.drive( // Drive
            new Translation2d(forwardOutput, horizontalOutput).times(Constants.Swerve.maxSpeed), 
            rotationalOutput * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
    }

    @Override
    public boolean isFinished() {
        return (horizontalPIDController.atGoal() && forwardPIDController.atGoal() && rotationPIDController.atGoal());
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
