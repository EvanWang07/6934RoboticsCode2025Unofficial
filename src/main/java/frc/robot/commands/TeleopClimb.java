package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class TeleopClimb extends Command {
    private final Climb c_Climb;
    private final DoubleSupplier speedSup;

    public TeleopClimb(Climb c_Climb, DoubleSupplier speedSup) {
        this.c_Climb = c_Climb;
        addRequirements(c_Climb);

        this.speedSup = speedSup;
    }

    @Override
    public void execute() {
        double climbSpeed = speedSup.getAsDouble();
        c_Climb.setClimberSpeed(climbSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        c_Climb.brakeClimber();
    }
}
