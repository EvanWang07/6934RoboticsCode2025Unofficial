package frc.robot.commands;

import frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TeleopElevator extends Command {
    private final Elevator e_Elevator;
    private DoubleSupplier speedSup;

    public TeleopElevator(Elevator e_Elevator, DoubleSupplier speedSup) {
        this.e_Elevator = e_Elevator;
        this.speedSup = speedSup;
        
        addRequirements(e_Elevator);
    }

    public void execute() {
        e_Elevator.setElevatorMotorSpeed(speedSup.getAsDouble());
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        e_Elevator.brakeElevator();
    }
}
