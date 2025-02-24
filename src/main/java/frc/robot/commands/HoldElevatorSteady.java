package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class HoldElevatorSteady extends Command {
    private final Elevator e_Elevator;

    public HoldElevatorSteady(Elevator e_Elevator) {
        this.e_Elevator = e_Elevator;
        addRequirements(e_Elevator);
    }

    @Override
    public void execute() {
        e_Elevator.setElevatorMotorSpeed(0);
    }

    @Override
    public boolean isFinished() { // A timeout should be implemented when this command is used
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        e_Elevator.brakeElevator();
    }
}
