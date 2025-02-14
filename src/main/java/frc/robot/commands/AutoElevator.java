package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class AutoElevator extends Command {
    private Elevator e_Elevator = new Elevator();
    private double angleSetPoint;
    ProfiledPIDController elevatorPIDController;
    // ElevatorFeedforward elevatorFeedForwardController;

    public AutoElevator(Elevator e_Elevator, double setPointInRotations) {
        this.e_Elevator = e_Elevator;
        addRequirements(e_Elevator);

        angleSetPoint = setPointInRotations;
        elevatorPIDController = new ProfiledPIDController(Constants.Elevator.kP, 
                                                          Constants.Elevator.kI, 
                                                          Constants.Elevator.kD, 
                                                          new TrapezoidProfile.Constraints(Constants.Elevator.PIDMaxSpeedInRotations, Constants.Elevator.PIDMaxAccelerationInRotations));
        // elevatorFeedForwardController = new ElevatorFeedforward(0, 0, Constants.Elevator.kV);
    }

    @Override
    public void initialize() {
        elevatorPIDController.setTolerance(Constants.Elevator.PIDToleranceInRotations);
        elevatorPIDController.reset(Units.degreesToRotations(e_Elevator.getElevatorPosition(true)));
    }

    @Override
    public void execute() {
        double distanceFromTarget = Units.degreesToRotations(e_Elevator.getElevatorPosition(true)) - angleSetPoint;
        e_Elevator.setElevatorMotorSpeed(elevatorPIDController.calculate(Units.degreesToRotations(e_Elevator.getElevatorPosition(true)), angleSetPoint));
        System.out.println(elevatorPIDController.calculate(Units.degreesToRotations(e_Elevator.getElevatorPosition(true)), angleSetPoint) + " " + distanceFromTarget + " " + elevatorPIDController.getPositionError());
        // PID Controller returns [-1, 1] output while FeedForward Controller returns a voltage
    }

    @Override
    public boolean isFinished() {
        return elevatorPIDController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        e_Elevator.brakeElevator();
    }
}
