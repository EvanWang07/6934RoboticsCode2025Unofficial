package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BasicOperations;
import frc.robot.Constants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
    private TalonFX elevatorMotorOne = new TalonFX(Constants.Elevator.elevatorMotorOneID);
    private TalonFX elevatorMotorTwo = new TalonFX(Constants.Elevator.elevatorMotorTwoID);    

    public Elevator() {
        elevatorMotorOne.getConfigurator().apply(Robot.ctreConfigs.elevatorConfig);
        elevatorMotorTwo.getConfigurator().apply(Robot.ctreConfigs.elevatorConfig);
        setElevatorPosition(0);
    }

    public void setElevatorMotorSpeed(double newSpeed) {
        if (checkElevatorMovement(newSpeed)) {
            elevatorMotorOne.set(newSpeed);
            elevatorMotorTwo.set(newSpeed);
        } else {
            brakeElevator();
        }
    }

    public void brakeElevator() {
        elevatorMotorOne.setVoltage(0);
        elevatorMotorTwo.setVoltage(0);
    }

    public void setElevatorPosition(double newPosition) { // newPosition is in degrees
        elevatorMotorOne.getConfigurator().setPosition(Units.degreesToRotations(newPosition));
    }

    public double getElevatorPosition() {
        double motorOnePosition = Units.rotationsToDegrees(elevatorMotorOne.getPosition().getValueAsDouble());
        double motorTwoPosition = Units.rotationsToDegrees(elevatorMotorTwo.getPosition().getValueAsDouble());
        return BasicOperations.findAverage(motorOnePosition, motorTwoPosition);
    }

    public boolean checkElevatorMovement(double newSpeed) {
        if (getElevatorPosition() <= Constants.Elevator.elevatorLowerBound) {
            if (newSpeed >= 0) {
                return true;
            } else {
                return false;
            }
        } else if (getElevatorPosition() >= Constants.Elevator.elevatorUpperBound) {
            if (newSpeed <= 0) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
}
