package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class Elevator extends SubsystemBase {
    private TalonFX elevatorMotorOne = new TalonFX(Constants.Elevator.elevatorMotorOneID);
    private TalonFX elevatorMotorTwo = new TalonFX(Constants.Elevator.elevatorMotorTwoID); 

    public Elevator() {
        elevatorMotorOne.getConfigurator().apply(Robot.ctreConfigs.elevatorConfig);
        elevatorMotorTwo.getConfigurator().apply(Robot.ctreConfigs.elevatorConfig);
        setElevatorPosition(Units.rotationsToDegrees(Constants.Elevator.elevatorStartingHeightInRotations));
    }

    public void setElevatorMotorSpeed(double newSpeed) {
        if (checkElevatorMovement(newSpeed)) {
            elevatorMotorOne.setVoltage(2 * newSpeed + Constants.Elevator.gravitationalOffsetVoltage);
            elevatorMotorTwo.setVoltage(2 * newSpeed + Constants.Elevator.gravitationalOffsetVoltage);
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

    public double getElevatorPosition(boolean physicalPosition) { // Return value in degrees
        double motorOnePosition = Units.rotationsToDegrees(elevatorMotorOne.getPosition().getValueAsDouble());
        double motorTwoPosition = Units.rotationsToDegrees(elevatorMotorTwo.getPosition().getValueAsDouble());
        double averageMotorPosition = (motorOnePosition + motorTwoPosition) / 2;
        if (physicalPosition) {
            return averageMotorPosition / Constants.Elevator.elevatorGearRatio;
        } else {
            return averageMotorPosition;
        }
    }

    public boolean checkElevatorMovement(double newSpeed) {
        if (getElevatorPosition(false) <= Constants.Elevator.elevatorLowerBound) {
            if (newSpeed >= 0) {
                return true;
            } else {
                return false;
            }
        } else if (getElevatorPosition(false) >= Constants.Elevator.elevatorUpperBound) {
            if (newSpeed < 0) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }
}
