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
        elevatorMotorOne.getConfigurator().setPosition(0);
        elevatorMotorTwo.getConfigurator().setPosition(0);
    }

    public void setElevatorMotorSpeed(double newSpeed) {
        if (checkMotorBounds(newSpeed)) {
            elevatorMotorOne.set(newSpeed);
            elevatorMotorTwo.set(newSpeed);
        } else {
            brakeElevator();
        }
    }

    public void setElevatorPosition(double newPosition) { // newPosition is in degrees
        elevatorMotorOne.getConfigurator().setPosition(Units.degreesToRotations(newPosition));
    }

    public void brakeElevator() {
        elevatorMotorOne.setVoltage(0);
        elevatorMotorTwo.setVoltage(0);
    }

    public boolean checkMotorBounds(double newSpeed) {
        if (Units.rotationsToDegrees(elevatorMotorOne.getPosition().getValueAsDouble()) <= Constants.Elevator.elevatorLowerBound) {
            if (newSpeed >= 0) {
                return true;
            } else {
                return false;
            }
        } else if (Units.rotationsToDegrees(elevatorMotorOne.getPosition().getValueAsDouble()) >= Constants.Elevator.elevatorUpperBound) {
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
