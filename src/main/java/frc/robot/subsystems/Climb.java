package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climb extends SubsystemBase {
    private TalonFX climbMotor = new TalonFX(Constants.ClimberConstants.climbMotorID, Constants.Swerve.canivoreName);

    public Climb() {
        climbMotor.getConfigurator().apply(Robot.ctreConfigs.climberConfig);
        climbMotor.setPosition(Constants.ClimberConstants.climberStartingPosition);
        System.out.println("Climb subsystem loaded!");
    }

    public void setClimberSpeed(double newSpeed) {
        if (checkClimberMovement(newSpeed)) {
            climbMotor.setVoltage(Constants.ClimberConstants.maxClimberVoltage * newSpeed);
        } else {
            brakeClimber();
        }
    }

    public void brakeClimber() {
        climbMotor.setVoltage(0);
    }

    public void setClimberPosition(double positionInRotations) {
        climbMotor.setPosition(positionInRotations);
    }

    public double getClimberPosition() { // Units are in rotations
        return climbMotor.getPosition().getValueAsDouble();
    }

    public boolean checkClimberMovement(double newSpeed) {
        if (getClimberPosition() < Constants.ClimberConstants.climberLowerBound) {
            if (newSpeed >= 0) {
                return true;
            } else {
                return false;
            }
        } else if (getClimberPosition() > Constants.ClimberConstants.climberUpperBound) {
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
