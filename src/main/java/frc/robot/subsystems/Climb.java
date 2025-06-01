package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climb extends SubsystemBase {
    private TalonFX climbMotor = new TalonFX(Constants.ClimberConstants.climbMotorID);

    public Climb() {
        climbMotor.getConfigurator().apply(Robot.ctreConfigs.climberConfig);
        System.out.println("Climb subsystem loaded!");
    }

    public void setClimberSpeed(double newSpeed) {
        climbMotor.setVoltage(Constants.ClimberConstants.maxClimberVoltage * newSpeed);
    }

    public void brakeClimber() {
        climbMotor.setVoltage(0);
    }
}
