package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private TalonFX elevatorMotorOne = new TalonFX(30);
    private TalonFX elevatorMotorTwo = new TalonFX(31);    

    public Elevator() {

    }

    public void setElevatorMotorSpeed (double newSpeed) {
        elevatorMotorOne.set(newSpeed);
        elevatorMotorTwo.set(newSpeed);
    }
}
