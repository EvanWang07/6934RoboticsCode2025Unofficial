package frc.robot.subsystems;

import frc.robot.Constants.MailboxConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Mailbox extends SubsystemBase {
    private final SparkMax intakeNeoVortex;
    private final DigitalInput coralSensor;

    public Mailbox() {
        intakeNeoVortex = new SparkMax(MailboxConstants.MailboxMotorID, MotorType.kBrushless);
        coralSensor = new DigitalInput(MailboxConstants.beamBreakerChannel);
    }

    public void setMailboxSpeed() {
        intakeNeoVortex.setVoltage(MailboxConstants.MailboxMotorMaxVoltage);
    }

    public void brakeMailbox() {
        intakeNeoVortex.set(0);
    }

    public boolean coralIsDetected() {
        return !coralSensor.get();
    }
}
