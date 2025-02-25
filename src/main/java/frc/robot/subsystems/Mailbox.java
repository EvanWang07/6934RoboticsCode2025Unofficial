package frc.robot.subsystems;

import frc.robot.Constants.MailboxConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Mailbox extends SubsystemBase {
    private final SparkFlex intakeNeoVortex;
    private final DigitalInput coralSensor;
    private final Timer coralDetectionTimer;

    public Mailbox() {
        intakeNeoVortex = new SparkFlex(MailboxConstants.MailboxMotorID, MotorType.kBrushless);
        coralSensor = new DigitalInput(MailboxConstants.beamBreakerChannel);
        coralDetectionTimer = new Timer();
        coralDetectionTimer.start();
    }

    public void setMailboxSpeed(double voltage) {
        intakeNeoVortex.setVoltage(voltage);
    }

    public void brakeMailbox() {
        intakeNeoVortex.set(0);
    }

    
    public boolean coralIsDetected() {
        return !coralSensor.get();
    }

    public boolean canScoreCoral() {
        if (coralIsDetected()) {
            coralDetectionTimer.reset();
        }
        return (coralDetectionTimer.get() < MailboxConstants.scoringLeewayTime); // Gives lee-way time for the end-effector to score
    }
    
}
