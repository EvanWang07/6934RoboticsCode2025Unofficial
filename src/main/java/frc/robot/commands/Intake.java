package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Mailbox;


public class Intake extends Command {
    private final Mailbox m_mailboxSubsystem;
    private final boolean directionIsIntake;

    public Intake(Mailbox m_mailboxSubsystem, boolean directionIsIntake) {
        this.m_mailboxSubsystem = m_mailboxSubsystem;
        addRequirements(m_mailboxSubsystem);

        this.directionIsIntake = directionIsIntake;
    }

    @Override
    public void execute() {
        if (directionIsIntake) { // Intake
            m_mailboxSubsystem.setMailboxSpeed(Constants.MailboxConstants.intakeVoltage);
        } else { // Score
            m_mailboxSubsystem.setMailboxSpeed(Constants.MailboxConstants.scoringVoltage);
        }
    }

    @Override
    public boolean isFinished() {
        if (directionIsIntake) { // Intake
            if (m_mailboxSubsystem.coralIsDetected()) {
                return true;
            } else {
                return false;
            }
        } else { // Score
            if (m_mailboxSubsystem.canScoreCoral()) { 
                return false;
            } else {
                return true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_mailboxSubsystem.brakeMailbox();
    }
}
