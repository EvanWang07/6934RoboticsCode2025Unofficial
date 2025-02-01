package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mailbox;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
        m_mailboxSubsystem.setMailboxSpeed();

        SmartDashboard.putBoolean("Coral in Intake: ", m_mailboxSubsystem.coralIsDetected());
    }

    @Override
    public boolean isFinished() {
        if (directionIsIntake) {
            if (m_mailboxSubsystem.coralIsDetected()) {
                return true;
            } else {
                return false;
            }
        } else {
            if (!m_mailboxSubsystem.coralIsDetected()) { 
                Commands.waitSeconds(3); 
                /* As the coral is being spat out, the sensor will stop detecting the coral
                 * This wait ensures the motors will still spin as the coral leaves
                 */
                return true;
            } else {
                return false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_mailboxSubsystem.brakeMailbox();
    }
}
