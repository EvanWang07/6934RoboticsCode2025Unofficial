package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.subsystems.Mailbox;

import edu.wpi.first.wpilibj2.command.Command;

public class TeleopMailbox extends Command {
    private final Mailbox c_Mailbox;
    private final DoubleSupplier speedSup;

    public TeleopMailbox(Mailbox c_Mailbox, DoubleSupplier speedSup) {
        this.c_Mailbox = c_Mailbox;
        addRequirements(c_Mailbox);

        this.speedSup = speedSup;
    }

    @Override
    public void execute() {
        double mailboxSpeed = speedSup.getAsDouble();
        c_Mailbox.setMailboxSpeed(mailboxSpeed * Constants.MailboxConstants.scoringVoltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        c_Mailbox.brakeMailbox();
    }
    
}
