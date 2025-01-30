package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.Mailbox;
import frc.robot.Constants.MailboxConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends Command {
    private final Mailbox m_mailboxSubsystem;
    private final DigitalInput beamBreaker;
    private Timer timer;

    private final boolean directionIsIntake;
    //private final CANrange m_CANrange;

    //private final CANrangeConfiguration canrangeConfiguration;

    //private final ProximityParamsConfigs proximityThreshold;

    public Intake (Mailbox m_mailboxSubsystem, boolean directionIsIntake){
        // m_CANrange = new CANrange(ElevatorAndMailboxConstants.CANrangeID);
        //canrangeConfiguration = new CANrangeConfiguration();
        //proximityThreshold = new ProximityParamsConfigs();

        this.m_mailboxSubsystem = m_mailboxSubsystem;
        beamBreaker = new DigitalInput(MailboxConstants.beamBreakerChannel);

        timer = new Timer();

        this.directionIsIntake = directionIsIntake;
        //proximityThreshold.withProximityThreshold(0.075);

        //canrangeConfiguration.withProximityParams(proximityThreshold);


        //m_CANrange.getConfigurator().apply(canrangeConfiguration);

    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        SmartDashboard.putBoolean("Mailbox sensor status (false = coral sensed)", beamBreaker.get());

    }

    @Override
    public void execute(){
        m_mailboxSubsystem.setMailboxSpeed();
        SmartDashboard.putBoolean("Mailbox sensor status (false = coral sensed)", beamBreaker.get());
    }

    @Override
    public void end(boolean interrupted){
        m_mailboxSubsystem.brakeMailbox();
        SmartDashboard.putBoolean("Mailbox sensor status (false = coral sensed)", beamBreaker.get());

    }

    @Override
    public boolean isFinished(){
        if(directionIsIntake){

            if((beamBreaker.get() == false) || (timer.get() > 10.0)){
                return true;
            } else {
                return false;
            }
        } else {
            if((beamBreaker.get() == true) || (timer.get() > 10.0)){
                Commands.waitSeconds(5);
                return true;
            } else {
                return false;
            }
        }
    }


}
