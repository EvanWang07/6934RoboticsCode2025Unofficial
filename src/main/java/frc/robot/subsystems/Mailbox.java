package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.MailboxConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;

public class Mailbox extends SubsystemBase{
    private final SparkMax intakeNeoVortex;


    public Mailbox(){
        intakeNeoVortex = new SparkMax(MailboxConstants.MailboxMotorID, MotorType.kBrushless);


    }

    public void setMailboxSpeed(){
        intakeNeoVortex.setVoltage(MailboxConstants.MailboxMotorMaxVoltage);
    }

    public void brakeMailbox(){
        intakeNeoVortex.set(0);
    }



}
