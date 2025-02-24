package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public final class Constants {

    public static final class QuickTuning {
        /* Controller Constants */
        public static final int driveControllerID = 0;
        public static final int weaponControllerID = 1;

        public static final double driveStickDeadband = 0.1;
        public static final double weaponStickDeadband = 0.1;

        public static final double driveSlowModeMultiplier = 0.1;
    }

    public static final class Vision {
        /* Limelight Configs & Location */
        public static final String limelightName = "limelight-scorps";
        public static final double limelightAngle = 10; // Angle between the ground and the limelight's orientation
        public static final double limelightHeight = Units.inchesToMeters(11.194); // Height the limelight is above the ground (in)

        /* Targetting Threshold */
        public static final int targetDetectionListSize = 10; // Amount of trials the list holds
        public static final double averageTVThreshold = 0.7; // Required targetting success rate for automatic alignment

        /* Finding Targets */
        public static final double targetSearchOutput = 0.25;

        /* Vision Alignment PID Constants */
        public static final double TXkP = 0.15;
        public static final double TXkI = 0.0;
        public static final double TXkD = 0.0;
        public static final double TXMaxSpeed = 1.0;
        public static final double TXMaxAcceleration = 0.5;

        public static final double TYkP = 0.25;
        public static final double TYkI = 0.0;
        public static final double TYkD = 0.0;
        public static final double TYMaxSpeed = 1.0;
        public static final double TYMaxAcceleration = 0.5;

        public static final double posekP = 0.15;
        public static final double posekI = 0.0;
        public static final double posekD = 0.0;
        public static final double poseMaxSpeed = 0.5;
        public static final double poseMaxAcceleration = 0.5;

        /* Alignment Error Tolerances */
        public static final double TXTolerance = 1; // Degrees
        public static final double TYTolerance = 1; // Degrees
        public static final double poseTolerance = 1; // Degrees

        /* Important April Tag Locations */
        public static final double reefAprilTagHeights = Units.inchesToMeters(12.13);
        public static final Pose2d redReefSix = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(300)));
        public static final Pose2d redReefSeven = new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(0)));
        public static final Pose2d redReefEight = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(60)));
        public static final Pose2d redReefNine = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(120)));
        public static final Pose2d redReefTen = new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d redReefEleven = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(240)));
        public static final Pose2d blueReefSeventeen = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(240)));
        public static final Pose2d blueReefEighteen = new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d blueReefNineteen = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(120)));
        public static final Pose2d blueReefTwenty = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(60)));
        public static final Pose2d blueReefTwentyone = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(0)));
        public static final Pose2d blueReefTwentytwo = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(300)));
    }

    public static final class Elevator {
        /* Elevator Mechanism Details */
        public static final double elevatorGearRatio = 6;
        public static final double cascadeElevatorRotationToDistanceMultiplier = 2; // Cascading elevators innately travel more distance per rotation than simply r(theta) by a constant factor
        public static final double elevatorGearRadius = Units.inchesToMeters(1.757 / 2);
        public static final double elevatorMetersToRotations = Units.radiansToRotations(1 / (elevatorGearRadius * cascadeElevatorRotationToDistanceMultiplier)); // Meters-to-rotations conversion ratio for the cascading elevator

        /* Elevator Bounds, Starting Position, & Bounds Tolerance */
        public static final double elevatorStartingHeightInRotations = 0; // Do NOT consider gear ratio here
        public static final double minimumElevatorHeightInRotations = 0; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER
        public static final double maxElevatorHeightInRotations = 3.625; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER
        public static final double elevatorMotorBoundsToleranceInRotations = 0.025; // Do NOT consider gear ratio here

        public static final double elevatorLowerBound = Units.rotationsToDegrees((minimumElevatorHeightInRotations + elevatorMotorBoundsToleranceInRotations) * elevatorGearRatio); // Angular position of the lower bound of elevator downward movement
        public static final double elevatorUpperBound = Units.rotationsToDegrees((maxElevatorHeightInRotations - elevatorMotorBoundsToleranceInRotations) * elevatorGearRatio); // Angular position of the upper bound of elevator upper movement

        /* Elevator Motor Configs */
        public static final int elevatorMotorOneID = 9;
        public static final int elevatorMotorTwoID = 10;

        public static final InvertedValue elevatorMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue elevatorMotorNeutralMode = NeutralModeValue.Brake;

        /* Target Elevator Heights (Robot Reaches L1, L2, and L3) (RELATIVE TO STARTING HEIGHT!!!) */
        public static final double intakeHeightInRotations = 0.2; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER
        public static final double levelOneHeightInRotations = 1.75; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER
        public static final double levelTwoHeightInRotations = 2.5; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER
        public static final double levelThreeHeightInRotations = 3.6; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER

        /* Elevator Feedforward & PID Tuning Constants */
        public static final double gravitationalOffsetVoltage = 0.27; // Offset feedforward voltage for gravity (currently a placeholder value)
        public static final double kS = 0; // Offset feedforward constant for static friction (currently a placeholder value)
        public static final double kV = 0; // Offset feedforward constant for kinetic friction (currently a placeholder value)
        public static final double kP = 2.0; // Proportional feedback constant (currently a placeholder value)
        public static final double kI = 0; // Integral feedback constant (currently a placeholder value)
        public static final double kD = 0; // Derivative feedback constant (currently a placeholder value)

        public static final double PIDMaxSpeed = 1.0; // Maximum speed the automatic elevator controller can move the elevator at (m / s)
        public static final double PIDMaxAcceleration = 1.0; // Maximum acceleration the automatic elevator controller can move the elevator at (m / s^2)

        public static final double PIDMaxSpeedInRotations = PIDMaxSpeed * elevatorMetersToRotations;
        public static final double PIDMaxAccelerationInRotations = PIDMaxAcceleration * elevatorMetersToRotations;

        public static final double PIDTolerance = Units.inchesToMeters(0.5); // Error tolerance for PID elevator (in)

        public static final double PIDToleranceInRotations = PIDTolerance * elevatorMetersToRotations;
    }

    public static final class MailboxConstants {
        /* Mailbox Motor Configs */
        public static final int MailboxMotorID = 62;
    
        /* Intake & Scoring */
        public static final double intakeVoltage = 3;
        public static final double scoringVoltage = 2;
    
        public static final double scoringLeewayTime = 0.65; // In seconds
    
        /* Beam Breaker Configs */
        public static final int beamBreakerChannel = 9;
      }

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final String canivoreName = "Second CANivor<3";

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double robotSideLength = Units.inchesToMeters(28);
        public static final double trackWidth = Units.inchesToMeters(22.75); 
        public static final double wheelBase = Units.inchesToMeters(22.875); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // Originally 0.12
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: Check later
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5;
        /** Radians per Second */
        public static final double maxAngularVelocity =  3 * Math.PI;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-47.37312 + 180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-39.63852);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-128.320 + 180); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(108.54504); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double maxModuleSpeed = 4.5; // Max module speed, in m/s
        public static final double driveBaseRadius = (Swerve.robotSideLength / 2) * Math.sqrt(2);
        public static final PPHolonomicDriveController pathPlannerConfig = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0)
        );
    }
}
