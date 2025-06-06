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

        /* Robot Starting Position */
        public static final Pose2d selectedStartingPose = GameField.topBlueBargeStartingPose;
    }

    public static final class Vision {
        /* Limelight Configs & Location */
        public static final String limelightName = "limelight-scorps";
        public static final double limelightAngle = 10; // Angle between the ground and the limelight's orientation
        public static final double limelightHeight = Units.inchesToMeters(11.194); // Height the limelight is above the ground (in)

        /* Targetting Threshold */
        public static final int targetDetectionListSize = 10; // Amount of trials the list holds
        public static final double averageTVThreshold = 0.7; // Required targetting success rate for automatic alignment

        /* Vision Alignment PID Constants */
        public static final double TXkP = 0.15;
        public static final double TXkI = 0.0;
        public static final double TXkD = 0.0;
        public static final double TXMaxSpeed = 1.0;
        public static final double TXMaxAcceleration = 0.5;

        /* Alignment Error Tolerances */
        public static final double TXTolerance = 1; // Degrees
        public static final double TYTolerance = 1; // Degrees
        public static final double poseTolerance = 1; // Degrees
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
        public static final double intakeHeightInRotations = 0.35; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER
        public static final double levelOneHeightInRotations = 1.8; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER
        public static final double levelTwoHeightInRotations = 2.4; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER
        public static final double levelThreeHeightInRotations = 3.45; // Do NOT consider gear ratio here; ALREADY CONSIDERS CASCADE ELEVATOR ROTATION-TO-DISTANCE MULTIPLIER

        /* Elevator Feedforward & PID Tuning Constants */
        public static final double gravitationalOffsetVoltage = 0.27; // Offset feedforward voltage for gravity (currently a placeholder value)
        public static final double kS = 0; // Offset feedforward constant for static friction (currently a placeholder value)
        public static final double kV = 0; // Offset feedforward constant for kinetic friction (currently a placeholder value)
        public static final double kP = 3; // Proportional feedback constant (currently a placeholder value)
        public static final double kI = 0; // Integral feedback constant (currently a placeholder value)
        public static final double kD = 0; // Derivative feedback constant (currently a placeholder value)

        public static final double PIDMaxSpeed = 3.25; // Maximum speed the automatic elevator controller can move the elevator at (m / s)
        public static final double PIDMaxAcceleration = 3.25; // Maximum acceleration the automatic elevator controller can move the elevator at (m / s^2)

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
        public static final double scoringVoltage = 2.5;
    
        public static final double scoringLeewayTime = 0.65; // In seconds
    
        /* Beam Breaker Configs */
        public static final int beamBreakerChannel = 9;
    }

    public static final class ClimberConstants {
        /* Climber Motor Configs */
        public static final int climbMotorID = 11;
        public static final InvertedValue climbMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue climbMotorNeutralMode = NeutralModeValue.Brake;

        /* Climbing */
        public static final double maxClimberVoltage = 8;
    }
    
    public static final class GameField {
        /* Robot Starting Poses (Pose Estimation) (USES BLUE ALLIANCE COORDINATES!!!) */
        public static final Pose2d topBlueBargeStartingPose = new Pose2d(8, 7.56, Rotation2d.fromDegrees(180));
        public static final Pose2d bottomBlueBargeStartingPose = new Pose2d(8, 4.8, Rotation2d.fromDegrees(180));
        public static final Pose2d topRedBargeStartingPose = new Pose2d(8, 4.25, Rotation2d.fromDegrees(180));
        public static final Pose2d bottomRedBargeStartingPose = new Pose2d(8, 0.5, Rotation2d.fromDegrees(180));

        /* Robot Space Offset (General) */
        public static final double robotOffsetMeters = 0.863 / 2; // Half the robot side length WITH bumpers
        public static final double additionalOffsetMeters = 0.025; // Additional distance from the wall

        public static final double leftBranchOffsetMeters = Units.inchesToMeters(2.5); // Direction: RIGHT (Robot-Perspective)
        public static final double rightBranchOffsetMeters = Units.inchesToMeters(15.5); // Direction: RIGHT (Robot-Perspective)
        public static final double algaeOffsetMeters = Units.inchesToMeters(12); // Direction: RIGHT (Robot-Perspective)

        public static final double leftStationOffsetMeters = Units.inchesToMeters(24); // Direction: LEFT (Robot-Perspective)
        public static final double rightStationOffsetMeters = Units.inchesToMeters(24); // Direction: RIGHT (Robot-Perspective)
        
        public static final double totalRobotOffsetMeters = robotOffsetMeters + additionalOffsetMeters; // TOTAL offset from the wall

        /* Robot Angle Offset (Reef) */
        public static final double leftMovementFieldAngle = Units.degreesToRadians(-90); // Robot angular direction of movement from forward when moving left (FACING THE REEF)
        public static final double rightMovementFieldAngle = Units.degreesToRadians(90); // Robot angular direction of movement from forward when moving right (FACING THE REEF)

        /* Field Dimensions (Reefscape) */
        public static final double fieldSizeX = Units.feetToMeters(57.573);
        public static double fieldSizeY = Units.feetToMeters(26.417);

        /* April Tag Locations (Reef) */
        public static final double reefAprilTagHeights = Units.inchesToMeters(12.13);

        public static final Pose2d redReefTagSix = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(300)));
        public static final Pose2d redReefTagSeven = new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(0)));
        public static final Pose2d redReefTagEight = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(60)));
        public static final Pose2d redReefTagNine = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(120)));
        public static final Pose2d redReefTagTen = new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d redReefTagEleven = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(240)));
        public static final Pose2d blueReefTagSeventeen = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(240)));
        public static final Pose2d blueReefTagEighteen = new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d blueReefTagNineteen = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(120)));
        public static final Pose2d blueReefTagTwenty = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), new Rotation2d(Units.degreesToRadians(60)));
        public static final Pose2d blueReefTagTwentyone = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), new Rotation2d(Units.degreesToRadians(0)));
        public static final Pose2d blueReefTagTwentytwo = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), new Rotation2d(Units.degreesToRadians(300)));

        /* Goal Robot Locations (CENTER Reef) */
        public static final Pose2d redReefRobotCenterSix = BasicOperations.findTranslatedPoseCenter(redReefTagSix, totalRobotOffsetMeters, 180);
        public static final Pose2d redReefRobotCenterSeven = BasicOperations.findTranslatedPoseCenter(redReefTagSeven, totalRobotOffsetMeters, 180);
        public static final Pose2d redReefRobotCenterEight = BasicOperations.findTranslatedPoseCenter(redReefTagEight, totalRobotOffsetMeters, 180);
        public static final Pose2d redReefRobotCenterNine = BasicOperations.findTranslatedPoseCenter(redReefTagNine, totalRobotOffsetMeters, 180);
        public static final Pose2d redReefRobotCenterTen = BasicOperations.findTranslatedPoseCenter(redReefTagTen, totalRobotOffsetMeters, 180);
        public static final Pose2d redReefRobotCenterEleven = BasicOperations.findTranslatedPoseCenter(redReefTagEleven, totalRobotOffsetMeters, 180);
        public static final Pose2d blueReefRobotCenterSeventeen = BasicOperations.findTranslatedPoseCenter(blueReefTagSeventeen, totalRobotOffsetMeters, 180);
        public static final Pose2d blueReefRobotCenterEighteen = BasicOperations.findTranslatedPoseCenter(blueReefTagEighteen, totalRobotOffsetMeters, 180);
        public static final Pose2d blueReefRobotCenterNineteen = BasicOperations.findTranslatedPoseCenter(blueReefTagNineteen, totalRobotOffsetMeters, 180);
        public static final Pose2d blueReefRobotCenterTwenty = BasicOperations.findTranslatedPoseCenter(blueReefTagTwenty, totalRobotOffsetMeters, 180);
        public static final Pose2d blueReefRobotCenterTwentyone = BasicOperations.findTranslatedPoseCenter(blueReefTagTwentyone, totalRobotOffsetMeters, 180);
        public static final Pose2d blueReefRobotCenterTwentytwo = BasicOperations.findTranslatedPoseCenter(blueReefTagTwentytwo, totalRobotOffsetMeters, 180);

        /* Goal Robot Locations (LEFT Reef Branches) */
        public static final Pose2d redReefRobotLeftSix = BasicOperations.findTranslatedPoseRight(redReefRobotCenterSix, leftBranchOffsetMeters, true);
        public static final Pose2d redReefRobotLeftSeven = BasicOperations.findTranslatedPoseRight(redReefRobotCenterSeven, leftBranchOffsetMeters, true);
        public static final Pose2d redReefRobotLeftEight = BasicOperations.findTranslatedPoseRight(redReefRobotCenterEight, leftBranchOffsetMeters, true);
        public static final Pose2d redReefRobotLeftNine = BasicOperations.findTranslatedPoseRight(redReefRobotCenterNine, leftBranchOffsetMeters, true);
        public static final Pose2d redReefRobotLeftTen = BasicOperations.findTranslatedPoseRight(redReefRobotCenterTen, leftBranchOffsetMeters, true);
        public static final Pose2d redReefRobotLeftEleven = BasicOperations.findTranslatedPoseRight(redReefRobotCenterEleven, leftBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotLeftSeventeen = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterSeventeen, leftBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotLeftEighteen = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterEighteen, leftBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotLeftNineteen = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterNineteen, leftBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotLeftTwenty = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterTwenty, leftBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotLeftTwentyone = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterTwentyone, leftBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotLeftTwentytwo = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterTwentytwo, leftBranchOffsetMeters, true);

        /* Goal Robot Locations (RIGHT Reef Branches) */
        public static final Pose2d redReefRobotRightSix = BasicOperations.findTranslatedPoseRight(redReefRobotCenterSix, rightBranchOffsetMeters, true);
        public static final Pose2d redReefRobotRightSeven = BasicOperations.findTranslatedPoseRight(redReefRobotCenterSeven, rightBranchOffsetMeters, true);
        public static final Pose2d redReefRobotRightEight = BasicOperations.findTranslatedPoseRight(redReefRobotCenterEight, rightBranchOffsetMeters, true);
        public static final Pose2d redReefRobotRightNine = BasicOperations.findTranslatedPoseRight(redReefRobotCenterNine, rightBranchOffsetMeters, true);
        public static final Pose2d redReefRobotRightTen = BasicOperations.findTranslatedPoseRight(redReefRobotCenterTen, rightBranchOffsetMeters, true);
        public static final Pose2d redReefRobotRightEleven = BasicOperations.findTranslatedPoseRight(redReefRobotCenterEleven, rightBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotRightSeventeen = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterSeventeen, rightBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotRightEighteen = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterEighteen, rightBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotRightNineteen = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterNineteen, rightBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotRightTwenty = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterTwenty, rightBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotRightTwentyone = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterTwentyone, rightBranchOffsetMeters, true);
        public static final Pose2d blueReefRobotRightTwentytwo = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterTwentytwo, rightBranchOffsetMeters, true);

        /* Goal Robot Locations (ALGAE) */
        public static final Pose2d redReefRobotAlgaeSix = BasicOperations.findTranslatedPoseRight(redReefRobotCenterSix, algaeOffsetMeters, true);
        public static final Pose2d redReefRobotAlgaeSeven = BasicOperations.findTranslatedPoseRight(redReefRobotCenterSeven, algaeOffsetMeters, true);
        public static final Pose2d redReefRobotAlgaeEight = BasicOperations.findTranslatedPoseRight(redReefRobotCenterEight, algaeOffsetMeters, true);
        public static final Pose2d redReefRobotAlgaeNine = BasicOperations.findTranslatedPoseRight(redReefRobotCenterNine, algaeOffsetMeters, true);
        public static final Pose2d redReefRobotAlgaeTen = BasicOperations.findTranslatedPoseRight(redReefRobotCenterTen, algaeOffsetMeters, true);
        public static final Pose2d redReefRobotAlgaeEleven = BasicOperations.findTranslatedPoseRight(redReefRobotCenterEleven, algaeOffsetMeters, true);
        public static final Pose2d blueReefRobotAlgaeSeventeen = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterSeventeen, algaeOffsetMeters, true);
        public static final Pose2d blueReefRobotAlgaeEighteen = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterEighteen, algaeOffsetMeters, true);
        public static final Pose2d blueReefRobotAlgaeNineteen = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterNineteen, algaeOffsetMeters, true);
        public static final Pose2d blueReefRobotAlgaeTwenty = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterTwenty, algaeOffsetMeters, true);
        public static final Pose2d blueReefRobotAlgaeTwentyone = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterTwentyone, algaeOffsetMeters, true);
        public static final Pose2d blueReefRobotAlgaeTwentytwo = BasicOperations.findTranslatedPoseRight(blueReefRobotCenterTwentytwo, algaeOffsetMeters, true);

        /* April Tag Locations (Coral Station) */
        public static final double stationAprilTagHeights = Units.inchesToMeters(58.5);

        public static final Pose2d redStationTagOne = new Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), new Rotation2d(Units.degreesToRadians(126)));
        public static final Pose2d redStationTagTwo = new Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20), new Rotation2d(Units.degreesToRadians(234)));
        public static final Pose2d blueStationTagTwelve = new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), new Rotation2d(Units.degreesToRadians(54)));
        public static final Pose2d blueStationTagThirteen = new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), new Rotation2d(Units.degreesToRadians(306)));

        /* Goal Robot Locations (CENTER Coral Station) */
        public static final Pose2d redStationRobotCenterOne = BasicOperations.findTranslatedPoseCenter(redStationTagOne, totalRobotOffsetMeters, 0);
        public static final Pose2d redStationRobotCenterTwo = BasicOperations.findTranslatedPoseCenter(redStationTagTwo, totalRobotOffsetMeters, 0);
        public static final Pose2d blueStationRobotCenterTwelve = BasicOperations.findTranslatedPoseCenter(blueStationTagTwelve, totalRobotOffsetMeters, 0);
        public static final Pose2d blueStationRobotCenterThirteen = BasicOperations.findTranslatedPoseCenter(blueStationTagThirteen, totalRobotOffsetMeters, 0);

        /* Goal Robot PATHFIND Locations (Coral Station) (USES BLUE ALLIANCE COORDINATES!!!) */
        public static final Pose2d robotNearLeftStation = BasicOperations.findTranslatedPoseCenter(blueStationRobotCenterThirteen, 0, 0);
        public static final Pose2d robotNearRightStation = BasicOperations.findTranslatedPoseCenter(blueStationRobotCenterTwelve, 0, 0);

        /* Goal Robot Locations (LEFT Coral Station) */
        public static final Pose2d redStationRobotLeftOne = BasicOperations.findTranslatedPoseLeft(redStationRobotCenterOne, leftStationOffsetMeters, false);
        public static final Pose2d redStationRobotLeftTwo = BasicOperations.findTranslatedPoseLeft(redStationRobotCenterTwo, leftStationOffsetMeters, false);
        public static final Pose2d blueStationRobotLeftTwelve = BasicOperations.findTranslatedPoseLeft(blueStationRobotCenterTwelve, leftStationOffsetMeters, false);
        public static final Pose2d blueStationRobotLeftThirteen = BasicOperations.findTranslatedPoseLeft(blueStationRobotCenterThirteen, leftStationOffsetMeters, false);

        /* Goal Robot Locations (RIGHT Coral Station) */
        public static final Pose2d redStationRobotRightOne = BasicOperations.findTranslatedPoseLeft(redStationRobotCenterOne, rightStationOffsetMeters, false);
        public static final Pose2d redStationRobotRightTwo = BasicOperations.findTranslatedPoseLeft(redStationRobotCenterTwo, rightStationOffsetMeters, false);
        public static final Pose2d blueStationRobotRightTwelve = BasicOperations.findTranslatedPoseLeft(blueStationRobotCenterTwelve, rightStationOffsetMeters, false);
        public static final Pose2d blueStationRobotRightThirteen = BasicOperations.findTranslatedPoseLeft(blueStationRobotCenterThirteen, rightStationOffsetMeters, false);
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
