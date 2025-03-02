package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Vision;
import frc.robot.Constants.GameField;

public final class VisionInfo {
    private static boolean[] targetValidResults = new boolean[Vision.targetDetectionListSize];
    private static double[] cameraToTargetDistances = new double[Vision.distanceListSize];

    public static int getTargetID() { // Gets the target ID
        return (int) LimelightHelpers.getFiducialID(Vision.limelightName);
    }

    public static double getTX(boolean asOutput) { // Gets the horizontal angle error
        if (asOutput) {
            return (LimelightHelpers.getTX(Vision.limelightName) / 31);
        } else {
            return LimelightHelpers.getTX(Vision.limelightName);
        }
    }

    public static double getTY(boolean asOutput) { // Gets the vertical angle error
        if (asOutput) {
            return (LimelightHelpers.getTY(Vision.limelightName) / 31);
        } else { 
            return LimelightHelpers.getTY(Vision.limelightName);
        }
    }

    public static double getPoseTheta() { // Gets the yaw of the limelight relative to the robot
        double[] robotPose = LimelightHelpers.getBotPose_TargetSpace(Vision.limelightName);
        return robotPose[4];
    }

    public static double getTA(boolean asOutput) { // Gets the % of the camera frame the target takes up (NOT USED)
        if (asOutput) {
            return (LimelightHelpers.getTA(Vision.limelightName) / 100);
        } else {
            return LimelightHelpers.getTA(Vision.limelightName);
        }
    }

    public static boolean hasValidTargets() { // Determines if there is a valid limelight target at the given time
        return LimelightHelpers.getTV(Vision.limelightName);
    }

    public static boolean willTarget() { // Determines (by averaging TV values) if the robot will seek a target
        BasicOperations.insertBooleanToConfinedList(targetValidResults, hasValidTargets());
        return (BasicOperations.getSuccessRate(targetValidResults) >= Vision.averageTVThreshold);
    }

    public static void updateDashboardValues() { // Sends limelight values to SmartDashboard
        SmartDashboard.putBoolean("Targetable Detected: ", willTarget());
        SmartDashboard.putNumber("TX: ", getTX(false));
        SmartDashboard.putNumber("TY: ", getTY(false));
        SmartDashboard.putNumber("Target Pose (Robot-Relative): ", getPoseTheta());
        SmartDashboard.putNumber("Camera Distance to Target: ", getDistanceCameraToTarget(GameField.reefAprilTagHeights));
    }

    public static double getDistanceCameraToTarget(double targetHeight) { // Gets the distance from the target to the physical limelight
        double angleInRadians = Units.degreesToRadians(Vision.limelightAngle + getTY(false));
        double distance = Math.abs((targetHeight - Vision.limelightHeight) / Math.tan(angleInRadians));
        BasicOperations.insertDoubleToConfinedList(cameraToTargetDistances, distance);
        return distance;
    }

    public static double getAverageDistanceCameraToTarget(double targetHeight) { // Averages the distance from the target to the physical limelight over several trials
        return BasicOperations.findAverageArray(cameraToTargetDistances);
    }

    public static double getDistanceCrosshairToTarget(double targetHeight) { // Gets the distance from the target to the limelight's crosshair
        double angleInRadians = Units.degreesToRadians(getTX(false));
        double distance = getDistanceCameraToTarget(targetHeight) * Math.tan(angleInRadians);
        return distance;
    }

    public static boolean isHorizontallyAligned() { // Checks camera alignment with the target along the x-axis
        boolean aligned = Math.abs(getTX(false)) < Vision.TXTolerance;
        return aligned;
    }

    public static boolean isVerticallyAligned() { // Checks camera alignment with the target along the y-axis
        boolean aligned = Math.abs(getTY(false)) < Vision.TYTolerance;
        return aligned;
    }

    public static boolean isZeroPose() { // Checks robot alignment with the target regarding yaw
        boolean isZeroPose = Math.abs(getPoseTheta()) <= Vision.poseTolerance;
        return isZeroPose;
    }

    public static double getRotationalCorrectionOutput() { // Gives an rotational output value to correct tx
        if (isHorizontallyAligned()) {
            return 0.0;
        } else {
            double correctionOutput = getTX(true) * Vision.TXkP;
            return correctionOutput;
        }
    }

    public static double getHorizontalCorrectionOutput() { // Gives a translational (left/right) output value to correct tx
        if (isHorizontallyAligned()) {
            return 0;
        } else {
            double correctionOutput = getTX(true) * Vision.TYkP;
            return correctionOutput;
        }
    }

    public static double getForwardCorrectionOutput() { // Gives a translational (forward/backward) output value to correct ty
        if (isVerticallyAligned()) {
            return 0;
        } else {
            double correctionOutput = getTY(true) * Vision.TYkP;
            return correctionOutput;
        }
    }

    public static double getPoseCorrectionOutput() { // Gives a rotational output value to correct the robot's yaw relative to the target
        if (isZeroPose()) {
            return 0;
        } else {
            double correctionOutput = getPoseTheta() * (1.0 / 20) * Vision.posekP;
            return correctionOutput;
        }
    }

    public static Pose2d getAprilTagLocation(int targetID) { // Gets the pose of a valid april tag relative to the field (blue origin) (UNUSED)
        switch (targetID) {
            case 1:
                return GameField.redStationTagOne;
            case 2:
                return GameField.redStationTagTwo;
            case 6:
                return GameField.redReefTagSix;
            case 7:
                return GameField.redReefTagSeven;
            case 8:
                return GameField.redReefTagEight;
            case 9:
                return GameField.redReefTagNine;
            case 10:
                return GameField.redReefTagTen;
            case 11:
                return GameField.redReefTagEleven;
            case 12:
                return GameField.blueStationTagTwelve;
            case 13:
                return GameField.blueStationRobotLeftThirteen;
            case 17:
                return GameField.blueReefTagSeventeen;
            case 18:
                return GameField.blueReefTagEighteen;
            case 19:
                return GameField.blueReefTagNineteen;
            case 20:
                return GameField.blueReefTagTwenty;
            case 21:
                return GameField.blueReefTagTwentyone;
            case 22:
                return GameField.blueReefTagTwentytwo;
            default:
                System.out.println("[WARNING] An invalid April Tag ID was given!");
                return null;
        }
    }

    public static Pose2d robotPoseToTargetError(Pose2d robotPose) {
        Pose2d aprilTagPose = getAprilTagLocation(getTargetID());
        if (aprilTagPose != null) {
            Rotation2d resultRotation = aprilTagPose.getRotation().minus(robotPose.getRotation());
            Translation2d resultTranslation = aprilTagPose.getTranslation().minus(robotPose.getTranslation());
            return new Pose2d(resultTranslation, resultRotation);
        } else {
            return null;
        }
    }

    public static Pose2d getRobotCenterGoal(int targetID) { // Gets the CENTER target robot pose of a valid april tag relative to the field (blue origin)
        switch (targetID) {
            case 1:
                return GameField.redStationRobotCenterOne;
            case 2:
                return GameField.redStationRobotCenterTwo;
            case 6:
                return GameField.redReefRobotAlgaeSix;
            case 7:
                return GameField.redReefRobotAlgaeSeven;
            case 8:
                return GameField.redReefRobotAlgaeEight;
            case 9:
                return GameField.redReefRobotAlgaeNine;
            case 10:
                return GameField.redReefRobotAlgaeTen;
            case 11:
                return GameField.redReefRobotAlgaeEleven;
            case 12:
                return GameField.blueStationRobotCenterTwelve;
            case 13:
                return GameField.blueStationRobotCenterThirteen;
            case 17:
                return GameField.blueReefRobotAlgaeSeventeen;
            case 18:
                return GameField.blueReefRobotAlgaeEighteen;
            case 19:
                return GameField.blueReefRobotAlgaeNineteen;
            case 20:
                return GameField.blueReefRobotAlgaeTwenty;
            case 21:
                return GameField.blueReefRobotAlgaeTwentyone;
            case 22:
                return GameField.blueReefRobotAlgaeTwentytwo;
            default:
                System.out.println("[WARNING] An invalid April Tag ID was given!");
                return null;
        }
    }

    public static Pose2d getRobotLeftGoal(int targetID) { // Gets the LEFT target robot pose of a valid april tag relative to the field (blue origin)
        switch (targetID) {
            case 1:
                return GameField.redStationRobotLeftOne;
            case 2:
                return GameField.redStationRobotLeftTwo;
            case 6:
                return GameField.redReefRobotLeftSix;
            case 7:
                return GameField.redReefRobotLeftSeven;
            case 8:
                return GameField.redReefRobotLeftEight;
            case 9:
                return GameField.redReefRobotLeftNine;
            case 10:
                return GameField.redReefRobotLeftTen;
            case 11:
                return GameField.redReefRobotLeftEleven;
            case 12:
                return GameField.blueStationRobotLeftTwelve;
            case 13:
                return GameField.blueStationRobotLeftThirteen;
            case 17:
                return GameField.blueReefRobotLeftSeventeen;
            case 18:
                return GameField.blueReefRobotLeftEighteen;
            case 19:
                return GameField.blueReefRobotLeftNineteen;
            case 20:
                return GameField.blueReefRobotLeftTwenty;
            case 21:
                return GameField.blueReefRobotLeftTwentyone;
            case 22:
                return GameField.blueReefRobotLeftTwentytwo;
            default:
                System.out.println("[WARNING] An invalid April Tag ID was given!");
                return null;
        }
    }

    public static Pose2d getRobotRightGoal(int targetID) { // Gets the RIGHT target robot pose of a valid april tag relative to the field (blue origin)
        switch (targetID) {
            case 1:
                return GameField.redStationRobotRightOne;
            case 2:
                return GameField.redStationRobotRightTwo;
            case 6:
                return GameField.redReefRobotRightSix;
            case 7:
                return GameField.redReefRobotRightSeven;
            case 8:
                return GameField.redReefRobotRightEight;
            case 9:
                return GameField.redReefRobotRightNine;
            case 10:
                return GameField.redReefRobotRightTen;
            case 11:
                return GameField.redReefRobotRightEleven;
            case 12:
                return GameField.blueStationRobotRightTwelve;
            case 13:
                return GameField.blueStationRobotRightThirteen;
            case 17:
                return GameField.blueReefRobotRightSeventeen;
            case 18:
                return GameField.blueReefRobotRightEighteen;
            case 19:
                return GameField.blueReefRobotRightNineteen;
            case 20:
                return GameField.blueReefRobotRightTwenty;
            case 21:
                return GameField.blueReefRobotRightTwentyone;
            case 22:
                return GameField.blueReefRobotRightTwentytwo;
            default:
                System.out.println("[WARNING] An invalid April Tag ID was given!");
                return null;
        }
    }
}
