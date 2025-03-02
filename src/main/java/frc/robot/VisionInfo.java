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

    public static Pose2d getAprilTagFieldLocation(int targetID) { // Gets the pose of a valid april tag relative to the field (blue origin) (UNUSED)
        if (targetID == 6) {
            return GameField.redReefTagSix;
        } else if (targetID == 7) {
            return GameField.redReefTagSeven;
        } else if (targetID == 8) {
            return GameField.redReefTagEight;
        } else if (targetID == 9) {
            return GameField.redReefTagNine;
        } else if (targetID == 10) {
            return GameField.redReefTagTen;
        } else if (targetID == 11) {
            return GameField.redReefTagEleven;
        } else if (targetID == 17) {
            return GameField.blueReefTagSeventeen;
        } else if (targetID == 18) {
            return GameField.blueReefTagEighteen;
        } else if (targetID == 19) {
            return GameField.blueReefTagNineteen;
        } else if (targetID == 20) {
            return GameField.blueReefTagTwenty;
        } else if (targetID == 21) {
            return GameField.blueReefTagTwentyone;
        } else if (targetID == 22) {
            return GameField.blueReefTagTwentytwo;
        } else {
            System.out.println("[WARNING] An invalid April Tag ID was given!");
            return new Pose2d();
        }
    }

    public static Pose2d robotPoseToTargetError(Pose2d robotPose) {
        Pose2d aprilTagPose = getAprilTagFieldLocation(getTargetID());
        Rotation2d resultRotation = aprilTagPose.getRotation().minus(robotPose.getRotation());
        Translation2d resultTranslation = aprilTagPose.getTranslation().minus(robotPose.getTranslation());
        return new Pose2d(resultTranslation, resultRotation);
    }
}
