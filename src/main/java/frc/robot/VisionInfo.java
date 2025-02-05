package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Vision;

public final class VisionInfo {
    private static boolean[] targetValidResults = new boolean[Vision.targetDetectionListSize];
    private static double[] txEstimates = new double[2];
    private static double[] tyEstimates = new double[2];
    private static double[] poseEstimates = new double[2];

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

    public static void updateSummaryValues() { // Sends limelight values to SmartDashboard
        SmartDashboard.putBoolean("Targetable Detected: ", willTarget());
        SmartDashboard.putNumber("TX: ", getTX(false));
        SmartDashboard.putNumber("TY: ", getTY(false));
        SmartDashboard.putNumber("Target Pose (Robot-Relative): ", getPoseTheta());
    } // Note: Possibly put a more descriptive label

    public static double getDistance(double targetHeight) { // Gets the distance from the target
        if (isHorizontallyAligned()) {
            double angleInRadians = Units.degreesToRadians(Vision.limelightAngle + getTY(false));
            double distance = Math.abs((targetHeight - Vision.limelightHeight) / Math.tan(angleInRadians));
            return distance;
        } else { // Only works for a two-dimensional scenario with flat ground
            System.out.println("[WARNING] Conditions were not met to get the target's distance!");
            return 0;
        }
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
            double correctionOutput = getTX(true) * Vision.visionAngleKP;
            return correctionOutput;
        }
    }

    public static double getHorizontalCorrectionOutput() { // Gives a translational (left/right) output value to correct tx
        if (isHorizontallyAligned()) {
            return 0;
        } else {
            double correctionOutput = getTX(true) * Vision.visionTranslationKP;
            return correctionOutput;
        }
    }

    public static double getForwardCorrectionOutput() { // Gives a translational (forward/backward) output value to correct ty
        if (isVerticallyAligned()) {
            return 0;
        } else {
            double correctionOutput = getTY(true) * Vision.visionTranslationKP;
            return correctionOutput;
        }
    }

    public static double getPoseCorrectionOutput() { // Gives a rotational output value to correct the robot's yaw relative to the target
        if (isZeroPose()) {
            return 0;
        } else {
            double correctionOutput = getPoseTheta() * (1.0 / 20) * Vision.visionPoseKP;
            return correctionOutput;
        }
    }

    /*
    public static Pose2d getRobotPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue(Vision.limelightName);
    }

    public static double getRobotLocationX() { // Gets the x component of the robot's location relative to the field (blue origin) (UNUSED)
        return getRobotPose().getX();
    }

    public static double getRobotLocationY() { // Gets the y component of the robot's location relative to the field (blue origin) (UNUSED)
        return getRobotPose().getY();
    }
    */

    public static Pose2d getAprilTagFieldLocation(int targetID) { // Gets the pose of a valid april tag relative to the field (blue origin) (UNUSED)
        if (targetID == 6) {
            return Vision.redReefSix;
        } else if (targetID == 7) {
            return Vision.redReefSeven;
        } else if (targetID == 8) {
            return Vision.redReefEight;
        } else if (targetID == 9) {
            return Vision.redReefNine;
        } else if (targetID == 10) {
            return Vision.redReefTen;
        } else if (targetID == 11) {
            return Vision.redReefEleven;
        } else if (targetID == 17) {
            return Vision.blueReefSeventeen;
        } else if (targetID == 18) {
            return Vision.blueReefEighteen;
        } else if (targetID == 19) {
            return Vision.blueReefNineteen;
        } else if (targetID == 20) {
            return Vision.blueReefTwenty;
        } else if (targetID == 21) {
            return Vision.blueReefTwentyone;
        } else if (targetID == 22) {
            return Vision.blueReefTwentytwo;
        } else {
            System.out.println("[WARNING] An invalid April Tag ID was given!");
            return new Pose2d();
        }
    }

    /*
    public static double getDistanceXExperimental() {
        Translation2d translationDifference = getRobotPose().getTranslation().minus(getAprilTagFieldLocation(getTargetID()).getTranslation());
        return translationDifference.getX();
    }

    public static double getDistanceYExperimental() {
        Translation2d translationDifference = getRobotPose().getTranslation().minus(getAprilTagFieldLocation(getTargetID()).getTranslation());
        return translationDifference.getY();
    }
    */
}
