package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Vision;

public final class VisionInfo {
    private static boolean[] targetValidResults = new boolean[Vision.targetDetectionListSize];

    public static int getTargetID() {
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

    public static boolean hasValidTargets() { // Determines if there is a valid limelight target at the given time
        return LimelightHelpers.getTV(Vision.limelightName);
    }

    public static boolean willTarget() { // Determines (by averaging TV values) if the robot will seek a target
        BasicOperations.insertBooleanToConfinedList(targetValidResults, hasValidTargets());
        return (BasicOperations.getSuccessRate(targetValidResults) >= Vision.averageTVThreshold);
    }

    public static double getTA(boolean asOutput) { // Gets the % of the camera frame the target takes up (NOT USED)
        if (asOutput) {
            return (LimelightHelpers.getTA(Vision.limelightName) / 100);
        } else {
            return LimelightHelpers.getTA(Vision.limelightName);
        }
    }

    public static void updateSummaryValues() { // Sends limelight values to SmartDashboard
        SmartDashboard.putBoolean("Can Auto-Align: ", willTarget());
        SmartDashboard.putNumber("TA: ", getTA(false));
        SmartDashboard.putNumber("TX: ", getTX(false));
        SmartDashboard.putNumber("TY: ", getTY(false));
    } // Note: Possibly put a more descriptive label

    public static void switchPipeline(int newPipeline) { // Swaps the camera "mode" (used if there are multiple targets) (NOT USED)
        LimelightHelpers.setPipelineIndex(Vision.limelightName, newPipeline);
    }

    public static double getDistance(double targetHeight) { // Only works for a two-dimensional scenario with flat ground; gets the distance from the target
        double angleInRadians = Units.degreesToRadians(Vision.limelightAngle + getTY(false));
        double distance = (targetHeight - Vision.limelightHeight) / Math.tan(angleInRadians);
        return distance;
    }

    public static double getDistanceX(double targetHeight) {
        double angleFromHorizontal = Units.degreesToRadians(90 - getPoseTheta());
        return (getDistance(targetHeight) * Math.cos(angleFromHorizontal));
    }

    public static double getDistanceY(double targetHeight) {
        double angleFromHorizontal = Units.degreesToRadians(90 - getPoseTheta());
        return (getDistance(targetHeight) * Math.sin(angleFromHorizontal));
    }

    public static boolean isHorizontallyAligned() { // Checks camera alignment with the target along the x-axis
        boolean aligned = Math.abs(getTX(false)) < Vision.TXTolerance;
        return aligned;
    }

    public static boolean isVerticallyAligned() { // Checks camera alignment with the target along the y-axis
        boolean aligned = Math.abs(getTY(false)) < Vision.TYTolerance;
        return aligned;
    }

    public static double getRotationalCorrectionOutput() { // Gives an rotational output value to correct tx
        if (isHorizontallyAligned()) {
            return 0.0;
        } else {
            double correctionOutput = getTX(true) * Vision.visionAngleKP;
            return correctionOutput;
        }
    }

    public static double getTranslationalCorrectionOutput() { // Gives a translational output value to correct ty
        if (isVerticallyAligned()) {
            return 0;
        } else {
            double correctionOutput = getTY(true) * Vision.visionTranslationKP;
            return correctionOutput;
        }
    }

    public static double getPoseTheta() { // Gets the yaw of the limelight relative to the robot
        double[] robotPose = LimelightHelpers.getBotPose_TargetSpace(Vision.limelightName);
        return robotPose[4];
    }

    public static boolean isZeroPose() { // Not used for now
        boolean isZeroPose = Math.abs(getPoseTheta()) <= Vision.poseTolerance;
        return isZeroPose;
    }

    public static double getRobotLocationX() {
        return LimelightHelpers.getBotPose2d_wpiBlue(Vision.limelightName).getX();
    }

    public static double getRobotLocationY() {
        return LimelightHelpers.getBotPose2d_wpiBlue(Vision.limelightName).getX();
    }

    public static Pose2d getAprilTagFieldLocation(int targetID) {
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
}
