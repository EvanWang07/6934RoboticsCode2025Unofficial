package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Vision;

public final class VisionInfo {
    private static boolean[] targetValidResults = new boolean[Vision.targetDetectionListSize];

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

    public static double getPoseX() {
        return LimelightHelpers.getBotPose2d_wpiBlue(Vision.limelightName).getX();
    }

    public static double getPoseY() {
        return LimelightHelpers.getBotPose2d_wpiBlue(Vision.limelightName).getY();
    }

    public static double getPoseTheta() {
        return LimelightHelpers.getBotPose2d_wpiBlue(Vision.limelightName).getRotation().getDegrees();
    }
}
