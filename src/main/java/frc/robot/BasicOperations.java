package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class BasicOperations {
    public static double getSuccessRate(boolean[] attemptsList) {
        int attempts = 0;
        int successes = 0;
        for (boolean attemptElement : attemptsList) {
            attempts++;
            if (attemptElement) {
                successes++;
            }
        }
        double successRate = (double) successes / attempts;
        return successRate;
    }

    public static void insertBooleanToConfinedList(boolean[] list, boolean newValue) {
        for (int i = list.length - 1; i > 0; i--) {
            list[i] = list[i - 1];
        }
        list[0] = newValue;
    }

    public static void insertDoubleToConfinedList(double[] list, double newValue) {
        for (int i = list.length - 1; i > 0; i--) {
            list[i] = list[i - 1];
        }
        list[0] = newValue;
    }

    public static double findAverage(double numOne, double numTwo) {
        return (numOne + numTwo) / 2;
    }

    public static double findAverageArray(double[] numList) {
        double total = 0;
        for (int i = 0; i < numList.length; i++) {
            total += numList[i];
        }
        double average = total / numList.length;
        return average;
    }

    public static Pose2d findTranslatedPoseCenter(Pose2d targetPose, double offset) { // For REEF ONLY
        double resultX = targetPose.getX() + (offset * Math.cos(targetPose.getRotation().getRadians()));
        double resultY = targetPose.getY() + (offset * Math.sin(targetPose.getRotation().getRadians()));
        double resultAngle = targetPose.getRotation().getDegrees() + 180;
        return new Pose2d(resultX, resultY, Rotation2d.fromDegrees(resultAngle));
    }
}
