package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.GameField;

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

    public static Pose2d findTranslatedPoseCenter(Pose2d targetPose, double robotOffset, double angularOffset) {
        double resultX = targetPose.getX() + (robotOffset * Math.cos(targetPose.getRotation().getRadians()));
        double resultY = targetPose.getY() + (robotOffset * Math.sin(targetPose.getRotation().getRadians()));
        double resultAngle = targetPose.getRotation().getDegrees() + angularOffset;
        return new Pose2d(resultX, resultY, Rotation2d.fromDegrees(resultAngle));
    }
    
    public static Pose2d findTranslatedPoseLeft(Pose2d translatedPoseCenter, double leftOffset, boolean accountForReefRotation) { // Use findTranslatedPoseCenter to find translatedPoseCenter
        double additionalAngleOffset = 0;
        if (accountForReefRotation) {
            additionalAngleOffset = Units.degreesToRadians(180);
        }
        double resultX = translatedPoseCenter.getX() + (leftOffset * Math.cos(translatedPoseCenter.getRotation().getRadians() + Constants.GameField.leftMovementFieldAngle + additionalAngleOffset));
        double resultY = translatedPoseCenter.getY() + (leftOffset * Math.sin(translatedPoseCenter.getRotation().getRadians() + Constants.GameField.leftMovementFieldAngle + additionalAngleOffset));
        return new Pose2d(resultX, resultY, translatedPoseCenter.getRotation());
    }

    public static Pose2d findTranslatedPoseRight(Pose2d translatedPoseCenter, double rightOffset, boolean accountForReefRotation) { // Use findTranslatedPoseCenter to find translatedPoseCenter
        double additionalAngleOffset = 0;
        if (accountForReefRotation) {
            additionalAngleOffset = Units.degreesToRadians(180);
        }
        double resultX = translatedPoseCenter.getX() + (rightOffset * Math.cos(translatedPoseCenter.getRotation().getRadians() + Constants.GameField.rightMovementFieldAngle + additionalAngleOffset));
        double resultY = translatedPoseCenter.getY() + (rightOffset * Math.sin(translatedPoseCenter.getRotation().getRadians() + Constants.GameField.rightMovementFieldAngle + additionalAngleOffset));
        return new Pose2d(resultX, resultY, translatedPoseCenter.getRotation());
    }

    public static Pose2d transformBlueToRedAlliancePose(Pose2d blueAlliancePose) { // Flips the blue alliance coordinates rotationally; this applies ONLY FOR REEFSCAPE!!!
        double resultX = GameField.fieldSizeX - blueAlliancePose.getX();
        double resultY = GameField.fieldSizeY - blueAlliancePose.getY();
        double resultRotation = blueAlliancePose.getRotation().getDegrees() - 180;
        return new Pose2d(resultX, resultY, Rotation2d.fromDegrees(resultRotation));
    }
}
