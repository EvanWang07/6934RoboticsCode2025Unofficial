package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Vision;
import frc.robot.Constants.GameField;

public final class VisionInfo {
    private static boolean[] targetValidResults = new boolean[Vision.targetDetectionListSize];

    /**
     * Gets the ID of the closest AprilTag detected
     * @return The ID of the closest AprilTag detected
     */
    public static int getTargetID() {
        return (int) LimelightHelpers.getFiducialID(Vision.limelightName);
    }

    /**
     * Gets the motor output or angle horizontal crosshair-to-target error
     * @param asOutput If the returned value should be given as a motor output (true) or as an angle (false) 
     * @return The motor output [-1, 1] or angle (degrees) based on the horizontal crosshair-to-target error
     */
    public static double getTX(boolean asOutput) {
        if (asOutput) {
            return (LimelightHelpers.getTX(Vision.limelightName) / 41); // LL 3G: +/- 41 degrees is the maximum horizontal offset angle (the camera's FoV)
        } else {
            return LimelightHelpers.getTX(Vision.limelightName);
        }
    }

    /**
     * Gets the motor output or angle vertical crosshair-to-target error
     * @param asOutput If the returned value should be given as a motor output (true) or as an angle (false) 
     * @return The motor output [-1, 1] or angle (degrees) based on the vertical crosshair-to-target error
     */
    public static double getTY(boolean asOutput) {
        if (asOutput) {
            return (LimelightHelpers.getTY(Vision.limelightName) / 28.1); // LL 3G: +/- 28.1 degrees is the maximum vertical offset angle (the camera's FoV)
        } else { 
            return LimelightHelpers.getTY(Vision.limelightName);
        }
    }

    /**
     * Uses Mega Tag 1 to get the current 2D rotation (yaw) of the robot. Make sure to configure the camera position relative to the robot
     * @deprecated The robot code uses Mega Tag 2 instead; see the Swerve.java subsystem for Mega Tag 2 implementation
     * @return The 2D rotation (yaw) (degrees) of the robot relative to the target
     * @see https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
     */
    public static double getPoseTheta() {
        double[] robotPose = LimelightHelpers.getBotPose_TargetSpace(Vision.limelightName);
        return robotPose[4];
    }

    /**
     * Gets the motor output or percentage of the camera frame the target takes up
     * @deprecated The robot code does not utilize TA
     * @param asOutput If the returned value should be given as a motor output (true) or as a percent (false) 
     * @return The motor output [0, 1] or a percentage (%) based on the area of the camera frame the target takes up
     */
    public static double getTA(boolean asOutput) {
        if (asOutput) {
            return (LimelightHelpers.getTA(Vision.limelightName) / 100);
        } else {
            return LimelightHelpers.getTA(Vision.limelightName);
        }
    }

    /**
     * Checks if the camera currently detects a target
     * @return True, if the camera currently detects a target; false, otherwise
     */
    public static boolean hasValidTargets() { // Determines if there is a valid limelight target at the given time
        return LimelightHelpers.getTV(Vision.limelightName);
    }

    /**
     * Checks if the camera has detected a target at least at a minimum rate over the past few ticks.
     * This reduces jittery-ness for code requiring that target detection over a period of time. Set the minimum rate (%) in 
     * Constants.Vision.java
     * @return True, if the camera has detected a target at least at a minimum rate over the past few ticks; false, otherwise
     */
    public static boolean willTarget() {
        BasicOperations.insertBooleanToConfinedList(targetValidResults, hasValidTargets());
        return (BasicOperations.getSuccessRate(targetValidResults) >= Vision.averageTVThreshold);
    }

    /**
     * Sends inputted 2D pose values to the SmartDashboard. Useful for robot localization testing and debugging
     * @param estimatedX The estimated x-coordinate (meters) of the robot relative to the blue origin of the competition field
     * @param estimatedY The estimated y-coordinate (meters) of the robot relative to the blue origin of the competition field
     * @param estimatedYaw The estimated 2D rotation (yaw) (degrees) of the robot relative to the blue origin of the competition field
     */
    public static void updateDashboardValues(double estimatedX, double estimatedY, double estimatedYaw) {
        SmartDashboard.putBoolean("Targetable Detected", willTarget());
        SmartDashboard.putNumber("Robot X-Coordinate (Blue Origin)", estimatedX);
        SmartDashboard.putNumber("Robot Y-Coordinate (Blue Origin)", estimatedY);
        SmartDashboard.putNumber("Robot Yaw (Blue Origin)", estimatedYaw);
        if (hasValidTargets()) {
            SmartDashboard.putNumber("Target ID", getTargetID());
        } else {
            SmartDashboard.putNumber("Target ID", 0);
        }
    }

    /**
     * Checks the camera's horizontal crosshair-to-target alignment. Set the angle tolerance (degrees) in Constants.Vision.java
     * @deprecated Unused
     * @return True, if the camera's crosshair is aligned (within tolerance) with the target; false, otherwise
     */
    public static boolean isHorizontallyAligned() {
        boolean aligned = Math.abs(getTX(false)) < Vision.TXTolerance;
        return aligned;
    }

    /**
     * Checks the camera's vertical crosshair-to-target alignment. Set the angle tolerance (degrees) in Constants.Vision.java
     * @deprecated Unused
     * @return True, if the camera's crosshair is aligned (within tolerance) with the target; false, otherwise
     */
    public static boolean isVerticallyAligned() {
        boolean aligned = Math.abs(getTY(false)) < Vision.TYTolerance;
        return aligned;
    }

    /**
     * Checks if the robot's chassis is aligned (parallel) with the target. 
     * Set the 2D rotation (yaw) tolerance (degrees) in Constants.Vision.java
     * @deprecated Unused
     * @return True, if the robot's chassis is aligned (within tolerance) with the target; false, otherwise
     */
    public static boolean isZeroPose() {
        boolean isZeroPose = Math.abs(getPoseTheta()) <= Vision.poseTolerance;
        return isZeroPose;
    }

    /**
     * Gets the 2D pose of a valid AprilTag relative to the blue origin of the competition field
     * @deprecated Unused
     * @param targetID The ID of the AprilTag
     * @return A Pose2D object representing the 2D pose <x (meters), y (meters), yaw (degrees)> of the center of the AprilTag of the inputted
     * ID relative to the blue origin of the competition field. Null, if there are no relevant AprilTags of the inputted ID. Note that 
     * AprilTags of IDs 3-5 and 14-16 are not considered relevant
     */
    public static Pose2d getAprilTagLocation(int targetID) { 
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

    /**
     * Gets the 2D pose error between an inputted pose and the detected AprilTag
     * @deprecated Unused
     * @param robotPose The current 2D pose of the robot relative to the blue origin of the competition field
     * @return A Pose2D object representing the 2D pose error <x (meters), y (meters), yaw (degrees)> between the center of the 
     * detected AprilTag and the robot. Positive rotation is relative to the blue origin of the competition field. 
     * Null, if a null value is found upon checking the AprilTag's 2D pose. Note that AprilTags of IDs 3-5 and 14-16 will give null Pose2D values
     */
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

    /**
     * Gets the goal center robot 2D pose relative to the blue origin of the competition field. These values are 
     * for positioning the robot directly in front of an AprilTag; note that the goal 2D rotation of the robot is toward the target if 
     * the AprilTag is a part of the reef and away from the target if a part of the coral station
     * @param targetID The ID of the AprilTag
     * @return A Pose2D object representing the goal center robot 2D pose <x (meters), y (meters), yaw (degrees)> for an AprilTag 
     * target of the inputted ID relative to the blue origin of the competition field. Null, if there are no relevant AprilTags of the 
     * inputted ID. Note that AprilTags of IDs 3-5 and 14-16 are not considered relevant
     */
    public static Pose2d getRobotCenterGoal(int targetID) {
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

    /**
     * Gets the goal left robot 2D pose relative to the blue origin of the competition field. These values are 
     * for positioning the robot in front and to the left of an AprilTag; note that the goal 2D rotation of the robot is toward the target if 
     * the AprilTag is a part of the reef and away from the target if a part of the coral station
     * @param targetID The ID of the AprilTag
     * @return A Pose2D object representing the goal left robot 2D pose <x (meters), y (meters), yaw (degrees)> for an AprilTag 
     * target of the inputted ID relative to the blue origin of the competition field. Null, if there are no relevant AprilTags of the 
     * inputted ID. Note that AprilTags of IDs 3-5 and 14-16 are not considered relevant
     */
    public static Pose2d getRobotLeftGoal(int targetID) { 
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

    /**
     * Gets the goal right robot 2D pose relative to the blue origin of the competition field. These values are 
     * for positioning the robot in front and to the right of an AprilTag; note that the goal 2D rotation of the robot is toward the target if 
     * the AprilTag is a part of the reef and away from the target if a part of the coral station
     * @param targetID The ID of the AprilTag
     * @return A Pose2D object representing the goal right robot 2D pose <x (meters), y (meters), yaw (degrees)> for an AprilTag 
     * target of the inputted ID relative to the blue origin of the competition field. Null, if there are no relevant AprilTags of the 
     * inputted ID. Note that AprilTags of IDs 3-5 and 14-16 are not considered relevant
     */
    public static Pose2d getRobotRightGoal(int targetID) {
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
