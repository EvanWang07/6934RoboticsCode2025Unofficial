package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.VisionInfo;
import frc.robot.BasicOperations;
import frc.robot.Constants;
import frc.robot.Constants.GameField;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Set;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry; // Keep this for now
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private double speedMultiplier;
    private Pose2d startingPose;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.canivoreName);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        speedMultiplier = 1; // Speed multiplier defaults to 1

        /* Check alliance color and transform (flip) the inputted starting 2D pose if needed */
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            startingPose = BasicOperations.transformBlueToRedAlliancePose(Constants.QuickTuning.selectedStartingPose);
        } else {
            startingPose = Constants.QuickTuning.selectedStartingPose;
        }

        mSwerveMods = new SwerveModule[] { // Create the SwerveModule objects based on inputted constants
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* Delay by 1 second and then create instantiate the swerve odometery and swerve pose estimator */
        Timer.delay(1);
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), 
                                                           startingPose, VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), 
                                                           VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        System.out.println("Swerve subsystem loaded!");
    
        /* Configure the PathPlanner AutoBuilder using the swerve pose estimator and various swerve methods */
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            
            AutoBuilder.configure(
                this::getSwervePoseEstimation, // Robot pose supplier
                this::setSwervePoseEstimate, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                Constants.AutoConstants.pathPlannerConfig,
                config, // Load the RobotConfig from the GUI settings.
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }    
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setSpeedMultiplier(double newSpeed) { // For slowmode
        speedMultiplier = newSpeed;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) { // For PathPlanner
        this.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false, false); // isOpenLoop should be false for ALL autonomous driving operations
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() { // For PathPlanner
        ChassisSpeeds fieldRelativeSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getGyroYaw());
        return robotRelativeSpeeds;
    }

    public Pose2d getPose() { // For PathPlanner
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) { // For PathPlanner
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setSwervePoseEstimate(Pose2d pose) { // EXPERIMENTAL
        swervePoseEstimator.resetPose(pose);
    }

    public void updateSwervePoseEstimator() { // EXPERIMENTAL
        swervePoseEstimator.update(getGyroYaw(), getModulePositions());
        LimelightHelpers.SetRobotOrientation(Constants.Vision.limelightName, swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate megaTag2Estimation = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.limelightName);
        if (VisionInfo.hasValidTargets() && (gyro.getAngularVelocityZWorld().getValueAsDouble() < 540) && (megaTag2Estimation.pose != null)) { // Ensures the robot is not spinning too quickly, that a target is detected, and that the pose estimation is not null
            swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999));
            swervePoseEstimator.addVisionMeasurement(megaTag2Estimation.pose, megaTag2Estimation.timestampSeconds);
        }
    }

    public Pose2d getSwervePoseEstimation() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public Command pathfindToLeftStation() {
        setSpeedMultiplier(1);
        Pose2d pathfindTargetPose = GameField.robotNearLeftStation;

        PathConstraints pathfindingConstraints = new PathConstraints(3, 2.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            return new DeferredCommand(() -> AutoBuilder.pathfindToPose(BasicOperations.transformBlueToRedAlliancePose(pathfindTargetPose), pathfindingConstraints), Set.of(this));
        } else {
            return new DeferredCommand(() -> AutoBuilder.pathfindToPose(pathfindTargetPose, pathfindingConstraints), Set.of(this));
        }
    }

    public Command pathfindToRightStation() {
        setSpeedMultiplier(1);
        Pose2d pathfindTargetPose = GameField.robotNearRightStation;

        PathConstraints pathfindingConstraints = new PathConstraints(3, 2.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            return new DeferredCommand(() -> AutoBuilder.pathfindToPose(BasicOperations.transformBlueToRedAlliancePose(pathfindTargetPose), pathfindingConstraints), Set.of(this));
        } else {
            return new DeferredCommand(() -> AutoBuilder.pathfindToPose(pathfindTargetPose, pathfindingConstraints), Set.of(this));
        }
    }

    public Command pathfindToReefPosition(Pose2d targetBlueLocationInFront, double metersInFront) {
        PathConstraints pathfindingConstraints = new PathConstraints(1.5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

        Pose2d targetPose = BasicOperations.findTranslatedPoseCenter(targetBlueLocationInFront, metersInFront, 180);
        
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
            return AutoBuilder.pathfindToPose(BasicOperations.transformBlueToRedAlliancePose(targetPose), pathfindingConstraints);
        } else {
            return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints);
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        updateSwervePoseEstimator(); // EXPERIMENTAL
        
        VisionInfo.updateDashboardValues(swervePoseEstimator.getEstimatedPosition().getX(), swervePoseEstimator.getEstimatedPosition().getY(), swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees()); // Puts relevant vision values into SmartDashboard

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}

