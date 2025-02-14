package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.OdometryThread;
import frc.lib.math.Optimizations;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.lib.math.Conversions.proportionalPowerToMps;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.MODULES;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class Swerve extends GenericSubsystem {
    private double lastTimestamp = Timer.getFPGATimestamp();

    public boolean isAtPose(Pose2d target, double allowedDistanceFromTargetMeters, double allowedRotationalErrorDegrees) {
         return POSE_ESTIMATOR.getCurrentPose().getTranslation().getDistance(target.getTranslation()) < allowedDistanceFromTargetMeters &&
                Math.abs(POSE_ESTIMATOR.getCurrentPose().getRotation().minus(target.getRotation()).getDegrees()) < allowedRotationalErrorDegrees;
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return SYSID_DRIVE_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        for (SwerveModule module : MODULES) {
            module.runDriveMotorForCharacterization(voltage);
        }
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        MODULES[0].logForSysId(log);
    }

    public void setGyroHeading(Rotation2d heading) {
        GYRO.setGyroYaw(heading.getRotations());
    }

    public double getGyroHeading() {
        return GYRO.getYawRotations();
    }

    public ChassisSpeeds getRobotRelativeVelocity() {
        return ROBOT_CONFIG.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeVelocity(), POSE_ESTIMATOR.getCurrentPose().getRotation());
    }

    public void runDriveMotorWheelCharacterization(double voltage) {
        for (SwerveModule module : MODULES)
            module.runDriveMotorForCharacterization(voltage);
    }

    public double[] getDriveWheelPositionsRadians() {
        final double[] driveWheelPositions = new double[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            driveWheelPositions[i] = MODULES[i].getDriveWheelPositionRadians();

        return driveWheelPositions;
    }

    @Override
    public void periodic() {
        final double[] odometryUpdatesYawRotations = GYRO.getInputs().threadGyroYawRotations;
        final int odometryUpdates = odometryUpdatesYawRotations.length;

        if (OdometryThread.getInstance().getLatestTimestamps().length == 0) return;

        final SwerveModulePosition[][] swerveWheelPositions = new SwerveModulePosition[odometryUpdates][];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
            gyroRotations[i] = Rotation2d.fromRotations(odometryUpdatesYawRotations[i]);
        }

        POSE_ESTIMATOR.updateFromOdometry(
                swerveWheelPositions,
                gyroRotations,
                OdometryThread.getInstance().getLatestTimestamps()
        );
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds, boolean shouldUseClosedLoop) {
        chassisSpeeds = discretize(chassisSpeeds);

        if (Optimizations.isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS);

        for (int i = 0; i < MODULES.length; i++)
            MODULES[i].setTargetState(swerveModuleStates[i], shouldUseClosedLoop);
    }

    protected void driveOpenLoop(double xPower, double yPower, double thetaPower, boolean robotCentric) {
        if (robotCentric)
            driveRobotRelative(xPower, yPower, thetaPower, false);
        else
            driveFieldRelative(xPower, yPower, thetaPower, false);
    }

    protected void driveWithTarget(double xPower, double yPower, boolean robotCentric) {
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();

        final double controllerOutput = Units.degreesToRadians(SWERVE_ROTATION_CONTROLLER.calculate(currentAngle.getDegrees()));

        if (robotCentric)
            driveRobotRelative(xPower, yPower, controllerOutput, false);
        else
            driveFieldRelative(xPower, yPower, controllerOutput, false);
    }

    protected void driveToPose(Pose2d target) {
        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();

        driveFieldRelative(
                PID_TRANSLATION_CONTROLLER.calculate(
                        currentPose.getX(),
                        target.getX()
                ),

                PID_TRANSLATION_CONTROLLER.calculate(
                        currentPose.getY(),
                        target.getY()
                ),

                SWERVE_ROTATION_CONTROLLER.calculate(currentPose.getRotation().getDegrees()),
                true
        );
    }

    protected void driveToPoseTrapezoidal(Pose2d target) {
        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();

        Logger.recordOutput("NGIGER1/PROFILED_TRANSLATION_CONTROLLER_output", PROFILED_TRANSLATION_CONTROLLER.calculate(currentPose.getX()));
        Logger.recordOutput("NGIGER1/PROFILED_TRANSLATION_CONTROLLER_current", currentPose.getX());
        Logger.recordOutput("NGIGER1/PROFILED_TRANSLATION_CONTROLLER_target", target.getX());
        Logger.recordOutput("NGIGER1/PROFILED_TRANSLATION_CONTROLLER_velocity", getFieldRelativeVelocity().vxMetersPerSecond);
        Logger.recordOutput("NGIGER1/PROFILED_TRANSLATION_CONTROLLER_isAtGoal", PROFILED_TRANSLATION_CONTROLLER.atGoal());

        Logger.recordOutput("NGIGER2/PROFILED_STRAFE_CONTROLLER_output", PROFILED_STRAFE_CONTROLLER.calculate(currentPose.getY()));
        Logger.recordOutput("NGIGER2/PROFILED_STRAFE_CONTROLLER_current", currentPose.getY());
        Logger.recordOutput("NGIGER2/PROFILED_STRAFE_CONTROLLER_target", target.getY());
        Logger.recordOutput("NGIGER2/PROFILED_STRAFE_CONTROLLER_velocity", getFieldRelativeVelocity().vyMetersPerSecond);
        Logger.recordOutput("NGIGER2/PROFILED_STRAFE_CONTROLLER_isAtGoal", PROFILED_STRAFE_CONTROLLER.atGoal());

        Logger.recordOutput("NGIGER3/SWERVE_ROTATION_CONTROLLER_output", SWERVE_ROTATION_CONTROLLER.calculate(currentPose.getRotation().getDegrees(), target.getRotation().getDegrees()));
        Logger.recordOutput("NGIGER3/SWERVE_ROTATION_CONTROLLER_current", currentPose.getRotation().getDegrees());
        Logger.recordOutput("NGIGER3/SWERVE_ROTATION_CONTROLLER_target", target.getRotation().getDegrees());
        Logger.recordOutput("NGIGER3/SWERVE_ROTATION_CONTROLLER_velocity", getFieldRelativeVelocity().omegaRadiansPerSecond);
        Logger.recordOutput("NGIGER3/SWERVE_ROTATION_CONTROLLER_isAtGoal", SWERVE_ROTATION_CONTROLLER.atGoal());

        driveFieldRelative(
                PROFILED_TRANSLATION_CONTROLLER.calculate(currentPose.getX()),
                PROFILED_STRAFE_CONTROLLER.calculate(currentPose.getY()),
                SWERVE_ROTATION_CONTROLLER.calculate(currentPose.getRotation().getDegrees()),
                true
        );
    }

    protected void driveFieldRelative(double xPower, double yPower, double thetaPower, boolean shouldUseClosedLoop) {
        ChassisSpeeds speeds = proportionalSpeedToMps(new ChassisSpeeds(xPower, yPower, thetaPower));
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation());

        driveRobotRelative(speeds, shouldUseClosedLoop);
    }

    protected void driveRobotRelative(double xPower, double yPower, double thetaPower, boolean shouldUseClosedLoop) {
        final ChassisSpeeds speeds = proportionalSpeedToMps(new ChassisSpeeds(xPower, yPower, thetaPower));
        driveRobotRelative(speeds, shouldUseClosedLoop);
    }

    protected void resetTranslationalControllers() {
        PROFILED_TRANSLATION_CONTROLLER.reset(POSE_ESTIMATOR.getCurrentPose().getX(), SWERVE.getFieldRelativeVelocity().vxMetersPerSecond);
        PROFILED_STRAFE_CONTROLLER.reset(POSE_ESTIMATOR.getCurrentPose().getY(), SWERVE.getFieldRelativeVelocity().vyMetersPerSecond);
    }

    protected void resetRotationController() {
        SWERVE_ROTATION_CONTROLLER.reset(POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees(), getFieldRelativeVelocity().omegaRadiansPerSecond);
    }

    protected void setGoalTranslationalControllers(Pose2d target) {
        PROFILED_TRANSLATION_CONTROLLER.setGoal(target.getX());
        PROFILED_STRAFE_CONTROLLER.setGoal(target.getY());
    }

    protected void setGoalRotationController(Pose2d target) {
        SWERVE_ROTATION_CONTROLLER.setGoal(target.getRotation().getDegrees());
    }

    protected SwerveModulePosition[] getSwerveWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[MODULES.length];

        for (int i = 0; i < MODULES.length; i++) {
            swerveModulePositions[i] = MODULES[i].getOdometryPosition(odometryUpdateIndex);
        }

        return swerveModulePositions;
    }

    protected ChassisSpeeds proportionalSpeedToMps(ChassisSpeeds chassisSpeeds) {
        return new ChassisSpeeds(
                proportionalPowerToMps(chassisSpeeds.vxMetersPerSecond, ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS),
                proportionalPowerToMps(chassisSpeeds.vyMetersPerSecond, ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS),
                chassisSpeeds.omegaRadiansPerSecond
        );
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            states[i] = MODULES[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    @SuppressWarnings("unused")
    protected SwerveModuleState[] getModuleTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[MODULES.length];

        for (int i = 0; i < MODULES.length; i++)
            states[i] = MODULES[i].getTargetState();

        return states;
    }

    protected void stop() {
        for (SwerveModule currentModule : MODULES)
            currentModule.stop();
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds The chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    protected ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        final double currentTimestamp = Timer.getFPGATimestamp();
        final double difference = currentTimestamp - lastTimestamp;

        lastTimestamp = currentTimestamp;

        return ChassisSpeeds.discretize(chassisSpeeds, difference);
    }
}
