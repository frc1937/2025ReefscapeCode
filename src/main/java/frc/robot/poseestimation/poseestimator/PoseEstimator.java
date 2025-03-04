package frc.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.poseestimation.photoncamera.PhotonCameraIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;

import static frc.lib.util.QuickSortHandler.sort;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.commands.pathfinding.PathfindingCommands.IS_ALIGNING_REEF;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_KINEMATICS;

public class PoseEstimator implements AutoCloseable {
    private final Field2d field = new Field2d();
    private final PhotonCameraIO[] aprilTagCameras;
    private final TimeInterpolatableBuffer<Pose2d> previousOdometryPoses = TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SECONDS);

    private Pose2d
            odometryPose = DEFAULT_POSE,
            estimatedPose = DEFAULT_POSE;

    private SwerveModulePosition[] lastSwerveModulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private Rotation2d lastGyroHeading = new Rotation2d();

    public PoseEstimator(PhotonCameraIO... aprilTagCameras) {
        this.aprilTagCameras = aprilTagCameras;

        initialize();
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        updateFromAprilTagCameras();
        field.setRobotPose(getCurrentPose());
    }

    public void resetPose(Pose2d newPose) {
        SWERVE.setGyroHeading(newPose.getRotation());
        odometryPose = newPose;
        estimatedPose = newPose;
        lastGyroHeading = newPose.getRotation();

        previousOdometryPoses.clear();
    }

    /**
     * @return The estimated pose of the robot, relative to the bottom blue corner
     */
    @AutoLogOutput
    public Pose2d getCurrentPose() {
        return estimatedPose;
    }

    /**
     * @return The odometry's estimated pose, relative to the bottom blue corner
     */
    @AutoLogOutput
    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    /**
     * Updates the pose estimator with the given SWERVE wheel positions and gyro rotations.
     *
     * @param swerveWheelPositions the SWERVE wheel positions accumulated since the last update
     * @param gyroRotations        the gyro rotations accumulated since the last update
     */
    public void updateFromOdometry(SwerveModulePosition[][] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++) {
            if (swerveWheelPositions[i] == null) continue;
            addOdometryObservation(swerveWheelPositions[i], gyroRotations[i], timestamps[i]);
        }
    }

    /**
     * Sets the estimated robot pose from the odometry at the given timestamp.
     *
     * @param swerveModulePositions the positions of each swerve module
     * @param gyroHeading           the heading of the gyro
     * @param timestamp             the timestamp of the odometry observation
     */
    private void addOdometryObservation(SwerveModulePosition[] swerveModulePositions, Rotation2d gyroHeading, double timestamp) {
        final Twist2d odometryDelta = SWERVE_KINEMATICS.toTwist2d(lastSwerveModulePositions, swerveModulePositions);
        final Twist2d gyroOdometryDelta = new Twist2d(odometryDelta.dx, odometryDelta.dy,
                gyroHeading.minus(lastGyroHeading).getRadians());

        odometryPose = odometryPose.exp(gyroOdometryDelta);
        estimatedPose = estimatedPose.exp(gyroOdometryDelta);

        lastSwerveModulePositions = swerveModulePositions;
        lastGyroHeading = gyroHeading;

        previousOdometryPoses.addSample(timestamp, odometryPose);
    }

    /**
     * Sets the estimated pose from vision at the given timestamp.
     *
     * @param observationPose the estimated robot pose
     * @param timestamp       the timestamp of the observation
     */
    private void addVisionObservation(Pose2d observationPose, double timestamp, StandardDeviations standardDeviations) {
        final Pose2d odometryPoseAtTimestamp = getOdometryPoseAtTimestamp(timestamp);
        final Pose2d estimatedPoseAtObservationTime = getEstimatedPoseAtTimestamp(odometryPoseAtTimestamp);

        if (estimatedPoseAtObservationTime == null) return;

        final Pose2d estimatedPoseAtTimestampWithAmbiguityCompensation =
                getCompensatedPoseAtTimestamp(estimatedPoseAtObservationTime, observationPose, standardDeviations);
        final Transform2d odometryPoseAtTimestampToCurrentOdometryPose = new Transform2d(odometryPoseAtTimestamp, odometryPose);

        this.estimatedPose = estimatedPoseAtTimestampWithAmbiguityCompensation.plus(odometryPoseAtTimestampToCurrentOdometryPose);
    }

    private void updateFromAprilTagCameras() {
        final PhotonCameraIO[] newResultCameras = getCamerasWithResults();

        sort(aprilTagCameras, PhotonCameraIO::getLastResultTimestamp);

        for (PhotonCameraIO aprilTagCamera : newResultCameras) {
            if (aprilTagCamera.getName() == "Cameras/REAR_LEFT" ||
                    aprilTagCamera.getName() == "Cameras/REAR_RIGHT" && IS_ALIGNING_REEF)
                return;

            addVisionObservation(
                    aprilTagCamera.getRobotPose(),
                    aprilTagCamera.getLastResultTimestamp(),
                    aprilTagCamera.getStandardDeviations()
            );
        }
    }

    private PhotonCameraIO[] getCamerasWithResults() {
        final PhotonCameraIO[] camerasWithNewResult = new PhotonCameraIO[aprilTagCameras.length];
        int index = 0;

        for (PhotonCameraIO aprilTagCamera : aprilTagCameras) {
            if (!aprilTagCamera.hasNewResult()) continue;

            camerasWithNewResult[index++] = aprilTagCamera;
        }

        return Arrays.copyOf(camerasWithNewResult, index);
    }

    private Pose2d getOdometryPoseAtTimestamp(double timestamp) {
        if (!hasPoseAtTimestamp(timestamp)) return null;

        final Optional<Pose2d> odometryPoseAtTimestamp = previousOdometryPoses.getSample(timestamp);
        return odometryPoseAtTimestamp.orElse(null);
    }

    private Pose2d getEstimatedPoseAtTimestamp(Pose2d odometryPoseAtTimestamp) {
        if (odometryPoseAtTimestamp == null) return null;

        final Transform2d currentPoseToSamplePose = new Transform2d(odometryPose, odometryPoseAtTimestamp);
        return estimatedPose.plus(currentPoseToSamplePose);
    }

    private boolean hasPoseAtTimestamp(double timestamp) {
        try {
            if (previousOdometryPoses.getInternalBuffer().lastKey() - POSE_BUFFER_SIZE_SECONDS > timestamp)
                return false;
        } catch (NoSuchElementException e) {
            return false;
        }

        return true;
    }

    /**
     * Calculates the estimated pose of a vision observation with compensation for its ambiguity.
     * This is done by finding the difference between the estimated pose at the time of the observation and the estimated pose of the observation and scaling that down using the calibrated standard deviations.
     * This will calculate for the pose at timestamp of `estimatedPoseAtObservationTime`.
     *
     * @param estimatedPoseAtObservationTime the estimated pose of the robot at the time of the observation
     * @param observationPose                the estimated robot pose from the observation
     * @param observationStandardDeviations  the ambiguity of the observation
     * @return the estimated pose with compensation for its ambiguity
     */
    private Pose2d getCompensatedPoseAtTimestamp(Pose2d estimatedPoseAtObservationTime, Pose2d observationPose, StandardDeviations observationStandardDeviations) {
        final Transform2d observationDifference = new Transform2d(estimatedPoseAtObservationTime, observationPose);
        final Transform2d allowedMovement = getCompensatedPoseDifference(observationDifference, observationStandardDeviations);

        return estimatedPoseAtObservationTime.plus(allowedMovement);
    }

    /**
     * Calculates the scaling needed to reduce noise in the estimated pose from the standard deviations of the observation.
     *
     * @param observationDifference    the difference between the estimated pose of the robot at the time of the observation and the estimated pose of the observation
     * @param cameraStandardDeviations the standard deviations of the camera's estimated pose
     * @return the maximum allowed movement of the estimated pose as a {@link Transform2d}
     */
    private Transform2d getCompensatedPoseDifference(Transform2d observationDifference, StandardDeviations cameraStandardDeviations) {
        final StandardDeviations combinedStandardDeviations = ODOMETRY_STANDARD_DEVIATIONS.combineWith(cameraStandardDeviations);
        return combinedStandardDeviations.scaleTransformFromStandardDeviations(observationDifference);
    }

    private void initialize() {
        SmartDashboard.putData("Field", field);

        for (Map.Entry<Integer, Pose3d> entry : TAG_ID_TO_POSE.entrySet()) {
            field.getObject("Tag " + entry.getKey()).setPose(entry.getValue().toPose2d());
        }

        PathPlannerLogging.setLogActivePathCallback(pathPoses -> {
            field.getObject("path").setPoses(pathPoses);
            Logger.recordOutput("PathPlanner/Path", pathPoses.toArray(new Pose2d[0]));
        });

        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));
    }
}
