package frc.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.flippable.Flippable;
import frc.robot.poseestimation.apriltagcamera.AprilTagCamera;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Map;

import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.TAG_ID_TO_POSE;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_KINEMATICS;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator implements AutoCloseable {
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator = createSwerveDrivePoseEstimator();
    private final SwerveDriveOdometry swerveDriveOdometry = createSwerveDriveOdometry();
    private final Field2d field = new Field2d();
    private final AprilTagCamera[] aprilTagCameras;

    /**
     * Constructs a new PoseEstimator.
     * This constructor disables the use of a relative robot pose source and instead uses april tags cameras for pose estimation.
     *
     * @param aprilTagCameras the cameras that should be used to update the pose estimator
     */
    public PoseEstimator(AprilTagCamera... aprilTagCameras) {
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

    public void resetHeading() {
        final Rotation2d resetRotation = Flippable.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
        swerveDrivePoseEstimator.resetRotation(resetRotation);
        swerveDriveOdometry.resetRotation(resetRotation);
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param newPose the pose to reset to, relative to the blue alliance's driver station right corner
     */
    public void resetPose(Pose2d newPose) {
        SWERVE.setGyroHeading(newPose.getRotation());

        swerveDrivePoseEstimator.resetPose(newPose); // TODO: Might not work as intended
        swerveDriveOdometry.resetPose(newPose);
    }

    /**
     * @return the estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput(key = "Poses/Robot/PoseEstimator/EstimatedRobotPose")
    public Pose2d getCurrentPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * @return the odometry's estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput(key = "Poses/Robot/PoseEstimator/EstimatedOdometryPose")
    public Pose2d getOdometryPose() {
        return swerveDriveOdometry.getPoseMeters();
    }

    /**
     * Updates the pose estimator with the given SWERVE wheel positions and gyro rotations.
     * This function accepts an array of SWERVE wheel positions and an array of gyro rotations because the odometry can be updated at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with all of them.
     *
     * @param swerveWheelPositions the SWERVE wheel positions accumulated since the last update
     * @param gyroRotations        the gyro rotations accumulated since the last update
     */
    public void updatePoseEstimatorStates(SwerveModulePosition[][] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++) {
            if (swerveWheelPositions[i] == null) return;
            swerveDrivePoseEstimator.updateWithTime(timestamps[i], gyroRotations[i], swerveWheelPositions[i]);
            swerveDriveOdometry.update(gyroRotations[i], swerveWheelPositions[i]);
        }
    }

    /**
     * Gets the estimated pose of the robot at the target timestamp.
     *
     * @param timestamp the target timestamp
     * @return the robot's estimated pose at the timestamp
     */
    public Pose2d getEstimatedPoseAtTimestamp(double timestamp) {
        return swerveDrivePoseEstimator.sampleAt(timestamp).orElse(null);
    }

    private void initialize() {
        putAprilTagsOnFieldWidget();
        SmartDashboard.putData("Field", field);
        logTargetPath();
    }

    private void putAprilTagsOnFieldWidget() {
        for (Map.Entry<Integer, Pose3d> entry : TAG_ID_TO_POSE.entrySet()) {
            final Pose2d tagPose = entry.getValue().toPose2d();
            field.getObject("Tag " + entry.getKey()).setPose(tagPose);
        }
    }

    /**
     * Logs and updates the field widget with the target PathPlanner path as an array of Pose2ds.
     */
    private void logTargetPath() {
        PathPlannerLogging.setLogActivePathCallback((pathPoses) -> {
            field.getObject("path").setPoses(pathPoses);
            Logger.recordOutput("PathPlanner/Path", pathPoses.toArray(new Pose2d[0]));
        });

        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));
    }

    private void updateFromAprilTagCameras() {
        final AprilTagCamera[] newResultCameras = getCamerasWithResults();

        for (AprilTagCamera aprilTagCamera : newResultCameras) {
            swerveDrivePoseEstimator.addVisionMeasurement(
                    aprilTagCamera.getEstimatedRobotPose(),
                    aprilTagCamera.getLatestResultTimestampSeconds(),
                    aprilTagCamera.calculateStandardDeviations().toMatrix()
            );
        }
    }

    private AprilTagCamera[] getCamerasWithResults() {
        final AprilTagCamera[] camerasWithNewResult = new AprilTagCamera[aprilTagCameras.length];
        int index = 0;

        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            aprilTagCamera.update();
            if (aprilTagCamera.hasValidResult() && aprilTagCamera.getEstimatedRobotPose() != null) {
                camerasWithNewResult[index] = aprilTagCamera;
                index++;
            }
        }

        return Arrays.copyOf(camerasWithNewResult, index);
    }

    private SwerveDriveOdometry createSwerveDriveOdometry() {
        final SwerveModulePosition[] swerveModulePositions = {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        return new SwerveDriveOdometry(
                SWERVE_KINEMATICS,
                new Rotation2d(),
                swerveModulePositions
        );
    }

    private SwerveDrivePoseEstimator createSwerveDrivePoseEstimator() {
        final SwerveModulePosition[] swerveModulePositions = {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        return new SwerveDrivePoseEstimator(
                SWERVE_KINEMATICS,
                new Rotation2d(),
                swerveModulePositions,
                new Pose2d(),
                PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS.toMatrix(),
                VecBuilder.fill(0, 0, 0)
        );
    }
}