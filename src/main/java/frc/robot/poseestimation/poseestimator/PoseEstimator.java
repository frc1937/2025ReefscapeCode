package frc.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.QuickSortHandler;
import frc.lib.util.flippable.Flippable;
import frc.robot.poseestimation.photoncamera.PhotonCameraIO;
import frc.robot.poseestimation.photoncamera.PhotonCameraIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;

import static frc.lib.util.QuickSortHandler.sort;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_KINEMATICS;

public class PoseEstimator implements AutoCloseable {
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator = createSwerveDrivePoseEstimator();
    private final SwerveDriveOdometry swerveDriveOdometry = createSwerveDriveOdometry();
    private final Field2d field = new Field2d();
    private final PhotonCameraIO[] aprilTagCameras;
    private final boolean shouldUseRelativeRobotPoseSource;

    /**
     * Constructs a new PoseEstimator and sets the relativeRobotPoseSource.
     * This constructor enables usage of a relative robot pose source and disables the use of april tags for pose estimation, and instead uses them to reset the relative robot pose source's offset.
     *
     * @param aprilTagCameras         the apriltag cameras that should be used to update the relative robot pose source
     */
    public PoseEstimator(PhotonCameraIO... aprilTagCameras) {
        this.aprilTagCameras = aprilTagCameras;
        this.shouldUseRelativeRobotPoseSource = true;

        initialize();
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        updateFromAprilTagCameras();
        field.setRobotPose(getEstimatedRobotPose());
    }

    public Pose2d getCurrentPose() {
        return getEstimatedRobotPose();
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

        swerveDrivePoseEstimator.resetPose(newPose);
        swerveDriveOdometry.resetPose(newPose);
    }

    /**
     * @return the estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput(key = "Poses/Robot/PoseEstimator/EstimatedRobotPose")
    public Pose2d getEstimatedRobotPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * @return the odometry's estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    @AutoLogOutput(key = "Poses/Robot/PoseEstimator/EstimatedOdometryPose")
    public Pose2d getEstimatedOdometryPose() {
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
        PathPlannerLogging.setLogActivePathCallback(pathPoses -> {
            field.getObject("path").setPoses(pathPoses);
            Logger.recordOutput("PathPlanner/Path", pathPoses.toArray(new Pose2d[0]));
        });

        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));

    }

    private void updateFromAprilTagCameras() {
        final PhotonCameraIO[] newResultCameras = getCamerasWithResults();
        sortCamerasByLatestResultTimestamp(newResultCameras);

        for (PhotonCameraIO aprilTagCamera : newResultCameras) {
            swerveDrivePoseEstimator.addVisionMeasurement(
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
            if (aprilTagCamera.hasNewResult()) {
                camerasWithNewResult[index] = aprilTagCamera;
                index++;
            }
        }

        return Arrays.copyOf(camerasWithNewResult, index);
    }

    private void sortCamerasByLatestResultTimestamp(PhotonCameraIO[] aprilTagCameras) {
        QuickSortHandler.sort(aprilTagCameras, PhotonCameraIO::getLastResultTimestamp);
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