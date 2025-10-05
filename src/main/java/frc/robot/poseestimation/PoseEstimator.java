package frc.robot.poseestimation;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.poseestimation.camera.Camera;
import frc.robot.poseestimation.camera.EstimateData;
import frc.robot.poseestimation.quest.Quest;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Map;

import static frc.robot.poseestimation.PoseEstimatorConstants.*;

public class PoseEstimator {
    private final Field2d field = new Field2d();

    private final SwerveModulePosition[] positions = {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.SWERVE_KINEMATICS,
            Rotation2d.kZero,
            positions,
            Pose2d.kZero,
            ODOMETRY_STD_DEVS,
            VecBuilder.fill(VISION_STD_LINEAR, VISION_STD_LINEAR, VISION_STD_ANGULAR)
    );

    private final Quest quest;
    private final Camera[] cameras;

    public PoseEstimator(Camera[] cameras, Quest quest) {
        this.quest = quest;
        this.cameras = cameras;

        initialize();
    }

    public void resetPose(Pose2d pose) {
        quest.setPose(pose);
        poseEstimator.resetPose(pose);
    }

    public Rotation2d getCurrentAngle() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    @AutoLogOutput(key = "PoseEstimator/CurrentPose")
    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void periodic() {
        updateFromVision();
        updateFromQuest();

        field.setRobotPose(getCurrentPose());
    }

    public void updateFromQuest() {
        if (quest == null) return;

        quest.refreshInputs();

        if (!quest.isResultValid()) return;

        poseEstimator.addVisionMeasurement(
                quest.getEstimatedPose(),
                quest.getTimestamp(),
                QUEST_STD_DEVS
        );
    }

    public void updateFromVision() {
        for (Camera camera : cameras) {
            camera.refreshInputs();

            if (!camera.cameraHasResults() || camera.getEstimates() == null) continue;

            for (EstimateData estimate : camera.getEstimates()) {
                if (!estimate.isValid()) continue;

                poseEstimator.addVisionMeasurement(
                        estimate.pose().toPose2d(),
                        estimate.timestamp(),
                        estimate.getStandardDeviations()
                );
            }
        }
    }

    public void updateFromOdometry(SwerveModulePosition[][] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamp) {
        if (swerveWheelPositions == null) return;

        for (int i = 0; i < swerveWheelPositions.length; i++) {
            if (swerveWheelPositions[i] == null) continue;

            poseEstimator.updateWithTime(
                    timestamp[i],
                    gyroRotations[i],
                    swerveWheelPositions[i]);
        }
    }

    private void initialize() {
        for (Map.Entry<Integer, Pose3d> entry : TAG_ID_TO_POSE.entrySet()) {
            field.getObject("Tag " + entry.getKey()).setPose(entry.getValue().toPose2d());
        }

        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback((pathPoses) -> {
            field.getObject("path").setPoses(pathPoses);
            Logger.recordOutput("PathPlanner/Path", pathPoses.toArray(new Pose2d[0]));
        });

        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));
    }
}