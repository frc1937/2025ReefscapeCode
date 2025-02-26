package frc.robot.poseestimation.photoncamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.HardwareManager;
import frc.robot.poseestimation.poseestimator.StandardDeviations;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.*;

public class PhotonCameraIO implements LoggableHardware {
    private final String name;
    private final Transform3d robotCenterToCamera;

    private final CameraInputsAutoLogged inputs = new CameraInputsAutoLogged();

    private double lastUpdatedTimestamp;

    public PhotonCameraIO(String name, Transform3d robotCenterToCamera) {
        this.name = name;
        this.robotCenterToCamera = robotCenterToCamera;

        HardwareManager.addHardware(this);
    }

    public double getLastResultTimestamp() {
        return inputs.lastResultTimestamp;
    }

    public int[] getVisibleTags() {
        return inputs.visibleTagIDs;
    }

    public double getAverageDistanceFromTags() {
        return inputs.averageDistanceFromTags;
    }

    public boolean hasNewResult() {
        return inputs.hasResult && inputs.averageDistanceFromTags != 0 && isNewTimestamp();
    }

    public Pose2d getRobotPose() {
        return inputs.estimatedRobotPose.toPose2d();
    }

    public StandardDeviations getStandardDeviations() {
        return new StandardDeviations(
                calculateStandardDeviation(TRANSLATION_STD_EXPONENT, inputs.averageDistanceFromTags, inputs.visibleTagIDs.length),
                calculateStandardDeviation(ROTATION_STD_EXPONENT, inputs.averageDistanceFromTags, inputs.visibleTagIDs.length));
    }

    protected void refreshInputs(CameraInputsAutoLogged inputs) { }

    private void logVisibleTags() {
        if (!inputs.hasResult) {
            Logger.recordOutput("UsedTags/" + name, new Pose3d[0]);
            return;
        }

        final Pose3d[] visibleTagPoses = new Pose3d[inputs.visibleTagIDs.length];

        for (int i = 0; i < visibleTagPoses.length; i++)
            visibleTagPoses[i] = TAG_ID_TO_POSE.get(inputs.visibleTagIDs[i]);

        Logger.recordOutput("UsedTags/" + name, visibleTagPoses);
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    private double calculateStandardDeviation(double exponent, double distance, int numberOfVisibleTags) {
        return exponent * (distance * distance) / numberOfVisibleTags;
    }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);

        logVisibleTags();
    }

    @Override
    public CameraInputsAutoLogged getInputs() {
        return inputs;
    }

    @AutoLog
    public static class CameraInputs {
        public boolean hasResult = false;

        public int[] visibleTagIDs = new int[0];

        public double lastResultTimestamp = 0;
        public double averageDistanceFromTags = 0;

        public Pose3d estimatedRobotPose = new Pose3d();
    }
}
