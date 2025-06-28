package frc.robot.poseestimation.camera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.poseestimation.PoseEstimatorConstants.MAX_AMBIGUITY;
import static frc.robot.poseestimation.PoseEstimatorConstants.TAG_ID_TO_POSE;

public class CameraPhotonReal extends CameraIO {
    private final PhotonCamera camera;
    private final Transform3d cameraToRobot;

    public CameraPhotonReal(String name, Transform3d cameraToRobot) {
        this.cameraToRobot = cameraToRobot;
        this.camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(CameraIOInputsAutoLogged inputs) {
        var results = camera.getAllUnreadResults();

        if (results == null) {
            inputs.hasResult = false;
            inputs.estimatedPose = null;
            return;
        }

        for (var result : results) {
            var bestTarget = result.getBestTarget();

            if (bestTarget == null || bestTarget.poseAmbiguity > MAX_AMBIGUITY) continue;

            int fiducialId = bestTarget.fiducialId;

            var bestCameraTransform = bestTarget.bestCameraToTarget;
            var alternateCameraTransform = bestTarget.altCameraToTarget;

            double yaw = POSE_ESTIMATOR.getCurrentAngle().getDegrees();

            double bestYawError = Math.abs(MathUtil.inputModulus(
                    bestCameraTransform.getRotation().getZ() - yaw,
                    -180.0,
                    180.0));

            double alternateYawError = Math.abs(MathUtil.inputModulus(
                    alternateCameraTransform.getRotation().getZ() - yaw,
                    -180.0,
                    180.0));

            var bestTransform = bestYawError < alternateYawError ? bestCameraTransform : alternateCameraTransform;

            final Pose3d tagPose = TAG_ID_TO_POSE.get(fiducialId);

            inputs.hasResult = true;

            inputs.estimatedPose = tagPose.transformBy(bestTransform.inverse()).transformBy(cameraToRobot);
            inputs.timestamp = result.getTimestampSeconds();

            inputs.estimatedDistanceFromTag = inputs.estimatedPose.getTranslation().getDistance(tagPose.getTranslation());
        }
    }
    //TODO: Better way to handlew multiple inptz; Currently they jsut override!
}
