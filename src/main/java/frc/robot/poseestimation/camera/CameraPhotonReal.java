package frc.robot.poseestimation.camera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;

import java.util.ArrayList;

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
            inputs.estimations = null;
            return;
        }

        final var estimations = new ArrayList<EstimateData>();

        for (var result : results) {
            final var bestTarget = result.getBestTarget();

            if (bestTarget == null || bestTarget.poseAmbiguity > MAX_AMBIGUITY) continue;

            final int fiducialId = bestTarget.fiducialId;

            final var bestCameraTransform = bestTarget.bestCameraToTarget;
            final var alternateCameraTransform = bestTarget.altCameraToTarget;

            final double currentYaw = POSE_ESTIMATOR.getCurrentAngle().getDegrees();

            final double bestYawError = Math.abs(MathUtil.inputModulus(
                    bestCameraTransform.getRotation().getZ() - currentYaw,
                    -180.0,
                    180.0));

            final double alternateYawError = Math.abs(MathUtil.inputModulus(
                    alternateCameraTransform.getRotation().getZ() - currentYaw,
                    -180.0,
                    180.0));

            final var bestTransform = bestYawError < alternateYawError ? bestCameraTransform : alternateCameraTransform;

            final Pose3d tagPose = TAG_ID_TO_POSE.get(fiducialId);

            inputs.hasResult = true;

            final var estimatedPose = tagPose.transformBy(bestTransform.inverse()).transformBy(cameraToRobot);

            estimations.add(new EstimateData(
                    estimatedPose,
                    result.getTimestampSeconds(),
                    estimatedPose.getTranslation().getDistance(tagPose.getTranslation())));
        }

        if (estimations.isEmpty()) {
            inputs.hasResult = false;
            return;
        }

        inputs.estimations = estimations.toArray(new EstimateData[0]);
    }
}
