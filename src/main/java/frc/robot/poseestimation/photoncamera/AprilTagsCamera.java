package frc.robot.poseestimation.photoncamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.GlobalConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.poseestimation.photoncamera.VisionConstants.VISION_SIMULATION;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.*;

public class AprilTagsCamera extends PhotonCameraIO {
    private final PhotonCamera photonCamera;

    private final Transform3d robotCenterToCamera;

    public AprilTagsCamera(String cameraName, Transform3d robotCenterToCamera) {
        super(cameraName, robotCenterToCamera);

        this.robotCenterToCamera = robotCenterToCamera;

        photonCamera = new PhotonCamera(cameraName);

        if (GlobalConstants.Mode.SIMULATION == CURRENT_MODE)
            VISION_SIMULATION.addCamera(photonCamera, robotCenterToCamera);
    }

    @Override
    protected void refreshInputs(CameraInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = getLatestResult();

        inputs.hasResult = isValidResult(latestResult);

        if (inputs.hasResult) {
            inputs.estimatedRobotPose = getPose(latestResult);
            inputs.lastResultTimestamp = latestResult.getTimestampSeconds();

            inputs.averageDistanceFromTags = getAverageDistanceFromTags(latestResult);
            inputs.visibleTagIDs = getVisibleTagIDs(latestResult);
        } else {
            updateWithNoResult(inputs);
        }
    }

    private boolean isValidResult(PhotonPipelineResult latestResult) {
        return latestResult != null && (latestResult.hasTargets() && latestResult.getBestTarget().getPoseAmbiguity() < MAXIMUM_AMBIGUITY);
    }

    private void updateWithNoResult(CameraInputsAutoLogged inputs) {
        inputs.hasResult = false;
        inputs.visibleTagIDs = new int[0];
        inputs.estimatedRobotPose = new Pose3d();
    }

    private int[] getVisibleTagIDs(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.targets;
        final int[] visibleTagIDs = new int[targets.size()];

        for (int i = 0; i < targets.size(); i++) {
            visibleTagIDs[i] = targets.get(i).getFiducialId();
        }

        return visibleTagIDs;
    }

    private PhotonPipelineResult getLatestResult() {
        final List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        return results.isEmpty() ? null : results.get(results.size() - 1);
    }

    private Pose3d getPose(PhotonPipelineResult result) {
        final Optional<MultiTargetPNPResult> multiTagResult = result.getMultiTagResult();

        if (multiTagResult.isPresent()) {
            final Transform3d cameraTransform = multiTagResult.get().estimatedPose.best;
            return new Pose3d().plus(cameraTransform).relativeTo(APRIL_TAG_FIELD_LAYOUT.getOrigin())
                    .transformBy(robotCenterToCamera.inverse());
        }

        final PhotonTrackedTarget bestTarget = result.getBestTarget();

        final Pose3d tagPose = TAG_ID_TO_POSE.get(bestTarget.getFiducialId());
        final Transform3d targetToCamera = bestTarget.getBestCameraToTarget().inverse();

        return tagPose.transformBy(targetToCamera).transformBy(robotCenterToCamera.inverse());
    }

    private double getAverageDistanceFromTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.targets;
        double distanceSum = 0;

        for (PhotonTrackedTarget currentTarget : targets) {
            final Translation2d distanceTranslation = currentTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
            distanceSum += distanceTranslation.getNorm();
        }

        return distanceSum / targets.size();
    }
}
