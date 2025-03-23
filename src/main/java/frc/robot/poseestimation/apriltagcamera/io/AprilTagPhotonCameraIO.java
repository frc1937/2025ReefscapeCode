package frc.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Arrays;
import java.util.List;

import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.TAG_ID_TO_POSE;

public class AprilTagPhotonCameraIO extends AprilTagCameraIO {
    private final Transform3d robotToCamera;
    final PhotonCamera photonCamera;

    private final LinearFilter movingAverage = LinearFilter.movingAverage(50);

    public AprilTagPhotonCameraIO(String cameraName, Transform3d robotToCamera) {
        photonCamera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;
    }

    @Override
    protected void refreshInputs(AprilTagCameraInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = getLatestPipelineResult();

        inputs.hasResult = latestResult != null && latestResult.hasTargets();
        if (inputs.hasResult) {
            updateHasResultInputs(inputs, latestResult);

            if (inputs.bestCameraSolvePNPPose != null) {
                movingAverage.calculate(Units.radiansToDegrees(inputs.bestCameraSolvePNPPose.getRotation().getZ()));
                Logger.recordOutput(photonCamera.getName() + " z rotation yaw", movingAverage.lastValue());
                Logger.recordOutput(photonCamera.getName() + " y rotation pitch", Units.radiansToDegrees(inputs.bestCameraSolvePNPPose.getRotation().getY()));
            }
             return;
        }

        updateNoResultInputs(inputs);
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private void updateNoResultInputs(AprilTagCameraInputsAutoLogged inputs) {
        inputs.bestCameraSolvePNPPose = new Pose3d();
        inputs.alternateCameraSolvePNPPose = inputs.bestCameraSolvePNPPose;
        inputs.visibleTagIDs = new int[0];
    }

    private void updateHasResultInputs(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        final PhotonTrackedTarget bestTag = getBestTag(latestResult);
        if (bestTag == null) {
            updateNoResultInputs(inputs);
            inputs.hasResult = false;
            return;
        }
        updateSolvePNPPoses(inputs, latestResult, bestTag);
        if (inputs.bestCameraSolvePNPPose == null) {
            updateNoResultInputs(inputs);
            inputs.hasResult = false;
            return;
        }

        inputs.latestResultTimestampSeconds = latestResult.getTimestampSeconds();
        inputs.visibleTagIDs = getVisibleTagIDs(latestResult);
        inputs.poseAmbiguity = latestResult.getMultiTagResult().isPresent() ? 0 : bestTag.getPoseAmbiguity();
        inputs.distancesFromTags = getDistancesFromTags(latestResult);
    }

    private PhotonTrackedTarget getBestTag(PhotonPipelineResult result) {
        for (PhotonTrackedTarget tag : result.getTargets())
            if (TAG_ID_TO_POSE.containsKey(tag.getFiducialId()))
                return tag;
        return null;
    }

    private void updateSolvePNPPoses(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult, PhotonTrackedTarget bestTag) {
        if (latestResult.getMultiTagResult().isPresent()) {
            final Transform3d cameraPoseTransform = latestResult.getMultiTagResult().get().estimatedPose.best;
            inputs.bestCameraSolvePNPPose = new Pose3d().plus(cameraPoseTransform).relativeTo(APRIL_TAG_FIELD_LAYOUT.getOrigin())
                    .transformBy(robotToCamera.inverse());
            inputs.alternateCameraSolvePNPPose = inputs.bestCameraSolvePNPPose;
            return;
        }

        final Pose3d tagPose = TAG_ID_TO_POSE.get(bestTag.getFiducialId());
        if (tagPose == null) {
            inputs.bestCameraSolvePNPPose = null;
            return;
        }

        final Transform3d bestTargetToCamera = bestTag.getBestCameraToTarget().inverse();
        final Transform3d alternateTargetToCamera = bestTag.getAlternateCameraToTarget().inverse();

        inputs.bestCameraSolvePNPPose = tagPose.transformBy(bestTargetToCamera)
                .transformBy(robotToCamera.inverse());
        ;
        inputs.alternateCameraSolvePNPPose = tagPose.transformBy(alternateTargetToCamera)
                .transformBy(robotToCamera.inverse());;
    }

    private int[] getVisibleTagIDs(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final int[] visibleTagIDs = new int[targets.size()];
        int indx = 0;

        for (PhotonTrackedTarget target : targets) {
            if (TAG_ID_TO_POSE.containsKey(target.getFiducialId())) {
                visibleTagIDs[indx] = target.getFiducialId();
                indx++;
            }
        }

        return Arrays.copyOf(visibleTagIDs, indx);
    }

    private double[] getDistancesFromTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final double[] distances = new double[targets.size()];

        for (int i = 0; i < targets.size(); i++)
            distances[i] = getDistanceFromTarget(targets.get(i));

        return distances;
    }

    private double getDistanceFromTarget(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getNorm();
    }
}
