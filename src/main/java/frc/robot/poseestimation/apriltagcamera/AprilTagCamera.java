package frc.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.poseestimation.poseestimator.StandardDeviations;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.TAG_ID_TO_POSE;

/**
 * An april tag camera is a class that provides the robot's pose from a camera using one or multiple apriltags.
 * An april tag is like a 2d QR-code used to find the robot's position on the field.
 * Since the tag's position on the field is known, we can calculate our position relative to it, therefore estimating our position on the field.
 */
public class AprilTagCamera {
    protected final String name;
    private final AprilTagCameraInputsAutoLogged inputs = new AprilTagCameraInputsAutoLogged();
    private final Transform2d cameraToRobotCenter;
    private final StandardDeviations standardDeviations;
    private final AprilTagCameraIO aprilTagCameraIO;
    private Pose2d estimatedRobotPose = new Pose2d();

    /**
     * Constructs a new AprilTagCamera.
     *
     * @param name                the camera's name
     * @param robotCenterToCamera the transform of the robot's origin point to the camera.
     *                            only the x, y and yaw values will be used for transforming the camera pose to the robot's center,
     *                            to avoid more inaccuracies like pitch and roll.
     *                            The reset will be used for creating a camera in simulation
     * @param standardDeviations  the initial calibrated standard deviations for the camera's estimated pose,
     *                            will be changed as the distance from the tag(s) changes and the number of tags changes
     */
    public AprilTagCamera(String name, Transform3d robotCenterToCamera,
                          StandardDeviations standardDeviations) {
        this.name = name;
        this.standardDeviations = standardDeviations;
        this.cameraToRobotCenter = toTransform2d(robotCenterToCamera).inverse();

        aprilTagCameraIO = AprilTagCameraIO.createCamera(name, robotCenterToCamera);
    }

    public void update() {
        aprilTagCameraIO.refreshInputs(inputs);

        estimatedRobotPose = calculateRobotPose();
        logCameraInfo();
    }

    public Pose2d getEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public String getName() {
        return name;
    }

    public double getLatestResultTimestampSeconds() {
        return inputs.latestResultTimestampSeconds;
    }

    public boolean hasValidResult() {
        return inputs.hasResult && inputs.poseAmbiguity < AprilTagCameraConstants.MAXIMUM_AMBIGUITY && inputs.distancesFromTags[0] < 1.9;
    }

    /**
     * Calculates the range of how inaccurate the estimated pose could be using the distance from the target, the number of targets, and a calibrated gain.
     *
     * @return the standard deviations of the current estimated pose
     */
    public StandardDeviations calculateStandardDeviations() {
        final double averageDistanceFromTags = calculateAverageDistanceFromTags();
        final double translationStandardDeviation = calculateStandardDeviation(standardDeviations.translationStandardDeviation(), averageDistanceFromTags, inputs.visibleTagIDs.length);
        final double thetaStandardDeviation = calculateStandardDeviation(standardDeviations.thetaStandardDeviation(), averageDistanceFromTags, inputs.visibleTagIDs.length);

        Logger.recordOutput("StandardDeviations/" + name + "/translations", translationStandardDeviation);
        Logger.recordOutput("StandardDeviations/" + name + "/theta", thetaStandardDeviation);

        return new StandardDeviations(translationStandardDeviation, thetaStandardDeviation);
    }

    /**
     * Calculates an aspect of the standard deviations of the estimated pose using a formula.
     * As we get further from the tag(s), this will return a less trusting (higher deviation) result.
     *
     * @param exponent            a calibrated gain
     * @param distance            the distance from the tag(s)
     * @param numberOfVisibleTags the number of visible tags
     * @return the standard deviation
     */
    private double calculateStandardDeviation(double exponent, double distance, int numberOfVisibleTags) {
        return exponent * (distance * distance) / numberOfVisibleTags;
    }

    private Pose2d calculateRobotPose() {
        if (!hasValidResult())
            return null;

        return chooseBestNormalSolvePNPPose();
    }

    private Pose2d chooseBestNormalSolvePNPPose() {
        if (inputs.bestCameraSolvePNPPose.equals(inputs.alternateCameraSolvePNPPose))
            return cameraPoseToRobotPose(inputs.bestCameraSolvePNPPose.toPose2d());

        final Pose2d bestPose = cameraPoseToRobotPose(inputs.bestCameraSolvePNPPose.toPose2d());

        if (inputs.bestCameraSolvePNPPose.equals(inputs.alternateCameraSolvePNPPose))
            return bestPose;
        if (inputs.alternateCameraSolvePNPPose.getTranslation().toTranslation2d().getDistance(TAG_ID_TO_POSE.get(inputs.visibleTagIDs[0]).getTranslation().toTranslation2d()) < 0.1 || DriverStation.isDisabled())
            return bestPose;

        final Pose2d poseAtTime = POSE_ESTIMATOR.getEstimatedPoseAtTimestamp(inputs.latestResultTimestampSeconds);

        if (poseAtTime == null) return null;

        final Pose2d alternatePose = cameraPoseToRobotPose(inputs.alternateCameraSolvePNPPose.toPose2d());
        final Rotation2d robotAngleAtResultTime = poseAtTime.getRotation();

        final double bestAngleDifference = Math.abs(bestPose.getRotation().minus(robotAngleAtResultTime).getRadians());
        final double alternateAngleDifference = Math.abs(alternatePose.getRotation().minus(robotAngleAtResultTime).getRadians());

        return bestAngleDifference > alternateAngleDifference ? alternatePose : bestPose;
    }

    private Pose2d cameraPoseToRobotPose(Pose2d cameraPose) {
        return cameraPose;
    }

    private double calculateAverageDistanceFromTags() {
        double totalDistance = 0;
        for (int visibleTagID : inputs.visibleTagIDs) {
            totalDistance += TAG_ID_TO_POSE.get(visibleTagID).getTranslation().getDistance(inputs.bestCameraSolvePNPPose.getTranslation());
        }
        return totalDistance / inputs.visibleTagIDs.length;
    }

    private void logCameraInfo() {
        Logger.processInputs("Cameras/" + name, inputs);
        if (!TAG_ID_TO_POSE.isEmpty())
            logUsedTags();

        if (hasValidResult()) {
            Logger.recordOutput("Poses/Robot/Cameras/" + name + "Pose", estimatedRobotPose);
            return;
        }

        Logger.recordOutput("Poses/Robot/Cameras/" + name + "Pose", Pose2d.kZero);
    }

    private void logUsedTags() {
        if (!hasValidResult()) {
            Logger.recordOutput("UsedTags/" + this.getName(), new Pose3d[0]);
            return;
        }

        final Pose3d[] usedTagPoses =  new Pose3d[inputs.visibleTagIDs.length];
        for (int i = 0; i < usedTagPoses.length; i++)
            usedTagPoses[i] = TAG_ID_TO_POSE.get(inputs.visibleTagIDs[i]);
        Logger.recordOutput("UsedTags/" + this.getName(), usedTagPoses);
    }

    private Transform2d toTransform2d(Transform3d transform3d) {
        final Translation2d robotCenterToCameraTranslation = transform3d.getTranslation().toTranslation2d();
        final Rotation2d robotCenterToCameraRotation = transform3d.getRotation().toRotation2d();

        return new Transform2d(robotCenterToCameraTranslation, robotCenterToCameraRotation);
    }
}