package frc.robot.poseestimation.camera;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.poseestimation.PoseEstimatorConstants.*;

public record EstimateData(Pose3d pose, double timestamp, double distanceFromTag) {
    public static final double FIELD_LENGTH = APRIL_TAG_FIELD_LAYOUT.getFieldLength();
    public static final double FIELD_WIDTH = APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

    public boolean isHighQuality() {
        boolean isValid = isValid();

        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();

        boolean isDstSmall = currentPose.getTranslation().getDistance(pose.toPose2d().getTranslation()) < 0.55;

        boolean isRotSmall = Math.abs(MathUtil.inputModulus(
                pose.getRotation().toRotation2d().minus(currentPose.getRotation()).getDegrees(),
                -180.0,
                180.0)) < 30;

        return isValid && isDstSmall && isRotSmall ;
    }

    public boolean isValid() {
        final boolean isOutsideField = Math.abs(pose.getZ()) > MAX_Z_ERROR
                || pose.getX() < 0.0
                || pose.getX() > FIELD_LENGTH
                || pose.getY() < 0.0
                || pose.getY() > FIELD_WIDTH;

        if (isOutsideField) return false;

        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();

        final double yawError = Math.abs(MathUtil.inputModulus(
                pose.getRotation().toRotation2d().minus(currentPose.getRotation()).getDegrees(),
                -180.0,
                180.0));

        if (yawError > 60.0)
            return false;

        return true;
    }

    public Matrix<N3, N1> getStandardDeviations() {
        final double standardDeviationFactor = Math.pow(distanceFromTag, 2.0);

        final double linearStandardDeviation = VISION_STD_LINEAR * standardDeviationFactor;
        final double angularStandardDeviation = VISION_STD_ANGULAR * standardDeviationFactor;

        return VecBuilder.fill(linearStandardDeviation, linearStandardDeviation, angularStandardDeviation);
    }
}