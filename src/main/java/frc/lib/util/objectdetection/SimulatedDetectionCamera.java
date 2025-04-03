package frc.lib.util.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static frc.lib.math.MathUtils.getAngleFromPoseToPose;
import static frc.lib.math.MathUtils.getPitchFromPoseToPose;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class SimulatedDetectionCamera extends DetectionCameraIO {
    private static final Rotation2d HORIZONTAL_FOV = Rotation2d.fromDegrees(75);
    private static final double
            MAXIMUM_DISTANCE_METERS = 5,
            MINIMUM_DISTANCE_METERS = 0.05;

    private static final Translation2d[] NOTES_ON_FIELD = new Translation2d[]{
            new Translation2d(2.9, 7),
            new Translation2d(2.9, 5.5),
            new Translation2d(2.9, 4.1),
            new Translation2d(8.3, 7.45),
            new Translation2d(8.3, 5.75),
            new Translation2d(8.3, 4.1),
            new Translation2d(8.3, 2.45),
            new Translation2d(8.3, 0.75),
            new Translation2d(13.65, 7),
            new Translation2d(13.65, 5.5),
            new Translation2d(13.65, 4.1)
    };

    private final String name;
    private final Transform3d robotToCamera;

    public SimulatedDetectionCamera(String name, Transform3d robotToCamera) {
        super(name);

        this.name = name;
        this.robotToCamera = robotToCamera;
    }

    @Override
    protected void refreshInputs(DetectionCameraInputsAutoLogged inputs) {
        if (robotToCamera == null) return;

        final double[] closestObjectValues = getClosestVisibleObjectYaw(POSE_ESTIMATOR.getCurrentPose());

        if (closestObjectValues[0] != -10069 && closestObjectValues[1] != -10069) {
            inputs.closestTargetYaw = -closestObjectValues[0];
            inputs.closestTargetPitch = closestObjectValues[1];
            inputs.yaws = new double[]{inputs.closestTargetYaw};
        }
    }

    private double[] getClosestVisibleObjectYaw(Pose2d robotPose) {
        Rotation2d closestObjectYaw = null;
        double closestDistance = Double.POSITIVE_INFINITY;
        double closestPitch = 0;

        final Pose3d cameraPose = new Pose3d(robotPose).transformBy(robotToCamera);

        for (Translation2d objectPlacement : NOTES_ON_FIELD) {
            final Rotation2d angleToObject = getAngleFromPoseToPose(objectPlacement, cameraPose.toPose2d().getTranslation());

            if (!isWithinHorizontalFOV(angleToObject, cameraPose.toPose2d()) || !isWithinDistance(objectPlacement, cameraPose.toPose2d()))
                continue;

            final double distance = getObjectDistance(objectPlacement, robotPose);

            if (distance < closestDistance) {
                closestObjectYaw = angleToObject.minus(robotPose.getRotation());

                 final Pose3d objectPose = new Pose3d(
                        new Translation3d(objectPlacement.getX(), objectPlacement.getY(), 0),
                        new Rotation3d()
                );

                closestPitch = getPitchFromPoseToPose(objectPose, cameraPose).getDegrees();
                closestDistance = distance;
            }
        }

        if (closestObjectYaw == null) return new double[]{-10069, -10069};

        return new double[]{closestObjectYaw.getDegrees(), closestPitch};
    }

    private boolean isWithinHorizontalFOV(Rotation2d objectYaw, Pose2d cameraPose) {
        return Math.abs(objectYaw.minus(cameraPose.getRotation()).getRadians()) <= HORIZONTAL_FOV.getRadians() / 2;
    }

    private boolean isWithinDistance(Translation2d objectPlacement, Pose2d cameraPose) {
        final double distance = getObjectDistance(objectPlacement, cameraPose);
        return distance <= MAXIMUM_DISTANCE_METERS && distance >= MINIMUM_DISTANCE_METERS;
    }

    private double getObjectDistance(Translation2d objectPlacement, Pose2d cameraPose) {
        final Translation2d difference = objectPlacement.minus(cameraPose.getTranslation());
        return difference.getNorm();
    }
}
