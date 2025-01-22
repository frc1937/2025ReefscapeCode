package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.flippable.Flippable;
import frc.robot.poseestimation.photoncamera.CameraFactory;
import frc.robot.poseestimation.photoncamera.PhotonCameraIO;

import java.util.HashMap;
import java.util.Map;

import static frc.lib.util.flippable.FlippableUtils.flipAboutYAxis;

public class PoseEstimatorConstants {
    static final double POSE_BUFFER_SIZE_SECONDS = 2;
    static final StandardDeviations ODOMETRY_STANDARD_DEVIATIONS = new StandardDeviations(Math.pow(0.003, 2), Math.pow(0.0002, 2));

    static final Pose2d EMPTY_POSE = Flippable.isRedAlliance() ? flipAboutYAxis(new Pose2d()) : new Pose2d();

    static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
            0.27, 0.37, 0.19,
            new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(30))
    ), ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
            -0.27, 0.37, 0.19,
            new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(150))
    ), ROBOT_TO_REAR_LEFT_CAMERA = new Transform3d(
            0.27, -0.37, 0.19,
            new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(-30))
    ), ROBOT_TO_REAR_RIGHT_CAMERA = new Transform3d(
            -0.27, -0.37, 0.19,
            new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(210))
    );


    public static final PhotonCameraIO FRONT_LEFT_CAMERA = CameraFactory.generateCamera("FrontLeft1937", ROBOT_TO_FRONT_LEFT_CAMERA),
            FRONT_RIGHT_CAMERA = CameraFactory.generateCamera("FrontRight1937", ROBOT_TO_FRONT_RIGHT_CAMERA),
            REAR_LEFT_CAMERA = CameraFactory.generateCamera("RearLeft1937", ROBOT_TO_REAR_LEFT_CAMERA),
            REAR_RIGHT_CAMERA = CameraFactory.generateCamera("RearRight1937", ROBOT_TO_REAR_RIGHT_CAMERA);

    public static final double TRANSLATION_STD_EXPONENT = 0.05;
    public static final double ROTATION_STD_EXPONENT = 0.01;

    public static final double MAXIMUM_AMBIGUITY = 0.4;

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Map<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();

    static {
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
    }
}

