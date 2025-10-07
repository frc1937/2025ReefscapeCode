package frc.robot.poseestimation;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.HashMap;
import java.util.List;

public class PoseEstimatorConstants {
    public static final Matrix<N3, N1> QUEST_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);
    public static final Matrix<N3, N1> ODOMETRY_STD_DEVS = VecBuilder.fill(0.03, 0.03, 0.0002);

    public static final double VISION_STD_LINEAR = 0.018;
    public static final double VISION_STD_ANGULAR = 0.012;

    public static double MAX_Z_ERROR = 0.75;
    public static double MAX_AMBIGUITY = 0.4;

    private static final List<Integer> TAGS_TO_IGNORE = List.of(
            13, 12, 16, 15, 14, 4, 5, 3, 2,1
    );

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    private static HashMap<Integer, Pose3d> fieldLayoutToTagIdToPoseMap() {
        final HashMap<Integer, Pose3d> tagIdToPose = new HashMap<>();

        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags()) {
            if (!TAGS_TO_IGNORE.contains(aprilTag.ID))
                tagIdToPose.put(aprilTag.ID, aprilTag.pose);
        }

        return tagIdToPose;
    }
}

