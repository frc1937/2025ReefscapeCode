package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.HashMap;
import java.util.List;

public class PoseEstimatorConstants {
    static final StandardDeviations ODOMETRY_STANDARD_DEVIATIONS
            = new StandardDeviations(0.003, 0.0002);

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

