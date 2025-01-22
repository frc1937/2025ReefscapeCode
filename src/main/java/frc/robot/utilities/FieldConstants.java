package frc.robot.utilities;

import edu.wpi.first.math.geometry.*;
import frc.lib.util.flippable.Flippable;

import static frc.lib.util.flippable.FlippableUtils.flipAboutBothAxis;

public class FieldConstants {
    private static final Transform2d ROBOT_TRANSFORM = new Transform2d(new Translation2d(0.5, 0), Rotation2d.fromDegrees(180));

    public static final Pose2d BLUE_BOTTOM_FEEDER_INTAKE_POSE = new Pose2d(new Translation2d(0.84319, 0.65078),
                    Rotation2d.fromDegrees(54)).transformBy(ROBOT_TRANSFORM);

    public static final double FIELD_WIDTH = 8.05;
    public static final double FIELD_LENGTH = 17.55;

    public static final Translation2d REEF_CENTER = new Translation2d(4.490, 4.027);

    public enum ReefFace {
        FACE_0(3.6587, 4.0269, Rotation2d.k180deg),
        FACE_1(4.0745, 4.7472, Rotation2d.fromDegrees(120)),
        FACE_2(4.9052, 4.7472, Rotation2d.fromDegrees(60)),
        FACE_3(5.3210, 4.0259, Rotation2d.kZero),
        FACE_4(4.9053, 3.3067, Rotation2d.fromDegrees(-60)),
        FACE_5(4.0746, 3.3064, Rotation2d.fromDegrees(-120));

        private final Pose2d facePose;
        private final Pose2d leftBranch;
        private final Pose2d rightBranch;

        ReefFace(double x, double y, Rotation2d rotation) {
            this.facePose = new Pose2d(x, y, rotation);

            final Pose2d faceDirection = new Pose2d(REEF_CENTER, facePose.getRotation());
            this.leftBranch = faceDirection.transformBy(new Transform2d(0.7808, -0.1643, Rotation2d.kZero)).transformBy(ROBOT_TRANSFORM);
            this.rightBranch = faceDirection.transformBy(new Transform2d(0.7808, 0.1643, Rotation2d.kZero)).transformBy(ROBOT_TRANSFORM);
        }

        public Pose2d getPose() {
            return Flippable.isRedAlliance() ? flipAboutBothAxis(facePose) : facePose;
        }

        public Pose2d getLeftBranch() {
            return Flippable.isRedAlliance() ? flipAboutBothAxis(leftBranch) : leftBranch;
        }

        public Pose2d getRightBranch() {
            return Flippable.isRedAlliance() ? flipAboutBothAxis(rightBranch) : rightBranch;
        }
    }
}