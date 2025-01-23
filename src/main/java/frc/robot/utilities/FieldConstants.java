package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.flippable.FlippablePose2d;
import frc.lib.util.flippable.FlippableTranslation2d;

public class FieldConstants {
    private static final Transform2d ROBOT_TRANSFORM = new Transform2d(new Translation2d(0.5, 0), Rotation2d.fromDegrees(180));

    private final static Transform2d LEFT_BRANCH_TRANSFORM = new Transform2d(0.7808, -0.1643, Rotation2d.kZero);
    private final static Transform2d RIGHT_BRANCH_TRANSFORM = new Transform2d(0.7808, 0.1643, Rotation2d.kZero);

    public static final Pose2d BLUE_BOTTOM_FEEDER_INTAKE_POSE = new Pose2d(new Translation2d(0.84319, 0.65078),
                    Rotation2d.fromDegrees(54)).transformBy(ROBOT_TRANSFORM);

    public static final double FIELD_WIDTH = 8.05;
    public static final double FIELD_LENGTH = 17.55;

    public static final Translation2d REEF_CENTER = new FlippableTranslation2d(4.490, 4.027, true).get();

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
            this.facePose = new FlippablePose2d(x, y, rotation,true).get();

            final Pose2d faceDirection = new Pose2d(REEF_CENTER, facePose.getRotation());
            this.leftBranch = new FlippablePose2d(faceDirection.transformBy(LEFT_BRANCH_TRANSFORM).transformBy(ROBOT_TRANSFORM), true).get();
            this.rightBranch = new FlippablePose2d(faceDirection.transformBy(RIGHT_BRANCH_TRANSFORM).transformBy(ROBOT_TRANSFORM), true).get();
        }

        public Pose2d getPose() {
            return facePose;
        }

        public Pose2d getLeftBranch() {
            return leftBranch;
        }

        public Pose2d getRightBranch() {
            return rightBranch;
        }
    }
}