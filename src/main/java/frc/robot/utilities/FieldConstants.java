package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.flippable.FlippablePose2d;
import frc.lib.util.flippable.FlippableTranslation2d;

import static frc.lib.util.flippable.FlippableUtils.flipAboutXAxis;

public class FieldConstants {
    public enum ReefFace {
        FACE_0(3.6587, 4.0269, Rotation2d.k180deg),
        FACE_1(4.0745, 4.7472, Rotation2d.fromDegrees(120)),
        FACE_2(4.9052, 4.7472, Rotation2d.fromDegrees(60)),
        FACE_3(5.3210, 4.0259, Rotation2d.kZero),
        FACE_4(4.9053, 3.3067, Rotation2d.fromDegrees(-60)),
        FACE_5(4.0746, 3.3064, Rotation2d.fromDegrees(-120));

        private final FlippablePose2d
                facePose,
                leftBranch,
                rightBranch;

        ReefFace(double x, double y, Rotation2d rotation) {
            this.facePose = new FlippablePose2d(new Pose2d(x, y, rotation).transformBy(ROBOT_TRANSFORM), true);

            final FlippablePose2d faceDirection = new FlippablePose2d(REEF_CENTER.get(), facePose.getRotation().get(), true);
            this.leftBranch = new FlippablePose2d(faceDirection.get().transformBy(LEFT_BRANCH_TRANSFORM).transformBy(ROBOT_REEF_TRANSFORM), true);
            this.rightBranch = new FlippablePose2d(faceDirection.get().transformBy(RIGHT_BRANCH_TRANSFORM).transformBy(ROBOT_REEF_TRANSFORM), true);
        }

        public Pose2d getPose() {
            return facePose.get();
        }

        public ReefFace getOpposite() {
            return switch (this) {
                case FACE_0 -> FACE_3;
                case FACE_1 -> FACE_4;
                case FACE_2 -> FACE_5;
                case FACE_3 -> FACE_0;
                case FACE_4 -> FACE_1;
                case FACE_5 -> FACE_2;
            };
        }

        public Pose2d getLeftBranch() {
            return leftBranch.get();
        }

        public Pose2d getRightBranch() {
            return rightBranch.get();
        }
    }

    public enum Feeder {
        TOP_FEEDER(new Pose2d(0.84319, 0.65078, Rotation2d.fromDegrees(54))),
        BOTTOM_FEEDER(flipAboutXAxis(new Pose2d(0.84319, 0.65078, Rotation2d.fromDegrees(54))));

        private final FlippablePose2d feederPose;

        Feeder(Pose2d pose) {
            this.feederPose = new FlippablePose2d(pose.transformBy(ROBOT_TRANSFORM), true);
        }

        public Pose2d getPose() {
            return feederPose.get();
        }
    }

    private static final Transform2d
            ROBOT_TRANSFORM = new Transform2d(new Translation2d(0.4, 0), Rotation2d.fromDegrees(180)),
            ROBOT_REEF_TRANSFORM = new Transform2d(new Translation2d(0.4, -0.25), Rotation2d.fromDegrees(90));

    private static final Transform2d
            LEFT_BRANCH_TRANSFORM = new Transform2d(0.7808, -0.1643, Rotation2d.kZero),
            RIGHT_BRANCH_TRANSFORM = new Transform2d(0.7808, 0.1643, Rotation2d.kZero);

    public static final double
            FIELD_WIDTH = 8.05,
            FIELD_LENGTH = 17.55;

    public static final FlippablePose2d
            FAR_CAGE = new FlippablePose2d(8.7738712, 7.2841866, Rotation2d.k180deg, true),
            MIDDLE_CAGE = new FlippablePose2d(8.7738712, 6.168517, Rotation2d.k180deg, true),
            CLOSE_CAGE = new FlippablePose2d(8.7738712, 5.0786538, Rotation2d.k180deg, true);

    public static final FlippableTranslation2d REEF_CENTER = new FlippableTranslation2d(4.490, 4.027, true);
}