package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.flippable.FlippablePose2d;
import frc.lib.util.flippable.FlippableTranslation2d;

public class FieldConstants {
    private static final Transform2d ROBOT_TRANSFORM = new Transform2d(new Translation2d(0.5, 0), Rotation2d.fromDegrees(180));

    private static final Transform2d LEFT_BRANCH_TRANSFORM = new Transform2d(0.7808, -0.1643, Rotation2d.kZero);
    private static final Transform2d RIGHT_BRANCH_TRANSFORM = new Transform2d(0.7808, 0.1643, Rotation2d.kZero);

    public static final Pose2d BLUE_BOTTOM_FEEDER_INTAKE_POSE = new Pose2d(new Translation2d(0.84319, 0.65078),
                    Rotation2d.fromDegrees(54)).transformBy(ROBOT_TRANSFORM);

    public static final double FIELD_WIDTH = 8.05;
    public static final double FIELD_LENGTH = 17.55;

    public static final FlippablePose2d
            FAR_CAGE = new FlippablePose2d(8.7738712, 7.2841866, Rotation2d.k180deg, true),
            MIDDLE_CAGE = new FlippablePose2d(8.7738712, 6.168517, Rotation2d.k180deg, true),
            CLOSE_CAGE = new FlippablePose2d(8.7738712, 5.0786538, Rotation2d.k180deg, true);

    public static final FlippableTranslation2d REEF_CENTER = new FlippableTranslation2d(4.490, 4.027, true);

    public enum ReefFace {
        FACE_0(3.6587, 4.0269, Rotation2d.k180deg),
        FACE_1(4.0745, 4.7472, Rotation2d.fromDegrees(120)),
        FACE_2(4.9052, 4.7472, Rotation2d.fromDegrees(60)),
        FACE_3(5.3210, 4.0259, Rotation2d.kZero),
        FACE_4(4.9053, 3.3067, Rotation2d.fromDegrees(-60)),
        FACE_5(4.0746, 3.3064, Rotation2d.fromDegrees(-120));

        private final FlippablePose2d facePose;
        private final FlippablePose2d leftBranch;
        private final FlippablePose2d rightBranch;

        ReefFace(double x, double y, Rotation2d rotation) {
            this.facePose = new FlippablePose2d(x, y, rotation, true);

            final FlippablePose2d faceDirection = new FlippablePose2d(REEF_CENTER.get(), facePose.getRotation().get(), true);
            this.leftBranch = new FlippablePose2d(faceDirection.get().transformBy(LEFT_BRANCH_TRANSFORM).transformBy(ROBOT_TRANSFORM), true);
            this.rightBranch = new FlippablePose2d(faceDirection.get().transformBy(RIGHT_BRANCH_TRANSFORM).transformBy(ROBOT_TRANSFORM), true);
        }

        public Pose2d getPose() {
            return facePose.get();
        }

        public Pose2d getLeftBranch() {
            return leftBranch.get();
        }

        public Pose2d getRightBranch() {
            return rightBranch.get();
        }
    }
}