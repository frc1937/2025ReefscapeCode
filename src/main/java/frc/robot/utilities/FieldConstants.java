package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.flippable.FlippablePose2d;
import frc.lib.util.flippable.FlippableTranslation2d;

import static frc.lib.util.flippable.Flippable.isRedAlliance;

public class FieldConstants {
    public enum ReefFace {
        FACE_0(Rotation2d.k180deg),
        FACE_1(Rotation2d.fromDegrees(120)),
        FACE_2(Rotation2d.fromDegrees(60)),
        FACE_3(Rotation2d.kZero),
        FACE_4(Rotation2d.fromDegrees(-60)),
        FACE_5(Rotation2d.fromDegrees(-120));

        private final FlippablePose2d rightBranch, leftBranch, centerPose;

        ReefFace(Rotation2d rotation) {
            final FlippablePose2d facePose = new FlippablePose2d(new Pose2d(new Translation2d(), rotation), true);
            final FlippablePose2d faceDirection =  new FlippablePose2d(new Pose2d(REEF_CENTER.get(), facePose.getRotation().get()), true);

            this.centerPose = new FlippablePose2d(faceDirection.get().transformBy(CENTER_POSE_TRANSFORM)
                    .transformBy(ROBOT_REEF_TRANSFORM), true);

            this.leftBranch = new FlippablePose2d(faceDirection.get().transformBy(LEFT_BRANCH_TRANSFORM)
                    .transformBy(ROBOT_REEF_TRANSFORM), true);
            this.rightBranch = new FlippablePose2d(faceDirection.get().transformBy(RIGHT_BRANCH_TRANSFORM)
                    .transformBy(ROBOT_REEF_TRANSFORM), true);
        }

        public ReefFace getAllianceCorrectedFace() {
            return isRedAlliance() ? values()[(ordinal() + 3) % 6] : this;
        }

        public Pose2d getLeftBranch() {
            return leftBranch.get();
        }

        public Pose2d getRightBranch() {
            return rightBranch.get();
        }

        public Pose2d getCenterPose() {
            return centerPose.get();
        }
    }

    public enum Feeder {
        BLUES_LEFT_REDS_RIGHT_FEEDER(
                new Pose2d(0.851, 7.396, Rotation2d.fromDegrees(-54)),
                new Pose2d(16.697, 7.396, Rotation2d.fromDegrees(-126))
        ),
        BLUES_RIGHT_REDS_LEFT_FEEDER(
                new Pose2d(0.851, 0.63605, Rotation2d.fromDegrees(54)),
                new Pose2d(16.697, 0.63605, Rotation2d.fromDegrees(126))
        );

        private final Pose2d blueFeederPose, redFeederPose;

        Feeder(Pose2d blueFeederPose, Pose2d redFeederPose) {
            this.blueFeederPose = blueFeederPose.transformBy(ROBOT_FEEDER_TRANSFORM);
            this.redFeederPose = redFeederPose.transformBy(ROBOT_FEEDER_TRANSFORM);
        }

        /**
         * Get alliance corrected feeder pose.
         *
         * @return alliance corrected feeder pose
         */
        public Pose2d getPose() {
            return isRedAlliance() ? redFeederPose : blueFeederPose;
        }
    }

    private static final Transform2d
            ROBOT_FEEDER_TRANSFORM = new Transform2d(new Translation2d(0.47, 0), Rotation2d.fromDegrees(270)),
            ROBOT_REEF_TRANSFORM = new Transform2d(new Translation2d(0.49, 0), Rotation2d.fromDegrees(180));

    private static final Transform2d
            LEFT_BRANCH_TRANSFORM = new Transform2d(0.7808, -(0.1643+0.1643+0.087), Rotation2d.kZero),
            CENTER_POSE_TRANSFORM = new Transform2d(0.7808, -0.24345, Rotation2d.kZero),
            RIGHT_BRANCH_TRANSFORM = new Transform2d(0.7808, -(0.0943), Rotation2d.kZero);

    public static final double
            FIELD_WIDTH = 8.05,
            FIELD_LENGTH = 17.55;

    public static final FlippablePose2d
            FAR_CAGE = new FlippablePose2d(8.7738712, 7.2841866, Rotation2d.k180deg, true),
            MIDDLE_CAGE = new FlippablePose2d(8.7738712, 6.168517, Rotation2d.k180deg, true),
            CLOSE_CAGE = new FlippablePose2d(8.7738712, 5.0786538, Rotation2d.k180deg, true);

    public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.490, 4.027);

    public static final FlippableTranslation2d REEF_CENTER = new FlippableTranslation2d(BLUE_REEF_CENTER, true);
}