package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.Set;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.FieldConstants.*;
import static frc.robot.utilities.FieldConstants.ReefFace.*;

public class PathfindingCommands {
    public static boolean IS_ALIGNING_REEF = false;

    public static DeferredCommand pathfindToBranch(PathfindingConstants.Branch branch, Transform2d offset) {
        return new DeferredCommand(
                () -> {
                    IS_ALIGNING_REEF = true;

                    return SwerveCommands.goToPosePID(branch.getBranchPose().transformBy(offset))
                            .andThen(new InstantCommand(() -> IS_ALIGNING_REEF = false));
                },

                Set.of(SWERVE)
        );
    }

    public static DeferredCommand pathfindToBranch(PathfindingConstants.Branch branch) {
        return new DeferredCommand(
                () -> {
                    IS_ALIGNING_REEF = true;

                    return SwerveCommands.goToPosePID(branch.getBranchPose())
                            .andThen(new InstantCommand(() -> IS_ALIGNING_REEF = false));
                },

                Set.of(SWERVE)
        );
    }

    public static DeferredCommand pathfindToBranchBezier(PathfindingConstants.Branch branch, ReefFace face, Transform2d offset) {
        return new DeferredCommand(() -> {
            IS_ALIGNING_REEF = true;

            final Pose2d targetPose = branch.getBranchPose(face).transformBy(offset);

            return SwerveCommands.goToPoseBezier(targetPose)
                    .until(() -> SWERVE.isAtPose(targetPose, 0.3, 2))
                    .andThen(SwerveCommands.goToPosePID(targetPose)
                            .onlyIf(() -> !SWERVE.isAtPose(targetPose, 0.08, 1))
                    )
//                    .andThen(SwerveCommands.driveWithTimeout(0.1,0,0,true,0.2))
                    .andThen(new InstantCommand(() -> IS_ALIGNING_REEF = false));
        }, Set.of(SWERVE));
    }

    public static DeferredCommand pathfindToFeederBezier(Feeder feeder) {
        return new DeferredCommand(() -> {
            final Pose2d targetPose = feeder.getPose();

            return SwerveCommands.goToPoseBezier(targetPose)
                    .withTimeout(2.5)
                    .andThen(SwerveCommands.goToPosePID(feeder.getPose())).withTimeout(3);
//                    .andThen(SwerveCommands.driveWithTimeout(0,-0.1,0,true,0.2));
        }, Set.of(SWERVE));
    }

    public static DeferredCommand pathfindToFeeder() {
        return new DeferredCommand(
                () -> SwerveCommands.goToPoseTrapezoidal(decideFeederPose(), 0.1, 0.5),
                Set.of(SWERVE)
        );
    }

    public static DeferredCommand pathfindToCage() {
        return new DeferredCommand(() -> {
            final Pose2d targetPose = decideCagePose();
            final Pose2d intermediateTarget = new Pose2d(POSE_ESTIMATOR.getCurrentPose().getX(), targetPose.getY(), targetPose.getRotation());

            final Command alignWithTargetY = SwerveCommands.goToPoseTrapezoidal(intermediateTarget, 0.1, 0.1);

            return alignWithTargetY
                    .andThen(SwerveCommands.goToPoseTrapezoidal(targetPose, 0.07, 0.01));
        }, Set.of(SWERVE));
    }

    public static ReefFace decideReefFace() {
        final Translation2d robotPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Translation2d distanceToReef = REEF_CENTER.get().minus(robotPose);

        final double angle = Math.toDegrees(Math.atan2(distanceToReef.getY(), distanceToReef.getX()));

        if (angle <= -150 || angle >= 150) return FACE_3.getAllianceCorrectedFace();
        if (angle <= -90) return FACE_2.getAllianceCorrectedFace();
        if (angle <= -30) return FACE_1.getAllianceCorrectedFace();
        if (angle <= 30) return FACE_0.getAllianceCorrectedFace();
        if (angle <= 90) return FACE_5.getAllianceCorrectedFace();

        return FACE_4.getAllianceCorrectedFace();
    }

    public static Pose2d decideFeederPose() {
        Pose2d originalPose = Feeder.BLUES_LEFT_REDS_RIGHT_FEEDER.getPose();

        if (POSE_ESTIMATOR.getCurrentPose().getY() - FIELD_WIDTH / 2 < 0)
            originalPose = Feeder.BLUES_RIGHT_REDS_LEFT_FEEDER.getPose();

        return originalPose;
    }

    private static Pose2d decideCagePose() {
        final Pose2d robotPose = POSE_ESTIMATOR.getCurrentPose();

        final double closeCageDistanceY = Math.abs(robotPose.getY() - CLOSE_CAGE.get().getY()),
                middleCageDistanceY = Math.abs(robotPose.getY() - MIDDLE_CAGE.get().getY()),
                farCageDistanceY = Math.abs(robotPose.getY() - FAR_CAGE.get().getY());

        if (closeCageDistanceY < farCageDistanceY && closeCageDistanceY < middleCageDistanceY)
            return CLOSE_CAGE.get();
        else if (middleCageDistanceY < farCageDistanceY)
            return MIDDLE_CAGE.get();

        return FAR_CAGE.get();
    }
}