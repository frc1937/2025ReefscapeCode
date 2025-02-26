package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.Set;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.FieldConstants.*;
import static frc.robot.utilities.FieldConstants.ReefFace.*;

public class PathfindingCommands {
    public static DeferredCommand pathfindToBranch(PathfindingConstants.Branch branch) {
        return new DeferredCommand(
                () -> SwerveCommands
                                .goToPoseTrapezoidal(branch.getBranchPose(), 0.01, 0.2)
                                .andThen(SwerveCommands.goToPosePID(branch.getBranchPose())),

                Set.of(SWERVE)
        );
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

    public static DeferredCommand pathfindToFeederBezier(Feeder feeder) {
        return new DeferredCommand(() -> {
            final Pose2d targetPose = feeder.getPose();

            return SwerveCommands.goToPoseBezier(targetPose)
                    .andThen(SwerveCommands.goToPosePID(targetPose))
                    .andThen(new WaitCommand(0.05));
        }, Set.of(SWERVE));
    }

    public static DeferredCommand pathfindToBranchBezier(PathfindingConstants.Branch branch, ReefFace face) {
        return new DeferredCommand(() -> {
            final Pose2d targetPose = branch.getBranchPose(face);

            return SwerveCommands.goToPoseBezier(targetPose)
                    .andThen(SwerveCommands.goToPosePID(targetPose))
                    .andThen(new WaitCommand(0.05));
        }, Set.of(SWERVE));
    }

    protected static ReefFace decideReefFace() {
        final Translation2d robotPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Translation2d distanceToReef = REEF_CENTER.get().minus(robotPose);

        final double angle = Math.toDegrees(Math.atan2(distanceToReef.getY(), distanceToReef.getX()));

        if (150 <= angle || angle < -150) return FACE_3.getAllianceCorrectedFace();
        if (-30 <= angle && angle < 30) return FACE_0.getAllianceCorrectedFace();
        if (90 <= angle) return  FACE_4.getAllianceCorrectedFace();
        if (-90 <= angle && angle < -30) return FACE_1.getAllianceCorrectedFace();
        if (-150 <= angle && angle < -90) return FACE_2.getAllianceCorrectedFace();

        return FACE_5.getAllianceCorrectedFace();
    }

    private static Pose2d decideFeederPose() {
        Pose2d originalPose = Feeder.TOP_FEEDER.getPose();

        if (POSE_ESTIMATOR.getCurrentPose().getY() - FIELD_WIDTH / 2 < 0)
            originalPose = Feeder.BOTTOM_FEEDER.getPose();

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