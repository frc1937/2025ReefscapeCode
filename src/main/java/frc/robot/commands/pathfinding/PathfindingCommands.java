package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.util.flippable.Flippable;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.Set;

import static frc.lib.util.flippable.FlippableUtils.flipAboutXAxis;
import static frc.lib.util.flippable.FlippableUtils.flipAboutYAxis;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.FieldConstants.*;
import static frc.robot.utilities.FieldConstants.ReefFace.*;
import static frc.robot.utilities.PathPlannerConstants.PATHPLANNER_CAGE_CONSTRAINTS;

public class PathfindingCommands {
    private static final double
            PID_PATHFIND_THRESHOLD_REEF = 1.1,
            PID_PATHFIND_THRESHOLD_FEEDER = 0.8;

    public static DeferredCommand pathfindToBranch(PathfindingConstants.BranchOption branch) {
        return new DeferredCommand(() -> SwerveCommands.goToPoseTrapezoidal(branch.getBranchPose()), Set.of(SWERVE));
    }

    public static DeferredCommand pathfindToBranchBezier(PathfindingConstants.BranchOption branch, ReefFace face) {
        return new DeferredCommand(() -> {
            final Pose2d targetPose = branch.getBranchPose(face);

            return isRobotInProximity(targetPose, PID_PATHFIND_THRESHOLD_REEF)
                    ? SwerveCommands.goToPosePID(targetPose).until(SWERVE.isRobotCloseToTarget(targetPose))
                    : SwerveCommands.goToPoseBezier(targetPose);
        }, Set.of(SWERVE));
    }

    public static DeferredCommand pathfindToFeeder() {
        return new DeferredCommand(() -> SwerveCommands.goToPoseTrapezoidal(decideFeederPose()), Set.of(SWERVE));
    }

    public static DeferredCommand pathfindToFeederBezier(Feeder feeder) {
        return new DeferredCommand(() -> {
            final Pose2d targetPose = feeder.getPose();

            return isRobotInProximity(targetPose, PID_PATHFIND_THRESHOLD_FEEDER)
                    ? SwerveCommands.goToPosePID(targetPose).until(SWERVE.isRobotCloseToTarget(targetPose))
                    : SwerveCommands.goToPoseBezier(targetPose);
        }, Set.of(SWERVE));
    }

    public static DeferredCommand pathfindToCage() {
        return new DeferredCommand(() -> {
            final Pose2d targetPose = decideCagePose();

            final Command alignWithTargetY = SwerveCommands.goToPosePIDWithConstraints(
                    new Pose2d(POSE_ESTIMATOR.getCurrentPose().getX(),
                            targetPose.getY(),
                            targetPose.getRotation()
                    ),
                    PATHPLANNER_CAGE_CONSTRAINTS
            );

            return alignWithTargetY
                    .until(SWERVE.isRobotCloseToTarget(targetPose))
                    .andThen(SwerveCommands.goToPosePIDWithConstraints(targetPose, PATHPLANNER_CAGE_CONSTRAINTS))
                    .until(SWERVE.isRobotCloseToTarget(targetPose));
        }, Set.of(SWERVE));
    }

    public static ReefFace decideReefFace() {
        final Translation2d robotPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Translation2d distanceToReef = REEF_CENTER.get().minus(robotPose);

        final double angle = Math.toDegrees(Math.atan2(distanceToReef.getY(), distanceToReef.getX()));
        final boolean isBlueAlliance = !Flippable.isRedAlliance();

        if (angle < -150 || angle >= 150) return isBlueAlliance ? FACE_0 : FACE_0.getOpposite();
        if (angle < -90) return isBlueAlliance ? FACE_5 : FACE_5.getOpposite();
        if (angle < -30) return isBlueAlliance ? FACE_4 : FACE_4.getOpposite();
        if (angle < 30) return isBlueAlliance ? FACE_3 : FACE_3.getOpposite();
        if (angle < 90) return isBlueAlliance ? FACE_2 : FACE_2.getOpposite();

        return isBlueAlliance ? FACE_1 : FACE_1.getOpposite();
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

    private static boolean isRobotInProximity(Pose2d pose2d, double thresholdMetres) {
        return POSE_ESTIMATOR.getCurrentPose().getTranslation().getDistance(pose2d.getTranslation()) < thresholdMetres;
    }
}