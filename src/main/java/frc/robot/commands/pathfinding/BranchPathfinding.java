package frc.robot.commands.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.FieldConstants;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.RobotContainer.*;
import static frc.robot.commands.ConveyorCommands.moveFromIntakeToL4;
import static frc.robot.commands.ConveyorCommands.scoreToL4;
import static frc.robot.commands.CoralManipulationCommands.CURRENT_SCORING_LEVEL;
import static frc.robot.commands.pathfinding.PathfindingCommands.pathfindToBranch;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.BlasterArmState.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L4;
import static frc.robot.utilities.PathPlannerConstants.PATHPLANNER_CONSTRAINTS;

public class BranchPathfinding {
    public static Transform2d L4DistanceFromReef = new Transform2d(-0.53,0, Rotation2d.kZero);

    public static Command pathAndScoreWithOverrideAutonomous(PathfindingConstants.Branch branch,
                                                   DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier,
                                                   Trigger shouldOverride) {

        final Command L4Sequence =
                (SwerveCommands.driveOpenLoop(xSupplier,ySupplier,rotationSupplier, ()-> false)
                        .onlyWhile(shouldOverride)
                        .andThen(getPathToBranch(branch, L4DistanceFromReef, 0.1))
                ).alongWith((moveFromIntakeToL4()
                                        .andThen(
                                                CORAL_INTAKE.compensateForWobblyArm(true)
                                                        .withDeadline(ALGAE_BLASTER.setArmTargetState(SCORE_L4_START)))),
                                (ELEVATOR.setTargetHeight(L4.getMiddlePoint())
                                        .until(() -> ELEVATOR.isAtTargetHeight(L4.getMiddlePoint())))
                        )
                        .until(() -> ALGAE_BLASTER.hasCoralInL4Mechanism() && ALGAE_BLASTER.isAtState(SCORE_L4_START) &&
                                SWERVE.isAtPose(branch.getBranchPose().transformBy(L4DistanceFromReef), 0.2,
                                        3))
                        .andThen(pathfindToBranch(branch).withDeadline(scoreToL4(branch)))
                        .andThen(ALGAE_BLASTER.algaeBlasterFullThrottle(DEFAULT_POSE, -0.1));


        return L4Sequence;
    }

    public static Command pathAndScoreWithOverride(PathfindingConstants.Branch branch,
                                                   DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier,
                                                   Trigger shouldOverride) {

        final Command L4Sequence =
                (SwerveCommands.driveOpenLoop(xSupplier,ySupplier,rotationSupplier, ()-> false)
                .onlyWhile(shouldOverride)
                .andThen(getPathToBranch(branch, L4DistanceFromReef, 0.1))
                ).alongWith((moveFromIntakeToL4()
                                .andThen(
                                    CORAL_INTAKE.compensateForWobblyArm(true)
                                      .withDeadline(ALGAE_BLASTER.setArmTargetState(SCORE_L4_START)))),
                                (ELEVATOR.setTargetHeight(L4.getMiddlePoint())
                                        .until(() -> ELEVATOR.isAtTargetHeight(L4.getMiddlePoint())))
                        )
                        .until(() -> ALGAE_BLASTER.hasCoralInL4Mechanism() && ALGAE_BLASTER.isAtState(SCORE_L4_START) &&
                                SWERVE.isAtPose(branch.getBranchPose().transformBy(L4DistanceFromReef), 0.2,
                                        3))
                        .andThen(pathfindToBranch(branch).withDeadline(scoreToL4(branch)))
                        .andThen(ALGAE_BLASTER.algaeBlasterFullThrottle(DEFAULT_POSE, -0.1));


        final Command L1L2L3Sequence =
                ((SwerveCommands.driveOpenLoop(xSupplier,ySupplier,rotationSupplier, ()-> false).onlyWhile(shouldOverride)
                .andThen((getPathToBranch(branch)
                        .andThen(SwerveCommands.goToPosePID(branch.getBranchPose()))
                        .onlyIf(() -> !SWERVE.isAtPose(branch.getBranchPose(), 0.01, 0.8)))))
                        .alongWith(
                                CORAL_INTAKE.prepareThenTakeBack(),
                                ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL.getMiddlePoint())
                        )
                        .until(() -> SWERVE.isAtPose(branch.getBranchPose(), 0.06, 0.8))
                        .andThen(
                             (ELEVATOR.setTargetHeight(() -> CURRENT_SCORING_LEVEL)
                             .until(() -> ELEVATOR.isAtTargetHeight(CURRENT_SCORING_LEVEL))
                            .andThen(ELEVATOR.maintainPosition()
                                    .withDeadline(CORAL_INTAKE.prepareThenTakeBack().andThen(CORAL_INTAKE.releaseGamePiece()))))))
                        .alongWith(ALGAE_BLASTER.setArmTargetState(HORIZONTAL_IN));


        return new ConditionalCommand(
                L4Sequence,
                L1L2L3Sequence,
                new Trigger(() -> CURRENT_SCORING_LEVEL == ElevatorConstants.ElevatorHeight.L4)
        );
    }

    public static Command getPathToFeeder(FieldConstants.Feeder feeder, double endVelocity) {
        return new DeferredCommand(
                () -> {
                    final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
                    final Pose2d targetPose = feeder.getPose();

                    return followPath(currentPose, targetPose, endVelocity);
                },
                Set.of(SWERVE)
        );
    }

    public static Command getPathToBranch(PathfindingConstants.Branch branch, Transform2d transform2d, double endVelocity) {
        return new DeferredCommand(
                () -> {
                    final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
                    final Pose2d targetPose = branch.getBranchPose().transformBy(transform2d);

                    return followPath(currentPose, targetPose, endVelocity);
                },
                Set.of(SWERVE)
        );
    }

    public static Command getPathToBranch(PathfindingConstants.Branch branch) {
        return getPathToBranch(branch, 0.05);
    }

    public static Command getPathToBranch(PathfindingConstants.Branch branch, double endVelocity) {
        return new DeferredCommand(
                () -> {
                    final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
                    final Pose2d targetPose = branch.getBranchPose();

                    return followPath(currentPose, targetPose, endVelocity);
                },
                Set.of(SWERVE)
        );
    }

    public static Command pathfindToTarget(Supplier<Pose2d> targetPose) {
        return pathfindToTarget(targetPose, 0);
    }

    public static Command pathfindToTarget(Supplier<Pose2d> targetPose, double endVelocity) {
        return new DeferredCommand(
                () -> followPath(POSE_ESTIMATOR.getCurrentPose(), targetPose.get(), endVelocity),
                Set.of(SWERVE)
        );
    }

    private static Command followPath(Pose2d currentPose, Pose2d targetPose, double endVelocity) {
        final ChassisSpeeds currentVelocity = SWERVE.getRobotRelativeVelocity();

        final List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                currentPose,
                targetPose
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.35) {
            return pidToPoseCommand(currentPose, targetPose);
        }

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                PATHPLANNER_CONSTRAINTS,
                new IdealStartingState(getTotalVelocity(currentVelocity), getHeading(currentVelocity, targetPose)),
                new GoalEndState(endVelocity, targetPose.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path)
                .andThen(pidToPoseCommand(currentPose, targetPose));
    }

    private static Command pidToPoseCommand(Pose2d currentPose, Pose2d targetPose) {
//        final PathPlannerTrajectoryState trajectoryGoalState = new PathPlannerTrajectoryState();
//
//        return new FunctionalCommand(
//                () -> {
//                    PATHPLANNER_DRIVE_CONTROLLER.reset(currentPose, SWERVE.getRobotRelativeVelocity());
//                    trajectoryGoalState.pose = targetPose;
//                    trajectoryGoalState.fieldSpeeds = new ChassisSpeeds();
//                },
//                () -> SWERVE.driveRobotRelative(
//                        PATHPLANNER_DRIVE_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, trajectoryGoalState), true),
//                (interrupt) -> SWERVE.stop(),
//                () -> SWERVE.isAtPose(targetPose, 0.1, 0.4),
//                SWERVE
//        );

        return SwerveCommands.goToPosePID(targetPose);
    }

    private static double getTotalVelocity(ChassisSpeeds currentVelocity) {
        return Math.hypot(currentVelocity.vxMetersPerSecond,
                currentVelocity.vyMetersPerSecond);
    }

    private static Rotation2d getHeading(ChassisSpeeds velocity, Pose2d targetPose) {
        if (getTotalVelocity(velocity) >= 0.25) {
            return new Rotation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
        }

        final Translation2d offset = targetPose.getTranslation().minus(POSE_ESTIMATOR.getCurrentPose().getTranslation());
        return (offset.getNorm() < 0.01) ? targetPose.getRotation() : offset.getAngle();
    }
}
