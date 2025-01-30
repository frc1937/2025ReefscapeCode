package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.Set;

import static frc.lib.util.flippable.FlippableUtils.flipAboutXAxis;
import static frc.lib.util.flippable.FlippableUtils.flipAboutYAxis;
import static frc.robot.RobotContainer.*;
import static frc.robot.utilities.FieldConstants.*;
import static frc.robot.utilities.FieldConstants.ReefFace.*;
import static frc.robot.utilities.PathPlannerConstants.PATHPLANNER_CONSTRAINTS;

public class PathfindingCommands {
    public static void setupFeederPathfinding(Trigger button) {
        button.whileTrue(
                new DeferredCommand(PathfindingCommands::pathfindToFeeder, Set.of(SWERVE))
        );
    }

    private static Command pathfindToFeeder() {
        final Pose2d targetPose = decideFeederPose();

        return isRobotInProximity(targetPose, 0.8) ?
                SwerveCommands.goToPosePID(targetPose) :
                SwerveCommands.goToPoseBezier(targetPose);
    }

    private static Pose2d decideFeederPose() {
        Pose2d originalPose = BLUE_BOTTOM_FEEDER_INTAKE_POSE;

        if (POSE_ESTIMATOR.getCurrentPose().getY() - FIELD_WIDTH / 2 > 0)
            originalPose = flipAboutXAxis(originalPose);

        if (Flippable.isRedAlliance())
            originalPose = flipAboutYAxis(originalPose);

        return originalPose;
    }

    public static void setupReefPathfinding(Trigger firstButton, Trigger secondButton) {
        firstButton.and(secondButton.negate()).whileTrue(
                new DeferredCommand(() -> pathfindToBranch(true), Set.of(SWERVE))
        );

        secondButton.and(firstButton.negate()).whileTrue(
                new DeferredCommand(() -> pathfindToBranch(false), Set.of(SWERVE))
        );
    }

    private static Command pathfindToBranch(boolean isLeftBranch) {
        final Pose2d targetPose = isLeftBranch ? decideReefFace().getLeftBranch() : decideReefFace().getRightBranch();

        return isRobotInProximity(targetPose, 0.8) ?
                SwerveCommands.goToPosePID(targetPose) :
                SwerveCommands.goToPoseBezier(targetPose);
    }

    private static ReefFace decideReefFace() {
        final Translation2d robotPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Translation2d distanceToReef = REEF_CENTER.get().minus(robotPose);

        final double angle = Math.toDegrees(Math.atan2(distanceToReef.getY(), distanceToReef.getX()));

        if (150 <= angle || angle < -150) return Flippable.isRedAlliance() ? FACE_0 : FACE_3;
        if (-30 <= angle && angle < 30) return Flippable.isRedAlliance() ? FACE_3 : FACE_0;
        if (90 <= angle) return Flippable.isRedAlliance() ? FACE_1 : FACE_4;
        if (-90 <= angle && angle < -30) return Flippable.isRedAlliance() ? FACE_4 : FACE_1;
        if (-150 <= angle && angle < -90) return Flippable.isRedAlliance() ? FACE_5 : FACE_2;
        return Flippable.isRedAlliance() ? FACE_2 : FACE_5;
    }

    public static void setupCagePathfinding(Trigger button) {
        button.whileTrue(
                new DeferredCommand((PathfindingCommands::pathfindToCage), Set.of(SWERVE))
        );
    }

    private static Command pathfindToCage() {
        final Pose2d targetPose = decideCagePose();
        final PathConstraints constraints = new PathConstraints(
                1.2,
                1.2,
                1.2 / 0.047,
                1.2
        );

        final Transform2d nextToCageTransform = new Transform2d(1.5, 0, Rotation2d.kZero);
        return AutoBuilder.pathfindToPose(targetPose.transformBy(nextToCageTransform), PATHPLANNER_CONSTRAINTS, 1).andThen(
                AutoBuilder.pathfindToPose(targetPose, constraints).andThen(ELEVATOR.setTargetPosition(ElevatorHeight.CLIMB))
        );
    }

    private static Pose2d decideCagePose() {
        final Pose2d robotPose = POSE_ESTIMATOR.getCurrentPose();

        final double
                closeCageDistanceY = Math.abs(robotPose.getY() - CLOSE_CAGE.get().getY()),
                middleCageDistanceY = Math.abs(robotPose.getY() - MIDDLE_CAGE.get().getY()),
                farCageDistanceY = Math.abs(robotPose.getY() - FAR_CAGE.get().getY());

        if (closeCageDistanceY < farCageDistanceY && closeCageDistanceY < middleCageDistanceY)
            return CLOSE_CAGE.get();
        else if (middleCageDistanceY < farCageDistanceY)
            return MIDDLE_CAGE.get();
        else
            return FAR_CAGE.get();
    }

    private static boolean isRobotInProximity(Pose2d pose2d, double thresholdMetres) {
        return POSE_ESTIMATOR.getCurrentPose().getTranslation().getDistance(pose2d.getTranslation()) < thresholdMetres;
    }
}