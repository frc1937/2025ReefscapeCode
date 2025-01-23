package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import frc.lib.util.flippable.FlippablePose2d;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.Set;

import static frc.lib.util.flippable.FlippableUtils.flipAboutXAxis;
import static frc.lib.util.flippable.FlippableUtils.flipAboutYAxis;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.FieldConstants.*;
import static frc.robot.utilities.FieldConstants.ReefFace.*;

public class PathfindingCommands {
    public static void setupFeederPathfinding(Trigger button) {
        button.whileTrue(
                new DeferredCommand(PathfindingCommands::pathfindToFeeder, Set.of(SWERVE))
        );
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
        final FlippablePose2d reef = new FlippablePose2d(new Pose2d(REEF_CENTER, Rotation2d.kZero), true);
        final Translation2d robotPose = POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final Translation2d distanceToReef = reef.get().getTranslation().minus(robotPose);

        final double angle = Math.toDegrees(Math.atan2(distanceToReef.getY(), distanceToReef.getX()));

        if (-30 <= angle && angle < 30) return Flippable.isRedAlliance() ? FACE_3 : FACE_0;
        else if (-90 <= angle && angle < -30) return Flippable.isRedAlliance() ? FACE_4 : FACE_1;
        else if (-150 <= angle && angle < -90) return Flippable.isRedAlliance() ? FACE_5 : FACE_2;
        else if (150 <= angle || angle < -150) return Flippable.isRedAlliance() ? FACE_0 : FACE_3;
        else if (90 <= angle) return Flippable.isRedAlliance() ? FACE_1 : FACE_4;
        else return Flippable.isRedAlliance() ? FACE_2 : FACE_5;
    }

    private static Command pathfindToFeeder() {
        final Pose2d targetPose = decideFeederPose();

        if (isRobotInProximity(targetPose, 0.8)) {
            return SwerveCommands.goToPosePID(targetPose);
        }

        return SwerveCommands.goToPoseBezier(targetPose);
    }

    private static Pose2d decideFeederPose() {
        Pose2d originalPose = BLUE_BOTTOM_FEEDER_INTAKE_POSE;

        if (POSE_ESTIMATOR.getCurrentPose().getY() - FIELD_WIDTH / 2 > 0)
            originalPose = flipAboutXAxis(originalPose);

        if (Flippable.isRedAlliance())
            originalPose = flipAboutYAxis(originalPose);

        return originalPose;
    }

    private static boolean isRobotInProximity(Pose2d pose2d, double thresholdMetres) {
        return POSE_ESTIMATOR.getCurrentPose().getTranslation().getDistance(pose2d.getTranslation()) < thresholdMetres;
    }
}