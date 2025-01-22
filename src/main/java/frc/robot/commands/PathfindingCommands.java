package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import frc.robot.subsystems.swerve.SwerveCommands;

import java.util.Set;

import static frc.lib.util.flippable.FlippableUtils.flipAboutXAxis;
import static frc.lib.util.flippable.FlippableUtils.flipAboutYAxis;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.utilities.FieldConstants.BLUE_BOTTOM_FEEDER_INTAKE_POSE;
import static frc.robot.utilities.FieldConstants.FIELD_WIDTH;

public class PathfindingCommands {
    public static void setupFeederPathfinding(Trigger button) {
        button.whileTrue(
                Commands.defer(PathfindingCommands::pathfindToFeeder, Set.of(SWERVE))
        );
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