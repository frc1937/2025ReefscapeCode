package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.commands.InitExecuteCommand;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.MODULES;
import static frc.robot.utilities.PathPlannerConstants.PATHPLANNER_CONSTRAINTS;

public class SwerveCommands {
    public static Command stopDriving() {
        return new InstantCommand(SWERVE::stop);
    }

    public static Command lockSwerve() {
        return Commands.run(
                () -> {
                    final SwerveModuleState
                            right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                            left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

                    MODULES[0].setTargetState(left);
                    MODULES[1].setTargetState(right);
                    MODULES[2].setTargetState(right);
                    MODULES[3].setTargetState(left);
                },
                SWERVE
        );
    }

    public static Command goToPoseBezier(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(targetPose, PATHPLANNER_CONSTRAINTS);
    }

    public static Command goToPosePID(Pose2d targetPose) {
        final Pose2d rotationCorrectedPose = new Pose2d(targetPose.getTranslation(), Rotation2d.fromDegrees(MathUtil.inputModulus(targetPose.getRotation().getDegrees(), -180, 180)));

        return new FunctionalCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.driveToPose(rotationCorrectedPose),
                interrupt -> {},
                () -> false,
                SWERVE
        );
    }

    public static Command goToPosePIDWithConstraints(Pose2d targetPose, PIDConstants constraints) {
        final Pose2d rotationCorrectedPose = new Pose2d(targetPose.getTranslation(), Rotation2d.fromDegrees(MathUtil.inputModulus(targetPose.getRotation().getDegrees(), -180, 180)));

        return new FunctionalCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.driveToPoseWithConstraints(rotationCorrectedPose, constraints),
                interrupt -> {},
                () -> false,
                SWERVE
        );
    }

    public static Command goToPoseTrapezoidal(Pose2d targetPose) {
        final Pose2d rotationCorrectedPose = new Pose2d(targetPose.getTranslation(), Rotation2d.fromDegrees(MathUtil.inputModulus(targetPose.getRotation().getDegrees(), -180, 180)));

        return new FunctionalCommand(
                () -> {
                    PROFILED_TRANSLATION_CONTROLLER.reset(POSE_ESTIMATOR.getCurrentPose().getX(), SWERVE.getFieldRelativeVelocity().vxMetersPerSecond);
                    PROFILED_STRAFE_CONTROLLER.reset(POSE_ESTIMATOR.getCurrentPose().getY(), SWERVE.getFieldRelativeVelocity().vyMetersPerSecond);

                    PROFILED_TRANSLATION_CONTROLLER.setGoal(targetPose.getX());
                    PROFILED_STRAFE_CONTROLLER.setGoal(targetPose.getY());

                    SWERVE.initializeDrive(true);
                },
                () -> SWERVE.driveToPoseTrapezoidal(rotationCorrectedPose),
                interrupt -> {},
                () -> PROFILED_TRANSLATION_CONTROLLER.atGoal() && PROFILED_STRAFE_CONTROLLER.atGoal() && SWERVE_ROTATION_CONTROLLER.atGoal(),
                SWERVE
        );
    }

    public static Command resetGyro() {
        return Commands.runOnce(() -> SWERVE.setGyroHeading(Rotation2d.fromDegrees(0)), SWERVE);
    }

    public static Command driveOpenLoop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, BooleanSupplier robotCentric) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.driveOrientationBased(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble(), robotCentric.getAsBoolean()),
                SWERVE
        );
    }

    public static Command driveWhilstRotatingToTarget(DoubleSupplier x, DoubleSupplier y, Pose2d target, BooleanSupplier robotCentric) {
        return new FunctionalCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.driveWithTarget(x.getAsDouble(), y.getAsDouble(), target, robotCentric.getAsBoolean()),
                interrupt -> {
                },
                () -> false,
                SWERVE
        );
    }

    public static Command rotateToTarget(Pose2d target) {
        return new FunctionalCommand(
                () -> SWERVE.initializeDrive(true),
                () -> SWERVE.driveWithTarget(0, 0, target, false),
                interrupt -> {
                },
                SWERVE_ROTATION_CONTROLLER::atGoal,
                SWERVE
        ).withTimeout(3);
    }
}
