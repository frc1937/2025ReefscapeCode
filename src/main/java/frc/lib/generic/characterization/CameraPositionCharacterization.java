package frc.lib.generic.characterization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

public class CameraPositionCharacterization extends Command {
    private static final LoggedNetworkNumber ROTATION_SPEED = new LoggedNetworkNumber("/SmartDashboard/CameraPositionCharacterization/RotationSpeed", 1);

    private final Supplier<Pose2d> cameraPoseSupplier;
    private final Rotation2d cameraMountAngle;
    private final DoubleConsumer rotateRobot;

    private Pose2d initialPose, endPose;

    public CameraPositionCharacterization(Supplier<Pose2d> cameraPoseSupplier, Rotation2d cameraMountAngle,
                                          DoubleConsumer rotateRobot, SubsystemBase requirement) {
        this.cameraPoseSupplier = cameraPoseSupplier;
        this.cameraMountAngle = cameraMountAngle;
        this.rotateRobot = rotateRobot;

        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        initialPose = cameraPoseSupplier.get();
    }

    @Override
    public void execute() {
        rotateRobot.accept(ROTATION_SPEED.get());

        final Pose2d currentPose = cameraPoseSupplier.get();

        if (initialPose == null) {
            initialPose = currentPose;
            return;
        }

        if (currentPose != null)
            endPose = currentPose;

        logRobotToCamera();
    }

    @Override
    public void end(boolean interrupted) {
        logRobotToCamera();
    }

    private void logRobotToCamera() {
        if (endPose == null || initialPose == null)
            return;

        final Transform2d robotToCamera = solveForRobotToCamera(initialPose, endPose, cameraMountAngle);
        Logger.recordOutput("CameraPositionCalculationCommand/RobotToCamera", robotToCamera);
    }

    private Transform2d solveForRobotToCamera(Pose2d initialPose, Pose2d endPose, Rotation2d cameraMountAngle) {
        final double initialCos = initialPose.getRotation().getCos();
        final double initialSin = initialPose.getRotation().getSin();

        final double endCos = endPose.getRotation().getCos();
        final double endSin = endPose.getRotation().getSin();

        final double cosDifference = initialCos - endCos;
        final double sinDifference = initialSin - endSin;

        final double denominator = solveForDenominator(cosDifference, sinDifference);

        final Translation2d translationDifference = endPose.getTranslation().minus(initialPose.getTranslation());
        final Translation2d transformTranslation = solveForTransformTranslation(denominator, translationDifference, cosDifference, sinDifference);
        final Translation2d rotatedTransformTranslation = transformTranslation.rotateBy(cameraMountAngle);

        return new Transform2d(rotatedTransformTranslation, cameraMountAngle);
    }

    private double solveForDenominator(double cosDifference, double sinDifference) {
        return (cosDifference * cosDifference) + (sinDifference * sinDifference);
    }

    private Translation2d solveForTransformTranslation(double denominator, Translation2d translationDifference, double cosDifference, double sinDifference) {
        final double transformX = ((translationDifference.getX() * cosDifference) + (translationDifference.getY() * sinDifference)) / -denominator;
        final double transformY = ((translationDifference.getX() * sinDifference) - (translationDifference.getY() * cosDifference)) / denominator;

        return new Translation2d(transformX, transformY);
    }
}
