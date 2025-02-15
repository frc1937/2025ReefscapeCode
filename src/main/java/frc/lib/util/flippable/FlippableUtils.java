package frc.lib.util.flippable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.utilities.FieldConstants.FIELD_LENGTH;
import static frc.robot.utilities.FieldConstants.FIELD_WIDTH;

public class FlippableUtils {
    /**
     * Rotate the pose about the Y axis
     *
     * @param pose The pose to rotate
     * @return The rotated pose
     */
    public static Pose2d flipAboutYAxis(Pose2d pose) {
        return new Pose2d(FIELD_LENGTH - pose.getX(), pose.getY(), Rotation2d.k180deg.minus(pose.getRotation()));
    }

    public static Pose2d flipAboutYAxisNoRotation(Pose2d pose) {
        return new Pose2d(FIELD_LENGTH - pose.getX(), pose.getY(), (pose.getRotation()));
    }

    /**
     * Rotate the pose about the X axis, including rotation.
     *
     * @param pose The pose to rotate
     * @return The rotated pose
     */
    public static Pose2d flipAboutXAxis(Pose2d pose) {
        return new Pose2d(pose.getX(), FIELD_WIDTH - pose.getY(), pose.getRotation().unaryMinus());
    }

    /**
     * Rotate the pose about the X and Y axes, including rotation.
     *
     * @param pose The pose to rotate
     * @return The rotated pose
     */
    public static Pose2d flipAboutBothAxes(Pose2d pose) {
        return new Pose2d(FIELD_LENGTH - pose.getX(), FIELD_WIDTH - pose.getY(), pose.getRotation().minus(Rotation2d.k180deg));
    }

    /**
     * Rotate the rotation about the Y axis
     *
     * @param rotation2d The rotation to rotate
     * @return The rotated rotation
     */
    public static Rotation2d flipAboutYAxis(Rotation2d rotation2d) {
        return Rotation2d.k180deg.minus(rotation2d);
    }

    /**
     * Rotate the rotation about the X axis
     *
     * @param rotation The rotation to rotate
     * @return The rotated rotation
     */
    public static Rotation2d flipAboutXAxis(Rotation2d rotation) {
        return rotation.unaryMinus();
    }

    /**
     * Rotate the rotation about the X and Y axes.
     *
     * @param rotation The rotation to rotate
     * @return The rotated rotation
     */
    public static Rotation2d flipAboutBothAxes(Rotation2d rotation) {
        return rotation.minus(Rotation2d.k180deg);
    }

    /**
     * Rotate the translation about the Y axis
     *
     * @param translation The translation to rotate
     * @return The rotated translation
     */
    public static Translation2d flipAboutYAxis(Translation2d translation) {
        return new Translation2d(FIELD_LENGTH - translation.getX(), translation.getY());
    }

    /**
     * Rotate the translation about the X axis
     *
     * @param translation The translation to rotate
     * @return The rotated translation
     */
    public static Translation2d flipAboutXAxis(Translation2d translation) {
        return new Translation2d(translation.getX(), FIELD_WIDTH - translation.getY());
    }

    /**
     * Rotate the translation about the X and Y axes.
     *
     * @param translation The translation to rotate
     * @return The rotated translation
     */
    public static Translation2d flipAboutBothAxes(Translation2d translation) {
        return new Translation2d(FIELD_LENGTH - translation.getX(), FIELD_WIDTH - translation.getY());
    }
}
