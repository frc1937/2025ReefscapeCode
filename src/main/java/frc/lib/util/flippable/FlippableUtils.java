package frc.lib.util.flippable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
    public static Pose2d flipAboutBothAxis(Pose2d pose) {
        return new Pose2d(FIELD_LENGTH - pose.getX(), FIELD_WIDTH - pose.getY(), pose.getRotation().minus(Rotation2d.kPi));
    }

    /**
     * Rotate the rotation2d about the Y axis
     *
     * @param rotation2d The rotation2d to rotate
     * @return The rotated rotation2d
     */
    public static Rotation2d flipAboutYAxis(Rotation2d rotation2d) {
        return Rotation2d.k180deg.minus(rotation2d);
    }

    /**
     * Rotate the pose about the X axis, including rotation.
     *
     * @param rotation The pose to rotate
     * @return The rotated pose
     */
    public static Rotation2d flipAboutXAxis(Rotation2d rotation) {
        return rotation.unaryMinus();
    }

    /**
     * Rotate the pose about the X and Y axes, including rotation.
     *
     * @param rotation The pose to rotate
     * @return The rotated pose
     */
    public static Rotation2d flipAboutBothAxis(Rotation2d rotation) {
        return rotation.minus(Rotation2d.kPi);
    }
}
