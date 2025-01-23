package frc.lib.util.flippable;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A class that represents a {@link Rotation2d} that can be flipped when the robot is on the red alliance.
 */
public class FlippableRotation2d extends Flippable<Rotation2d> {
    /**
     * Constructs and returns a FlippableRotation2d with the given degree value.
     *
     * @param degrees                   the value of the angle in degrees
     * @param shouldFlipWhenRedAlliance should the rotation be flipped when the robot is on the red alliance
     * @return the rotation object with the desired angle value
     */
    public static FlippableRotation2d fromDegrees(double degrees, boolean shouldFlipWhenRedAlliance) {
        return new FlippableRotation2d(Rotation2d.fromDegrees(degrees), shouldFlipWhenRedAlliance);
    }

    /**
     * Constructs and returns a FlippableRotation2d with the given radian value.
     *
     * @param radians                   the value of the angle in radians
     * @param shouldFlipWhenRedAlliance should the rotation be flipped when the robot is on the red alliance
     * @return the rotation object with the desired angle value
     */
    public static FlippableRotation2d fromRadians(double radians, boolean shouldFlipWhenRedAlliance) {
        return new FlippableRotation2d(Rotation2d.fromRadians(radians), shouldFlipWhenRedAlliance);
    }

    /**
     * Constructs a FlippableRotation2d with the given number of rotations.
     *
     * @param rotations                 the value of the angle in rotations
     * @param shouldFlipWhenRedAlliance should the rotation be flipped when the robot is on the red alliance
     * @return the rotation object with the desired angle value
     */
    public static FlippableRotation2d fromRotations(double rotations, boolean shouldFlipWhenRedAlliance) {
        return new FlippableRotation2d(Rotation2d.fromRotations(rotations), shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippableRotation2d with the given rotation value.
     *
     * @param radians                   the value of the angle in radians
     * @param shouldFlipWhenRedAlliance should the rotation be flipped when the robot is on the red alliance
     */
    public FlippableRotation2d(double radians, boolean shouldFlipWhenRedAlliance) {
        this(new Rotation2d(radians), shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippableRotation2d with the given Rotation2d.
     *
     * @param nonFlippedRotation        the non flipped rotation when the robot is on the blue alliance
     * @param shouldFlipWhenRedAlliance should the rotation be flipped when the robot is on the red alliance
     */
    public FlippableRotation2d(Rotation2d nonFlippedRotation, boolean shouldFlipWhenRedAlliance) {
        super(nonFlippedRotation, shouldFlipWhenRedAlliance);
    }

    @Override
    protected Rotation2d flip(Rotation2d rotation) {
        return FlippingUtil.flipFieldRotation(rotation);
    }
}
