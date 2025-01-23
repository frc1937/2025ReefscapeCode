package frc.lib.util.flippable;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * A class that represents a {@link Translation3d} that can be flipped when the robot is on the red alliance.
 * The Z value will have no change.
 */
public class FlippableTranslation3d extends Flippable<Translation3d> {
    /**
     * Creates a new FlippableTranslation3d with the given x, y, and z values.
     *
     * @param x                         the x value of the translation
     * @param y                         the y value of the translation
     * @param z                         the z value of the translation
     * @param shouldFlipWhenRedAlliance should the position be flipped when the robot is on the red alliance
     */
    public FlippableTranslation3d(double x, double y, double z, boolean shouldFlipWhenRedAlliance) {
        this(new Translation3d(x, y, z), shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippableTranslation3d with the given translation.
     *
     * @param nonFlippedTranslation     the translation to flip
     * @param shouldFlipWhenRedAlliance should the position be flipped when the robot is on the red alliance
     */
    public FlippableTranslation3d(Translation3d nonFlippedTranslation, boolean shouldFlipWhenRedAlliance) {
        super(nonFlippedTranslation, shouldFlipWhenRedAlliance);
    }

    /**
     * Flips the given translation. The translation will be flipped if the robot is on the red alliance and {@link #shouldFlipWhenRedAlliance} is true.
     * The Z value will have no change.
     *
     * @param translation the object to flip
     * @return the flipped translation
     */
    @Override
    protected Translation3d flip(Translation3d translation) {
        final Translation2d flippedTranslation2d = FlippingUtil.flipFieldPosition(translation.toTranslation2d());
        return new Translation3d(flippedTranslation2d.getX(), flippedTranslation2d.getY(), translation.getZ());
    }
}