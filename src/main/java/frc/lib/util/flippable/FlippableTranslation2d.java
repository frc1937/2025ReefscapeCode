package frc.lib.util.flippable;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A class that represents a {@link Translation2d} that can be flipped when the robot is on the red alliance.
 */
public class FlippableTranslation2d extends Flippable<Translation2d> {
    /**
     * Creates a new FlippableTranslation2d with the given x and y values.
     *
     * @param x                         the x value of the translation
     * @param y                         the y value of the translation
     * @param shouldFlipWhenRedAlliance should the position be flipped when the robot is on the red alliance
     */
    public FlippableTranslation2d(double x, double y, boolean shouldFlipWhenRedAlliance) {
        this(new Translation2d(x, y), shouldFlipWhenRedAlliance);
    }

    /**
     * Creates a new FlippableTranslation2d with the given translation.
     *
     * @param nonFlippedTranslation     the translation to flip
     * @param shouldFlipWhenRedAlliance should the position be flipped when the robot is on the red alliance
     */
    public FlippableTranslation2d(Translation2d nonFlippedTranslation, boolean shouldFlipWhenRedAlliance) {
        super(nonFlippedTranslation, shouldFlipWhenRedAlliance);
    }

    @Override
    protected Translation2d flip(Translation2d translation) {
        return FlippingUtil.flipFieldPosition(translation);
    }
}
