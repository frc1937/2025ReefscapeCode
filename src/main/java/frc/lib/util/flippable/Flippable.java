package frc.lib.util.flippable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;

/**
 * A class that allows for objects to be flipped across the center of the field when the robot is on the red alliance.
 * This is useful for placing field elements and other objects that are flipped across the field, or for flipping the target heading to face a field element.
 * This either represents a field that is mirrored vertically over the center of the field, or a field that is rotationally symmetric: the red alliance side is the blue alliance side rotated by 180 degrees.
 * The code will automatically determine which type of field the robot is on and flip the object accordingly.
 *
 * @param <T> the type of object to flip
 */
public abstract class Flippable<T> {
    public static final Trigger IS_RED_ALLIANCE_TRIGGER = new Trigger(Flippable::isRedAlliance);
    private static final Timer UPDATE_ALLIANCE_TIMER = new Timer();
    private static boolean IS_RED_ALLIANCE = notCachedIsRedAlliance();
    protected final T nonFlippedObject, flippedObject;

    protected final boolean shouldFlipWhenRedAlliance;


    /**
     * Initializes the Flippable class. This should be called once in RobotContainer.
     */
    public static void init() {
        UPDATE_ALLIANCE_TIMER.start();

        new Trigger(() -> UPDATE_ALLIANCE_TIMER.advanceIfElapsed(0.5))
                .onTrue(getUpdateAllianceCommand());
    }

    /**
     * @return whether the robot is on the red alliance. This is cached every 0.5 seconds
     */
    public static boolean isRedAlliance() {
        return IS_RED_ALLIANCE;
    }

    /**
     * Gets a command that updates the current alliance. This is used to cache the alliance every 0.5 seconds. Ignoring disable is used to update the current alliance when the robot is disabled.
     *
     * @return the command
     */
    private static Command getUpdateAllianceCommand() {
        return new InstantCommand(() -> IS_RED_ALLIANCE = notCachedIsRedAlliance()).ignoringDisable(true);
    }

    /**
     * @return whether the robot is on the red alliance. This is not cached
     */
    private static boolean notCachedIsRedAlliance() {
        final Optional<DriverStation.Alliance> optionalAlliance = DriverStation.getAlliance();
        return optionalAlliance.orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Red);
    }

    /**
     * Creates a new Flippable object.
     *
     * @param nonFlippedObject          the object when the robot is on the blue alliance; the non-flipped object
     * @param shouldFlipWhenRedAlliance should the object should be flipped when the robot is on the red alliance
     */
    protected Flippable(T nonFlippedObject, boolean shouldFlipWhenRedAlliance) {
        this.nonFlippedObject = nonFlippedObject;
        this.flippedObject = flip(nonFlippedObject);
        this.shouldFlipWhenRedAlliance = shouldFlipWhenRedAlliance;
    }

    /**
     * If the robot is on the red alliance and the object should be flipped, the flipped object is returned.
     * Otherwise, the non-flipped object is returned.
     *
     * @return the current object
     */
    public T get() {
        return isRedAlliance() && shouldFlipWhenRedAlliance ? flippedObject : nonFlippedObject;
    }

    /**
     * Flips the object across the center of the field.
     *
     * @param object the object to flip
     * @return the flipped object
     */
    protected abstract T flip(T object);
}