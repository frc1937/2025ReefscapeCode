package frc.robot.subsystems.leds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Colour;
import frc.lib.util.CustomLEDPatterns;

import java.util.function.Function;
import java.util.function.Supplier;

import static frc.lib.util.CustomLEDPatterns.*;
import static frc.robot.RobotContainer.LEDS;
import static frc.robot.utilities.PortsConstants.LEDSTRIP_PORT_PWM;

public class Leds extends SubsystemBase {
    public enum LEDMode {
        END_OF_MATCH(timeout -> getCommandFromColours(() -> generateLoadingAnimationBuffer(
                Colour.YELLOW.toGRB(),
                Colour.GOLD.toGRB()
        ), timeout)),

        AUTOMATION(timeout -> getCommandFromColours(CustomLEDPatterns::generateRainbowBuffer, timeout)),

        EATING(timeout -> getCommandFromColours(() -> generateBreathingBuffer(
                Colour.BLACK.toGRB(),
                Colour.PURPLE.toGRB()
        ), timeout)),

        INTAKE_LOADED(timeout -> getCommandFromColours(() -> generateOutwardsPointsBuffer(
                Colour.ORANGE.toGRB()
        ), timeout)),

        INTAKE_EMPTIED(timeout -> getCommandFromColours(() -> generateOutwardsPointsBuffer(
                Colour.GRAY.toGRB()
        ), timeout)),

        AUTO_START(timeout -> getCommandFromColours(() -> generateScrollBuffer(new Colour[]{
                Colour.WHITE.toGRB(),
                Colour.CYAN.toGRB()}
        ), timeout)),

        DEBUG_MODE(timeout -> getCommandFromColours(() -> generateBreathingBuffer(
                new Colour(57, 255, 20),
                Colour.BLACK.toGRB()
        ), timeout)),

        BATTERY_LOW(timeout -> getCommandFromColours(() -> generateOutwardsPointsBuffer(
                Colour.MAGENTA.toGRB()
        ), timeout)),

        OUTTAKE(timeout -> getCommandFromColours(
                () -> generateFlashingBuffer(Colour.DARK_GREEN.toGRB(), Colour.GREEN.toGRB()), timeout)),

        DEFAULT(timeout ->
                getCommandFromColours(() -> generateScrollBuffer(getAllianceThemedLeds()), 0));

        private final Function<Double, Command> ledCommandFunction;

        LEDMode(Function<Double, Command> ledCommandFunction) {
            this.ledCommandFunction = ledCommandFunction;
        }

        public Command getLedCommand(double timeout) {
            return ledCommandFunction.apply(timeout);
        }
    }

    private static final AddressableLED ledstrip = new AddressableLED(LEDSTRIP_PORT_PWM);
    private static final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_COUNT);

    public Leds() {
        ledstrip.setLength(LEDS_COUNT);
        ledstrip.setData(buffer);
        ledstrip.start();
    }

    public Command setLEDStatus(LEDMode mode, double timeout) {
        return mode.getLedCommand(timeout);
    }

    /**
     * Sets the LED strip to indicate the robot's position relative to the target position.
     * This is command-less as the target position may change during the assignment of the autonomous.
     *
     * @param robotPosition  The current robot position.
     * @param targetPosition The target position, where the robot should be.
     */
    public void setLEDToPositionIndicator(Translation2d robotPosition, Translation2d targetPosition) {
        flashLEDStrip(
                generatePositionIndicatorBuffer(
                        Colour.RED,
                        Colour.GOLD,
                        robotPosition,
                        targetPosition
                ));
    }

    private static Command getCommandFromColours(Supplier<Colour[]> colours, double timeout) {
        if (timeout == 0)
            return Commands.run(() -> flashLEDStrip(colours.get()), LEDS).ignoringDisable(true);

        return Commands.run(() -> flashLEDStrip(colours.get()), LEDS).withTimeout(timeout).ignoringDisable(true);
    }

    private static void flashLEDStrip(Colour[] colours) {
        ledstrip.setData(getBufferFromColours(buffer, colours));
    }

    private static Colour[] getAllianceThemedLeds() {
//        return new Colour[]{
//                Colour.DARK_RED.toGRB(),
//                Colour.RED.toGRB(),
//                Colour.DARK_RED.toGRB(),
//                Colour.ORANGE.toGRB()};
        return new Colour[]{Colour.SKY_BLUE.toGRB(),
                Colour.CORNFLOWER_BLUE.toGRB(),
                Colour.BLUE.toGRB(),
                Colour.LIGHT_BLUE.toGRB(),
                Colour.SILVER.toGRB()};
    }
}
