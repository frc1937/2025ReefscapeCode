package frc.robot.subsystems.leds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CustomLEDPatterns;

import java.util.function.Function;
import java.util.function.Supplier;

import static frc.lib.util.CustomLEDPatterns.*;
import static frc.robot.RobotContainer.LEDS;
import static frc.robot.utilities.PortsConstants.LEDSTRIP_PORT_PWM;

public class Leds extends SubsystemBase {
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
                        new Color8Bit(Color.kRed),
                        new Color8Bit(Color.kGreen),
                        robotPosition,
                        targetPosition
                ));
    }

    public enum LEDMode {
        END_OF_MATCH(timeout -> getCommandFromColours(() -> generateFlashingBuffer(new Color8Bit(Color.kGreen), new Color8Bit(Color.kBlue)), timeout)),

        INTAKE_LOADED(timeout -> getCommandFromColours(() -> generateFlashingBuffer(
                new Color8Bit(Color.kOrange),
                new Color8Bit(Color.kRed)
        ), timeout)),

        INTAKE_EMPTIED(timeout -> getCommandFromColours(() -> generateCirclingBuffer(
                new Color8Bit(Color.kCyan),
                new Color8Bit(Color.kWhite),
                new Color8Bit(Color.kDeepPink)
        ), timeout)),

        DEBUG_MODE(timeout -> getCommandFromColours(() -> generateBreathingBuffer(
                new Color8Bit(Color.kCyan),
                new Color8Bit(Color.kWhite)
        ), timeout)),

        BATTERY_LOW(timeout -> getCommandFromColours(() -> generateOutwardsPointsBuffer(new Color8Bit(Color.kRed)), timeout)),
        DEFAULT(timeout -> getCommandFromColours(CustomLEDPatterns::generateRainbowBuffer, 0));

        private final Function<Double, Command> ledCommandFunction;

        LEDMode(Function<Double, Command> ledCommandFunction) {
            this.ledCommandFunction = ledCommandFunction;
        }

        public Command getLedCommand(double timeout) {
            return ledCommandFunction.apply(timeout);
        }
    }

    private static Command getCommandFromColours(Supplier<Color8Bit[]> colours, double timeout) {
        if (timeout == 0)
            return Commands.run(() -> flashLEDStrip(colours.get()), LEDS).ignoringDisable(true);

        return Commands.run(() -> flashLEDStrip(colours.get()), LEDS).withTimeout(timeout).ignoringDisable(true);
    }

    private static void flashLEDStrip(Color8Bit[] colours) {
        ledstrip.setData(getBufferFromColors(buffer, colours));
    }
}
