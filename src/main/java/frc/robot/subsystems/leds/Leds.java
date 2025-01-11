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

import java.util.function.Supplier;

import static frc.lib.util.CustomLEDPatterns.LEDS_COUNT;
import static frc.lib.util.CustomLEDPatterns.generateBreathingBuffer;
import static frc.lib.util.CustomLEDPatterns.generateCirclingBuffer;
import static frc.lib.util.CustomLEDPatterns.generateFlashingBuffer;
import static frc.lib.util.CustomLEDPatterns.generateOutwardsPointsBuffer;
import static frc.lib.util.CustomLEDPatterns.generatePositionIndicatorBuffer;
import static frc.lib.util.CustomLEDPatterns.getBufferFromColors;

public class Leds extends SubsystemBase {
    private static final AddressableLED ledstrip = new AddressableLED(0);
    private static final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_COUNT);

    public Leds() {
        ledstrip.setLength(LEDS_COUNT);
        ledstrip.setData(buffer);
        ledstrip.start();
    }

    public Command setLEDStatus(LEDMode mode, double timeout) {
        return switch (mode) {
            case SHOOTER_LOADED -> getCommandFromColours(() -> generateFlashingBuffer(
                    new Color8Bit(Color.kOrange),
                    new Color8Bit(Color.kRed)
            ), timeout);

            case SHOOTER_EMPTY -> getCommandFromColours(() -> generateCirclingBuffer(
                    new Color8Bit(Color.kCyan),
                    new Color8Bit(Color.kWhite),
                    new Color8Bit(Color.kDeepPink)
            ), timeout);

            case DEBUG_MODE -> getCommandFromColours(() -> generateBreathingBuffer(
                    new Color8Bit(Color.kCyan),
                    new Color8Bit(Color.kWhite)
            ), timeout);

            case BATTERY_LOW ->
                    getCommandFromColours(() -> generateOutwardsPointsBuffer(new Color8Bit(Color.kRed)), timeout);

            default -> getCommandFromColours(CustomLEDPatterns::generateRainbowBuffer, 0);
        };
    }

    /**
     * Sets the LED strip to indicate the robot's position relative to the target position.
     * This is command-less as the target position may change during the assignment of the autonomous.
     * @param robotPosition - The current robot position.
     * @param targetPosition - The target position, where the robot should be.
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
        SHOOTER_LOADED,
        SHOOTER_EMPTY,
        DEBUG_MODE,
        BATTERY_LOW,
        DEFAULT,
    }

    private Command getCommandFromColours(Supplier<Color8Bit[]> colours, double timeout) {
        if (timeout == 0)
            return Commands.run(() -> flashLEDStrip(colours.get()), this).ignoringDisable(true);

        return Commands.run(
                () -> flashLEDStrip(colours.get()), this).withTimeout(timeout).ignoringDisable(true);
    }

    private void flashLEDStrip(Color8Bit[] colours) {
        ledstrip.setData(getBufferFromColors(buffer, colours));
    }
}
