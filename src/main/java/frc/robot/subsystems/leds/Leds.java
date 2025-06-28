package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Colour;
import frc.lib.util.CustomLEDPatterns;
import frc.lib.util.flippable.Flippable;

import java.util.function.Supplier;

import static frc.lib.util.CustomLEDPatterns.*;
import static frc.lib.util.CustomLEDPatterns.generateInterpolatedBuffer;
import static frc.robot.utilities.PortsConstants.LEDSTRIP_PORT_PWM;

public class Leds extends SubsystemBase {
    public enum LEDMode {
        END_OF_MATCH(() -> generateOutwardsPointsBuffer(
                Colour.GOLD
        )),

        AUTOMATION(CustomLEDPatterns::generateRainbowBuffer),

        EATING(() -> generateBreathingBuffer(
                Colour.BLACK,
                Colour.PURPLE
        )),

        INTAKE_LOADED(() -> generateOutwardsPointsBuffer(
                Colour.ORANGE
        )),

        INTAKE_EMPTIED(() -> generateOutwardsPointsBuffer(
                Colour.GRAY
        )),

        AUTO_START(() -> generateInterpolatedBuffer(
                Colour.WHITE,
                Colour.CYAN
        )),

        DEBUG_MODE(() -> generateBreathingBuffer(
                new Colour(57, 255, 20),
                Colour.BLACK
        )),

        BATTERY_LOW(() -> generateOutwardsPointsBuffer(
                Colour.MAGENTA
        )),

        OUTTAKE(() -> generateFlashingBuffer(
                Colour.DARK_GREEN,
                Colour.GREEN)
        ),

        DEFAULT(() -> generateInterpolatedBuffer(
                getAllianceThemedLeds()
        ));

        private final Supplier<Colour[]> colours;

        LEDMode(Supplier<Colour[]> colors) {
            this.colours = colors;
        }

        public Supplier<Colour[]> getColours() {
            return colours;
        }
    }

    private static final AddressableLED ledstrip = new AddressableLED(LEDSTRIP_PORT_PWM);
    private static final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_COUNT);

    public static LEDMode currentMode;
    public static LEDMode previousMode;

    public Leds() {
        ledstrip.setLength(LEDS_COUNT);
        ledstrip.setData(buffer);
        ledstrip.start();
    }

    private static Colour[] getAllianceThemedLeds() {
        if (Flippable.isRedAlliance()) {
            return new Colour[]{
                    Colour.DARK_RED,
                    Colour.RED,
                    Colour.DARK_RED,
                    Colour.ORANGE};
        }

        return new Colour[]{Colour.SKY_BLUE,
                Colour.CORNFLOWER_BLUE,
                Colour.BLUE,
                Colour.LIGHT_BLUE,
                Colour.SILVER};
    }

    public void periodic() {
        ledstrip.setData(getBufferFromColours(buffer, currentMode.getColours().get()));
    }

    public void setToDefault() {
        currentMode = LEDMode.DEFAULT;
    }

    public Command setLEDStatus(LEDMode mode, double durationSeconds) {
        return Commands.runOnce(() -> setMode(mode))
                .andThen(Commands.waitSeconds(durationSeconds))
                .andThen(Commands.runOnce(this::restorePreviousMode)
                );
    }

    private void setMode(LEDMode mode) {
        previousMode = currentMode;
        currentMode = mode;
    }

    private void restorePreviousMode() {
        setMode(previousMode);
    }
}