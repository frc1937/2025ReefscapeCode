package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.Arrays;

public class CustomLEDPatterns {
    public static final int LEDS_COUNT = 46;
    private static final double MAX_GREEN_RANGE_METERS = 2;

    private static final Timer timer = new Timer();

    private static Color8Bit[] buffer = new Color8Bit[LEDS_COUNT];

    private static int counter;
    private static int previousColor = 0;
    private static int rainbowFirstPixel;

    static {
        timer.start();
    }

    public static Color8Bit[] reduceBrightness(Color8Bit[] colors, int brightnessPercentage) {
        final double brightnessFactor = brightnessPercentage / 100.0;

        final Color8Bit[] adjustedColors = new Color8Bit[colors.length];

        for (int i = 0; i < colors.length; i++) {
            final Color8Bit originalColor = colors[i];

            int newRed = (int) (originalColor.red * brightnessFactor);
            int newGreen = (int) (originalColor.green * brightnessFactor);
            int newBlue = (int) (originalColor.blue * brightnessFactor);

            newRed = Math.min(255, Math.max(0, newRed));
            newGreen = Math.min(255, Math.max(0, newGreen));
            newBlue = Math.min(255, Math.max(0, newBlue));

            adjustedColors[i] = new Color8Bit(newRed, newGreen, newBlue);
        }

        return adjustedColors;
    }

    /**
     * Generates a buffer that indicates how far in each direction (left, right, forwards, backwards)
     * the robot is from the correct position. This should be called periodically to update the indicator.
     * If a certain direction of the position is correct , it turns off that direction's LEDs.
     *
     * @param startColor The color when the robot is at the furthest position.
     * @param endColor   The color when the robot is at the closest position.
     * @param robotPosition  The current robot position.
     * @param targetPosition The target position.
     * @return The filled buffer with corrective LED colors.
     */
    public static Color8Bit[] generatePositionIndicatorBuffer(Color8Bit startColor, Color8Bit endColor, Translation2d robotPosition, Translation2d targetPosition) {
        buffer = generateSingleColorBuffer(new Color8Bit(Color.kBlack));

        final double deltaX = robotPosition.getX() - targetPosition.getX();
        final double deltaY = robotPosition.getY() - targetPosition.getY();

        final double normalizedY = Math.min(Math.abs(deltaY) / MAX_GREEN_RANGE_METERS, 1.0);
        final double normalizedX = Math.min(Math.abs(deltaX) / MAX_GREEN_RANGE_METERS, 1.0);

        final Color8Bit leftColor = deltaX > 0 ? interpolateColors(startColor, endColor, normalizedX) : new Color8Bit(Color.kBlack);
        final Color8Bit rightColor = deltaX < 0 ? interpolateColors(startColor, endColor, normalizedX) : new Color8Bit(Color.kBlack);
        final Color8Bit frontColor = deltaY < 0 ? interpolateColors(startColor, endColor, normalizedY) : new Color8Bit(Color.kBlack);
        final Color8Bit backColor = deltaY > 0 ? interpolateColors(startColor, endColor, normalizedY) : new Color8Bit(Color.kBlack);

        buffer[23] = leftColor;
        buffer[0] = leftColor;
        buffer[22] = rightColor;
        buffer[45] = rightColor;
        buffer[10] = frontColor;
        buffer[11] = frontColor;
        buffer[12] = frontColor;
        buffer[33] = backColor;
        buffer[34] = backColor;
        buffer[35] = backColor;

        return buffer;
    }

    /**
     * Generates a buffer with a single color.
     *
     * @param color The color to fill the buffer.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateSingleColorBuffer(Color8Bit color) {
        Arrays.fill(buffer, color);
        return buffer;
    }

    /**
     * Set the buffer from the color.
     *
     * @param ledBuffer The LED buffer to set.
     * @param buffer The color buffer.
     * @return The updated LED buffer.
     */
    public static AddressableLEDBuffer getBufferFromColors(AddressableLEDBuffer ledBuffer, Color8Bit[] buffer) {
        for (int i = 0; i < buffer.length; i++) {
            ledBuffer.setLED(i, buffer[i]);
        }

        return ledBuffer;
    }

    /**
     * Fill the buffer with RAINBOW colors. This needs to be called periodically for the rainbow effect
     * to be dynamic.
     *
     * @return The filled buffer.
     */
    public static Color8Bit[] generateRainbowBuffer() {
        int hue;

        for (int i = 0; i < LEDS_COUNT; i++) {
            hue = (rainbowFirstPixel + (i * 180 / LEDS_COUNT)) % 180;

            buffer[i] = new Color8Bit(Color.fromHSV(hue, 255, 128));
        }

        rainbowFirstPixel += 3;
        rainbowFirstPixel %= 180;

        return buffer;
    }

    /**
     * Set the buffer to flash between a set of colors. This needs to be called periodically for the
     * flashing effect to work.
     *
     * @param colors The colors to switch between.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateFlashingBuffer(Color8Bit... colors) {
        if (counter % 25 == 0) buffer = generateSingleColorBuffer(colors[previousColor++]);

        previousColor %= colors.length;
        counter++;

        return buffer;
    }

    /**
     * Generates a loading animation that moves outwards from the center of the LED strip.
     * This should be called periodically to update the animation.
     *
     * @param color1 The color for the first direction.
     * @param color2 The color for the second direction.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateLoadingAnimationBuffer(Color8Bit color1, Color8Bit color2) {
        buffer = generateSingleColorBuffer(new Color8Bit(Color.kBlack));

        final int midPoint = LEDS_COUNT / 2;
        final double time = timer.get() * 5;
        final int progress = (int) time % (LEDS_COUNT / 2);

        for (int i = midPoint - progress; i <= midPoint; i++) {
            if (i >= 0) buffer[i] = color1;
        }

        for (int i = midPoint + progress; i >= midPoint; i--) {
            if (i < LEDS_COUNT) buffer[i] = color2;
        }

        return buffer;
    }

    /**
     * Slowly switches between two colors, creating a breathing effect.
     *
     * @param firstColor  The first color.
     * @param secondColor The second color.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateBreathingBuffer(Color8Bit firstColor, Color8Bit secondColor) {
        final double x = timer.get();
        return generateSingleColorBuffer(interpolateColors(firstColor, secondColor, cosInterpolate(x)));
    }

    /**
     * Circles through N colors across the LED strip, utilizing a smooth effect.
     * This should be called periodically.
     *
     * @param colors The colors to cycle through.
     * @return The filled buffer.
     */
    public static Color8Bit[] generateCirclingBuffer(Color8Bit... colors) {
        final int colorsLength = colors.length;
        final double timerValue = timer.get();
        final double timerPosition = timerValue * 13 % LEDS_COUNT;

        int index, colorIndex1, colorIndex2;
        double colorIndex, fraction;

        for (int i = 0; i < LEDS_COUNT; i++) {
            index = wrapIndex(i);

            colorIndex = (timerPosition + i) % LEDS_COUNT / LEDS_COUNT * colorsLength;

            colorIndex1 = (int) colorIndex;
            colorIndex2 = (colorIndex1 + 1) % colorsLength;

            fraction = cosInterpolate(colorIndex - colorIndex1);

            Color8Bit color1 = colors[colorIndex1];
            Color8Bit color2 = colors[colorIndex2];

            buffer[index] = interpolateColors(color1, color2, fraction);
        }

        return buffer;
    }

    /**
     * Clears the buffer, then moves a color from the middle outwards.
     * Creating a nice loading effect. Should be used periodically.
     *
     * @param color The color to use
     * @return The current state of the buffer
     */
    public static Color8Bit[] generateOutwardsPointsBuffer(Color8Bit color) {
        buffer = generateSingleColorBuffer(new Color8Bit(Color.kBlack));

        final int quarter = LEDS_COUNT / 4;

        final double time = timer.get();

        final int x = time == (int) time ? ((int) (time) % 11) : ((int) (time * 16 % 11));

        for (int i = quarter - 1 - x; i < quarter + 1 + x; i++) {
            buffer[i] = new Color8Bit(new Color(color.red, color.green, color.blue));
        }

        for (int i = quarter * 3 - x; i < 2 + quarter * 3 + x; i++) {
            buffer[i] = new Color8Bit(new Color(color.red, color.green, color.blue));
        }

        return buffer;
    }

    private static int wrapIndex(int i) {
        while (i >= LEDS_COUNT) i -= LEDS_COUNT;

        while (i < 0) i += LEDS_COUNT;

        return i;
    }

    private static Color8Bit interpolateColors(Color8Bit startColor, Color8Bit endColor, double colorWeight) {
        final int red = (int) (endColor.red * (1 - colorWeight) + startColor.red * colorWeight);
        final int green = (int) (endColor.green * (1 - colorWeight) + startColor.green * colorWeight);
        final int blue = (int) (endColor.blue * (1 - colorWeight) + startColor.blue * colorWeight);

        return new Color8Bit(red, green, blue);
    }

    private static double cosInterpolate(double x) {
        return (1 - Math.cos(x * Math.PI)) * 0.5;
    }
}
