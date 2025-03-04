package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;

public class CustomLEDPatterns {
    public static final int LEDS_COUNT = 69;
    private static final double MAX_GREEN_RANGE_METERS = 2;

    private static final Timer timer = new Timer();

    private static Color[] buffer = new Color[LEDS_COUNT];
    private static final Color[] scrollingBuffer = buffer.clone();
    private static final Color[] rainbowBuffer = buffer.clone();

    private static int counter;
    private static int previousColor = 0;
    private static int rainbowFirstPixel;
    private static int scrollingFirstPixel;

    private static boolean hasBeenCalledScrolling = false;
    private static boolean hasBeenCalledRainbow = false;

    static {
        timer.start();
    }

    public static Color[] reduceBrightness(Color[] colors, int brightnessPercentage) {
        final double brightnessFactor = brightnessPercentage / 100.0;

        final Color[] adjustedColors = new Color[colors.length];

        for (int i = 0; i < colors.length; i++) {
            final Color originalColor = colors[i];

            int newRed = (int) (originalColor.red * brightnessFactor);
            int newGreen = (int) (originalColor.green * brightnessFactor);
            int newBlue = (int) (originalColor.blue * brightnessFactor);

            newRed = Math.min(255, Math.max(0, newRed));
            newGreen = Math.min(255, Math.max(0, newGreen));
            newBlue = Math.min(255, Math.max(0, newBlue));

            adjustedColors[i] = new Color(newRed, newGreen, newBlue);
        }

        return adjustedColors;
    }

    /**
     * Generates a buffer that indicates how far in each direction (left, right, forwards, backwards)
     * the robot is from the correct position. This should be called periodically to update the indicator.
     * If a certain direction of the position is correct , it turns off that direction's LEDs.
     *
     * @param startColor     The color when the robot is at the furthest position.
     * @param endColor       The color when the robot is at the closest position.
     * @param robotPosition  The current robot position.
     * @param targetPosition The target position.
     * @return The filled buffer with corrective LED colors.
     */
    public static Color[] generatePositionIndicatorBuffer(Color startColor, Color endColor, Translation2d robotPosition, Translation2d targetPosition) {
        buffer = generateSingleColorBuffer(Color.kBlack);

        final double deltaX = robotPosition.getX() - targetPosition.getX();
        final double deltaY = robotPosition.getY() - targetPosition.getY();

        final double normalizedY = Math.min(Math.abs(deltaY) / MAX_GREEN_RANGE_METERS, 1.0);
        final double normalizedX = Math.min(Math.abs(deltaX) / MAX_GREEN_RANGE_METERS, 1.0);

        final Color leftColor = deltaX > 0 ? interpolateColors(startColor, endColor, normalizedX) : Color.kBlack;
        final Color rightColor = deltaX < 0 ? interpolateColors(startColor, endColor, normalizedX) : Color.kBlack;
        final Color frontColor = deltaY < 0 ? interpolateColors(startColor, endColor, normalizedY) : Color.kBlack;
        final Color backColor = deltaY > 0 ? interpolateColors(startColor, endColor, normalizedY) : Color.kBlack;

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
    public static Color[] generateSingleColorBuffer(Color color) {
        Arrays.fill(buffer, color);
        return buffer;
    }

    /**
     * Set the buffer from the color.
     *
     * @param ledBuffer The LED buffer to set.
     * @param buffer    The color buffer.
     * @return The updated LED buffer.
     */
    public static AddressableLEDBuffer getBufferFromColors(AddressableLEDBuffer ledBuffer, Color[] buffer) {
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
    public static Color[] generateRainbowBuffer() {
        if (!hasBeenCalledRainbow) {
            for (int i = 0; i < LEDS_COUNT; i++) {
                int hue = (rainbowFirstPixel + (i * 180 / LEDS_COUNT)) % 180;
                rainbowBuffer[i] = Color.fromHSV(hue, 255, 128);
            }

            buffer = rainbowBuffer.clone();
            hasBeenCalledRainbow = true;
        } else {
            for (int i = 0; i < LEDS_COUNT; i++) {
                buffer[i] = rainbowBuffer[(i + rainbowFirstPixel) % LEDS_COUNT];
            }

            rainbowFirstPixel = (rainbowFirstPixel + 1) % LEDS_COUNT;
        }

        return buffer;
    }

    public static Color[] generateScrollBuffer(Color[] colors) {
        if (!hasBeenCalledScrolling) {
            final int totalColors = colors.length;

            for (int i = 0; i < LEDS_COUNT; i++) {
                final double position = (double) i / LEDS_COUNT * totalColors;

                final int startColorIndex = (int) Math.floor(position);
                final int endColorIndex = (startColorIndex + 1) % totalColors;

                final double ratio = position - startColorIndex;

                scrollingBuffer[i] = interpolateColors(colors[endColorIndex], colors[startColorIndex], ratio);
            }

            buffer = scrollingBuffer.clone();
            hasBeenCalledScrolling = true;
        } else {
            for (int i = 0; i < LEDS_COUNT; i++) {
                buffer[i] = scrollingBuffer[(i + scrollingFirstPixel) % LEDS_COUNT];
            }

            scrollingFirstPixel = (scrollingFirstPixel + 1) % LEDS_COUNT;
        }

        return buffer;
    }

    /**
     * Set the buffer to flash between a set of colors. This needs to be called periodically for the
     * flashing effect to work.
     *
     * @param colors The colors to switch between.
     * @return The filled buffer.
     */
    public static Color[] generateFlashingBuffer(Color... colors) {
        if (previousColor++ >= colors.length) return buffer;

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
    public static Color[] generateLoadingAnimationBuffer(Color color1, Color color2) {
        buffer = generateSingleColorBuffer(Color.kBlack);

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
    public static Color[] generateBreathingBuffer(Color firstColor, Color secondColor) {
        final double x = timer.get();
        return generateSingleColorBuffer(interpolateColors(firstColor, secondColor, cosInterpolate(x)));
    }

    /**
     * Clears the buffer, then moves a color from the middle outwards.
     * Creating a nice loading effect. Should be used periodically.
     *
     * @param color The color to use
     * @return The current state of the buffer
     */
    public static Color[] generateOutwardsPointsBuffer(Color color) {
        buffer = generateSingleColorBuffer(Color.kBlack);

        final int quarter = LEDS_COUNT / 4;

        final double time = timer.get();

        final int x = time == (int) time ? ((int) (time) % 11) : ((int) (time * 32 % 11));

        for (int i = quarter - 1 - x; i < quarter + 1 + x; i++) {
            buffer[i] = new Color(color.red, color.green, color.blue);
        }

        for (int i = quarter * 3 - x; i < 2 + quarter * 3 + x; i++) {
            buffer[i] = new Color(color.red, color.green, color.blue);
        }

        return buffer;
    }

    private static int wrapIndex(int i) {
        while (i >= LEDS_COUNT) i -= LEDS_COUNT;

        while (i < 0) i += LEDS_COUNT;

        return i;
    }

    private static Color interpolateColors(Color startColor, Color endColor, double colorWeight) {
        final int red = (int) ((255 * endColor.red) * (1 - colorWeight) + (255 * startColor.red) * colorWeight);
        final int green = (int) ((255 * endColor.green) * (1 - colorWeight) + (255 * startColor.green) * colorWeight);
        final int blue = (int) ((255 * endColor.blue) * (1 - colorWeight) + (255 * startColor.blue) * colorWeight);

        return new Color(red, green, blue);
    }

    private static double cosInterpolate(double x) {
        return (1 - Math.cos(x * Math.PI)) * 0.5;
    }
}
