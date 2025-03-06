package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class CustomLEDPatterns {
    public static final int LEDS_COUNT = 69;
    private static final double MAX_GREEN_RANGE_METERS = 2;

    private static final Timer timer = new Timer();

    private static Colour[] buffer = new Colour[LEDS_COUNT];
    private static final Colour[] scrollingBuffer = buffer.clone();
    private static final Colour[] rainbowBuffer = buffer.clone();

    private static int counter;
    private static int previousColour = 0;
    private static int rainbowFirstPixel;
    private static int scrollingFirstPixel;

    private static boolean wasScrollInitialized = false;
    private static boolean wasRainbowInitialized = false;

    static {
        timer.start();
    }

    public static Colour[] reduceBrightness(Colour[] colours, int brightnessPercentage) {
        final double brightnessFactor = brightnessPercentage / 100.0;

        final Colour[] adjustedColours = new Colour[colours.length];

        for (int i = 0; i < colours.length; i++) {
            final Colour originalColour = colours[i];

            int newRed = (int) (originalColour.getRed() * brightnessFactor);
            int newGreen = (int) (originalColour.getGreen() * brightnessFactor);
            int newBlue = (int) (originalColour.getBlue() * brightnessFactor);

            newRed = Math.min(255, Math.max(0, newRed));
            newGreen = Math.min(255, Math.max(0, newGreen));
            newBlue = Math.min(255, Math.max(0, newBlue));

            adjustedColours[i] = new Colour(newRed, newGreen, newBlue);
        }

        return adjustedColours;
    }

    /**
     * Generates a buffer that indicates how far in each direction (left, right, forwards, backwards)
     * the robot is from the correct position. This should be called periodically to update the indicator.
     * If a certain direction of the position is correct , it turns off that direction's LEDs.
     *
     * @param startColour     The colour when the robot is at the furthest position.
     * @param endColour       The colour when the robot is at the closest position.
     * @param robotPosition  The current robot position.
     * @param targetPosition The target position.
     * @return The filled buffer with corrective LED colours.
     */
    public static Colour[] generatePositionIndicatorBuffer(Colour startColour, Colour endColour, Translation2d robotPosition, Translation2d targetPosition) {
        buffer = generateSingleColourBuffer(Colour.BLACK);

        final double deltaX = robotPosition.getX() - targetPosition.getX();
        final double deltaY = robotPosition.getY() - targetPosition.getY();

        final double normalizedY = Math.min(Math.abs(deltaY) / MAX_GREEN_RANGE_METERS, 1.0);
        final double normalizedX = Math.min(Math.abs(deltaX) / MAX_GREEN_RANGE_METERS, 1.0);

        final Colour leftColour = deltaX > 0 ? interpolateColours(startColour, endColour, normalizedX) : Colour.BLACK;
        final Colour rightColour = deltaX < 0 ? interpolateColours(startColour, endColour, normalizedX) : Colour.BLACK;
        final Colour frontColour = deltaY < 0 ? interpolateColours(startColour, endColour, normalizedY) : Colour.BLACK;
        final Colour backColour = deltaY > 0 ? interpolateColours(startColour, endColour, normalizedY) : Colour.BLACK;

        buffer[23] = leftColour;
        buffer[0] = leftColour;
        buffer[22] = rightColour;
        buffer[45] = rightColour;
        buffer[10] = frontColour;
        buffer[11] = frontColour;
        buffer[12] = frontColour;
        buffer[33] = backColour;
        buffer[34] = backColour;
        buffer[35] = backColour;

        return buffer;
    }

    /**
     * Generates a buffer with a single colour.
     *
     * @param colour The colour to fill the buffer.
     * @return The filled buffer.
     */
    public static Colour[] generateSingleColourBuffer(Colour colour) {
        Arrays.fill(buffer, colour);
        return buffer;
    }

    /**
     * Set the buffer from the colour.
     *
     * @param ledBuffer The LED buffer to set.
     * @param buffer    The colour buffer.
     * @return The updated LED buffer.
     */
    public static AddressableLEDBuffer getBufferFromColours(AddressableLEDBuffer ledBuffer, Colour[] buffer) {
        for (int i = 0; i < buffer.length; i++) {
            ledBuffer.setLED(i, buffer[i].getColor());
        }

        return ledBuffer;
    }

    /**
     * Fill the buffer with RAINBOW colours. This needs to be called periodically for the rainbow effect
     * to be dynamic.
     *
     * @return The filled buffer.
     */
    public static Colour[] generateRainbowBuffer() {
        if (!wasRainbowInitialized) {
            final Colour[] colours = new Colour[]{Colour.RED, Colour.ORANGE, Colour.YELLOW, Colour.GREEN, Colour.BLUE, Colour.INDIGO, Colour.VIOLET};

            for (int i = 0; i < LEDS_COUNT; i++) {
                final double position = (double) i / LEDS_COUNT * colours.length;

                final int startColourIndex = (int) Math.floor(position);
                final int endColourIndex = (startColourIndex + 1) % colours.length;

                final double ratio = position - startColourIndex;

                rainbowBuffer[i] = interpolateColours(colours[endColourIndex], colours[startColourIndex], ratio);
            }

            buffer = rainbowBuffer.clone();
            wasRainbowInitialized = true;
        } else {
            for (int i = 0; i < LEDS_COUNT; i++) {
                buffer[i] = rainbowBuffer[(i + rainbowFirstPixel) % LEDS_COUNT];
            }

            rainbowFirstPixel = (rainbowFirstPixel + 1) % LEDS_COUNT;
        }

        return buffer;
    }

    public static Colour[] generateScrollBuffer(Colour[] colours) {
        if (!wasScrollInitialized) {
            final int totalColours = colours.length;

            for (int i = 0; i < LEDS_COUNT; i++) {
                final double position = (double) i / LEDS_COUNT * totalColours;

                final int startColourIndex = (int) Math.floor(position);
                final int endColourIndex = (startColourIndex + 1) % totalColours;

                final double ratio = position - startColourIndex;

                scrollingBuffer[i] = interpolateColours(colours[endColourIndex], colours[startColourIndex], ratio);
            }

            buffer = scrollingBuffer.clone();
            wasScrollInitialized = true;
        } else {
            for (int i = 0; i < LEDS_COUNT; i++) {
                buffer[i] = scrollingBuffer[(i + scrollingFirstPixel) % LEDS_COUNT];
            }

            scrollingFirstPixel = (scrollingFirstPixel + 1) % LEDS_COUNT;
        }

        return buffer;
    }

    /**
     * Set the buffer to flash between a set of colours. This needs to be called periodically for the
     * flashing effect to work.
     *
     * @param colours The colours to switch between.
     * @return The filled buffer.
     */
    public static Colour[] generateFlashingBuffer(Colour... colours) {
        if (previousColour++ >= colours.length) return buffer;

        if (counter % 25 == 0) buffer = generateSingleColourBuffer(colours[previousColour++]);

        previousColour %= colours.length;
        counter++;

        return buffer;
    }

    /**
     * Generates a loading animation that moves outwards from the center of the LED strip.
     * This should be called periodically to update the animation.
     *
     * @param colour1 The colour for the first direction.
     * @param colour2 The colour for the second direction.
     * @return The filled buffer.
     */
    public static Colour[] generateLoadingAnimationBuffer(Colour colour1, Colour colour2) {
        buffer = generateSingleColourBuffer(Colour.BLACK);

        final int midPoint = LEDS_COUNT / 2;
        final double time = timer.get() * 5;
        final int progress = (int) time % (LEDS_COUNT / 2);

        for (int i = midPoint - progress; i <= midPoint; i++) {
            if (i >= 0) buffer[i] = colour1;
        }

        for (int i = midPoint + progress; i >= midPoint; i--) {
            if (i < LEDS_COUNT) buffer[i] = colour2;
        }

        return buffer;
    }

    /**
     * Slowly switches between two colours, creating a breathing effect.
     *
     * @param firstColour  The first colour.
     * @param secondColour The second colour.
     * @return The filled buffer.
     */
    public static Colour[] generateBreathingBuffer(Colour firstColour, Colour secondColour) {
        final double x = timer.get();
        return generateSingleColourBuffer(interpolateColours(firstColour, secondColour, cosInterpolate(x)));
    }

    /**
     * Clears the buffer, then moves a colour from the middle outwards.
     * Creating a nice loading effect. Should be used periodically.
     *
     * @param colour The colour to use
     * @return The current state of the buffer
     */
    public static Colour[] generateOutwardsPointsBuffer(Colour colour) {
        buffer = generateSingleColourBuffer(Colour.BLACK);

        final int quarter = LEDS_COUNT / 4;

        final double time = timer.get();

        final int x = time == (int) time ? ((int) (time) % 11) : ((int) (time * 32 % 11));

        for (int i = quarter - 1 - x; i < quarter + 1 + x; i++) {
            buffer[i] = new Colour(colour.getRed(), colour.getGreen(), colour.getBlue());
        }

        for (int i = quarter * 3 - x; i < 2 + quarter * 3 + x; i++) {
            buffer[i] = new Colour(colour.getRed(), colour.getGreen(), colour.getBlue());
        }

        return buffer;
    }

    private static int wrapIndex(int i) {
        while (i >= LEDS_COUNT) i -= LEDS_COUNT;

        while (i < 0) i += LEDS_COUNT;

        return i;
    }

    public static Colour interpolateColours(Colour startColour, Colour endColour, double colourWeight) {
        final int red = (int) (endColour.getRed() * (1 - colourWeight) + startColour.getRed() * colourWeight);
        final int green = (int) (endColour.getGreen() * (1 - colourWeight) + startColour.getGreen() * colourWeight);
        final int blue = (int) (endColour.getBlue() * (1 - colourWeight) + startColour.getBlue() * colourWeight);

        return new Colour(red, green, blue);
    }

    private static double cosInterpolate(double x) {
        return (1 - Math.cos(x * Math.PI)) * 0.5;
    }
}
