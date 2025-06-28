package frc.lib.util;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.util.Arrays;

public class CustomLEDPatterns {
    public static final int LEDS_COUNT = 69;
    private static Colour[] buffer = new Colour[LEDS_COUNT];

    private static final Colour[]
            interpolatedBuffer = buffer.clone(),
            rainbowBuffer = buffer.clone();

    private static double
            scrollingCounter,
            flashingCounter,
            breathingCounter,
            outwardPointsCounter;

    private static int
            previousColour = 0,
            scrollingFirstPixel;

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
     * Fill the buffer with RAINBOW colours and makes it scroll. This needs to be called periodically for the scrolling effect
     * to be dynamic.
     *
     * @return The filled buffer.
     */
    public static Colour[] generateRainbowBuffer() {
        final Colour[] colours = new Colour[]{Colour.RED, Colour.ORANGE, Colour.YELLOW, Colour.GREEN, Colour.BLUE, Colour.INDIGO, Colour.VIOLET};

        for (int i = 0; i < LEDS_COUNT; i++) {
            final double position = (double) i / LEDS_COUNT * colours.length;

            final int startColourIndex = (int) Math.floor(position);
            final int endColourIndex = (startColourIndex + 1) % colours.length;

            final double ratio = position - startColourIndex;

            rainbowBuffer[i] = interpolateColours(colours[endColourIndex], colours[startColourIndex], ratio);
        }


        return scrollBuffer(rainbowBuffer.clone());
    }

    /**
     * Fill the buffer with a smooth gradient between the given colours and makes it scroll. This needs to be called periodically for the scrolling effect
     * to be dynamic.
     *
     * @param colours The colours to interpolate between.
     * @return The filled buffer.
     */
    public static Colour[] generateInterpolatedBuffer(Colour... colours) {
        if (colours.length == 0) return buffer;

        final int totalColours = colours.length;

        for (int i = 0; i < LEDS_COUNT; i++) {
            final double position = (double) i / LEDS_COUNT * totalColours;

            final int startColourIndex = (int) Math.floor(position);
            final int endColourIndex = (startColourIndex + 1) % totalColours;

            final double ratio = position - startColourIndex;

            interpolatedBuffer[i] = interpolateColours(colours[endColourIndex], colours[startColourIndex], ratio);
        }

        return scrollBuffer(interpolatedBuffer.clone());
    }

    private static Colour[] scrollBuffer(Colour[] colours) {
        if (scrollingCounter % (double) 2 == 0) {
            for (int i = 0; i < LEDS_COUNT; i++) {
                buffer[i] = colours[(i + scrollingFirstPixel) % LEDS_COUNT];
            }
            scrollingFirstPixel = (scrollingFirstPixel + 1) % LEDS_COUNT;
        }
        scrollingCounter++;
        return buffer.clone();
    }

    /**
     * Set the buffer to flash between a set of colours. This needs to be called periodically for the
     * flashing effect to work.
     *
     * @param colours The colours to switch between.
     * @return The filled buffer.
     */
    public static Colour[] generateFlashingBuffer(Colour... colours) {
        if (colours.length == 0) return buffer;

        if (flashingCounter % 25 == 0) {
            buffer = generateSingleColourBuffer(colours[previousColour]);
            previousColour = (previousColour + 1) % colours.length;
        }

        flashingCounter++;

        return buffer;
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

        final int x = (int) (outwardPointsCounter % 11);

        for (int i = quarter - 1 - x; i < quarter + 1 + x; i++) {
            buffer[i] = new Colour(colour.red, colour.green, colour.blue);
        }

        for (int i = quarter * 3 - x; i < 2 + quarter * 3 + x; i++) {
            buffer[i] = new Colour(colour.red, colour.green, colour.blue);
        }

        outwardPointsCounter += 0.25;

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
        breathingCounter += 0.02;
        return generateSingleColourBuffer(interpolateColours(firstColour, secondColour, cosInterpolate(breathingCounter)));
    }

    private static Colour interpolateColours(Colour startColour, Colour endColour, double colourWeight) {
        final int red = (int) (endColour.red * (1 - colourWeight) + startColour.red * colourWeight);
        final int green = (int) (endColour.green * (1 - colourWeight) + startColour.green * colourWeight);
        final int blue = (int) (endColour.blue * (1 - colourWeight) + startColour.blue * colourWeight);

        return new Colour(red, green, blue);
    }

    private static double cosInterpolate(double x) {
        return (1 - Math.cos(x * Math.PI)) * 0.5;
    }
}
