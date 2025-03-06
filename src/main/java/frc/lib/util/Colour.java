package frc.lib.util;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Colour {
    public static final Colour
            RED = new Colour(255, 0, 0),
            GREEN = new Colour(0, 255, 0),
            BLUE = new Colour(0, 0, 255),
            BLACK = new Colour(0, 0, 0),
            WHITE = new Colour(255, 255, 255),
            GRAY = new Colour(128, 128, 128),
            LIGHT_BLUE = new Colour(173, 216, 230),
            CYAN = new Colour(0, 255, 255),
            DARK_RED = new Colour(139, 0, 0),
            DARK_GREEN = new Colour(0, 100, 0),
            DARK_BLUE = new Colour(0, 0, 139),
            ORANGE = new Colour(255, 165, 0),
            YELLOW = new Colour(255, 255, 0),
            PINK = new Colour(255, 192, 203),
            PURPLE = new Colour(128, 0, 128),
            BROWN = new Colour(165, 42, 42),
            LIME = new Colour(0, 255, 0),
            MAGENTA = new Colour(255, 0, 255),
            SILVER = new Colour(192, 192, 192),
            GOLD = new Colour(255, 215, 0),
            INDIGO = new Colour(75, 0, 130),
            CORNFLOWER_BLUE = new Colour(100, 149, 237),
            ROYAL_BLUE = new Colour(65, 105, 225),
            MEDIUM_BLUE = new Colour(0, 0, 205),
            VIOLET = new Colour(238, 130, 238);

    private final int red, green, blue;

    public Colour(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public Colour(Color colour) {
        this.red = (int) (255 * colour.red);
        this.green = (int) (255 * colour.green);
        this.blue = (int) (255 * colour.blue);
    }

    public int getRed() {
        return red;
    }

    public int getGreen() {
        return green;
    }

    public int getBlue() {
        return blue;
    }

    public Color getColor() {
        return new Color(red, green, blue);
    }

    public Color8Bit getColor8Bit() {
        return new Color8Bit(red, green, blue);
    }

    public Colour toGRB() {
        return new Colour(green, red, blue);
    }
}
