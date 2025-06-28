package frc.lib.util;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Colour {
    public static final Colour
            RED = new Colour(255, 0, 0).toGRB(),
            GREEN = new Colour(0, 255, 0).toGRB(),
            BLUE = new Colour(0, 0, 255).toGRB(),
            BLACK = new Colour(0, 0, 0).toGRB(),
            WHITE = new Colour(255, 255, 255).toGRB(),
            GRAY = new Colour(128, 128, 128).toGRB(),
            LIGHT_BLUE = new Colour(173, 216, 230).toGRB(),
            CYAN = new Colour(0, 255, 255).toGRB(),
            DARK_RED = new Colour(139, 0, 0).toGRB(),
            DARK_GREEN = new Colour(0, 100, 0).toGRB(),
            DARK_BLUE = new Colour(0, 0, 139).toGRB(),
            ORANGE = new Colour(255, 165, 0).toGRB(),
            YELLOW = new Colour(255, 255, 0).toGRB(),
            PINK = new Colour(255, 192, 203).toGRB(),
            PURPLE = new Colour(128, 0, 128).toGRB(),
            BROWN = new Colour(165, 42, 42).toGRB(),
            LIME = new Colour(0, 255, 0).toGRB(),
            MAGENTA = new Colour(255, 0, 255).toGRB(),
            SILVER = new Colour(192, 192, 192).toGRB(),
            GOLD = new Colour(255, 215, 0).toGRB(),
            INDIGO = new Colour(75, 0, 130).toGRB(),
            CORNFLOWER_BLUE = new Colour(100, 149, 237).toGRB(),
            ROYAL_BLUE = new Colour(65, 105, 225).toGRB(),
            MEDIUM_BLUE = new Colour(0, 0, 205).toGRB(),
            VIOLET = new Colour(238, 130, 238).toGRB(),
            NAVY_BLUE = new Colour(0, 0, 128).toGRB(),
            SKY_BLUE = new Colour(135, 206, 235).toGRB(),
            AQUA = new Colour(0, 255, 255).toGRB();

    public final int red, green, blue;

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
