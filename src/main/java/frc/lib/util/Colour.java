package frc.lib.util;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Colour {
    public static final Colour
            RED = new Colour(Color.kRed),
            GREEN = new Colour(Color.kGreen),
            BLUE = new Colour(Color.kBlue),
            BLACK = new Colour(Color.kBlack),
            WHITE = new Colour(Color.kWhite),
            GRAY = new Colour(Color.kGray),
            LIGHT_BLUE = new Colour(Color.kLightBlue),
            CYAN = new Colour(Color.kCyan),
            DARK_RED = new Colour(Color.kDarkRed),
            DARK_GREEN = new Colour(Color.kDarkGreen),
            DARK_BLUE = new Colour(Color.kDarkBlue),
            ORANGE = new Colour(Color.kOrange),
            YELLOW = new Colour(Color.kGold),
            PINK = new Colour(Color.kPink),
            PURPLE = new Colour(Color.kPurple),
            BROWN = new Colour(Color.kBrown),
            LIME = new Colour(Color.kLime),
            MAGENTA = new Colour(Color.kMagenta),
            SILVER = new Colour(Color.kSilver),
            GOLD = new Colour(Color.kGold),
            INDIGO = new Colour(Color.kIndigo),
            CORNFLOWER_BLUE = new Colour(Color.kCornflowerBlue),
            ROYAL_BLUE = new Colour(Color.kRoyalBlue),
            MEDIUM_BLUE = new Colour(Color.kMediumBlue),
            VIOLET = new Colour(Color.kViolet);

    private final int red;
    private final int green;
    private final int blue;

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
