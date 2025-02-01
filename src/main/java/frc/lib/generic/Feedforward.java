package frc.lib.generic;

/**
 * kS; Volts required to overcome the force of static friction <p>
 * kV; Volts required to maintain a velocity of one unit <p>
 * kA; Volts required to accelerate the mechanism by one unit <p>
 * kG; Volts required to overcome the force of gravity
 */
public class Feedforward {
    public static class FeedForwardConstants {
        public final double kS, kV, kA, kG;

        public FeedForwardConstants(double kS, double kV, double kA, double kG) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = kG;
        }

        @Override
        public String toString() {
            return "kS: " + kS + " kG: " + kG + " kA" + kA + " Kv " + kV;
        }
    }

    public enum Type {
        SIMPLE,
        ARM,
        ELEVATOR
    }

    private final FeedForwardConstants constants;
    private final Type type;

    public Feedforward(Type type, FeedForwardConstants constants) {
        this.type = type;
        this.constants = constants;
    }

    public FeedForwardConstants getConstants() {
        return constants;
    }

    public double calculate(double currentPositionRotations, double velocityRPS, double accelerationRPSPS) {
        double feedforward = constants.kS * Math.signum(velocityRPS) + constants.kV * velocityRPS + constants.kA * accelerationRPSPS;

        if (type == Type.ARM) {
            feedforward += constants.kG * Math.cos(currentPositionRotations * 2 * Math.PI);
        } else if (type == Type.ELEVATOR) {
            feedforward += constants.kG;
        }

        return feedforward;
    }

    public double calculate(double velocityRPS, double accelerationRPSPS) {
        return calculate(0, velocityRPS, accelerationRPSPS);
    }
}
