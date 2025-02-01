package frc.lib.generic.hardware.motor;

public class MotorProperties {
    public enum IdleMode {
        COAST,
        BRAKE
    }

    public enum SparkType {
        MAX,
        FLEX;
    }

    public enum GravityType {
        SIMPLE,
        ARM,
        ELEVATOR
    }

    /**
     * Enumeration of different control modes for a motor.
     */
    public enum ControlMode {
        /**
         * Control the motor output based on the desired current.
         * <p>Control type: Current control</p>
         * <p>Units: Amperes (A)</p>
         */
        CURRENT(),
//TODO: Realize how to do control wtihout siwtch statement in here.
        /**
         * Control the motor output based on the desired voltage.
         * <p>Control type: Voltage control</p>
         * <p>Units: Volts (V)</p>
         */
        VOLTAGE(),

        /**
         * Control the motor to achieve a specific position.
         * <p>Control type: Position control</p>
         * <p>Units: Rotations</p>
         */
        POSITION(),

        /**
         * Control the motor to achieve a specific velocity.
         * <p>Control type: Velocity control</p>
         * <p>Units: Rotations per second (RPS)</p>
         */
        VELOCITY();
    }

    public static final class Slot {
        private final double kP, kD, kI, kV, kA, kS, kG;
        private final GravityType gravityType;

        public Slot(double kP, double kI, double kD, double kV, double kA, double kS, double kG, GravityType gravityType) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kA = kA;
            this.kS = kS;
            this.kG = kG;
            this.gravityType = gravityType;
        }

        public Slot(double kP, double kI, double kD, double kV, double kA, double kS) {
            this(kP, kI, kD, kV, kA, kS, 0, GravityType.SIMPLE);
        }

        public Slot(double kP, double kI, double kD) {
            this(kP, kI, kD, 0, 0, 0, 0, null);
        }

        public double kP() {
            return kP;
        }

        public double kD() {
            return kD;
        }

        public double kI() {
            return kI;
        }

        public double kV() {
            return kV;
        }

        public double kA() {
            return kA;
        }

        public double kS() {
            return kS;
        }

        public double kG() {
            return kG;
        }

        public GravityType gravityType() {
            return gravityType;
        }
    }
}
