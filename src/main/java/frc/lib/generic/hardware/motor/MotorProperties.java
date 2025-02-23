package frc.lib.generic.hardware.motor;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.hardware.rev.GenericSparkFlex;
import frc.lib.generic.hardware.motor.hardware.rev.GenericSparkMax;

public class MotorProperties {
    public enum IdleMode {
        COAST {
            @Override
            public SparkBaseConfig.IdleMode getSparkIdleMode() { return SparkBaseConfig.IdleMode.kCoast; }

            @Override
            public NeutralModeValue getCTREIdleMode() { return NeutralModeValue.Coast; }
        },

        BRAKE {
            @Override
            public SparkBaseConfig.IdleMode getSparkIdleMode() {return SparkBaseConfig.IdleMode.kBrake;}

            @Override
            public NeutralModeValue getCTREIdleMode() { return NeutralModeValue.Brake;}
        };

        public abstract SparkBaseConfig.IdleMode getSparkIdleMode();
        public abstract NeutralModeValue getCTREIdleMode();
    }

    public enum SparkType {
        MAX {
            @Override
            public Motor getSpark(String name, int deviceId) {
                return new GenericSparkMax(name, deviceId);
            }
        },

        FLEX {
            @Override
            public Motor getSpark(String name, int deviceId) {
                return new GenericSparkFlex(name, deviceId);
            }
        };

        public abstract Motor getSpark(String name, int deviceId);
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
        public final double kP, kD, kI, kV, kA, kS, kG;
        public final Feedforward.Type feedforwardType;

        public Slot(double kP, double kI, double kD, double kV, double kA, double kS, double kG, Feedforward.Type feedforwardType) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kA = kA;
            this.kS = kS;
            this.kG = kG;
            this.feedforwardType = feedforwardType;
        }

        public Slot(double kP, double kI, double kD, double kV, double kA, double kS) {
            this(kP, kI, kD, kV, kA, kS, 0, Feedforward.Type.SIMPLE);
        }

        public Slot(double kP, double kI, double kD) {
            this(kP, kI, kD, 0, 0, 0, 0, null);
        }
    }
}
