package frc.lib.generic.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulationProperties {
    public enum SimulationType {
        SIMPLE_MOTOR, ELEVATOR, ARM
    }

    public static class Slot {
        private final SimulationType type;

        private final DCMotor gearbox;
        private final double gearRatio;

        private double momentOfInertia;

        private double armLengthMeters;
        private Rotation2d minimumAngle;
        private Rotation2d maximumAngle;

        private boolean simulateGravity;

        private double carriageMassKilograms;
        private double drumRadiusMeters;
        private double minimumHeightMeters;
        private double maximumHeightMeters;
        private double kv;
        private double ka;

        // Constructor for arm
        public Slot(SimulationType type, DCMotor gearbox, double gearRatio, double armLengthMeters,
                    double momentOfInertia, Rotation2d minimumAngle, Rotation2d maximumAngle, boolean simulateGravity) {
            this.type = type;
            this.gearbox = gearbox;
            this.gearRatio = gearRatio;
            this.armLengthMeters = armLengthMeters;
            this.momentOfInertia = momentOfInertia;
            this.minimumAngle = minimumAngle;
            this.maximumAngle = maximumAngle;
            this.simulateGravity = simulateGravity;
        }

        // Constructor for elevator
        public Slot(SimulationType type, DCMotor gearbox, double gearRatio, double carriageMassKilograms, double drumRadiusMeters,
                    double minimumHeightMeters, double maximumHeightMeters, boolean simulateGravity) {
            this.type = type;
            this.gearbox = gearbox;
            this.gearRatio = gearRatio;
            this.carriageMassKilograms = carriageMassKilograms;
            this.drumRadiusMeters = drumRadiusMeters;
            this.minimumHeightMeters = minimumHeightMeters;
            this.maximumHeightMeters = maximumHeightMeters;
            this.simulateGravity = simulateGravity;
        }

        // Constructor for simple + flywheel
        public Slot(SimulationType type, DCMotor gearbox, double gearRatio, double momentOfInertia) {
            this.type = type;
            this.gearbox = gearbox;
            this.gearRatio = gearRatio;
            this.momentOfInertia = momentOfInertia;
        }

        // Constructor for flywheel
        public Slot(SimulationType type, DCMotor gearbox, double gearRatio, double kv, double ka) {
            this.type = type;
            this.gearbox = gearbox;
            this.gearRatio = gearRatio;
            this.kv = kv;
            this.ka = ka;
        }

        public GenericPhysicsSimulation getSimulationType() {
            if (type == null)
                return null;

            return switch (type) {
                case SIMPLE_MOTOR -> {
                    if (kv == 0 && ka == 0) {
                        yield new SimpleMotorSimulation(gearbox, gearRatio, momentOfInertia);
                    } else {
                        yield new SimpleMotorSimulation(kv, ka, gearbox, gearRatio);
                    }
                }

                case ELEVATOR ->
                        new ElevatorSimulation(gearbox, gearRatio, carriageMassKilograms, drumRadiusMeters, minimumHeightMeters, maximumHeightMeters, simulateGravity);
                case ARM ->
                        new SingleJointedArmSimulation(gearbox, gearRatio,
                                armLengthMeters, momentOfInertia,
                                minimumAngle, maximumAngle, simulateGravity);
            };
        }
    }
}
