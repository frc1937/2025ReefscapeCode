package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.ElevatorMechanism2d;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.PortsConstants.ElevatorPorts.MASTER_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.ElevatorPorts.SLAVE_MOTOR_PORT;

public class ElevatorConstants {
    public enum ElevatorHeight {
        L1(0.3),
        L2(1.03),
        L2_AND_A_HALF(2.96),
        L3(2.96),
        L4(3.11),
        REMOVE_ALGAE_FROM_L3(1.92),
        REMOVE_ALGAE_FROM_L2(0.1),
        FEEDER(0.13),
        GO_LOW(0.2);

        private final double rotations;

        ElevatorHeight(double mechanismRotations) {
            this.rotations = mechanismRotations;
        }

        public double getRotations() {
            return rotations;
        }

        public ElevatorHeight getMiddlePoint() {
            if (this == L1) return L1;
            if (this == L2) return L2;
            if (this == L3) return L2_AND_A_HALF;
            if (this == L4) return L2_AND_A_HALF;

            return L1;
        }
    }

    protected static final SysIdRoutine.Config ELEVATOR_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(0.5),
            Volts.of(1),
            Second.of(7)
    );

    public static final Motor
            MASTER_MOTOR = MotorFactory.createSpark("ELEVATOR_MASTER_MOTOR", MASTER_MOTOR_PORT, MotorProperties.SparkType.FLEX),
            SLAVE_MOTOR = MotorFactory.createSpark("ELEVATOR_SLAVE_MOTOR", SLAVE_MOTOR_PORT, MotorProperties.SparkType.FLEX);

    protected static final double
            ELEVATOR_MAX_EXTENSION_ROTATIONS = 2.2483952045440674,
            WHEEL_DIAMETER = 0.0328,
            GEAR_RATIO = 4*(10/3.0),
            KG = 0.18348;

    protected static final ElevatorMechanism2d ELEVATOR_MECHANISM = MechanismFactory.createElevatorMechanism("Elevator Mechanism", 1);

    static {
        configureMotors();
    }

    private static void configureMotors() {
        final MotorConfiguration ELEVATOR_MOTORS_CONFIGURATION = new MotorConfiguration();

        ELEVATOR_MOTORS_CONFIGURATION.closedLoopTolerance = 0.05;

        ELEVATOR_MOTORS_CONFIGURATION.idleMode = MotorProperties.IdleMode.BRAKE;

        ELEVATOR_MOTORS_CONFIGURATION.profileMaxVelocity = 8;
        ELEVATOR_MOTORS_CONFIGURATION.profileMaxAcceleration = 75;

        ELEVATOR_MOTORS_CONFIGURATION.supplyCurrentLimit = 63;

        ELEVATOR_MOTORS_CONFIGURATION.inverted = true;
        ELEVATOR_MOTORS_CONFIGURATION.gearRatio = GEAR_RATIO;

        ELEVATOR_MOTORS_CONFIGURATION.slot = new MotorProperties.Slot(
                //1.1196,0,0,
                2.1,0,0,
                1.466,0,0.094165, KG, Feedforward.Type.ELEVATOR);

        ELEVATOR_MOTORS_CONFIGURATION.simulationSlot = new MotorProperties.Slot(17.5, 0, 0.6, 0, 0, 0, 0, Feedforward.Type.ELEVATOR);// S=1.313
        ELEVATOR_MOTORS_CONFIGURATION.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ELEVATOR,
                DCMotor.getNeoVortex(2),
                1,
                6,
                WHEEL_DIAMETER / 2,
                0.1,
                1.9,
                false
        );

        MASTER_MOTOR.configure(ELEVATOR_MOTORS_CONFIGURATION);
        SLAVE_MOTOR.configure(ELEVATOR_MOTORS_CONFIGURATION);

        MASTER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        MASTER_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        MASTER_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        MASTER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        MASTER_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);
        MASTER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        SLAVE_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        SLAVE_MOTOR.setFollower(MASTER_MOTOR, true);
    }
}