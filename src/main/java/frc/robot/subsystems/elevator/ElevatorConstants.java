package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorFactory;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.ElevatorMechanism2d;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.math.Conversions;
import frc.robot.GlobalConstants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.utilities.PortsConstants.ElevatorPorts.*;

public class ElevatorConstants {
    public enum ElevatorHeight {
        L1(0),
        L2(0.1769),
        L3(0.579),
        FEEDER(0.172),
        CLIMB(-0.02);

        private final double rotations;
        private final double meters;

        ElevatorHeight(double meters) {
            this.rotations = Conversions.metresToRotations(meters, WHEEL_DIAMETER);
            this.meters = meters;
        }

        public double getRotations() {
            return rotations;
        }

        public double getMeters() {
            return meters;
        }
    }

    protected static final SysIdRoutine.Config ELEVATOR_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(7)
    );

    protected static final Motor
            MASTER_MOTOR = MotorFactory.createSpark("ELEVATOR_MASTER_MOTOR", MASTER_MOTOR_PORT, MotorProperties.SparkType.MAX),
            SLAVE_MOTOR = MotorFactory.createSpark("ELEVATOR_SLAVE_MOTOR", SLAVE_MOTOR_PORT, MotorProperties.SparkType.MAX);

    protected static final Sensor
            TOP_BEAM_BREAK = SensorFactory.createDigitalInput("TOP_BEAM_BREAKER", TOP_BEAM_BREAK_DIO_PORT),
            BOTTOM_BEAM_BREAK = SensorFactory.createDigitalInput("BUTTON_BEAM_BREAKER", BOTTOM_BEAM_BREAK_DIO_PORT);

    protected static final double
            ELEVATOR_MAX_EXTENSION_ROTATIONS = 10, //TODO: TUNE
            ELEVATOR_MIN_EXTENSION_ROTATIONS = 0, //TODO: TUNE
            WHEEL_DIAMETER = 0.04;

    protected static final ElevatorMechanism2d ELEVATOR_MECHANISM = MechanismFactory.createElevatorMechanism("Elevator Mechanism", 1);

    static {
        configureMotors();
    }

    private static void configureMotors() {
        final MotorConfiguration ELEVATOR_MOTORS_CONFIGURATION = new MotorConfiguration();

        SLAVE_MOTOR.setFollowerOf(MASTER_MOTOR, true);

        ELEVATOR_MOTORS_CONFIGURATION.forwardSoftLimit = ELEVATOR_MAX_EXTENSION_ROTATIONS;
        ELEVATOR_MOTORS_CONFIGURATION.reverseSoftLimit = ELEVATOR_MIN_EXTENSION_ROTATIONS; //ASSUMING FORWARD IS +Voltage. TODO TUNE

        ELEVATOR_MOTORS_CONFIGURATION.closedLoopTolerance = 0.05;

        ELEVATOR_MOTORS_CONFIGURATION.idleMode = MotorProperties.IdleMode.BRAKE;
        ELEVATOR_MOTORS_CONFIGURATION.simulationSlot = new MotorProperties.Slot(17.5, 0, 0.6, 0, 0, 0, 0, MotorProperties.GravityType.ELEVATOR);// S=1.313

        ELEVATOR_MOTORS_CONFIGURATION.profileMaxVelocity = 25;
        ELEVATOR_MOTORS_CONFIGURATION.profileMaxAcceleration = 25;
        ELEVATOR_MOTORS_CONFIGURATION.profileMaxJerk =
                CURRENT_MODE == GlobalConstants.Mode.SIMULATION ? 250 : 0;

        ELEVATOR_MOTORS_CONFIGURATION.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ELEVATOR,
                DCMotor.getNeoVortex(2),
                1,
                6,
                WHEEL_DIAMETER / 2,
                0.1,
                1.9,
                true
        );

        MASTER_MOTOR.registerSignal(MotorSignal.VOLTAGE);
        MASTER_MOTOR.registerSignal(MotorSignal.POSITION);
        MASTER_MOTOR.registerSignal(MotorSignal.VELOCITY);
        MASTER_MOTOR.registerSignal(MotorSignal.ACCELERATION);
        MASTER_MOTOR.registerSignal(MotorSignal.CLOSED_LOOP_TARGET);

        MASTER_MOTOR.configure(ELEVATOR_MOTORS_CONFIGURATION);
        SLAVE_MOTOR.configure(ELEVATOR_MOTORS_CONFIGURATION);
    }
}