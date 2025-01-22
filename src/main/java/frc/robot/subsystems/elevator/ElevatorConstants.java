package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorFactory;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.ElevatorMechanism2d;
import frc.lib.math.Conversions;
import frc.robot.GlobalConstants;
import frc.robot.utilities.PortsConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.utilities.PortsConstants.ElevatorPorts.*;

public class ElevatorConstants {
    protected static final Motor
            MASTER_MOTOR = MotorFactory.createSpark("Elevator Master Motor", PortsConstants.ElevatorPorts.MASTER_MOTOR_PORT, MotorProperties.SparkType.MAX),
            SLAVE_MOTOR = MotorFactory.createSpark("Elevator Slave Motor", SLAVE_MOTOR_PORT, MotorProperties.SparkType.MAX);

    protected static final Sensor
            TOP_BEAM_BREAK = SensorFactory.createDigitalInput("Top Beam Breaker", TOP_BEAM_BREAK_DIO_PORT),
            BOTTOM_BEAM_BREAK = SensorFactory.createDigitalInput("Button Beam Breaker", BOTTOM_BEAM_BREAK_DIO_PORT);

    protected static final double
            ELEVATOR_MAX_EXTENSION_METERS = 0.86,
            WHEEL_DIAMETER = 0.04;

    protected static final ElevatorMechanism2d ELEVATOR_MECHANISM = new ElevatorMechanism2d("Elevator Mechanism", 1);

    public enum ElevatorHeight {
        L1(0.457), L2(0.793), L3(1.196),
        FEEDER(0.93), CLIMB(0);

        public final double rotations;
        public final double meters;

        ElevatorHeight(double meters) {
            this.rotations = Conversions.metresToRotations(meters, WHEEL_DIAMETER);
            this.meters = meters;
        }
    }

    static {
        configureMotorConfiguration();
    }

    private static void configureMotorConfiguration() {
        final MotorConfiguration ELEVATOR_MOTORS_CONFIGURATION = new MotorConfiguration();

        SLAVE_MOTOR.setFollowerOf(MASTER_MOTOR, true);

        ELEVATOR_MOTORS_CONFIGURATION.idleMode = MotorProperties.IdleMode.BRAKE;
        ELEVATOR_MOTORS_CONFIGURATION.simulationSlot = new MotorProperties.Slot(12.33, 0, 2.5, 0, 0,  1.311, 0, MotorProperties.GravityType.ELEVATOR);// S=1.313

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

        MASTER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        MASTER_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        MASTER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        MASTER_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);
        MASTER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        MASTER_MOTOR.configure(ELEVATOR_MOTORS_CONFIGURATION);
        SLAVE_MOTOR.configure(ELEVATOR_MOTORS_CONFIGURATION);
    }
}