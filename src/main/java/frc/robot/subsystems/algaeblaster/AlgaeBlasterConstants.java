package frc.robot.subsystems.algaeblaster;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorFactory;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.FLEX;
import static frc.robot.utilities.PortsConstants.AlgaePorts.L4_BEAM_BREAK_DIO_PORT;
import static frc.robot.utilities.PortsConstants.AlgaePorts.ALGAE_BLASTER_MOTOR_PORT;

public class AlgaeBlasterConstants {
    public enum BlasterArmState {
        HORIZONTAL_IN(0.1),
        DEFAULT_POSE(2.7),
        VERTICAL(9.54),
        SCORE_L4_START(8.562639236450195),
        SCORE_L4_END(12.623024826049805),
        INTAKE_L4(23.988644790649414),
        HORIZONTAL_OUT(19.721818923950195);

        private final double rotation;

        BlasterArmState(double rotation) {
            this.rotation = rotation;
        }

        public double getRotations() {
            return rotation;
        }
    }

    protected static final SysIdRoutine.Config BLASTER_SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(0.6),
            Second.of(4)
    );

    public static final Motor ARM_BLASTER_MOTOR = MotorFactory.createSpark("ALGAE_BLASTER_MOTOR", ALGAE_BLASTER_MOTOR_PORT,FLEX);
    protected static final Sensor L4_BEAM_BREAK = SensorFactory.createDigitalInput("L4_BEAM_BREAK", L4_BEAM_BREAK_DIO_PORT);

    protected static final double
            ARM_MINIMUM_ROTATION = -0.1,
            ARM_MAXIMUM_ROTATION = 0.6;

    protected static final SingleJointedArmMechanism2d BLASTER_ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Algae Blaster Mechanism", 4);

    static {
        configureBlasterMotor();
    }

    private static void configureBlasterMotor() {
        final MotorConfiguration blasterMotorConfiguration = new MotorConfiguration();

        blasterMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        blasterMotorConfiguration.slot = new MotorProperties.Slot(0.145, 0, 0, 0.084973, 0, 0.13081, 0, Feedforward.Type.ARM);

        blasterMotorConfiguration.profileMaxVelocity = 60;
        blasterMotorConfiguration.profileMaxAcceleration = 37;

        blasterMotorConfiguration.closedLoopTolerance = 0.03;

        blasterMotorConfiguration.supplyCurrentLimit = 44;

        blasterMotorConfiguration.simulationSlot = new MotorProperties.Slot(80, 0, 1, 0, 0, 0);
        blasterMotorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ARM,
                DCMotor.getFalcon500(1),
                1,
                0.5,
                0.01,
                Rotation2d.fromRotations(ARM_MINIMUM_ROTATION),
                Rotation2d.fromRotations(ARM_MAXIMUM_ROTATION),
                true);

        ARM_BLASTER_MOTOR.configure(blasterMotorConfiguration);

        ARM_BLASTER_MOTOR.setMotorEncoderPosition(0);

        ARM_BLASTER_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ARM_BLASTER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ARM_BLASTER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ARM_BLASTER_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        ARM_BLASTER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        ARM_BLASTER_MOTOR.setMotorEncoderPosition(0);
    }
}