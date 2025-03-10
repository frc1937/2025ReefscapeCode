package frc.robot.subsystems.algaeblaster;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.PortsConstants.AlgaePorts.ALGAE_BLASTER_MOTOR_PORT;

public class AlgaeBlasterConstants {
    public enum BlasterArmState {
        HORIZONTAL_IN(0),
        VERTICAL(0.47),
        HORIZONTAL_OUT(0.97);

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
            Volts.of(1),
            Second.of(3)
    );

    public static final Motor ARM_BLASTER_MOTOR = MotorFactory.createTalonFX("ALGAE_BLASTER_MOTOR", ALGAE_BLASTER_MOTOR_PORT);

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

        ARM_BLASTER_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ARM_BLASTER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ARM_BLASTER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ARM_BLASTER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        blasterMotorConfiguration.slot = new MotorProperties.Slot(9, 0, 0, 0, 0, 0.153, 0.201, Feedforward.Type.ARM);

        blasterMotorConfiguration.profileMaxVelocity = 2.8;
        blasterMotorConfiguration.profileMaxAcceleration = 2.7;

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
    }
}