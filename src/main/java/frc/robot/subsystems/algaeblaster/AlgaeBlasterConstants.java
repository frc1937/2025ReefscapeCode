package frc.robot.subsystems.algaeblaster;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.robot.utilities.PortsConstants.AlgaePorts.ALGAE_BLASTER_MOTOR_PORT;

public class AlgaeBlasterConstants {
    public enum BlasterArmState {
        VERTICAL(Rotation2d.kCW_90deg),
        HORIZONTAL_IN(Rotation2d.k180deg),
        HORIZONTAL_OUT(Rotation2d.kZero);

        private final Rotation2d rotation;

        BlasterArmState(Rotation2d rotation) {
            this.rotation = rotation;
        }

        public Rotation2d getRotation2d() {
            return rotation;
        }
    }

    protected static final SysIdRoutine.Config BLASTER_SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(7)
    );

    protected static final Motor BLASTER_MOTOR = MotorFactory.createSpark("ALGAE_BLASTER_MOTOR", ALGAE_BLASTER_MOTOR_PORT, MAX);
    protected static final Rotation2d ARM_MINIMUM_ROTATION = Rotation2d.fromDegrees(0),
            ARM_MAXIMUM_ROTATION = Rotation2d.fromDegrees(180);

    protected static final SingleJointedArmMechanism2d BLASTER_ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("ALGAE_BLASTER_MECHANISM", 4);

    static {
        configureBlasterMotor();
    }

    private static void configureBlasterMotor() {
        final MotorConfiguration blasterMotorConfiguration = new MotorConfiguration();
        blasterMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        BLASTER_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        BLASTER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        BLASTER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        BLASTER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        blasterMotorConfiguration.slot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        blasterMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        blasterMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 1, 0.5, 0.2, ARM_MINIMUM_ROTATION, ARM_MAXIMUM_ROTATION, true);

        blasterMotorConfiguration.profileMaxVelocity = 2;
        blasterMotorConfiguration.profileMaxAcceleration = 3;
        blasterMotorConfiguration.closedLoopTolerance = 0.5;

        blasterMotorConfiguration.supplyCurrentLimit = 30;

        blasterMotorConfiguration.forwardSoftLimit = ARM_MAXIMUM_ROTATION.getRotations();
        blasterMotorConfiguration.reverseSoftLimit = ARM_MINIMUM_ROTATION.getRotations();

        BLASTER_MOTOR.configure(blasterMotorConfiguration);
    }
}