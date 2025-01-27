package frc.robot.subsystems.algaearms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.robot.utilities.PortsConstants.AlgaePorts.ALGAE_OUTTAKE_MOTOR_PORT;

public class AlgaeOuttakeConstants {
    protected static final Motor OUTTAKE_MOTOR = MotorFactory.createSpark("ALGAE_OUTTAKE_MOTOR", ALGAE_OUTTAKE_MOTOR_PORT, MAX);

    protected static final Rotation2d
            MINIMUM_ROTATION = Rotation2d.fromDegrees(0),
            MAXIMUM_ROTATION = Rotation2d.fromDegrees(180);

    protected static final SingleJointedArmMechanism2d OUTTAKE_ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("ALGAE_OUTTAKE_MECHANISM", 4);

    static {
        configureOuttakeMotor();
    }

    private static void configureOuttakeMotor() {
        final MotorConfiguration outtakeMotorConfiguration = new MotorConfiguration();
        outtakeMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        OUTTAKE_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        OUTTAKE_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        OUTTAKE_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        OUTTAKE_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        outtakeMotorConfiguration.slot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        outtakeMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        outtakeMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 1, 0.5, 0.2, MINIMUM_ROTATION, MAXIMUM_ROTATION, true);
        outtakeMotorConfiguration.supplyCurrentLimit = 30;

        OUTTAKE_MOTOR.configure(outtakeMotorConfiguration);
    }

    public enum OuttakeArmState {
        VERTICAL(Rotation2d.kCW_90deg),
        HORIZONTAL_IN(Rotation2d.k180deg),
        HORIZONTAL_OUT(Rotation2d.kZero);

        private final Rotation2d rotation;

        OuttakeArmState(Rotation2d rotation) {
            this.rotation = rotation;
        }

        public Rotation2d getRotation2d() {
            return rotation;
        }
    }
}