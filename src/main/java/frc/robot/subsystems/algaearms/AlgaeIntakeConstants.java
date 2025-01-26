package frc.robot.subsystems.algaearms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.robot.utilities.PortsConstants.AlgaePorts.ALGAE_ARM_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.AlgaePorts.ALGAE_INTAKE_MOTOR_PORT;

public class AlgaeIntakeConstants {
    protected static final Motor INTAKE_ARM_MOTOR = MotorFactory.createSpark("ALGAE_INTAKE_ARM_MOTOR", ALGAE_ARM_MOTOR_PORT, MAX);
    protected static final Motor INTAKE_MOTOR = MotorFactory.createSpark("ALGAE_INTAKE_MOTOR", ALGAE_INTAKE_MOTOR_PORT, MAX);

    protected static final SingleJointedArmMechanism2d INTAKE_ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("INTAKE_ARM_MECHANISM", 3);

    static {
        configureIntakeArmMotor();
        configureIntakeMotor();
    }

    private static void configureIntakeArmMotor() {
        final MotorConfiguration intakeArmMotorConfiguration = new MotorConfiguration();
        intakeArmMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        INTAKE_ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        INTAKE_ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        INTAKE_ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        INTAKE_ARM_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        intakeArmMotorConfiguration.slot = new MotorProperties.Slot(0.2, 0.01, 0.0, 0.0, 0.0, 0.0);
        intakeArmMotorConfiguration.simulationSlot = new MotorProperties.Slot(0.2, 0.01, 0.0, 0.0, 0.0, 0.0);
        intakeArmMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 1.0, 0.1, 0.2, -90, 90, false);

        INTAKE_ARM_MOTOR.configure(intakeArmMotorConfiguration);
    }

    private static void configureIntakeMotor() {
        final MotorConfiguration intakeMotorConfiguration = new MotorConfiguration();
        intakeMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;

        INTAKE_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);

        intakeMotorConfiguration.slot = new MotorProperties.Slot(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
        intakeMotorConfiguration.simulationSlot = new MotorProperties.Slot(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
        intakeMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), 1, 0, 0);

        INTAKE_MOTOR.configure(intakeMotorConfiguration);
    }

    public enum IntakeArmState {
        EXTENDED(Rotation2d.kZero),
        RETRACTED(Rotation2d.kCW_90deg);

        private final Rotation2d rotation;

        IntakeArmState(Rotation2d rotation) {
            this.rotation = rotation;
        }

        public Rotation2d getRotation2d() {
            return rotation;
        }
    }
}