package frc.robot.subsystems.algaearms;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.robot.utilities.PortsConstants.ArmPorts.ALGAE_ARM_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.ArmPorts.ALGAE_INTAKE_MOTOR_PORT;

public class AlgaeIntakeConstants {
    protected static final Motor ARM_MOTOR = MotorFactory.createSpark("ALGAE_ARM_MOTOR", ALGAE_ARM_MOTOR_PORT, MAX);
    protected static final Motor INTAKE_MOTOR = MotorFactory.createSpark("ALGAE_INTAKE_MOTOR", ALGAE_INTAKE_MOTOR_PORT, MAX);

    protected static final SingleJointedArmMechanism2d ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("ALGAE_ARM_MECHANISM", 3);

    static {
        configureArmMotor();
        configureIntakeMotor();
    }

    private static void configureArmMotor() {
        final MotorConfiguration armMotorConfiguration = new MotorConfiguration();
        armMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);

        armMotorConfiguration.slot = new MotorProperties.Slot(0.2, 0.01, 0.0, 0.0, 0.0, 0.0);
        armMotorConfiguration.simulationSlot = new MotorProperties.Slot(0.2, 0.01, 0.0, 0.0, 0.0, 0.0);
        armMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 1.0, 0.1, 0.2, -90, 90, false);

        ARM_MOTOR.configure(armMotorConfiguration);
    }

    private static void configureIntakeMotor() {
        final MotorConfiguration intakeMotorConfiguration = new MotorConfiguration();
        intakeMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;

        INTAKE_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        INTAKE_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);

        intakeMotorConfiguration.slot = new MotorProperties.Slot(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
        intakeMotorConfiguration.simulationSlot = new MotorProperties.Slot(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
        intakeMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), 1, 0, 0);

        INTAKE_MOTOR.configure(intakeMotorConfiguration);
    }
}