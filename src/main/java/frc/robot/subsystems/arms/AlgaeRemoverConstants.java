package frc.robot.subsystems.arms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.MAX;
import static frc.robot.utilities.PortsConstants.ArmPorts.ALGAE_REMOVER_MOTOR_PORT;

public class AlgaeRemoverConstants {
    protected static final Motor ALGAE_REMOVER_MOTOR = MotorFactory.createSpark("ALGAE_REMOVER_MOTOR", ALGAE_REMOVER_MOTOR_PORT, MAX);

    protected static final Rotation2d
            MINIMUM_ROTATION = Rotation2d.fromDegrees(0),
            MAXIMUM_ROTATION = Rotation2d.fromDegrees(180);

    protected static final SingleJointedArmMechanism2d algaeRemoverMechanism = new SingleJointedArmMechanism2d("ALGAE_REMOVER_MECHANISM", 10);

    static {
        configureAlgaeArmMotor();
    }

    private static void configureAlgaeArmMotor() {
        final MotorConfiguration algaeRemoverMotorConfiguration = new MotorConfiguration();
        algaeRemoverMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        ALGAE_REMOVER_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ALGAE_REMOVER_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ALGAE_REMOVER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ALGAE_REMOVER_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        algaeRemoverMotorConfiguration.simulationSlot = new MotorProperties.Slot(100, 0, 0, 0, 0, 0);
        algaeRemoverMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM, DCMotor.getFalcon500(1), 1, 0.5, 0.2, MINIMUM_ROTATION, MAXIMUM_ROTATION, true);

        ALGAE_REMOVER_MOTOR.configure(algaeRemoverMotorConfiguration);
    }
}
