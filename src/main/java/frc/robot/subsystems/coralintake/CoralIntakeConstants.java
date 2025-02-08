package frc.robot.subsystems.coralintake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorFactory;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.robot.utilities.PortsConstants.IntakePorts.BEAM_BREAK_DIO_PORT;
import static frc.robot.utilities.PortsConstants.IntakePorts.INTAKE_MOTOR_PORT;

public class CoralIntakeConstants {
    protected static final Motor INTAKE_MOTOR = MotorFactory.createTalonFX("INTAKE_MOTOR", INTAKE_MOTOR_PORT);
    protected static final Sensor INTAKE_BEAM_BREAK = SensorFactory.createDigitalInput("INTAKE_BEAM_BREAK", BEAM_BREAK_DIO_PORT);

    static {
        configureIntakeMotorConfiguration();
    }

    private static void configureIntakeMotorConfiguration() {
        final MotorConfiguration intakeMotorConfiguration = new MotorConfiguration();

        intakeMotorConfiguration.supplyCurrentLimit = 40;
        intakeMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;
        intakeMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), 1, 0.2);

        INTAKE_MOTOR.registerSignal(MotorSignal.VOLTAGE);

        INTAKE_MOTOR.configure(intakeMotorConfiguration);
    }
}