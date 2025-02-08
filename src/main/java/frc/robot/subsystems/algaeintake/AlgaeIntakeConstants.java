package frc.robot.subsystems.algaeintake;

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
import static frc.robot.utilities.PortsConstants.AlgaePorts.ALGAE_ARM_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.AlgaePorts.ALGAE_INTAKE_MOTOR_PORT;

public class AlgaeIntakeConstants {
    public enum IntakeArmState {
        EXTENDED(0.03, -2),
        RETRACTED(-0.25, 0);

        private final double
                armTargetPosition,
                intakeSpeed;

        IntakeArmState(double armTargetPosition, double intakeSpeed) {
            this.armTargetPosition = armTargetPosition;
            this.intakeSpeed = intakeSpeed;
        }

        public double getTargetArmPositionRotations() {
            return armTargetPosition;
        }

        public double getRollerVoltage() {
            return intakeSpeed;
        }
    }

    protected static final SysIdRoutine.Config INTAKE_ARM_SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(7)
    );

    protected static final Motor
            INTAKE_ARM_MOTOR = MotorFactory.createSpark("ALGAE_INTAKE_ARM_MOTOR", ALGAE_ARM_MOTOR_PORT, MAX),
            INTAKE_MOTOR = MotorFactory.createSpark("ALGAE_INTAKE_MOTOR", ALGAE_INTAKE_MOTOR_PORT, MAX);

    protected static final double
            ARM_MAXIMUM_POSITION_ROTATIONS = 0.7, //TODO TUNE
            ARM_MINIMUM_POSITION_ROTATIONS = -0.01; //TODO TUNE

    protected static final SingleJointedArmMechanism2d INTAKE_ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("IntakeArmMechanism", 3);

    static {
        configureIntakeArmMotor();
        configureIntakeMotor();
    }

    private static void configureIntakeArmMotor() {
        final MotorConfiguration intakeArmMotorConfiguration = new MotorConfiguration();

        intakeArmMotorConfiguration.slot = new MotorProperties.Slot(0.2, 0, 0, 0, 0, 0);
        intakeArmMotorConfiguration.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        intakeArmMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.ARM,
                DCMotor.getFalcon500(1),
                1.0, 0.1, 0.2, Rotation2d.fromRotations(ARM_MINIMUM_POSITION_ROTATIONS),
                Rotation2d.fromRotations(ARM_MAXIMUM_POSITION_ROTATIONS), false);

        intakeArmMotorConfiguration.profileMaxVelocity = 1;
        intakeArmMotorConfiguration.profileMaxAcceleration = 3;
        intakeArmMotorConfiguration.closedLoopTolerance = 0.1;

        intakeArmMotorConfiguration.supplyCurrentLimit = 30;
        intakeArmMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        INTAKE_ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        INTAKE_ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        INTAKE_ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        INTAKE_ARM_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        INTAKE_ARM_MOTOR.configure(intakeArmMotorConfiguration);
    }

    private static void configureIntakeMotor() {
        final MotorConfiguration intakeMotorConfiguration = new MotorConfiguration();
        intakeMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;

        INTAKE_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);

        intakeMotorConfiguration.slot = new MotorProperties.Slot(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
        intakeMotorConfiguration.simulationSlot = new MotorProperties.Slot(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
        intakeMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getFalcon500(1), 1, 0.003);

        intakeMotorConfiguration.supplyCurrentLimit = 20;

        INTAKE_MOTOR.configure(intakeMotorConfiguration);
    }
}