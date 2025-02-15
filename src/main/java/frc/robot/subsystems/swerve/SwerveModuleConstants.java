package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.encoder.*;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorSignal.*;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.STEER_GEAR_RATIO;
import static frc.robot.utilities.PortsConstants.SwervePorts.FL_DRIVE_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.FL_STEER_ENCODER_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.FL_STEER_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.FR_DRIVE_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.FR_STEER_ENCODER_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.FR_STEER_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.RL_DRIVE_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.RL_STEER_ENCODER_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.RL_STEER_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.RR_DRIVE_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.RR_STEER_ENCODER_PORT;
import static frc.robot.utilities.PortsConstants.SwervePorts.RR_STEER_MOTOR_PORT;

public class SwerveModuleConstants {
    static final MotorConfiguration steerMotorConfiguration = new MotorConfiguration();
    static final MotorConfiguration driveMotorConfiguration = new MotorConfiguration();

    static final MotorProperties.IdleMode ANGLE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;
    static final MotorProperties.IdleMode DRIVE_NEUTRAL_MODE = MotorProperties.IdleMode.BRAKE;

    static final double OPEN_LOOP_RAMP = 0.1;
    static final double CLOSED_LOOP_RAMP = 0.0;

    static final boolean CAN_CODER_INVERT = false;
    static final boolean ANGLE_MOTOR_INVERT = true;
    static final boolean DRIVE_MOTOR_INVERT = false;

    static final int ANGLE_CURRENT_LIMIT = 40;
    static final int DRIVE_SUPPLY_CURRENT_LIMIT = 60;
    static final int DRIVE_STATOR_CURRENT_LIMIT = 60;

    static final MotorProperties.Slot DRIVE_SLOT = new MotorProperties.Slot(
            0.053067, 0.0, 0.0,
            0.10861,
            0.023132,
            0.27053);

    protected static final Motor
            FL_STEER_MOTOR = MotorFactory.createSpark("FL_STEER_MOTOR", FL_STEER_MOTOR_PORT, MotorProperties.SparkType.MAX),
            FR_STEER_MOTOR = MotorFactory.createSpark("FR_STEER_MOTOR", FR_STEER_MOTOR_PORT, MotorProperties.SparkType.MAX),
            RL_STEER_MOTOR = MotorFactory.createSpark("RL_STEER_MOTOR", RL_STEER_MOTOR_PORT, MotorProperties.SparkType.MAX),
            RR_STEER_MOTOR = MotorFactory.createSpark("RR_STEER_MOTOR", RR_STEER_MOTOR_PORT, MotorProperties.SparkType.MAX);

    protected static final Motor
            FL_DRIVE_MOTOR = MotorFactory.createTalonFX("FL_DRIVE_MOTOR", FL_DRIVE_MOTOR_PORT),
            FR_DRIVE_MOTOR = MotorFactory.createTalonFX("FR_DRIVE_MOTOR", FR_DRIVE_MOTOR_PORT),
            RL_DRIVE_MOTOR = MotorFactory.createTalonFX("RL_DRIVE_MOTOR", RL_DRIVE_MOTOR_PORT),
            RR_DRIVE_MOTOR = MotorFactory.createTalonFX("RR_DRIVE_MOTOR", RR_DRIVE_MOTOR_PORT);

    protected static final Encoder
            FL_STEER_ENCODER = EncoderFactory.createCanCoder("FL_STEER_ENCODER", FL_STEER_ENCODER_PORT),
            FR_STEER_ENCODER = EncoderFactory.createCanCoder("FR_STEER_ENCODER", FR_STEER_ENCODER_PORT),
            RL_STEER_ENCODER = EncoderFactory.createCanCoder("RL_STEER_ENCODER", RL_STEER_ENCODER_PORT),
            RR_STEER_ENCODER = EncoderFactory.createCanCoder("RR_STEER_ENCODER", RR_STEER_ENCODER_PORT);

    static final double[] STEER_ENCODER_OFFSET = {0.677246, 0.282715, 0.533447, 0.313721};

    static final Encoder[] STEER_ENCODERS = {FL_STEER_ENCODER, FR_STEER_ENCODER, RL_STEER_ENCODER, RR_STEER_ENCODER};
    static final Motor[] STEER_MOTORS = {FL_STEER_MOTOR, FR_STEER_MOTOR, RL_STEER_MOTOR, RR_STEER_MOTOR};
    static final Motor[] DRIVE_MOTORS = {FL_DRIVE_MOTOR, FR_DRIVE_MOTOR, RL_DRIVE_MOTOR, RR_DRIVE_MOTOR};

    static {
        configureSteerConfiguration();
        configureDriveConfiguration();

        for (int i = 0; i < 4; i++) {
            configureSteerEncoder(STEER_ENCODERS[i], Rotation2d.fromRotations(STEER_ENCODER_OFFSET[i]));
            configureDriveMotor(DRIVE_MOTORS[i]);
            configureSteerMotor(STEER_MOTORS[i], STEER_ENCODERS[i]);

            setSimulatedEncoderSources(STEER_ENCODERS[i], STEER_MOTORS[i]);
        }
    }

    protected static final SwerveModule[] MODULES = new SwerveModule[]{
            new SwerveModule(FL_DRIVE_MOTOR, FL_STEER_MOTOR, FL_STEER_ENCODER),
            new SwerveModule(FR_DRIVE_MOTOR, FR_STEER_MOTOR, FR_STEER_ENCODER),
            new SwerveModule(RL_DRIVE_MOTOR, RL_STEER_MOTOR, RL_STEER_ENCODER),
            new SwerveModule(RR_DRIVE_MOTOR, RR_STEER_MOTOR, RR_STEER_ENCODER)
    };

    private static void configureSteerEncoder(Encoder steerEncoder, Rotation2d angleOffset) {
        final EncoderConfiguration encoderConfiguration = new EncoderConfiguration();

        encoderConfiguration.invert = CAN_CODER_INVERT;
        encoderConfiguration.sensorRange = EncoderProperties.SensorRange.ZERO_TO_ONE;
        encoderConfiguration.offsetRotations = -angleOffset.getRotations();

        steerEncoder.configure(encoderConfiguration);

        steerEncoder.setupSignalUpdates(EncoderSignal.POSITION, true);
    }

    private static void setSimulatedEncoderSources(Encoder steerEncoder, Motor simulationSource) {
        steerEncoder.setSimulatedEncoderPositionSource(simulationSource::getSystemPosition);
        steerEncoder.setSimulatedEncoderVelocitySource(simulationSource::getSystemVelocity);
    }

    private static void configureDriveMotor(Motor driveMotor) {
        driveMotor.setupSignalUpdates(POSITION, true);

        driveMotor.setupSignalUpdates(CLOSED_LOOP_TARGET);
        driveMotor.setupSignalUpdates(VOLTAGE);
        driveMotor.setupSignalUpdates(VELOCITY);
        driveMotor.setupSignalUpdates(TEMPERATURE);

        driveMotor.configure(driveMotorConfiguration);
    }

    private static void configureSteerMotor(Motor steerMotor, Encoder encoder) {
        steerMotor.setupSignalUpdates(POSITION);
        steerMotor.setupSignalUpdates(VELOCITY);
        steerMotor.setupSignalUpdates(VOLTAGE);
        steerMotor.setupSignalUpdates(CLOSED_LOOP_TARGET);
        steerMotor.configure(steerMotorConfiguration);

        steerMotor.setExternalPositionSupplier(encoder::getEncoderPosition);
    }

    private static void configureDriveConfiguration() {
        driveMotorConfiguration.idleMode = DRIVE_NEUTRAL_MODE;
        driveMotorConfiguration.inverted = DRIVE_MOTOR_INVERT;

        driveMotorConfiguration.gearRatio = DRIVE_GEAR_RATIO;

        driveMotorConfiguration.statorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
        driveMotorConfiguration.supplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;

        driveMotorConfiguration.slot = DRIVE_SLOT;

        driveMotorConfiguration.dutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        driveMotorConfiguration.dutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;

        driveMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getKrakenX60(1),
                DRIVE_GEAR_RATIO,
                0.003);
        driveMotorConfiguration.simulationSlot = new MotorProperties.Slot(0, 0, 0, 0.74095, 0.019661, 0.010919);
    }

    private static void configureSteerConfiguration() {
        steerMotorConfiguration.slot = new MotorProperties.Slot(30, 0, 0, 0, 0, 0);

        steerMotorConfiguration.supplyCurrentLimit = ANGLE_CURRENT_LIMIT;
        steerMotorConfiguration.inverted = ANGLE_MOTOR_INVERT;
        steerMotorConfiguration.idleMode = ANGLE_NEUTRAL_MODE;

        steerMotorConfiguration.gearRatio = STEER_GEAR_RATIO;

        steerMotorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getCIM(1),
                STEER_GEAR_RATIO,
                0.003);
        steerMotorConfiguration.simulationSlot = new MotorProperties.Slot(120, 0, 0, 0, 0, 0);

        steerMotorConfiguration.closedLoopContinuousWrap = true;
    }
}
