package frc.lib.generic.hardware.motor.hardware.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SignalsConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.hardware.motor.hardware.MotorUtilities;
import frc.lib.scurve.InputParameter;
import frc.lib.scurve.OutputParameter;
import frc.lib.scurve.SCurveGenerator;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.motor.MotorInputs.MOTOR_INPUTS_LENGTH;

public abstract class GenericSparkBase extends Motor {
    protected final SignalsConfig signalsConfig = new SignalsConfig();

    private MotorUtilities.MotionType motionType;

    private final SparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController sparkController;
    private final int deviceId;

    private final boolean[] signalsToLog = new boolean[MOTOR_INPUTS_LENGTH];

    protected DoubleSupplier externalPositionSupplier, externalVelocitySupplier;
    private Feedforward feedforward;

    private double previousVelocity = 0;

    private SCurveGenerator scurveGenerator;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State goalState;

    private boolean hasStoppedOccurred = false;

    private MotorConfiguration currentConfiguration;

    protected GenericSparkBase(String name, int deviceId) {
        super(name);

        this.deviceId = deviceId;

        spark = getSpark();
        encoder = getEncoder();
        sparkController = getSparkController();

        optimizeBusUsage();
    }

    @Override
    public void setExternalPositionSupplier(DoubleSupplier positionSupplier) {
        this.externalPositionSupplier = positionSupplier;
    }

    @Override
    public void setExternalVelocitySupplier(DoubleSupplier velocitySupplier) {
        this.externalVelocitySupplier = velocitySupplier;
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, 0);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        setNewGoal(output);

        switch (mode) {
            case POSITION, VELOCITY -> handleSmoothMotion(motionType, goalState, motionProfile, this.feedforward);
            case VOLTAGE -> sparkController.setReference(output, SparkBase.ControlType.kVoltage, ClosedLoopSlot.kSlot0, 0);
            case CURRENT -> sparkController.setReference(output, SparkBase.ControlType.kCurrent, ClosedLoopSlot.kSlot0, 0);
        }
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        return configureMotor(configuration, null, false);
    }

    protected void setFeedforward(MotorProperties.Slot slot) {
        feedforward = new Feedforward(slot.feedforwardType,
                new Feedforward.FeedForwardConstants(slot.kS, slot.kV, slot.kA, slot.kG));
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void setFollower(Motor motor, boolean invert) {
        if (!(motor instanceof GenericSparkBase))
            return;

        configureMotor(currentConfiguration, ((GenericSparkFlex) motor).getSpark(), invert);
    }

    @Override
    public void stopMotor() {
        hasStoppedOccurred = true;
        this.setOutput(MotorProperties.ControlMode.VOLTAGE,0);
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public int getDeviceID() {
        return deviceId;
    }

    private void setNewGoal(double goal) {
        if (hasNoNewGoal(new TrapezoidProfile.State(goal, 0))) return;

        hasStoppedOccurred = false;
        setNewGoalExtras();

        if (motionType == MotorUtilities.MotionType.POSITION_TRAPEZOIDAL) {
            setPreviousSetpoint(new TrapezoidProfile.State(getEffectivePosition(), getEffectiveVelocity()));
        } else if (motionType == MotorUtilities.MotionType.VELOCITY_TRAPEZOIDAL) {
            setPreviousSetpoint(new TrapezoidProfile.State(getEffectiveVelocity(), getEffectiveAcceleration()));
        } else if (motionType == MotorUtilities.MotionType.POSITION_S_CURVE) {
            setSCurveInputs(new InputParameter(
                    getEffectivePosition(),
                    getEffectiveVelocity(),
                    getEffectiveAcceleration(),
                    goal
            ));

            setSCurveOutputs(new OutputParameter());
        }

        goalState = new TrapezoidProfile.State(goal, 0);
    }

    /**
     * Explanation here: <a href="https://docs.revrobotics.com/brushless/spark-max/control-interfaces">REV DOCS</a>
     */
    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        if (useFasterThread)
            new UnsupportedOperationException("Spark doesn't support faster thread");

        final int ms = 1000 / (useFasterThread ? 200 : 50);

        signalsToLog[signal.getId()] = true;

        switch (signal) {
            case CURRENT -> signalsConfig.outputCurrentPeriodMs(ms);
            case TEMPERATURE -> signalsConfig.motorTemperaturePeriodMs(ms);

            case POSITION -> {
                signalsConfig.primaryEncoderPositionAlwaysOn(true);
                signalsConfig.primaryEncoderPositionPeriodMs(ms);
            }

            case VELOCITY, ACCELERATION -> {
                signalsConfig.primaryEncoderVelocityPeriodMs(ms);
                signalsConfig.primaryEncoderVelocityAlwaysOn(true);
            }

            case VOLTAGE -> {
                signalsConfig.appliedOutputPeriodMs(ms);
                signalsConfig.busVoltagePeriodMs(ms);
            }
        }

        configure(currentConfiguration);
    }

    @Override
    protected boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        if (spark == null) return;

        refreshExtras();

        inputs.setSignalsToLog(signalsToLog);

        inputs.voltage = getVoltagePrivate();
        inputs.current = spark.getOutputCurrent();
        inputs.temperature = spark.getMotorTemperature();
        if (goalState != null) inputs.target = goalState.position;
        inputs.systemPosition = getEffectivePosition();
        inputs.systemVelocity = getEffectiveVelocity();
        inputs.systemAcceleration = getEffectiveAcceleration();
    }

    private double getVoltagePrivate() {
        return spark.getBusVoltage() * spark.getAppliedOutput();
    }

    private double getSystemPositionPrivate() {
        return encoder.getPosition();
    }

    private double getSystemVelocityPrivate() {
        return (encoder.getVelocity());
    }

    double getEffectivePosition() {
        return externalPositionSupplier == null ? getSystemPositionPrivate() : externalPositionSupplier.getAsDouble();
    }

    double getEffectiveVelocity() {
        return externalVelocitySupplier == null ? getSystemVelocityPrivate() : externalVelocitySupplier.getAsDouble();
    }

    private double getEffectiveAcceleration() {
        final double acceleration = externalVelocitySupplier == null ? getSystemVelocityPrivate() - previousVelocity : externalVelocitySupplier.getAsDouble() - previousVelocity;

        previousVelocity = externalVelocitySupplier == null ? getSystemVelocityPrivate() : externalVelocitySupplier.getAsDouble();

        return acceleration;
    }

    private void configureProfile(MotorConfiguration configuration) {
        if (configuration.profileMaxVelocity != 0 && configuration.profileMaxAcceleration != 0) {
            if (configuration.profileMaxJerk != 0) {
                scurveGenerator = new SCurveGenerator(0.02,
                        configuration.profileMaxVelocity,
                        configuration.profileMaxAcceleration,
                        configuration.profileMaxJerk);

                motionType = MotorUtilities.MotionType.POSITION_S_CURVE;
            } else {
                motionProfile = new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                configuration.profileMaxVelocity,
                                configuration.profileMaxAcceleration
                        )
                );

                motionType = MotorUtilities.MotionType.POSITION_TRAPEZOIDAL;
            }
        } else if (configuration.profileMaxAcceleration != 0 && configuration.profileMaxJerk != 0) {
            motionProfile =
                    new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                            configuration.profileMaxAcceleration,
                            configuration.profileMaxJerk
                        )
                );

            motionType = MotorUtilities.MotionType.VELOCITY_TRAPEZOIDAL;
        } else if (feedforward.getConstants().kG != 0 && feedforward.getConstants().kV == 0 && feedforward.getConstants().kA == 0 && feedforward.getConstants().kS == 0) {
            motionType = MotorUtilities.MotionType.POSITION_PID_WITH_KG;
        } else if (feedforward.getConstants().kS == 0 && feedforward.getConstants().kG == 0 && feedforward.getConstants().kV == 0 && feedforward.getConstants().kA == 0) {
            motionType = MotorUtilities.MotionType.POSITION_PID;
        } else {
            motionType = MotorUtilities.MotionType.VELOCITY_PID_FF;
        }
    }

    private boolean hasNoNewGoal(TrapezoidProfile.State newGoal) {
        return goalState != null
                && goalState.equals(newGoal)
                && !hasStoppedOccurred
                && (Logger.getTimestamp() - getLastProfileCalculationTimestamp() <= 100000); //(0.1 sec has passed)
    }

    protected SCurveGenerator getSCurveGenerator() {
        return scurveGenerator;
    }

    protected abstract void setSCurveInputs(InputParameter scurveInputs);

    protected abstract void setSCurveOutputs(OutputParameter outputParameter);

    protected abstract SparkBase getSpark();

    protected abstract RelativeEncoder getEncoder();

    protected abstract SparkClosedLoopController getSparkController();

    protected abstract void refreshExtras();

    protected abstract void setNewGoalExtras();

    /**
     * This exists because REV doesn't work properly with THEIR OW NFUCKING abstract configuration object
     */
    protected abstract boolean configureMotorInternal(MotorConfiguration configuration, SparkFlex master, boolean invertFollower);

    protected abstract void handleSmoothMotion(MotorUtilities.MotionType motionType, TrapezoidProfile.State goalState, TrapezoidProfile motionProfile, final Feedforward feedforward);

    protected abstract double getLastProfileCalculationTimestamp();

    protected abstract void setPreviousSetpoint(TrapezoidProfile.State previousSetpoint);

    private void optimizeBusUsage() {
        final int disabledMs = 1000;

        //Status0:
        signalsConfig.appliedOutputPeriodMs(50);
        signalsConfig.busVoltagePeriodMs(50);
        signalsConfig.motorTemperaturePeriodMs(50);
        signalsConfig.limitsPeriodMs(50);
        signalsConfig.outputCurrentPeriodMs(50);

        //Status1:
        signalsConfig.warningsPeriodMs(disabledMs);
        signalsConfig.faultsPeriodMs(disabledMs);
        signalsConfig.warningsAlwaysOn(false);
        signalsConfig.faultsAlwaysOn(false);

        //Status2:
        signalsConfig.primaryEncoderPositionPeriodMs(disabledMs);
        signalsConfig.primaryEncoderVelocityPeriodMs(disabledMs);
        signalsConfig.primaryEncoderPositionAlwaysOn(false);
        signalsConfig.primaryEncoderVelocityAlwaysOn(false);

        //Status3:
        signalsConfig.analogPositionPeriodMs(disabledMs);
        signalsConfig.analogVelocityPeriodMs(disabledMs);
        signalsConfig.analogVoltagePeriodMs(disabledMs);
        signalsConfig.analogPositionAlwaysOn(false);
        signalsConfig.analogVelocityAlwaysOn(false);
        signalsConfig.analogVoltageAlwaysOn(false);

        //Status4:
        signalsConfig.externalOrAltEncoderPosition(disabledMs);
        signalsConfig.externalOrAltEncoderVelocity(disabledMs);
        signalsConfig.externalOrAltEncoderPositionAlwaysOn(false);
        signalsConfig.externalOrAltEncoderVelocityAlwaysOn(false);

        //Status5:
        signalsConfig.absoluteEncoderPositionPeriodMs(disabledMs);
        signalsConfig.absoluteEncoderVelocityPeriodMs(disabledMs);
        signalsConfig.absoluteEncoderPositionAlwaysOn(false);
        signalsConfig.absoluteEncoderVelocityAlwaysOn(false);

        //Status7:
        signalsConfig.iAccumulationPeriodMs(disabledMs);
        signalsConfig.iAccumulationAlwaysOn(false);
    }

    private boolean configureMotor(MotorConfiguration configuration, SparkFlex masterId, boolean invertFollower) {
        currentConfiguration = configuration;

        setFeedforward(configuration.slot);
        configureProfile(configuration);

        return configureMotorInternal(configuration, masterId, invertFollower);
    }
}
