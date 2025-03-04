package frc.lib.generic.hardware.motor.hardware.rev;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.hardware.MotorUtilities;
import frc.lib.math.Conversions;
import frc.lib.scurve.InputParameter;
import frc.lib.scurve.OutputParameter;
import frc.lib.scurve.UpdateResult;
import org.littletonrobotics.junction.Logger;

public class GenericSparkFlex extends GenericSparkBase {
    private SparkFlex spark;
    private RelativeEncoder encoder;
    private SparkClosedLoopController sparkController;

    private InputParameter scurveInputs;
    private OutputParameter scurveOutput = new OutputParameter();

    private double lastProfileCalculationTimestamp;
    private TrapezoidProfile.State previousSetpoint;

    private final Timer timer = new Timer();

    public GenericSparkFlex(String name, int deviceId) {
        super(name, deviceId);
    }

    @Override
    protected double getLastProfileCalculationTimestamp() {
        return lastProfileCalculationTimestamp;
    }

    @Override
    protected void setPreviousSetpoint(TrapezoidProfile.State previousSetpoint) {
        this.previousSetpoint = previousSetpoint;
    }

    @Override
    protected SparkFlex getSpark() {
        if (spark == null) spark = new SparkFlex(getDeviceID(), SparkLowLevel.MotorType.kBrushless);
        return spark;
    }

    @Override
    protected RelativeEncoder getEncoder() {
        if (encoder == null) encoder = spark.getEncoder();
        return encoder;
    }

    @Override
    protected SparkClosedLoopController getSparkController() {
        if (sparkController == null) sparkController = spark.getClosedLoopController();
        return sparkController;
    }

    @Override
    protected void refreshExtras() {
        if (super.externalPositionSupplier == null) return;

        if (!timer.isRunning()) timer.start();

        if (timer.advanceIfElapsed(4)) {
            encoder.setPosition(getEffectivePosition());
        }
    }

    @Override
    protected void setNewGoalExtras() {
    }

    @Override
    protected boolean configureMotorInternal(MotorConfiguration configuration, SparkFlex master, boolean invertFollower) {
        encoder.setPosition(getEffectivePosition());

        final SparkFlexConfig sparkConfig = new SparkFlexConfig();

        sparkConfig.closedLoop.maxMotion.maxVelocity(configuration.profileMaxVelocity);
        sparkConfig.closedLoop.maxMotion.maxAcceleration(configuration.profileMaxAcceleration);
        sparkConfig.closedLoop.maxMotion.positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal);

        sparkConfig.idleMode(configuration.idleMode.getSparkIdleMode());

        sparkConfig.closedLoop.maxMotion.allowedClosedLoopError(configuration.closedLoopTolerance);
        sparkConfig.closedLoop.pid(configuration.slot.kP, configuration.slot.kI, configuration.slot.kD);
        sparkConfig.closedLoop.positionWrappingEnabled(configuration.closedLoopContinuousWrap);

        sparkConfig.encoder.positionConversionFactor(1.0 / configuration.gearRatio);
        sparkConfig.encoder.velocityConversionFactor(1.0 / (Conversions.SEC_PER_MIN * configuration.gearRatio));

        sparkConfig.openLoopRampRate(configuration.dutyCycleOpenLoopRampPeriod);
        sparkConfig.closedLoopRampRate(configuration.dutyCycleClosedLoopRampPeriod);

        sparkConfig.voltageCompensation(12);

        sparkConfig.signals.apply(signalsConfig);

        sparkConfig.inverted(configuration.inverted);

        if (master != null)
            sparkConfig.follow(master, true);

        if (configuration.statorCurrentLimit != -1) sparkConfig.smartCurrentLimit((int) configuration.statorCurrentLimit);
        if (configuration.supplyCurrentLimit != -1) sparkConfig.smartCurrentLimit((int) configuration.supplyCurrentLimit);

        if (configuration.forwardSoftLimit != null) {
            sparkConfig.softLimit.forwardSoftLimitEnabled(true);
            sparkConfig.softLimit.forwardSoftLimit(configuration.forwardSoftLimit * configuration.gearRatio);
        }

        if (configuration.reverseSoftLimit != null) {
            sparkConfig.softLimit.reverseSoftLimitEnabled(true);
            sparkConfig.softLimit.reverseSoftLimit(configuration.reverseSoftLimit * configuration.gearRatio);
        }

        int i = 0;

        while (i <= 3 &&
                spark.configureAsync(sparkConfig,
                        SparkBase.ResetMode.kResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters)
                        != REVLibError.kOk) {
            i++;
        }

        return spark.configureAsync(sparkConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters) == REVLibError.kOk;
    }

    protected void handleSmoothMotion(MotorUtilities.MotionType motionType,
                                      TrapezoidProfile.State goalState, TrapezoidProfile motionProfile,
                                      Feedforward feedforward) {
        if (goalState == null) return;

        double feedforwardOutput, acceleration;

        switch (motionType) {
            case POSITION_PID, POSITION_PID_WITH_KG -> sparkController.setReference(goalState.position,
                    SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0,
                    feedforward.calculate(getEffectivePosition(), 0, 0),
                    SparkClosedLoopController.ArbFFUnits.kVoltage); //todo: TEST, we merged these two.

            case VELOCITY_PID_FF ->
                sparkController.setReference(goalState.position, //todo: TEST, removed *60 cuz setThingy
                        SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                        feedforward.calculate(goalState.position, goalState.velocity),
                        SparkClosedLoopController.ArbFFUnits.kVoltage);


            case POSITION_TRAPEZOIDAL -> {
                final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(0.02, previousSetpoint, goalState);

                acceleration = (currentSetpoint.velocity - previousSetpoint.velocity) / 0.02;
                feedforwardOutput = feedforward.calculate(getEffectivePosition(), currentSetpoint.velocity, acceleration);

                sparkController.setReference(currentSetpoint.position,
                        SparkBase.ControlType.kPosition,
                        ClosedLoopSlot.kSlot0, feedforwardOutput,
                        SparkClosedLoopController.ArbFFUnits.kVoltage);

                previousSetpoint = currentSetpoint;
                lastProfileCalculationTimestamp = Logger.getTimestamp();
            }

            case VELOCITY_TRAPEZOIDAL -> {
                final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(0.02, previousSetpoint, goalState);

                feedforwardOutput = feedforward.calculate(0, currentSetpoint.position, currentSetpoint.velocity);

                sparkController.setReference(currentSetpoint.position * 60,
                        SparkBase.ControlType.kVelocity,
                        ClosedLoopSlot.kSlot0, feedforwardOutput,
                        SparkClosedLoopController.ArbFFUnits.kVoltage);

                previousSetpoint = currentSetpoint;
                lastProfileCalculationTimestamp = Logger.getTimestamp();
            }

            case POSITION_S_CURVE -> {
                final UpdateResult result = getSCurveGenerator().update(scurveInputs, scurveOutput);

                scurveInputs = result.input_parameter;
                scurveOutput = result.output_parameter;

                feedforwardOutput = feedforward.calculate(getEffectivePosition(), scurveOutput.new_velocity, scurveOutput.new_acceleration);

                sparkController.setReference(scurveOutput.new_position,
                        com.revrobotics.spark.SparkBase.ControlType.kPosition,
                        ClosedLoopSlot.kSlot0, feedforwardOutput,
                        SparkClosedLoopController.ArbFFUnits.kVoltage);

                lastProfileCalculationTimestamp = Logger.getTimestamp();
            }
        }
    }

    protected void setSCurveInputs(InputParameter scurveInputs) {
        this.scurveInputs = scurveInputs;
    }

    protected void setSCurveOutputs(OutputParameter scurveOutput) {
        this.scurveOutput = scurveOutput;
    }
}
