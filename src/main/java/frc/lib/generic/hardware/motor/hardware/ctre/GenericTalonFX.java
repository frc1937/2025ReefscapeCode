package frc.lib.generic.hardware.motor.hardware.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.*;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorInputs;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.hardware.motor.MotorSignal;
import frc.lib.generic.hardware.motor.hardware.MotorUtilities;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.hardware.motor.MotorProperties.GravityType.ARM;

public class GenericTalonFX extends Motor {
    private final TalonFX talonFX;

    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    private final boolean[] signalsToLog = new boolean[MotorInputs.MOTOR_INPUTS_LENGTH];
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final List<BaseStatusSignal> signalsToUpdateList = new ArrayList<>();

    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator talonConfigurator;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage positionMMRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityMMRequest = new MotionMagicVelocityVoltage(0);

    private MotorConfiguration currentConfiguration;

    private boolean shouldUseProfile = false;

    private double target;

    public GenericTalonFX(String name, int deviceId, String canbusName) {
        super(name);

        talonFX = new TalonFX(deviceId);

        talonConfigurator = talonFX.getConfigurator();

        positionSignal = talonFX.getPosition().clone();
        velocitySignal = talonFX.getVelocity().clone();
        accelerationSignal = talonFX.getAcceleration().clone();
        voltageSignal = talonFX.getMotorVoltage().clone();
        currentSignal = talonFX.getStatorCurrent().clone();
        temperatureSignal = talonFX.getDeviceTemp().clone();
    }

    public GenericTalonFX(String name, int deviceId) {
        this(name, deviceId, "");
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output) {
        target = output;

        switch (mode) {
            case VOLTAGE -> talonFX.setControl(voltageRequest.withOutput(output));

            case POSITION -> {
                if (shouldUseProfile)
                    talonFX.setControl(positionMMRequest.withPosition(output).withSlot(0));
                else
                    talonFX.setControl(positionVoltageRequest.withPosition(output).withSlot(0));
            }

            case VELOCITY -> {
                if (shouldUseProfile)
                    talonFX.setControl(velocityMMRequest.withVelocity(output).withSlot(0));
                else
                    talonFX.setControl(velocityVoltageRequest.withVelocity(output).withSlot(0));
            }

            case CURRENT -> new UnsupportedOperationException("CTRE LOVES money and wants $150!!! dollars for this.. wtf.").printStackTrace();
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        if (mode != MotorProperties.ControlMode.POSITION && mode != MotorProperties.ControlMode.VELOCITY)
            setOutput(mode, output);

        target = output;

        switch (mode) {
            case POSITION -> {
                if (shouldUseProfile) {
                    talonFX.setControl(positionMMRequest.withPosition(output).withSlot(0).withFeedForward(feedforward));
                } else {
                    talonFX.setControl(positionVoltageRequest.withPosition(output).withSlot(0).withFeedForward(feedforward));
                }
            }

            case VELOCITY -> {
                if (shouldUseProfile) {
                    talonFX.setControl(velocityMMRequest.withVelocity(output).withSlot(0).withFeedForward(feedforward));
                } else {
                    talonFX.setControl(velocityVoltageRequest.withVelocity(output).withSlot(0).withFeedForward(feedforward));
                }
            }
        }
    }

    @Override
    public void stopMotor() {
        this.setOutput(MotorProperties.ControlMode.VOLTAGE,0);
    }

    @Override
    public void setExternalPositionSupplier(DoubleSupplier positionSupplier) {
        new UnsupportedOperationException("Setting external position source is not supported in TalonFX.\n Periodically call #setMotorEncoderPosition() instead.").printStackTrace();
    }

    @Override
    public void setExternalVelocitySupplier(DoubleSupplier velocitySupplier) {
        new UnsupportedOperationException("Setting external velocity source is not supported in TalonFX").printStackTrace();
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        talonConfigurator.setPosition(position);
    }

    @Override
    public int getDeviceID() {
        return talonFX.getDeviceID();
    }

    @Override
    public void setFollowerOf(Motor motor, boolean invert) {
        if (!(motor instanceof GenericTalonFX))
            return;

        talonFX.setControl(new Follower(motor.getDeviceID(), invert));
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        this.currentConfiguration = configuration;

        talonConfig.MotorOutput.Inverted = configuration.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = configuration.idleMode.equals(MotorProperties.IdleMode.BRAKE) ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //Who the FUCK added this feature. CTRE should fucking fire him bruh
        talonConfig.Audio.BeepOnBoot = false;
        talonConfig.Audio.BeepOnConfig = false;

        talonConfig.Voltage.PeakForwardVoltage = 12;
        talonConfig.Voltage.PeakReverseVoltage = -12;

        talonConfig.Feedback.SensorToMechanismRatio = configuration.gearRatio;

        configureMotionMagic();

        setConfig0();
        applyCurrentLimits();

        talonConfig.ClosedLoopGeneral.ContinuousWrap = configuration.closedLoopContinuousWrap;

        talonFX.optimizeBusUtilization();

        return applyConfig();
    }

    private void configureMotionMagic() {
        if (currentConfiguration.profileMaxVelocity == 0 && currentConfiguration.profileMaxAcceleration == 0 && currentConfiguration.profileMaxJerk == 0 ||
                currentConfiguration.profileMaxVelocity != 0 && currentConfiguration.profileMaxAcceleration == 0 && currentConfiguration.profileMaxJerk == 0)
            return;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = currentConfiguration.profileMaxVelocity;
        talonConfig.MotionMagic.MotionMagicAcceleration = currentConfiguration.profileMaxAcceleration;
        talonConfig.MotionMagic.MotionMagicJerk = currentConfiguration.profileMaxJerk;

        shouldUseProfile = true;
    }

    private void setConfig0() {
        talonConfig.Slot0.kP = currentConfiguration.slot.kP();
        talonConfig.Slot0.kI = currentConfiguration.slot.kI();
        talonConfig.Slot0.kD = currentConfiguration.slot.kD();

        talonConfig.Slot0.kA = currentConfiguration.slot.kA();
        talonConfig.Slot0.kS = currentConfiguration.slot.kS();
        talonConfig.Slot0.kV = currentConfiguration.slot.kV();
        talonConfig.Slot0.kG = currentConfiguration.slot.kG();

        if (currentConfiguration.slot.gravityType() != null)
            talonConfig.Slot0.GravityType = currentConfiguration.slot.gravityType() == ARM ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;

        if (currentConfiguration.slot.kS() != 0)
            talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    }

    private void applyCurrentLimits() {
        talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = currentConfiguration.dutyCycleOpenLoopRampPeriod;
        talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = currentConfiguration.dutyCycleClosedLoopRampPeriod;

        if (currentConfiguration.statorCurrentLimit != -1) {
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            talonConfig.CurrentLimits.StatorCurrentLimit = currentConfiguration.statorCurrentLimit;
        }

        if (currentConfiguration.supplyCurrentLimit != -1) {
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            talonConfig.CurrentLimits.SupplyCurrentLimit = currentConfiguration.supplyCurrentLimit;
        }
    }

    private boolean applyConfig() {
        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = talonConfigurator.apply(talonConfig);
            counter--;
        }

        return statusCode == StatusCode.OK;
    }

    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        final int updateFrequency = useFasterThread ? 200 : 50;
        signalsToLog[signal.getId()] = true;

        switch (signal) {
            case VELOCITY -> setupSignal(velocitySignal, updateFrequency);
            case POSITION -> setupSignal(positionSignal, updateFrequency);
            case ACCELERATION -> setupSignal(accelerationSignal, updateFrequency);
            case VOLTAGE -> setupSignal(voltageSignal, updateFrequency);
            case CURRENT -> setupSignal(currentSignal, updateFrequency);
            case TEMPERATURE -> setupSignal(temperatureSignal, updateFrequency);
        }

        if (!useFasterThread) return;

        signalsToLog[signal.getId() + MotorInputs.MOTOR_INPUTS_LENGTH / 2] = true;

        switch (signal) {
            case VELOCITY -> signalQueueList.put("velocity", OdometryThread.getInstance().registerSignal(this::getSystemVelocityPrivate));
            case POSITION -> signalQueueList.put("position", OdometryThread.getInstance().registerSignal(this::getSystemPositionPrivate));
            case ACCELERATION -> signalQueueList.put("acceleration", OdometryThread.getInstance().registerSignal(this::getSystemAccelerationPrivate));
            case VOLTAGE -> signalQueueList.put("voltage", OdometryThread.getInstance().registerSignal(this::getVoltagePrivate));
            case CURRENT -> signalQueueList.put("current", OdometryThread.getInstance().registerSignal(this::getCurrentPrivate));
            case TEMPERATURE -> signalQueueList.put("temperature", OdometryThread.getInstance().registerSignal(this::getTemperaturePrivate));
        }
    }

    @Override
    protected boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        if (talonFX == null) return;

        inputs.setSignalsToLog(signalsToLog);

        BaseStatusSignal.refreshAll(signalsToUpdateList.toArray(new BaseStatusSignal[0]));

        inputs.voltage = getVoltagePrivate();
        inputs.current = getCurrentPrivate();
        inputs.temperature = getTemperaturePrivate();
        inputs.target = target;
        inputs.systemPosition = getSystemPositionPrivate();
        inputs.systemVelocity = getSystemVelocityPrivate();
        inputs.systemAcceleration = getSystemAccelerationPrivate();

        MotorUtilities.handleThreadedInputs(inputs, signalQueueList);
    }

    private double getSystemPositionPrivate() {
        return positionSignal.getValue().in(Rotations);
    }

    private double getSystemVelocityPrivate() {
        return velocitySignal.getValue().in(RotationsPerSecond);
    }

    private double getSystemAccelerationPrivate() { return accelerationSignal.getValue().in(RotationsPerSecondPerSecond); }

    private double getVoltagePrivate() {
        return voltageSignal.getValue().in(Volts);
    }

    private double getTemperaturePrivate() {
        return temperatureSignal.getValue().in(Celsius);
    }

    private double getCurrentPrivate() {
        return currentSignal.getValue().in(Amp);
    }

    private void setupSignal(final BaseStatusSignal correspondingSignal, final int updateFrequency) {
        correspondingSignal.setUpdateFrequency(updateFrequency);
        signalsToUpdateList.add(correspondingSignal);
    }
}
