package frc.lib.generic.hardware.motor.hardware.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.lib.generic.hardware.motor.*;

import java.util.function.DoubleSupplier;

public class GenericTalonSRX extends Motor {
    private final WPI_TalonSRX talonSRX;

    private DoubleSupplier externalPositionSupplier, externalVelocitySupplier;
    private MotorConfiguration currentConfiguration;

    private double closedLoopTarget = 0;
    private double conversionFactor = 1;

    public GenericTalonSRX(String name, int deviceNumber) {
        super(name);

        talonSRX = new WPI_TalonSRX(deviceNumber);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        closedLoopTarget = output;

        switch (controlMode) {
            case POSITION -> talonSRX.set(ControlMode.Position, output);
            case VELOCITY -> talonSRX.set(ControlMode.Velocity, output);
            case CURRENT -> talonSRX.set(ControlMode.Current, output);
            case VOLTAGE -> talonSRX.setVoltage(output);
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward) {
        new UnsupportedOperationException("I ain't implementing this lmfao").printStackTrace();
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
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void stopMotor() {
        talonSRX.stopMotor();
    }

    @Override
    public void setFollower(Motor motor, boolean invert) {
        if (!(motor instanceof GenericTalonSRX))
            return;

        talonSRX.follow(((GenericTalonSRX) motor).talonSRX);
    }

    @Override
    public double getClosedLoopTarget() {
        return talonSRX.getClosedLoopTarget();
    }

    @Override
    public double getTemperature() {
        return talonSRX.getTemperature();
    }

    @Override
    public double getMotorPosition() {
        if (externalPositionSupplier != null)
            return externalPositionSupplier.getAsDouble() * conversionFactor;

        new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders. \nUse an external position supplier").printStackTrace();
        return 0;
    }

    @Override
    public double getMotorVelocity() {
        if (externalVelocitySupplier != null)
            return externalVelocitySupplier.getAsDouble() * conversionFactor;

        new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders. \nUse an external velocity supplier").printStackTrace();
        return 0;
    }

    @Override
    public double getSystemPosition() {
        if (externalPositionSupplier != null)
            return externalPositionSupplier.getAsDouble();

        new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders. \nUse an external position supplier").printStackTrace();
        return 0;
    }

    @Override
    public double getSystemVelocity() {
        if (externalVelocitySupplier != null)
            return externalVelocitySupplier.getAsDouble();

        new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders. \nUse an external velocity supplier").printStackTrace();
        return 0;
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.").printStackTrace();
    }

    @Override
    public int getDeviceID() {
        return talonSRX.getDeviceID();
    }

    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        new UnsupportedOperationException("Talon SRX does NOT use signals. Use GenericTalonFX instead").printStackTrace();
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        talonSRX.configFactoryDefault();

        currentConfiguration = configuration;

        configureSlots(configuration);

        conversionFactor = currentConfiguration.gearRatio;

        TalonSRXConfiguration talonSRXConfiguration = new TalonSRXConfiguration();

        talonSRXConfiguration.openloopRamp = configuration.dutyCycleOpenLoopRampPeriod;
        talonSRXConfiguration.closedloopRamp = configuration.dutyCycleClosedLoopRampPeriod;

        talonSRXConfiguration.feedbackNotContinuous = !configuration.closedLoopContinuousWrap;

        talonSRXConfiguration.peakCurrentLimit = (int) configuration.statorCurrentLimit;

        talonSRX.setInverted(configuration.inverted);

        return talonSRX.configAllSettings(talonSRXConfiguration) == ErrorCode.OK;
    }

    private void configureSlots(MotorConfiguration configuration) {
        talonSRX.config_kP(0, configuration.slot.kP);
        talonSRX.config_kI(0, configuration.slot.kI);
        talonSRX.config_kD(0, configuration.slot.kD);
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        if (talonSRX == null) return;

        if (externalPositionSupplier != null)
            inputs.systemPosition = externalPositionSupplier.getAsDouble();

        if (externalVelocitySupplier != null)
            inputs.systemVelocity = externalVelocitySupplier.getAsDouble();

        inputs.target = closedLoopTarget;

        inputs.current = talonSRX.getStatorCurrent();
        inputs.voltage = talonSRX.getMotorOutputVoltage();
        inputs.temperature = talonSRX.getTemperature();
    }
}
