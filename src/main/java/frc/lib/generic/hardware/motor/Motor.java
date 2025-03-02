package frc.lib.generic.hardware.motor;

import frc.lib.generic.hardware.HardwareManager;
import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import java.util.NoSuchElementException;
import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.motor.MotorInputs.MOTOR_INPUTS_LENGTH;
import static frc.robot.GlobalConstants.CURRENT_MODE;

/**
 * Custom Motor class to allow switching and replacing motors quickly,
 * in addition of better uniformity across the code.
 */
public class Motor implements LoggableHardware {
    private final MotorInputs inputs = new MotorInputs();
    private final String name;

    private MotorConfiguration configuration;

    public Motor(String name) { //every motor has a port. reflect this here.
        this.name = name;

        periodic();
        HardwareManager.addHardware(this);
    }

    public String getName() {
        return name;
    }

    /**
     * Supplies an external position for the motor control system. This method allows
     * the feedforward and PID controllers to use an external encoder position value instead
     * of the system's position, allowing for more precise control using external {@link Encoder Encoders}.
     *
     * @param positionSupplier A {@link DoubleSupplier} providing the position to be used
     *                 by the motor control system.
     */
    public void setExternalPositionSupplier(DoubleSupplier positionSupplier) { }

    /**
     * Supplies velocity from an external source for the motor control system. This method allows
     * the feedforward and PID controllers to use the externally supplied velocity value instead
     * of the system's calculated velocity, allowing for more precise control using external {@link Encoder Encoders}.
     *
     * @param velocitySupplier A {@link DoubleSupplier} providing the velocity to be used
     *                 by the motor control system.
     */
    public void setExternalVelocitySupplier(DoubleSupplier velocitySupplier) { }

    /**
     * Sets the output of the motor based on the specified control mode and desired output value.
     *
     * <p>This method utilizes the built-in feedforward and PID controller to achieve precise control
     * over the motor. The control mode determines how the output value is interpreted and applied
     * to the motor. The supported control modes include:
     * <ul>
     *   <li>{@link MotorProperties.ControlMode#CURRENT CURRENT} Achieve a specific current.
     *   <li>{@link MotorProperties.ControlMode#VOLTAGE VOLTAGE} Achieve a specific voltage.
     *   <li>{@link MotorProperties.ControlMode#POSITION POSITION} Achieve a specific position using advanced control.
     *   <li>{@link MotorProperties.ControlMode#VELOCITY VELOCITY} Achieve a specific velocity using advanced control.
     * </ul>
     * </p>
     *
     * <p>For {@link MotorProperties.ControlMode#POSITION POSITION} and {@link MotorProperties.ControlMode#VELOCITY VELOCITY} control modes,
     * a trapezoidal motion profile can optionally be used. To enable it, ensure both {@link MotorConfiguration#profileMaxVelocity profiledMaxVelocity}
     * and {@link MotorConfiguration#profileMaxAcceleration profiledTargetAcceleration} are set.
     * The motor will calculate the needed feedforward based on the provided gains.
     * </p>
     *
     * @param controlMode the control mode for the motor
     * @param output      the desired output value
     */
    public void setOutput(MotorProperties.ControlMode controlMode, double output) { }


    /**
     * Sets the output of the motor based on the specified control mode, desired output value, and custom feedforward.
     *
     * <p>This method utilizes the built-in feedforward and PID controller to achieve precise control
     * over the motor. The control mode determines how the output value is interpreted and applied
     * to the motor. The supported control modes include:
     * <ul>
     *   <li>{@link MotorProperties.ControlMode#CURRENT CURRENT} Achieve a specific current.
     *   <li>{@link MotorProperties.ControlMode#VOLTAGE VOLTAGE} Achieve a specific voltage.
     *   <li>{@link MotorProperties.ControlMode#POSITION POSITION} Achieve a specific position using advanced control.
     *   <li>{@link MotorProperties.ControlMode#VELOCITY VELOCITY} Achieve a specific velocity using advanced control.
     * </ul>
     * </p>
     *
     * <p>For {@link MotorProperties.ControlMode#POSITION POSITION} and {@link MotorProperties.ControlMode#VELOCITY VELOCITY} control modes,
     * a trapezoidal motion profile can optionally be used. To enable it, ensure both {@link MotorConfiguration#profileMaxVelocity profiledMaxVelocity}
     * and {@link MotorConfiguration#profileMaxAcceleration profiledTargetAcceleration} are set.
     * </p>
     *
     * <p>The custom feedforward is used to provide additional control over the motor output, allowing for fine-tuned
     * performance. Feedforward is applied only in {@link MotorProperties.ControlMode#POSITION POSITION} and {@link MotorProperties.ControlMode#VELOCITY VELOCITY} control modes.
     * </p>
     *
     * @param controlMode the control mode for the motor
     * @param output      the desired output value (amperes for {@link MotorProperties.ControlMode#CURRENT CURRENT}, volts for {@link MotorProperties.ControlMode#VOLTAGE VOLTAGE},
     *                    rotations for {@link MotorProperties.ControlMode#POSITION POSITION}
     *                    or rotations per second for {@link MotorProperties.ControlMode#VELOCITY VELOCITY})
     * @param feedforward the custom feedforward to be applied to the motor output
     */
    public void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward) { }

    /**
     * Set the idle mode of the motor
     *
     * @param idleMode The new idle mode
     */
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        getCurrentConfiguration().idleMode = idleMode;
        configure(getCurrentConfiguration());
    }

    /**
     * Stop the motor
     */
    public void stopMotor() { }

    /**
     * Sets the encoder position of the motor to a specified value.
     *
     * <p>This method allows for manually setting the encoder position of the motor.
     * This can be useful for resetting the encoder position to a known reference point
     * or for calibrating the motor position in applications that require precise positional control.
     * </p>
     *
     * @param position the desired encoder position to set, in rotations.
     */
    public void setMotorEncoderPosition(double position) { }

    /**
     * Get the ID of the motor
     *
     * @return The ID of the motor
     */
    public int getDeviceID() { return -1; }

    /**
     * Retrieves the current position of the motor without any gearing applied.
     *
     * <p>This method returns the position of the motor as measured by the encoder,
     * without taking into account any gearing reductions or multipliers.
     * </p>
     *
     * @return the current position of the motor in rotations
     */
    public double getMotorPosition() { return getSystemPosition() / getCurrentConfiguration().gearRatio; }

    /**
     * Retrieves the current velocity of the motor, with no gearing applied.
     *
     * <p>This method returns the velocity of the motor as measured by the encoder,
     * without taking into account any gearing reductions or multipliers.
     * </p>
     *
     * @return the current velocity of the motor, in rotations per second (RPS).
     */
    public double getMotorVelocity() { return getSystemVelocity() / getCurrentConfiguration().gearRatio; }

    /**
     * Get the voltage running through the motor
     *
     * @Units In volts
     */
    public double getVoltage() {
        if (!getSignalsToLog()[0]) printSignalError("VOLTAGE");
        return inputs.voltage;
    }

    /**
     * Get the current running through the motor (STATOR current)
     *
     * @Units In amps
     */
    public double getCurrent() {
        if (!getSignalsToLog()[1]) printSignalError("CURRENT");
        return inputs.current;
    }

    /**
     * Get the temperature of the motor
     * @Units In celsius
     */
    public double getTemperature() {
        if (!getSignalsToLog()[2]) printSignalError("TEMPERATURE");
        return inputs.temperature;
    }

    /**
     * Get the current target of the closed-loop PID
     */
    public double getClosedLoopTarget() {
        if (!getSignalsToLog()[3]) printSignalError("CLOSED_LOOP_TARGET");
        return inputs.target;
    }

    /**
     * Gearing applied
     *
     * @Units In rotations
     */
    public double getSystemPosition() {
        if (!getSignalsToLog()[4]) printSignalError("POSITION");
        return inputs.systemPosition;
    }

    /**
     * Gearing applied
     *
     * @Units In rotations per second
     */
    public double getSystemVelocity() {
        if (!getSignalsToLog()[5]) printSignalError("VELOCITY");
        return inputs.systemVelocity;
    }

    /**
     * Gearing applied
     *
     * @Units In rotations per second
     */
    public double getSystemAcceleration() {
        if (!getSignalsToLog()[6]) printSignalError("ACCELERATION");
        return inputs.systemAcceleration;
    }

    public void setFollower(Motor motor, boolean invert) { }

    /**
     Signals are lazily loaded - only these explicitly called will be updated.
     Thus, you must call this method. when using a signal.
     Can only be used with CTRE products
     */
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) { }

    public void setupSignalUpdates(MotorSignal signal) { setupSignalUpdates(signal, false); }

    public boolean configure(MotorConfiguration configuration) {
        this.configuration = configuration;
        return true;
    }

    /**
     * Gets the currently used configuration used by the motor. If this is not set, it will return null.
     *
     * @return The configuration
     */
    public MotorConfiguration getCurrentConfiguration() { return configuration; }

    public boolean isAtPositionSetpoint() {
        if (getCurrentConfiguration() == null || getCurrentConfiguration().closedLoopTolerance == 0)
            new UnsupportedOperationException("You must set the tolerance before checking if the mechanism is at the setpoint.").printStackTrace();

        return Math.abs(getClosedLoopTarget() - getSystemPosition()) < getCurrentConfiguration().closedLoopTolerance;
    }

    public boolean isAtVelocitySetpoint() {
        if (getCurrentConfiguration() == null || getCurrentConfiguration().closedLoopTolerance == 0)
            new UnsupportedOperationException("You must set the tolerance before checking if the mechanism is at the setpoint.").printStackTrace();

        return Math.abs(getClosedLoopTarget() - getSystemVelocity()) < getCurrentConfiguration().closedLoopTolerance;
    }

    protected void refreshInputs(MotorInputs inputs) { }

    protected boolean[] getSignalsToLog() { return new boolean[MOTOR_INPUTS_LENGTH]; }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs("Motors/" + name, inputs);
    }

    @Override
    public MotorInputs getInputs() {
        return inputs;
    }

    private void printSignalError(String signalName) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) return;

        new NoSuchElementException("--------------------\n" +
                "ERROR: TRYING TO RETRIEVE UNINITIALIZED SIGNAL " + signalName + "| AT " + getClass().getName() + name +
                "\n--------------------")
                .printStackTrace();
    }
}
