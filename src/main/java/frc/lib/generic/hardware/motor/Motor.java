package frc.lib.generic.hardware.motor;

import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.HardwareManager;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.signals.InputsBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * Custom Motor class to allow switching and replacing motors quickly,
 * in addition of better uniformity across the code.
 */
public class Motor implements LoggableHardware {
    private final String name;

    private MotorConfiguration configuration;

    public Motor(String name) {
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
        return getInputs().getSignal(MotorSignal.VOLTAGE);
    }

    /**
     * Get the current running through the motor (STATOR current)
     *
     * @Units In amps
     */
    public double getCurrent() {
        return getInputs().getSignal(MotorSignal.CURRENT);
    }

    /**
     * Get the temperature of the motor
     * @Units In celsius
     */
    public double getTemperature() {
        return getInputs().getSignal(MotorSignal.TEMPERATURE);
    }

    /**
     * Get the current target of the closed-loop PID
     */
    public double getClosedLoopTarget() {
        return getInputs().getSignal(MotorSignal.CLOSED_LOOP_TARGET);
    }

    /**
     * Gearing applied
     *
     * @Units In rotations
     */
    public double getSystemPosition() {
        return getInputs().getSignal(MotorSignal.POSITION);
    }

    /**
     * Gearing applied
     *
     * @Units In rotations per second
     */
    public double getSystemVelocity() {
        return getInputs().getSignal(MotorSignal.VELOCITY);
    }

    /**
     * Gearing applied
     *
     * @Units In rotations per second
     */
    public double getSystemAcceleration() {
        return getInputs().getSignal(MotorSignal.ACCELERATION);
    }

    public void setFollowerOf(Motor motor, boolean invert) { }

    public void registerSignal(MotorSignal signal, boolean useFasterThread) {}

    public void registerSignal(MotorSignal signal) { registerSignal(signal, false); }

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

    @Override
    public void periodic() {
        if (getInputs() != null) {
            Logger.processInputs("Motors/" + name, getInputs());
        }
    }

    @Override
    public InputsBase getInputs() {
        return null;
    }
}
