package frc.lib.generic.hardware.encoder;

import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.HardwareManager;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import java.util.NoSuchElementException;
import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.encoder.EncoderInputs.ENCODER_INPUTS_LENGTH;
import static frc.robot.GlobalConstants.CURRENT_MODE;

public class Encoder implements LoggableHardware {
    private final EncoderInputs inputs = new EncoderInputs();
    private final String name;

    public Encoder(String name) {
        this.name = "Encoders/" + name;

        periodic();
        HardwareManager.addHardware(this);
    }

    /** This is required for sim to function correctly. In real, this won't do anything. */
    public void setSimulatedEncoderPositionSource(DoubleSupplier positionSource) {}
    /** This is required for sim to function correctly. In real, this won't do anything. */
    public void setSimulatedEncoderVelocitySource(DoubleSupplier velocitySource) {}

    /** Returns the encoder position, in Rotations*/
    public double getEncoderPosition() {
        if (!getSignalsToLog()[0]) printSignalError("POSITION");
        return inputs.position;
    }

    public double getEncoderVelocity() {
        if (!getSignalsToLog()[1]) printSignalError("VELOCITY");
        return inputs.velocity;
    }

    /**
     * Registers and automatically updates telemetry signals for logging.
     * <p>
     * This method is used to automate the process of tracking important robot signals
     * such as sensor readings or motor outputs. It ensures these values are
     * consistently updated and logged without needing manual updates in each robot loop.
     * </p>
     *
     * <p>
     * Benefits include:
     * <ul>
     *   <li><b>Debugging:</b> Easily diagnose issues with a record of sensor and system values.</li>
     *   <li><b>Performance Tuning:</b> Analyze robot behavior during matches or tests for optimization.</li>
     * </ul>
     * </p>
     *
     * @param signal The signal to log.
     * @param useFasterThread Whether to use a faster thread.
     */
    public void setupSignalUpdates(EncoderSignal signal, boolean useFasterThread) { }

    /**
     * Registers and automatically updates telemetry signals for logging.
     * <p>
     * This method is used to automate the process of tracking important robot signals
     * such as sensor readings or motor outputs. It ensures these values are
     * consistently updated and logged without needing manual updates in each robot loop.
     * </p>
     *
     * <p>
     * Benefits include:
     * <ul>
     *   <li><b>Debugging:</b> Easily diagnose issues with a record of sensor and system values.</li>
     *   <li><b>Performance Tuning:</b> Analyze robot behavior during matches or tests for optimization.</li>
     * </ul>
     * </p>
     *
     * @param signal The signal to log.
     */
    public void setupSignalUpdates(EncoderSignal signal) { setupSignalUpdates(signal, false); }

    public boolean configure(EncoderConfiguration encoderConfiguration) { return true; }

    protected void refreshInputs(EncoderInputs inputs) { }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public EncoderInputs getInputs() {
        return inputs;
    }

    protected boolean[] getSignalsToLog() { return new boolean[ENCODER_INPUTS_LENGTH]; }

    private void printSignalError(String signalName) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) return;

        new NoSuchElementException("--------------\n" +
                "ERROR - TRYING TO RETRIEVE UNINITIALIZED SIGNAL " + signalName + "| AT ENCODER " + name +
                "\n--------------").printStackTrace();
    }
}
