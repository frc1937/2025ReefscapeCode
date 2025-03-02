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

    /** Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.*/
    public void setupSignalUpdates(EncoderSignal signal, boolean useFasterThread) { }

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
