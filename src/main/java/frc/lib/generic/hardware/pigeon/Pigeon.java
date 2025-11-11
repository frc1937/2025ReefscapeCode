package frc.lib.generic.hardware.pigeon;

import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.hardware.HardwareManager;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import java.util.NoSuchElementException;

import static frc.lib.generic.hardware.pigeon.PigeonInputs.PIGEON_INPUTS_LENGTH;
import static frc.robot.GlobalConstants.CURRENT_MODE;

public class Pigeon implements LoggableHardware {
    private final PigeonInputs inputs = new PigeonInputs();
    private final String name;

    public Pigeon(String name) {
        this.name = "Pigeons/" + name;

        periodic();
        HardwareManager.addHardware(this);
    }

    public void configurePigeon(PigeonConfiguration pigeonConfiguration) {}

    public double getYawRotations() {
        if (!getSignalsToLog()[0]) printSignalError("YAW");
        return inputs.gyroYawRotations;
    }

    public double getRollRotations() {
        if (!getSignalsToLog()[1]) printSignalError("ROLL");
        return inputs.gyroRollRotations;
    }

    public double getPitchRotations() {
        if (!getSignalsToLog()[2]) printSignalError("PITCH");
        return inputs.gyroPitchRotations;
    }

    public void setGyroYaw(double yawRotations) {}

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
    public void setupSignalUpdates(PigeonSignal signal, boolean useFasterThread) {}

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
    public void setupSignalUpdates(PigeonSignal signal) {setupSignalUpdates(signal, false);}

    public boolean[] getSignalsToLog() {
        return new boolean[PIGEON_INPUTS_LENGTH];
    }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public PigeonInputs getInputs() {
        return inputs;
    }
    
    protected void refreshInputs(PigeonInputs inputs) {}

    private void printSignalError(String signalName) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) return;

        new NoSuchElementException("--------------\n" +
                "ERROR - TRYING TO RETRIEVE UNINITIALIZED SIGNAL " + signalName + "| AT " + getClass().getName() + name +
                "\n--------------")
                .printStackTrace();
    }
}
