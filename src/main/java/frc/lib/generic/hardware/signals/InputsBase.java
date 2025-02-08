package frc.lib.generic.hardware.signals;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.hardware.motor.MotorSignal;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.NoSuchElementException;

public abstract class InputsBase implements LoggableInputs {
    protected LogTable latestTable = null;
    private final String name;
    private double lastErrorTimestamp = 0;

    /**
     * Creates a new SignalsBase instance.
     *
     * @param name the name of the instance. Used for error messages
     */
    protected InputsBase(String name) {
        this.name = name;
    }

    @Override
    public void fromLog(LogTable table) {
        latestTable = table;
    }

    /**
     * Gets a signal from the inputs.
     *
     * @param signalType the name of the signal
     * @return the signal
     */
    public double getSignal(MotorSignal signalType) {
        if (latestTable == null) {
            printError(signalType.name());
            return 0;
        }

        final LogTable.LogValue value = latestTable.get(signalType.name());

        if (value == null) {
            printError(signalType.name());
            return 0;
        }

        return value.getDouble();
    }

    /**
     * Gets a threaded signal.
     * Threaded signals use threading to process certain signals separately at a faster rate.
     *
     * @param signalType the name of the threaded signal
     * @return the threaded signal
     */
    public double[] getThreadedSignal(MotorSignal signalType) {
        if (latestTable == null) {
            printError(signalType.name());
            return new double[0];
        }

        final LogTable.LogValue value = latestTable.get(signalType.name().concat("_Threaded"));

        if (value == null) {
            printError(signalType.name());
            return new double[0];
        }

        return value.getDoubleArray();
    }

    private boolean shouldPrintError() {
        final double currentTime = Timer.getTimestamp();
        final boolean shouldPrint = currentTime - lastErrorTimestamp > 5;

        if (shouldPrint)
            lastErrorTimestamp = currentTime;

        return shouldPrint;
    }

    private void printError(String signalName) {
        if (!shouldPrintError()) return;

        String message = String.format(
                "The device \"%s\" is trying to retrieve signal \"%s\" which doesn't exist. This is likely due to the device not being logged.",
                name, signalName
        );

        new NoSuchElementException(message).printStackTrace();
    }
}
