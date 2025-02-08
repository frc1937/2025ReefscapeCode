package frc.lib.generic.hardware.signals.rev;

import frc.lib.generic.hardware.motor.MotorSignal;
import frc.lib.generic.hardware.signals.FasterSignalsThread;
import frc.lib.generic.hardware.signals.InputsBase;
import frc.lib.generic.hardware.signals.SignalUtils;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.LogTable;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class REVInputs extends InputsBase {
    private static DoubleStatusSignal[] REV_SIGNALS = new DoubleStatusSignal[0];

    private final Map<String, Queue<Double>> threadedSignals = new HashMap<>();

    private int inputStartingIndex = -1;
    private int inputsSize = 0;

    public REVInputs(String name) {
        super(name);
    }

    @Override
    public void toLog(LogTable table) {
        if (inputsSize == 0) return;

        for (Map.Entry<String, Queue<Double>> entry : threadedSignals.entrySet())
            table.put(entry.getKey(), SignalUtils.queueToDoubleArray(entry.getValue()));

        for (int i = inputStartingIndex; i < inputStartingIndex + inputsSize; i++) {
            final DoubleStatusSignal signal = REV_SIGNALS[i];
            table.put(signal.getName(), signal.getValue());
        }

        latestTable = table;
    }

    public synchronized void registerThreadedREVSignal(MotorSignal signal, DoubleSupplier value) {
        if (signal == null || GlobalConstants.IS_REPLAY)
            return;

        registerREVSignal(signal, value);
        threadedSignals.put(signal.name().concat("_Threaded"), FasterSignalsThread.getInstance().registerSignal(value));
    }

    public synchronized void registerREVSignal(MotorSignal signal, DoubleSupplier value) {
        if (signal == null || GlobalConstants.IS_REPLAY)
            return;

        if (inputStartingIndex == -1)
            inputStartingIndex = REV_SIGNALS.length;

        inputsSize++;

        final DoubleStatusSignal[] newSignals = new DoubleStatusSignal[REV_SIGNALS.length + 1];

        System.arraycopy(REV_SIGNALS, 0, newSignals, 0, REV_SIGNALS.length);
        newSignals[REV_SIGNALS.length] = new DoubleStatusSignal(signal, value);

        REV_SIGNALS = newSignals;
    }
}
