package frc.lib.generic.hardware.signals.ctre;

import frc.lib.generic.hardware.motor.MotorSignal;
import frc.lib.generic.hardware.signals.FasterSignalsThread;
import frc.lib.generic.hardware.signals.InputsBase;
import frc.lib.generic.hardware.signals.SignalUtils;
import frc.lib.generic.hardware.signals.rev.DoubleStatusSignal;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.LogTable;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class LegacySRXInputs extends InputsBase {
    private static DoubleStatusSignal[] SRX_SIGNALS = new DoubleStatusSignal[0];

    private final Map<String, Queue<Double>> threadedSignals = new HashMap<>();

    private int inputStartingIndex = -1;
    private int inputsSize = 0;

    public LegacySRXInputs(String name) {
        super(name);
    }

    @Override
    public void toLog(LogTable table) {
        if (inputsSize == 0) return;

        for (Map.Entry<String, Queue<Double>> entry : threadedSignals.entrySet())
            table.put(entry.getKey(), SignalUtils.queueToDoubleArray(entry.getValue()));

        for (int i = inputStartingIndex; i < inputStartingIndex + inputsSize; i++) {
            final DoubleStatusSignal signal = SRX_SIGNALS[i];
            table.put(signal.getName(), signal.getValue());
        }

        latestTable = table;
    }

    public synchronized void registerThreadedSRXSignal(MotorSignal signalType, DoubleSupplier value) {
        if (value == null || GlobalConstants.IS_REPLAY)
            return;

        registerSRXSignal(signalType, value);
        threadedSignals.put(signalType.name().concat("_Threaded"), FasterSignalsThread.getInstance().registerSignal(value));
    }

    public synchronized void registerSRXSignal(MotorSignal signal, DoubleSupplier value) {
        if (value == null || GlobalConstants.IS_REPLAY)
            return;

        if (inputStartingIndex == -1)
            inputStartingIndex = SRX_SIGNALS.length;

        inputsSize++;

        final DoubleStatusSignal[] newSignals = new DoubleStatusSignal[SRX_SIGNALS.length + 1];

        System.arraycopy(SRX_SIGNALS, 0, newSignals, 0, SRX_SIGNALS.length);
        newSignals[SRX_SIGNALS.length] = new DoubleStatusSignal(signal, value);

        SRX_SIGNALS = newSignals;
    }
}
