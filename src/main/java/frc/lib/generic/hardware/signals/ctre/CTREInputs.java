package frc.lib.generic.hardware.signals.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
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

public class CTREInputs extends InputsBase {
    private static BaseStatusSignal[] CTRE_SIGNALS = new BaseStatusSignal[0];
    private static String[] CTRE_SIGNAL_NAMES = new String[0];
    private static DoubleStatusSignal[] SIMULATION_SIGNALS = new DoubleStatusSignal[0];

    private final Map<String, Queue<Double>> threadedSignals = new HashMap<>();
    private int inputStartingIndex = -1;
    private int inputsSize = 0;

    private int simulationInputStartingIndex = -1;
    private int simulationInputsSize = 0;

    public CTREInputs(String name) {
        super(name);
    }

    public static void refreshAllCTRESignals() {
        if (GlobalConstants.IS_REPLAY)
            return;

        BaseStatusSignal.refreshAll(CTRE_SIGNALS);
    }

    @Override
    public void toLog(LogTable table) {
        if (inputsSize == 0) return;

        for (Map.Entry<String, Queue<Double>> entry : threadedSignals.entrySet())
            table.put(entry.getKey(), SignalUtils.queueToDoubleArray(entry.getValue()));

        for (int i = inputStartingIndex; i < inputStartingIndex + inputsSize; i++) {
            table.put(CTRE_SIGNAL_NAMES[i], CTRE_SIGNALS[i].getValueAsDouble());
        }

        for (int i = simulationInputStartingIndex; i < simulationInputStartingIndex + simulationInputsSize; i++) {
            final DoubleStatusSignal signal = SIMULATION_SIGNALS[i];
            table.put(signal.getName(), signal.getValue());
        }

        latestTable = table;
    }

    public synchronized void registerSupplierForSimulation(MotorSignal signalType, DoubleSupplier supplier) {
        if (signalType == null || GlobalConstants.IS_REPLAY)
            return;

        if (simulationInputStartingIndex == -1)
            simulationInputStartingIndex = SIMULATION_SIGNALS.length;

        simulationInputsSize++;

        final DoubleStatusSignal[] newSignals = new DoubleStatusSignal[SIMULATION_SIGNALS.length + 1];

        System.arraycopy(SIMULATION_SIGNALS, 0, newSignals, 0, SIMULATION_SIGNALS.length);
        newSignals[SIMULATION_SIGNALS.length] = new DoubleStatusSignal(signalType, supplier);

        SIMULATION_SIGNALS = newSignals;

        System.out.println("HAS PUT NAME: " + signalType.name().concat("_Threaded"));
        threadedSignals.put(signalType.name().concat("_Threaded"), FasterSignalsThread.getInstance().registerSignal(supplier));
    }

    public synchronized void registerThreadedCTRESignal(MotorSignal signalType, BaseStatusSignal signal) {
        if (signal == null || GlobalConstants.IS_REPLAY)
            return;

        registerCTRESignal(signalType, signal, 200);
        threadedSignals.put(signalType.name().concat("_Threaded"), FasterSignalsThread.getInstance().registerSignal(signal::getValueAsDouble));
    }

    public synchronized void registerCTRESignal(MotorSignal signalType, BaseStatusSignal signal, double updateFrequency) {
        if (signal == null || GlobalConstants.IS_REPLAY)
            return;

        signal.setUpdateFrequency(updateFrequency);

        if (inputStartingIndex == -1)
            inputStartingIndex = CTRE_SIGNALS.length;

        inputsSize++;

        final BaseStatusSignal[] newSignals = new BaseStatusSignal[CTRE_SIGNALS.length + 1];

        System.arraycopy(CTRE_SIGNALS, 0, newSignals, 0, CTRE_SIGNALS.length);
        newSignals[CTRE_SIGNALS.length] = signal;

        CTRE_SIGNALS = newSignals;

       updateStringsArray(signalType);
    }

    private static void updateStringsArray(MotorSignal signalType) {
        final String[] newStringSignals = new String[CTRE_SIGNAL_NAMES.length + 1];

        System.arraycopy(CTRE_SIGNAL_NAMES, 0, newStringSignals, 0, CTRE_SIGNAL_NAMES.length);
        newStringSignals[CTRE_SIGNAL_NAMES.length] = signalType.name();

        CTRE_SIGNAL_NAMES = newStringSignals;
    }
}
