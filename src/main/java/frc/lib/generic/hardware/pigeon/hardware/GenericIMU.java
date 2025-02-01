package frc.lib.generic.hardware.pigeon.hardware;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonConfiguration;
import frc.lib.generic.hardware.pigeon.PigeonInputs;
import frc.lib.generic.hardware.pigeon.PigeonSignal;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

import static frc.lib.generic.hardware.pigeon.PigeonInputs.PIGEON_INPUTS_LENGTH;
import static frc.lib.generic.hardware.pigeon.hardware.PigeonUtilities.handleThreadedInputs;

public class GenericIMU extends Pigeon {
    private final WPI_PigeonIMU pigeon;

    private final boolean[] signalsToLog = new boolean[PIGEON_INPUTS_LENGTH];
    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    public GenericIMU(String name, int deviceNumber) {
        super(name);

        pigeon = new WPI_PigeonIMU(deviceNumber);
    }

    @Override
    public void configurePigeon(PigeonConfiguration pigeonConfiguration) {
        pigeon.configFactoryDefault();
    }

    @Override
    public boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    public void setGyroYaw(double yawRotations) {
        pigeon.setYaw(yawRotations * 360);
    }

    @Override
    public void setupSignalUpdates(PigeonSignal signal, boolean useFasterThread) {
        signalsToLog[signal.getId()] = true;

        if (!useFasterThread) return;

        signalsToLog[signal.getId() + PIGEON_INPUTS_LENGTH / 2] = true;

        switch (signal) {
            case YAW -> signalQueueList.put("yaw", OdometryThread.getInstance().registerSignal(pigeon::getYaw));
            case ROLL -> signalQueueList.put("roll", OdometryThread.getInstance().registerSignal(pigeon::getRoll));
            case PITCH -> signalQueueList.put("pitch", OdometryThread.getInstance().registerSignal(pigeon::getPitch));
        }
    }

    @Override
    protected void refreshInputs(PigeonInputs inputs) {
        if (pigeon == null) return;

        inputs.setSignalsToLog(signalsToLog);

        inputs.gyroYawRotations = pigeon.getYaw() / 360.0;
        inputs.gyroRollRotations = pigeon.getRoll() / 360.0;
        inputs.gyroPitchRotations = pigeon.getPitch() / 360.0;

        handleThreadedInputs(inputs, signalQueueList);
    }
}
