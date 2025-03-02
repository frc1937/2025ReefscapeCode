package frc.lib.generic.hardware.pigeon.hardware;

import frc.lib.generic.hardware.pigeon.PigeonInputs;

import java.util.Map;
import java.util.Queue;

import static frc.lib.util.QueueUtilities.queueToDoubleArray;

public class PigeonUtilities {
    public static void handleThreadedInputs(PigeonInputs inputs, Map<String, Queue<Double>> signalQueueList) {
        if (signalQueueList.isEmpty()) return;

        inputs.threadGyroYawRotations = queueToDoubleArray(signalQueueList.get("yaw_pigeon2"));
        inputs.threadGyroPitchRotations = queueToDoubleArray(signalQueueList.get("pitch_pigeon2"));
        inputs.threadGyroRollRotations = queueToDoubleArray(signalQueueList.get("roll_pigeon2"));
    }
}
