package frc.lib.generic.hardware.pigeon.hardware;

import frc.lib.generic.hardware.pigeon.PigeonInputs;

import java.util.Map;
import java.util.Queue;

import static frc.lib.util.QueueUtilities.toArray;

public class PigeonUtilities {
    public static void handleThreadedInputs(PigeonInputs inputs, Map<String, Queue<Double>> signalQueueList) {
        if (signalQueueList.isEmpty()) return;

        inputs.threadGyroYawRotations = toArray(signalQueueList.get("yaw_pigeon2"));
        inputs.threadGyroPitchRotations = toArray(signalQueueList.get("pitch_pigeon2"));
        inputs.threadGyroRollRotations = toArray(signalQueueList.get("roll_pigeon2"));

        for (Queue<Double> queue : signalQueueList.values()) {
            queue.clear();
        }
    }
}
