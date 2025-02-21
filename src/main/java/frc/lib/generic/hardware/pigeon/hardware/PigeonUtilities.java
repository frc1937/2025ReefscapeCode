package frc.lib.generic.hardware.pigeon.hardware;

import frc.lib.generic.hardware.pigeon.PigeonInputs;

import java.util.Map;
import java.util.Queue;

import static frc.lib.util.QueueUtilities.queueToArrayAndClearQueue;

public class PigeonUtilities {
    public static void handleThreadedInputs(PigeonInputs inputs, Map<String, Queue<Double>> signalQueueList) {
        if (signalQueueList.isEmpty()) return;

        inputs.threadGyroYawRotations = queueToArrayAndClearQueue(signalQueueList.get("yaw_pigeon2"));
        inputs.threadGyroPitchRotations = queueToArrayAndClearQueue(signalQueueList.get("pitch_pigeon2"));
        inputs.threadGyroRollRotations = queueToArrayAndClearQueue(signalQueueList.get("roll_pigeon2"));
    }
}
