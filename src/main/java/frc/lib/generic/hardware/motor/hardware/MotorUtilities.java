package frc.lib.generic.hardware.motor.hardware;

import frc.lib.generic.hardware.motor.MotorInputs;

import java.util.Map;
import java.util.Queue;

import static frc.lib.util.QueueUtilities.toArray;

public class MotorUtilities {
    public enum MotionType {
        POSITION_PID,
        POSITION_PID_WITH_KG,
        POSITION_S_CURVE,
        POSITION_TRAPEZOIDAL,
        VELOCITY_PID_FF,
        VELOCITY_TRAPEZOIDAL
    }

    public static void handleThreadedInputs(MotorInputs inputs, Map<String, Queue<Double>> signalQueueList) {
        if (signalQueueList.isEmpty()) return;

        inputs.threadSystemPosition = toArray(signalQueueList.get("position"));
        inputs.threadSystemVelocity = toArray(signalQueueList.get("velocity"));
        inputs.threadSystemAcceleration = toArray(signalQueueList.get("acceleration"));
        inputs.threadVoltage = toArray(signalQueueList.get("voltage"));
        inputs.threadCurrent = toArray(signalQueueList.get("current"));
        inputs.threadTemperature = toArray(signalQueueList.get("temperature"));
        inputs.threadTarget = toArray(signalQueueList.get("target"));

        for (Queue<Double> queue : signalQueueList.values()) {
            queue.clear();
        }
    }
}
