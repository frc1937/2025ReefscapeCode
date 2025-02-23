package frc.lib.generic.hardware.motor.hardware;

import frc.lib.generic.hardware.motor.MotorInputs;

import java.util.Map;
import java.util.Queue;

import static frc.lib.util.QueueUtilities.queueToArrayAndClearQueue;

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

        inputs.threadSystemPosition = queueToArrayAndClearQueue(signalQueueList.get("position"));
        inputs.threadSystemVelocity = queueToArrayAndClearQueue(signalQueueList.get("velocity"));
        inputs.threadSystemAcceleration = queueToArrayAndClearQueue(signalQueueList.get("acceleration"));
        inputs.threadVoltage = queueToArrayAndClearQueue(signalQueueList.get("voltage"));
        inputs.threadCurrent = queueToArrayAndClearQueue(signalQueueList.get("current"));
        inputs.threadTemperature = queueToArrayAndClearQueue(signalQueueList.get("temperature"));
        inputs.threadTarget = queueToArrayAndClearQueue(signalQueueList.get("target"));
    }
}
