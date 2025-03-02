package frc.lib.generic.hardware.motor.hardware;

import frc.lib.generic.hardware.motor.MotorInputs;

import java.util.Map;
import java.util.Queue;

import static frc.lib.util.QueueUtilities.queueToDoubleArray;

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

        inputs.threadSystemPosition = queueToDoubleArray(signalQueueList.get("position"));
        inputs.threadSystemVelocity = queueToDoubleArray(signalQueueList.get("velocity"));
        inputs.threadSystemAcceleration = queueToDoubleArray(signalQueueList.get("acceleration"));
        inputs.threadVoltage = queueToDoubleArray(signalQueueList.get("voltage"));
        inputs.threadCurrent = queueToDoubleArray(signalQueueList.get("current"));
        inputs.threadTemperature = queueToDoubleArray(signalQueueList.get("temperature"));
        inputs.threadTarget = queueToDoubleArray(signalQueueList.get("target"));
    }
}
