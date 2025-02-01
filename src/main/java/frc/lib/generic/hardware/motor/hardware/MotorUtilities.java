package frc.lib.generic.hardware.motor.hardware;

import frc.lib.generic.hardware.motor.MotorInputs;

import java.util.Map;
import java.util.Queue;

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

        signalQueueList.values().forEach(Queue::clear);
    }

    private static double[] toArray(Queue<Double> queue) {
        if (queue == null || queue.isEmpty())
            return new double[0];

        final double[] array = new double[queue.size()];
        int i = 0;
        for (Double value : queue)
            array[i++] = value;

        return array;
    }
}
