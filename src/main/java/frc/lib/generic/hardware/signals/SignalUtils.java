package frc.lib.generic.hardware.signals;

import java.util.Queue;

public class SignalUtils {

    public static double[] queueToDoubleArray(Queue<Double> queue) {
        final int queueSize = queue.size();
        final double[] array = new double[queueSize];

        for (int i = 0; i < queueSize; i++)
            array[i] = queue.poll();

        return array;
    }
}
