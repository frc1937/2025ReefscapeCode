package frc.lib.util;

import java.util.Queue;

public class QueueUtilities {
    @SuppressWarnings("ConstantConditions")
    public static double[] queueToDoubleArray(Queue<Double> queue) {
        if (queue == null || queue.isEmpty()) return new double[0];
        final double[] array = new double[queue.size()];

        int i = 0;

        for (double value : queue)
            array[i++] = value;

        queue.clear();

        return array;
    }
}
