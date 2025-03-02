package frc.lib.util;

import java.util.Queue;

public class QueueUtilities {
    @SuppressWarnings("ConstantConditions")
    public static double[] queueToDoubleArray(Queue<Double> queue) {
        if (queue == null || queue.isEmpty()) return new double[0];
        final double[] array = new double[queue.size()];

        for (int i = 0; i < array.length; i++)
            array[i] = queue.poll();

        return array;
    }
}
