package frc.lib.util;

import java.util.Queue;

public class QueueUtilities {
    public static double[] toArray(Queue<Double> queue) {
        if (queue == null || queue.isEmpty()) return new double[0];

        final double[] array = new double[queue.size()];

        int i = 0;
        for (Double value : queue)
            array[i++] = value;

        return array;
    }
}
