package frc.lib.util;

import java.util.function.ToDoubleFunction;

public class QuickSortHandler {
    public static <T> void sort(T[] array, ToDoubleFunction<T> toDoubleFunction) {
        quickSortByDoubleValue(array, toDoubleFunction, 0, array.length - 1);
    }

    private static <T> void quickSortByDoubleValue(T[] array, ToDoubleFunction<T> toDoubleFunction, int startIndex, int endIndex) {
        if (startIndex < endIndex) {
            int pivot = partitionArrayAndGetNewPivot(array, toDoubleFunction, startIndex, endIndex);

            quickSortByDoubleValue(array, toDoubleFunction, startIndex, pivot - 1);
            quickSortByDoubleValue(array, toDoubleFunction, pivot + 1, endIndex);
        }
    }

    private static <T> int partitionArrayAndGetNewPivot(T[] array, ToDoubleFunction<T> toDoubleFunction, int startIndex, int endIndex) {
        double[] doubleArray = convertArrayToDoubleArray(array, toDoubleFunction);
        double pivot = doubleArray[endIndex];
        int i = startIndex;

        for(int j = startIndex; j < endIndex; ++j) {
            if (doubleArray[j] < pivot) {
                swapArrayValues(array, i, j);
                ++i;
            }
        }

        swapArrayValues(array, i, endIndex);
        return i;
    }

    private static <T> double[] convertArrayToDoubleArray(T[] array, ToDoubleFunction<T> toDoubleFunction) {
        double[] doubleArray = new double[array.length];

        for(int i = 0; i < array.length; ++i) {
            doubleArray[i] = toDoubleFunction.applyAsDouble(array[i]);
        }

        return doubleArray;
    }

    private static <T> void swapArrayValues(T[] array, int firstIndex, int secondIndex) {
        T temp = array[firstIndex];
        array[firstIndex] = array[secondIndex];
        array[secondIndex] = temp;
    }
}
