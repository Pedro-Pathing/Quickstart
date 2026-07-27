package com.sun.tools.javac.util;

import java.lang.reflect.Array;

/* JADX INFO: loaded from: classes.dex */
public class ArrayUtils {
    private static int calculateNewLength(int currentLength, int maxIndex) {
        while (currentLength < maxIndex + 1) {
            currentLength *= 2;
        }
        return currentLength;
    }

    public static <T> T[] ensureCapacity(T[] tArr, int i) {
        if (i < tArr.length) {
            return tArr;
        }
        T[] tArr2 = (T[]) ((Object[]) Array.newInstance(tArr.getClass().getComponentType(), calculateNewLength(tArr.length, i)));
        System.arraycopy(tArr, 0, tArr2, 0, tArr.length);
        return tArr2;
    }

    public static byte[] ensureCapacity(byte[] array, int maxIndex) {
        if (maxIndex < array.length) {
            return array;
        }
        int newLength = calculateNewLength(array.length, maxIndex);
        byte[] result = new byte[newLength];
        System.arraycopy(array, 0, result, 0, array.length);
        return result;
    }

    public static char[] ensureCapacity(char[] array, int maxIndex) {
        if (maxIndex < array.length) {
            return array;
        }
        int newLength = calculateNewLength(array.length, maxIndex);
        char[] result = new char[newLength];
        System.arraycopy(array, 0, result, 0, array.length);
        return result;
    }

    public static int[] ensureCapacity(int[] array, int maxIndex) {
        if (maxIndex < array.length) {
            return array;
        }
        int newLength = calculateNewLength(array.length, maxIndex);
        int[] result = new int[newLength];
        System.arraycopy(array, 0, result, 0, array.length);
        return result;
    }
}
