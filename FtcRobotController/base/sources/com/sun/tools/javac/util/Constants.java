package com.sun.tools.javac.util;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.sun.tools.javac.code.Type;
import org.firstinspires.ftc.onbotjava.RequestConditions;

/* JADX INFO: loaded from: classes.dex */
public class Constants {
    public static Object decode(Object value, Type type) {
        if (value instanceof Integer) {
            int i = ((Integer) value).intValue();
            switch (type.getTag()) {
                case BOOLEAN:
                    return Boolean.valueOf(i != 0);
                case CHAR:
                    return Character.valueOf((char) i);
                case BYTE:
                    return Byte.valueOf((byte) i);
                case SHORT:
                    return Short.valueOf((short) i);
            }
        }
        return value;
    }

    public static String format(Object value, Type type) {
        Object value2 = decode(value, type);
        switch (type.getTag()) {
            case CHAR:
                return formatChar(((Character) value2).charValue());
            case BYTE:
                return formatByte(((Byte) value2).byteValue());
            case SHORT:
            default:
                if (value2 instanceof String) {
                    return formatString((String) value2);
                }
                return value2 + "";
            case LONG:
                return formatLong(((Long) value2).longValue());
            case FLOAT:
                return formatFloat(((Float) value2).floatValue());
            case DOUBLE:
                return formatDouble(((Double) value2).doubleValue());
        }
    }

    public static String format(Object value) {
        if (value instanceof Byte) {
            return formatByte(((Byte) value).byteValue());
        }
        if (value instanceof Short) {
            return formatShort(((Short) value).shortValue());
        }
        if (value instanceof Long) {
            return formatLong(((Long) value).longValue());
        }
        if (value instanceof Float) {
            return formatFloat(((Float) value).floatValue());
        }
        if (value instanceof Double) {
            return formatDouble(((Double) value).doubleValue());
        }
        if (value instanceof Character) {
            return formatChar(((Character) value).charValue());
        }
        if (value instanceof String) {
            return formatString((String) value);
        }
        if ((value instanceof Integer) || (value instanceof Boolean)) {
            return value.toString();
        }
        throw new IllegalArgumentException("Argument is not a primitive type or a string; it " + (value == null ? "is a null value." : "has class " + value.getClass().getName()) + ".");
    }

    private static String formatByte(byte b) {
        return String.format("(byte)0x%02x", Byte.valueOf(b));
    }

    private static String formatShort(short s) {
        return String.format("(short)%d", Short.valueOf(s));
    }

    private static String formatLong(long lng) {
        return lng + "L";
    }

    private static String formatFloat(float f) {
        if (Float.isNaN(f)) {
            return "0.0f/0.0f";
        }
        if (Float.isInfinite(f)) {
            return f < 0.0f ? "-1.0f/0.0f" : "1.0f/0.0f";
        }
        return f + RequestConditions.REQUEST_KEY_FILE;
    }

    private static String formatDouble(double d) {
        if (Double.isNaN(d)) {
            return "0.0/0.0";
        }
        if (Double.isInfinite(d)) {
            return d < LynxServoController.apiPositionFirst ? "-1.0/0.0" : "1.0/0.0";
        }
        return d + "";
    }

    private static String formatChar(char c) {
        return '\'' + Convert.quote(c) + '\'';
    }

    private static String formatString(String s) {
        return '\"' + Convert.quote(s) + '\"';
    }
}
