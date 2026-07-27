package com.google.gson.internal.bind.util;

import com.qualcomm.robotcore.util.Dimmer;
import dk.sgjesse.r8api.DescriptorUtils;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Locale;
import java.util.TimeZone;

/* JADX INFO: loaded from: classes8.dex */
public class ISO8601Utils {
    private static final String UTC_ID = "UTC";
    private static final TimeZone TIMEZONE_UTC = TimeZone.getTimeZone(UTC_ID);

    public static String format(Date date) {
        return format(date, false, TIMEZONE_UTC);
    }

    public static String format(Date date, boolean millis) {
        return format(date, millis, TIMEZONE_UTC);
    }

    public static String format(Date date, boolean millis, TimeZone tz) {
        Calendar calendar = new GregorianCalendar(tz, Locale.US);
        calendar.setTime(date);
        int capacity = "yyyy-MM-ddThh:mm:ss".length();
        StringBuilder formatted = new StringBuilder(capacity + (millis ? ".sss".length() : 0) + (tz.getRawOffset() == 0 ? "Z" : "+hh:mm").length());
        padInt(formatted, calendar.get(1), "yyyy".length());
        formatted.append('-');
        padInt(formatted, calendar.get(2) + 1, "MM".length());
        formatted.append('-');
        padInt(formatted, calendar.get(5), "dd".length());
        formatted.append('T');
        padInt(formatted, calendar.get(11), "hh".length());
        formatted.append(':');
        padInt(formatted, calendar.get(12), "mm".length());
        formatted.append(':');
        padInt(formatted, calendar.get(13), "ss".length());
        if (millis) {
            formatted.append(DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
            padInt(formatted, calendar.get(14), "sss".length());
        }
        int offset = tz.getOffset(calendar.getTimeInMillis());
        if (offset != 0) {
            int hours = Math.abs((offset / Dimmer.LONG_BRIGHT_TIME) / 60);
            int minutes = Math.abs((offset / Dimmer.LONG_BRIGHT_TIME) % 60);
            formatted.append(offset >= 0 ? '+' : '-');
            padInt(formatted, hours, "hh".length());
            formatted.append(':');
            padInt(formatted, minutes, "mm".length());
        } else {
            formatted.append('Z');
        }
        return formatted.toString();
    }

    /* JADX WARN: Removed duplicated region for block: B:113:0x0232  */
    /* JADX WARN: Removed duplicated region for block: B:114:0x0234  */
    /* JADX WARN: Removed duplicated region for block: B:117:0x0251  */
    /* JADX WARN: Removed duplicated region for block: B:119:0x0257  */
    /* JADX WARN: Removed duplicated region for block: B:23:0x0063 A[Catch: IllegalArgumentException -> 0x0050, NumberFormatException -> 0x0055, IndexOutOfBoundsException -> 0x005a, TryCatch #6 {IndexOutOfBoundsException -> 0x005a, NumberFormatException -> 0x0055, IllegalArgumentException -> 0x0050, blocks: (B:11:0x003b, B:13:0x0041, B:23:0x0063, B:25:0x0074, B:26:0x0076, B:28:0x0083, B:30:0x0088, B:32:0x008e, B:38:0x009c, B:44:0x00af, B:46:0x00b7, B:59:0x00e9), top: B:122:0x003b }] */
    /* JADX WARN: Removed duplicated region for block: B:56:0x00e0 A[Catch: IllegalArgumentException -> 0x0221, NumberFormatException -> 0x0226, IndexOutOfBoundsException -> 0x022b, TRY_LEAVE, TryCatch #5 {IllegalArgumentException -> 0x0221, IndexOutOfBoundsException -> 0x022b, NumberFormatException -> 0x0226, blocks: (B:3:0x0007, B:5:0x0019, B:6:0x001b, B:8:0x0027, B:9:0x0029, B:54:0x00da, B:56:0x00e0, B:66:0x0101), top: B:123:0x0007 }] */
    /* JADX WARN: Removed duplicated region for block: B:94:0x020f A[Catch: IllegalArgumentException -> 0x021b, NumberFormatException -> 0x021d, IndexOutOfBoundsException -> 0x021f, TryCatch #4 {IllegalArgumentException -> 0x021b, IndexOutOfBoundsException -> 0x021f, NumberFormatException -> 0x021d, blocks: (B:92:0x01dc, B:71:0x0126, B:75:0x0146, B:77:0x0154, B:90:0x01d7, B:80:0x0164, B:82:0x018b, B:85:0x019e, B:86:0x01c8, B:74:0x0133, B:68:0x0105, B:69:0x0121, B:94:0x020f, B:95:0x021a), top: B:125:0x00de }] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public static java.util.Date parse(java.lang.String r24, java.text.ParsePosition r25) throws java.text.ParseException {
        /*
            Method dump skipped, instruction units count: 682
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.gson.internal.bind.util.ISO8601Utils.parse(java.lang.String, java.text.ParsePosition):java.util.Date");
    }

    private static boolean checkOffset(String value, int offset, char expected) {
        return offset < value.length() && value.charAt(offset) == expected;
    }

    private static int parseInt(String value, int beginIndex, int endIndex) throws NumberFormatException {
        if (beginIndex < 0 || endIndex > value.length() || beginIndex > endIndex) {
            throw new NumberFormatException(value);
        }
        int digit = beginIndex;
        int result = 0;
        if (digit < endIndex) {
            int i = digit + 1;
            int digit2 = Character.digit(value.charAt(digit), 10);
            if (digit2 < 0) {
                throw new NumberFormatException("Invalid number: " + value.substring(beginIndex, endIndex));
            }
            result = -digit2;
            digit = i;
        }
        while (digit < endIndex) {
            int i2 = digit + 1;
            int digit3 = Character.digit(value.charAt(digit), 10);
            if (digit3 < 0) {
                throw new NumberFormatException("Invalid number: " + value.substring(beginIndex, endIndex));
            }
            result = (result * 10) - digit3;
            digit = i2;
        }
        return -result;
    }

    private static void padInt(StringBuilder buffer, int value, int length) {
        String strValue = Integer.toString(value);
        for (int i = length - strValue.length(); i > 0; i--) {
            buffer.append('0');
        }
        buffer.append(strValue);
    }

    private static int indexOfNonDigit(String string, int offset) {
        for (int i = offset; i < string.length(); i++) {
            char c = string.charAt(i);
            if (c < '0' || c > '9') {
                return i;
            }
        }
        int i2 = string.length();
        return i2;
    }
}
