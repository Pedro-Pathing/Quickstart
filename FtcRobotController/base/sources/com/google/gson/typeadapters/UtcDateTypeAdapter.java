package com.google.gson.typeadapters;

import com.google.gson.JsonParseException;
import com.google.gson.TypeAdapter;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;
import com.qualcomm.robotcore.util.Dimmer;
import dk.sgjesse.r8api.DescriptorUtils;
import java.io.IOException;
import java.text.ParseException;
import java.text.ParsePosition;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Locale;
import java.util.TimeZone;

/* JADX INFO: loaded from: classes8.dex */
public final class UtcDateTypeAdapter extends TypeAdapter<Date> {
    private static final String GMT_ID = "GMT";
    private final TimeZone UTC_TIME_ZONE = TimeZone.getTimeZone("UTC");

    @Override // com.google.gson.TypeAdapter
    public void write(JsonWriter out, Date date) throws IOException {
        if (date == null) {
            out.nullValue();
        } else {
            String value = format(date, true, this.UTC_TIME_ZONE);
            out.value(value);
        }
    }

    @Override // com.google.gson.TypeAdapter
    /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
    public Date read2(JsonReader in) throws IOException {
        try {
            switch (in.peek()) {
                case NULL:
                    in.nextNull();
                    return null;
                default:
                    String date = in.nextString();
                    return parse(date, new ParsePosition(0));
            }
        } catch (ParseException e) {
            throw new JsonParseException(e);
        }
    }

    private static String format(Date date, boolean millis, TimeZone tz) {
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

    private static void padInt(StringBuilder buffer, int value, int length) {
        String strValue = Integer.toString(value);
        for (int i = length - strValue.length(); i > 0; i--) {
            buffer.append('0');
        }
        buffer.append(strValue);
    }

    /* JADX WARN: Removed duplicated region for block: B:67:0x0140  */
    /* JADX WARN: Removed duplicated region for block: B:68:0x0142  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private static java.util.Date parse(java.lang.String r16, java.text.ParsePosition r17) throws java.text.ParseException {
        /*
            Method dump skipped, instruction units count: 390
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.gson.typeadapters.UtcDateTypeAdapter.parse(java.lang.String, java.text.ParsePosition):java.util.Date");
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
                throw new NumberFormatException("Invalid number: " + value);
            }
            result = -digit2;
            digit = i;
        }
        while (digit < endIndex) {
            int i2 = digit + 1;
            int digit3 = Character.digit(value.charAt(digit), 10);
            if (digit3 < 0) {
                throw new NumberFormatException("Invalid number: " + value);
            }
            result = (result * 10) - digit3;
            digit = i2;
        }
        return -result;
    }
}
