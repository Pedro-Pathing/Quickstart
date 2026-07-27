package com.google.zxing.client.result;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Locale;
import java.util.TimeZone;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public final class CalendarParsedResult extends ParsedResult {
    private final String[] attendees;
    private final String description;
    private final long end;
    private final boolean endAllDay;
    private final double latitude;
    private final String location;
    private final double longitude;
    private final String organizer;
    private final long start;
    private final boolean startAllDay;
    private final String summary;
    private static final Pattern RFC2445_DURATION = Pattern.compile("P(?:(\\d+)W)?(?:(\\d+)D)?(?:T(?:(\\d+)H)?(?:(\\d+)M)?(?:(\\d+)S)?)?");
    private static final long[] RFC2445_DURATION_FIELD_UNITS = {604800000, 86400000, 3600000, 60000, 1000};
    private static final Pattern DATE_TIME = Pattern.compile("[0-9]{8}(T[0-9]{6}Z?)?");

    public CalendarParsedResult(String summary, String startString, String endString, String durationString, String location, String organizer, String[] attendees, String description, double latitude, double longitude) {
        super(ParsedResultType.CALENDAR);
        this.summary = summary;
        try {
            this.start = parseDate(startString);
            if (endString != null) {
                try {
                    this.end = parseDate(endString);
                } catch (ParseException pe) {
                    throw new IllegalArgumentException(pe.toString());
                }
            } else {
                long durationMS = parseDurationMS(durationString);
                this.end = durationMS < 0 ? -1L : this.start + durationMS;
            }
            this.startAllDay = startString.length() == 8;
            this.endAllDay = endString != null && endString.length() == 8;
            this.location = location;
            this.organizer = organizer;
            this.attendees = attendees;
            this.description = description;
            this.latitude = latitude;
            this.longitude = longitude;
        } catch (ParseException pe2) {
            throw new IllegalArgumentException(pe2.toString());
        }
    }

    public String getSummary() {
        return this.summary;
    }

    @Deprecated
    public Date getStart() {
        return new Date(this.start);
    }

    public long getStartTimestamp() {
        return this.start;
    }

    public boolean isStartAllDay() {
        return this.startAllDay;
    }

    @Deprecated
    public Date getEnd() {
        if (this.end < 0) {
            return null;
        }
        return new Date(this.end);
    }

    public long getEndTimestamp() {
        return this.end;
    }

    public boolean isEndAllDay() {
        return this.endAllDay;
    }

    public String getLocation() {
        return this.location;
    }

    public String getOrganizer() {
        return this.organizer;
    }

    public String[] getAttendees() {
        return this.attendees;
    }

    public String getDescription() {
        return this.description;
    }

    public double getLatitude() {
        return this.latitude;
    }

    public double getLongitude() {
        return this.longitude;
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        StringBuilder result = new StringBuilder(100);
        maybeAppend(this.summary, result);
        maybeAppend(format(this.startAllDay, this.start), result);
        maybeAppend(format(this.endAllDay, this.end), result);
        maybeAppend(this.location, result);
        maybeAppend(this.organizer, result);
        maybeAppend(this.attendees, result);
        maybeAppend(this.description, result);
        return result.toString();
    }

    private static long parseDate(String when) throws ParseException {
        if (!DATE_TIME.matcher(when).matches()) {
            throw new ParseException(when, 0);
        }
        if (when.length() == 8) {
            DateFormat format = new SimpleDateFormat("yyyyMMdd", Locale.ENGLISH);
            format.setTimeZone(TimeZone.getTimeZone("GMT"));
            return format.parse(when).getTime();
        }
        if (when.length() == 16 && when.charAt(15) == 'Z') {
            long milliseconds = parseDateTimeString(when.substring(0, 15));
            Calendar calendar = new GregorianCalendar();
            long milliseconds2 = milliseconds + ((long) calendar.get(15));
            calendar.setTime(new Date(milliseconds2));
            return ((long) calendar.get(16)) + milliseconds2;
        }
        return parseDateTimeString(when);
    }

    private static String format(boolean allDay, long date) {
        DateFormat dateTimeInstance;
        if (date < 0) {
            return null;
        }
        if (allDay) {
            dateTimeInstance = DateFormat.getDateInstance(2);
        } else {
            dateTimeInstance = DateFormat.getDateTimeInstance(2, 2);
        }
        return dateTimeInstance.format(Long.valueOf(date));
    }

    private static long parseDurationMS(CharSequence durationString) {
        if (durationString == null) {
            return -1L;
        }
        Matcher m = RFC2445_DURATION.matcher(durationString);
        if (!m.matches()) {
            return -1L;
        }
        long durationMS = 0;
        for (int i = 0; i < RFC2445_DURATION_FIELD_UNITS.length; i++) {
            String fieldValue = m.group(i + 1);
            if (fieldValue != null) {
                durationMS += RFC2445_DURATION_FIELD_UNITS[i] * ((long) Integer.parseInt(fieldValue));
            }
        }
        return durationMS;
    }

    private static long parseDateTimeString(String dateTimeString) throws ParseException {
        return new SimpleDateFormat("yyyyMMdd'T'HHmmss", Locale.ENGLISH).parse(dateTimeString).getTime();
    }
}
