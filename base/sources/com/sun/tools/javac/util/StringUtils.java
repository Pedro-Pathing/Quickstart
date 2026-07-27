package com.sun.tools.javac.util;

import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes.dex */
public class StringUtils {
    public static String toLowerCase(String source) {
        return source.toLowerCase(Locale.US);
    }

    public static String toUpperCase(String source) {
        return source.toUpperCase(Locale.US);
    }

    public static int indexOfIgnoreCase(String text, String str) {
        return indexOfIgnoreCase(text, str, 0);
    }

    public static int indexOfIgnoreCase(String text, String str, int startIndex) {
        Matcher m = Pattern.compile(Pattern.quote(str), 2).matcher(text);
        if (m.find(startIndex)) {
            return m.start();
        }
        return -1;
    }
}
