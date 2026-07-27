package com.sun.tools.javac.api;

import java.util.Locale;
import java.util.MissingResourceException;

/* JADX INFO: loaded from: classes.dex */
public interface Messages {
    void add(String str) throws MissingResourceException;

    String getLocalizedString(Locale locale, String str, Object... objArr);
}
