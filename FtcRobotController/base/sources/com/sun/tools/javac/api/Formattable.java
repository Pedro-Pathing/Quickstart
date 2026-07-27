package com.sun.tools.javac.api;

import java.util.Locale;

/* JADX INFO: loaded from: classes.dex */
public interface Formattable {
    String getKind();

    String toString(Locale locale, Messages messages);

    public static class LocalizedString implements Formattable {
        String key;

        public LocalizedString(String key) {
            this.key = key;
        }

        @Override // com.sun.tools.javac.api.Formattable
        public String toString(Locale l, Messages messages) {
            return messages.getLocalizedString(l, this.key, new Object[0]);
        }

        @Override // com.sun.tools.javac.api.Formattable
        public String getKind() {
            return "LocalizedString";
        }

        public String toString() {
            return this.key;
        }
    }
}
