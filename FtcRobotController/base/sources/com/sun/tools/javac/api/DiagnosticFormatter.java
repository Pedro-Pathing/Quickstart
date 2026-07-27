package com.sun.tools.javac.api;

import java.util.Locale;
import java.util.Set;
import javax.tools.Diagnostic;

/* JADX INFO: loaded from: classes.dex */
public interface DiagnosticFormatter<D extends Diagnostic<?>> {

    public interface Configuration {

        public enum DiagnosticPart {
            SUMMARY,
            DETAILS,
            SOURCE,
            SUBDIAGNOSTICS,
            JLS
        }

        public enum MultilineLimit {
            DEPTH,
            LENGTH
        }

        int getMultilineLimit(MultilineLimit multilineLimit);

        Set<DiagnosticPart> getVisible();

        void setMultilineLimit(MultilineLimit multilineLimit, int i);

        void setVisible(Set<DiagnosticPart> set);
    }

    public enum PositionKind {
        START,
        END,
        LINE,
        COLUMN,
        OFFSET
    }

    boolean displaySource(D d);

    String format(D d, Locale locale);

    String formatKind(D d, Locale locale);

    String formatMessage(D d, Locale locale);

    String formatPosition(D d, PositionKind positionKind, Locale locale);

    String formatSource(D d, boolean z, Locale locale);

    Configuration getConfiguration();
}
