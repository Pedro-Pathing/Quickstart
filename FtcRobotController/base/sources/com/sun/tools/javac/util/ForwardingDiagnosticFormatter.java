package com.sun.tools.javac.util;

import com.sun.tools.javac.api.DiagnosticFormatter;
import java.util.Locale;
import java.util.Set;
import javax.tools.Diagnostic;

/* JADX INFO: loaded from: classes.dex */
public class ForwardingDiagnosticFormatter<D extends Diagnostic<?>, F extends DiagnosticFormatter<D>> implements DiagnosticFormatter<D> {
    protected ForwardingConfiguration configuration;
    protected F formatter;

    public ForwardingDiagnosticFormatter(F formatter) {
        this.formatter = formatter;
        this.configuration = new ForwardingConfiguration(formatter.getConfiguration());
    }

    public F getDelegatedFormatter() {
        return this.formatter;
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public DiagnosticFormatter.Configuration getConfiguration() {
        return this.configuration;
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public boolean displaySource(D diag) {
        return this.formatter.displaySource(diag);
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String format(D diag, Locale l) {
        return this.formatter.format(diag, l);
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String formatKind(D diag, Locale l) {
        return this.formatter.formatKind(diag, l);
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String formatMessage(D diag, Locale l) {
        return this.formatter.formatMessage(diag, l);
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String formatPosition(D diag, DiagnosticFormatter.PositionKind pk, Locale l) {
        return this.formatter.formatPosition(diag, pk, l);
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String formatSource(D diag, boolean fullname, Locale l) {
        return this.formatter.formatSource(diag, fullname, l);
    }

    public static class ForwardingConfiguration implements DiagnosticFormatter.Configuration {
        protected DiagnosticFormatter.Configuration configuration;

        public ForwardingConfiguration(DiagnosticFormatter.Configuration configuration) {
            this.configuration = configuration;
        }

        public DiagnosticFormatter.Configuration getDelegatedConfiguration() {
            return this.configuration;
        }

        @Override // com.sun.tools.javac.api.DiagnosticFormatter.Configuration
        public int getMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit limit) {
            return this.configuration.getMultilineLimit(limit);
        }

        @Override // com.sun.tools.javac.api.DiagnosticFormatter.Configuration
        public Set<DiagnosticFormatter.Configuration.DiagnosticPart> getVisible() {
            return this.configuration.getVisible();
        }

        @Override // com.sun.tools.javac.api.DiagnosticFormatter.Configuration
        public void setMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit limit, int value) {
            this.configuration.setMultilineLimit(limit, value);
        }

        @Override // com.sun.tools.javac.api.DiagnosticFormatter.Configuration
        public void setVisible(Set<DiagnosticFormatter.Configuration.DiagnosticPart> diagParts) {
            this.configuration.setVisible(diagParts);
        }
    }
}
