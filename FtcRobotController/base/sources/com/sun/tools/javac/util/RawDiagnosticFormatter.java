package com.sun.tools.javac.util;

import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.api.DiagnosticFormatter;
import com.sun.tools.javac.api.Formattable;
import com.sun.tools.javac.file.BaseFileObject;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.AbstractDiagnosticFormatter;
import java.util.Collection;
import java.util.EnumSet;
import java.util.Locale;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public final class RawDiagnosticFormatter extends AbstractDiagnosticFormatter {
    public RawDiagnosticFormatter(Options options) {
        super(null, new AbstractDiagnosticFormatter.SimpleConfiguration(options, EnumSet.of(DiagnosticFormatter.Configuration.DiagnosticPart.SUMMARY, DiagnosticFormatter.Configuration.DiagnosticPart.DETAILS, DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS)));
    }

    @Override // com.sun.tools.javac.util.AbstractDiagnosticFormatter
    public String formatDiagnostic(JCDiagnostic d, Locale l) {
        try {
            StringBuilder buf = new StringBuilder();
            if (d.getPosition() != -1) {
                buf.append(formatSource(d, false, (Locale) null));
                buf.append(':');
                buf.append(formatPosition(d, DiagnosticFormatter.PositionKind.LINE, (Locale) null));
                buf.append(':');
                buf.append(formatPosition(d, DiagnosticFormatter.PositionKind.COLUMN, (Locale) null));
                buf.append(':');
            } else if (d.getSource() != null && d.getSource().getKind() == JavaFileObject.Kind.CLASS) {
                buf.append(formatSource(d, false, (Locale) null));
                buf.append(":-:-:");
            } else {
                buf.append('-');
            }
            buf.append(' ');
            buf.append(formatMessage(d, (Locale) null));
            if (displaySource(d)) {
                buf.append("\n");
                buf.append(formatSourceLine(d, 0));
            }
            return buf.toString();
        } catch (Exception e) {
            return null;
        }
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String formatMessage(JCDiagnostic d, Locale l) {
        StringBuilder buf = new StringBuilder();
        Collection<String> args = formatArguments(d, l);
        buf.append(localize(null, d.getCode(), args.toArray()));
        if (d.isMultiline() && getConfiguration().getVisible().contains(DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS)) {
            List<String> subDiags = formatSubdiagnostics(d, null);
            if (subDiags.nonEmpty()) {
                String sep = "";
                buf.append(",{");
                for (String sub : formatSubdiagnostics(d, null)) {
                    buf.append(sep);
                    buf.append("(");
                    buf.append(sub);
                    buf.append(")");
                    sep = DocLint.TAGS_SEPARATOR;
                }
                buf.append('}');
            }
        }
        return buf.toString();
    }

    @Override // com.sun.tools.javac.util.AbstractDiagnosticFormatter
    protected String formatArgument(JCDiagnostic diag, Object arg, Locale l) {
        String s;
        if (arg instanceof Formattable) {
            s = arg.toString();
        } else if (arg instanceof JCTree.JCExpression) {
            JCTree.JCExpression tree = (JCTree.JCExpression) arg;
            s = "@" + tree.getStartPosition();
        } else if (arg instanceof BaseFileObject) {
            s = ((BaseFileObject) arg).getShortName();
        } else {
            s = super.formatArgument(diag, arg, null);
        }
        return arg instanceof JCDiagnostic ? "(" + s + ")" : s;
    }

    @Override // com.sun.tools.javac.util.AbstractDiagnosticFormatter
    protected String localize(Locale l, String key, Object... args) {
        StringBuilder buf = new StringBuilder();
        buf.append(key);
        String sep = ": ";
        for (Object o : args) {
            buf.append(sep);
            buf.append(o);
            sep = ", ";
        }
        return buf.toString();
    }

    @Override // com.sun.tools.javac.util.AbstractDiagnosticFormatter
    public boolean isRaw() {
        return true;
    }
}
