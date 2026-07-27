package com.sun.tools.javac.util;

import com.sun.tools.javac.api.DiagnosticFormatter;
import com.sun.tools.javac.util.AbstractDiagnosticFormatter;
import java.util.Collection;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class BasicDiagnosticFormatter extends AbstractDiagnosticFormatter {
    public BasicDiagnosticFormatter(Options options, JavacMessages msgs) {
        super(msgs, new BasicConfiguration(options));
    }

    public BasicDiagnosticFormatter(JavacMessages msgs) {
        super(msgs, new BasicConfiguration());
    }

    @Override // com.sun.tools.javac.util.AbstractDiagnosticFormatter
    public String formatDiagnostic(JCDiagnostic d, Locale l) {
        if (l == null) {
            l = this.messages.getCurrentLocale();
        }
        String format = selectFormat(d);
        StringBuilder buf = new StringBuilder();
        int i = 0;
        while (i < format.length()) {
            char c = format.charAt(i);
            boolean meta = false;
            if (c == '%' && i < format.length() - 1) {
                meta = true;
                i++;
                c = format.charAt(i);
            }
            buf.append(meta ? formatMeta(c, d, l) : String.valueOf(c));
            i++;
        }
        int i2 = this.depth;
        if (i2 == 0) {
            return addSourceLineIfNeeded(d, buf.toString());
        }
        return buf.toString();
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String formatMessage(JCDiagnostic d, Locale l) {
        int currentIndentation = 0;
        StringBuilder buf = new StringBuilder();
        Collection<String> args = formatArguments(d, l);
        String msg = localize(l, d.getCode(), args.toArray());
        String[] lines = msg.split("\n");
        if (getConfiguration().getVisible().contains(DiagnosticFormatter.Configuration.DiagnosticPart.SUMMARY)) {
            currentIndentation = 0 + getConfiguration().getIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.SUMMARY);
            buf.append(indent(lines[0], currentIndentation));
        }
        if (lines.length > 1 && getConfiguration().getVisible().contains(DiagnosticFormatter.Configuration.DiagnosticPart.DETAILS)) {
            currentIndentation += getConfiguration().getIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.DETAILS);
            for (int i = 1; i < lines.length; i++) {
                buf.append("\n" + indent(lines[i], currentIndentation));
            }
        }
        if (d.isMultiline() && getConfiguration().getVisible().contains(DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS)) {
            int currentIndentation2 = currentIndentation + getConfiguration().getIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS);
            for (String sub : formatSubdiagnostics(d, l)) {
                buf.append("\n" + indent(sub, currentIndentation2));
            }
        }
        return buf.toString();
    }

    protected String addSourceLineIfNeeded(JCDiagnostic d, String msg) {
        if (!displaySource(d)) {
            return msg;
        }
        BasicConfiguration conf = getConfiguration();
        int indentSource = conf.getIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE);
        String sourceLine = "\n" + formatSourceLine(d, indentSource);
        boolean singleLine = msg.indexOf("\n") == -1;
        if (singleLine || getConfiguration().getSourcePosition() == BasicConfiguration.SourcePosition.BOTTOM) {
            return msg + sourceLine;
        }
        return msg.replaceFirst("\n", Matcher.quoteReplacement(sourceLine) + "\n");
    }

    protected String formatMeta(char c, JCDiagnostic d, Locale l) {
        boolean usePrefix = true;
        switch (c) {
            case '%':
                return "%";
            case 'L':
                return formatLintCategory(d, l);
            case '_':
                return " ";
            case 'b':
                return formatSource(d, false, l);
            case 'c':
                return formatPosition(d, DiagnosticFormatter.PositionKind.COLUMN, l);
            case 'e':
                return formatPosition(d, DiagnosticFormatter.PositionKind.END, l);
            case 'f':
                return formatSource(d, true, l);
            case 'l':
                return formatPosition(d, DiagnosticFormatter.PositionKind.LINE, l);
            case 'm':
                return formatMessage(d, l);
            case 'o':
                return formatPosition(d, DiagnosticFormatter.PositionKind.OFFSET, l);
            case 'p':
                return formatKind(d, l);
            case 's':
                return formatPosition(d, DiagnosticFormatter.PositionKind.START, l);
            case 't':
                switch (d.getType()) {
                    case FRAGMENT:
                        usePrefix = false;
                        break;
                    case ERROR:
                        if (d.getIntPosition() != -1) {
                            usePrefix = false;
                        }
                        break;
                    default:
                        usePrefix = true;
                        break;
                }
                if (usePrefix) {
                    return formatKind(d, l);
                }
                return "";
            default:
                return String.valueOf(c);
        }
    }

    private String selectFormat(JCDiagnostic d) {
        DiagnosticSource source = d.getDiagnosticSource();
        String format = getConfiguration().getFormat(BasicConfiguration.BasicFormatKind.DEFAULT_NO_POS_FORMAT);
        if (source != null && source != DiagnosticSource.NO_SOURCE) {
            if (d.getIntPosition() != -1) {
                return getConfiguration().getFormat(BasicConfiguration.BasicFormatKind.DEFAULT_POS_FORMAT);
            }
            if (source.getFile() != null && source.getFile().getKind() == JavaFileObject.Kind.CLASS) {
                return getConfiguration().getFormat(BasicConfiguration.BasicFormatKind.DEFAULT_CLASS_FORMAT);
            }
            return format;
        }
        return format;
    }

    @Override // com.sun.tools.javac.util.AbstractDiagnosticFormatter, com.sun.tools.javac.api.DiagnosticFormatter
    public BasicConfiguration getConfiguration() {
        return (BasicConfiguration) super.getConfiguration();
    }

    public static class BasicConfiguration extends AbstractDiagnosticFormatter.SimpleConfiguration {
        protected Map<BasicFormatKind, String> availableFormats;
        protected Map<DiagnosticFormatter.Configuration.DiagnosticPart, Integer> indentationLevels;
        protected SourcePosition sourcePosition;

        public enum BasicFormatKind {
            DEFAULT_POS_FORMAT,
            DEFAULT_NO_POS_FORMAT,
            DEFAULT_CLASS_FORMAT
        }

        public enum SourcePosition {
            BOTTOM,
            AFTER_SUMMARY
        }

        public BasicConfiguration(Options options) {
            super(options, EnumSet.of(DiagnosticFormatter.Configuration.DiagnosticPart.SUMMARY, DiagnosticFormatter.Configuration.DiagnosticPart.DETAILS, DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS, DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE));
            initFormat();
            initIndentation();
            if (options.isSet("oldDiags")) {
                initOldFormat();
            }
            String fmt = options.get("diagsFormat");
            if (fmt != null) {
                if (fmt.equals("OLD")) {
                    initOldFormat();
                } else {
                    initFormats(fmt);
                }
            }
            String srcPos = options.get("sourcePosition");
            if (srcPos != null && srcPos.equals("bottom")) {
                setSourcePosition(SourcePosition.BOTTOM);
            } else {
                setSourcePosition(SourcePosition.AFTER_SUMMARY);
            }
            String indent = options.get("diagsIndentation");
            if (indent != null) {
                String[] levels = indent.split("\\|");
                try {
                    switch (levels.length) {
                        case 5:
                            setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.JLS, Integer.parseInt(levels[4]));
                        case 4:
                            setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS, Integer.parseInt(levels[3]));
                        case 3:
                            setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE, Integer.parseInt(levels[2]));
                        case 2:
                            setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.DETAILS, Integer.parseInt(levels[1]));
                            break;
                    }
                    setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.SUMMARY, Integer.parseInt(levels[0]));
                } catch (NumberFormatException e) {
                    initIndentation();
                }
            }
        }

        public BasicConfiguration() {
            super(EnumSet.of(DiagnosticFormatter.Configuration.DiagnosticPart.SUMMARY, DiagnosticFormatter.Configuration.DiagnosticPart.DETAILS, DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS, DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE));
            initFormat();
            initIndentation();
        }

        private void initFormat() {
            initFormats("%f:%l:%_%p%L%m", "%p%L%m", "%f:%_%p%L%m");
        }

        private void initOldFormat() {
            initFormats("%f:%l:%_%t%L%m", "%p%L%m", "%f:%_%t%L%m");
        }

        private void initFormats(String pos, String nopos, String clazz) {
            this.availableFormats = new EnumMap(BasicFormatKind.class);
            setFormat(BasicFormatKind.DEFAULT_POS_FORMAT, pos);
            setFormat(BasicFormatKind.DEFAULT_NO_POS_FORMAT, nopos);
            setFormat(BasicFormatKind.DEFAULT_CLASS_FORMAT, clazz);
        }

        private void initFormats(String fmt) {
            String[] formats = fmt.split("\\|");
            switch (formats.length) {
                case 3:
                    setFormat(BasicFormatKind.DEFAULT_CLASS_FORMAT, formats[2]);
                case 2:
                    setFormat(BasicFormatKind.DEFAULT_NO_POS_FORMAT, formats[1]);
                    break;
            }
            setFormat(BasicFormatKind.DEFAULT_POS_FORMAT, formats[0]);
        }

        private void initIndentation() {
            this.indentationLevels = new HashMap();
            setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.SUMMARY, 0);
            setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.DETAILS, 2);
            setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS, 4);
            setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE, 0);
        }

        public int getIndentation(DiagnosticFormatter.Configuration.DiagnosticPart diagPart) {
            return this.indentationLevels.get(diagPart).intValue();
        }

        public void setIndentation(DiagnosticFormatter.Configuration.DiagnosticPart diagPart, int nSpaces) {
            this.indentationLevels.put(diagPart, Integer.valueOf(nSpaces));
        }

        public void setSourcePosition(SourcePosition sourcePos) {
            this.sourcePosition = sourcePos;
        }

        public SourcePosition getSourcePosition() {
            return this.sourcePosition;
        }

        public void setFormat(BasicFormatKind kind, String s) {
            this.availableFormats.put(kind, s);
        }

        public String getFormat(BasicFormatKind kind) {
            return this.availableFormats.get(kind);
        }
    }
}
