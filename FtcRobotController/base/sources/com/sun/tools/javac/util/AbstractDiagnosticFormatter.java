package com.sun.tools.javac.util;

import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.api.DiagnosticFormatter;
import com.sun.tools.javac.api.Formattable;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Printer;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.file.BaseFileObject;
import com.sun.tools.javac.jvm.Profile;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.Pretty;
import com.sun.tools.javac.util.JCDiagnostic;
import java.util.Arrays;
import java.util.Collection;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import javax.tools.JavaFileObject;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes.dex */
public abstract class AbstractDiagnosticFormatter implements DiagnosticFormatter<JCDiagnostic> {
    private SimpleConfiguration config;
    protected JavacMessages messages;
    protected int depth = 0;
    private List<Type> allCaptured = List.nil();
    protected Printer printer = new Printer() { // from class: com.sun.tools.javac.util.AbstractDiagnosticFormatter.1
        @Override // com.sun.tools.javac.code.Printer
        protected String localize(Locale locale, String key, Object... args) {
            return AbstractDiagnosticFormatter.this.localize(locale, key, args);
        }

        @Override // com.sun.tools.javac.code.Printer
        protected String capturedVarId(Type.CapturedType t, Locale locale) {
            return "" + (AbstractDiagnosticFormatter.this.allCaptured.indexOf(t) + 1);
        }

        @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
        public String visitCapturedType(Type.CapturedType t, Locale locale) {
            if (!AbstractDiagnosticFormatter.this.allCaptured.contains(t)) {
                AbstractDiagnosticFormatter.this.allCaptured = AbstractDiagnosticFormatter.this.allCaptured.append(t);
            }
            return super.visitCapturedType(t, locale);
        }
    };

    protected abstract String formatDiagnostic(JCDiagnostic jCDiagnostic, Locale locale);

    protected AbstractDiagnosticFormatter(JavacMessages messages, SimpleConfiguration config) {
        this.messages = messages;
        this.config = config;
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String formatKind(JCDiagnostic d, Locale l) {
        switch (d.getType()) {
            case FRAGMENT:
                return "";
            case NOTE:
                return localize(l, "compiler.note.note", new Object[0]);
            case WARNING:
                return localize(l, "compiler.warn.warning", new Object[0]);
            case ERROR:
                return localize(l, "compiler.err.error", new Object[0]);
            default:
                throw new AssertionError("Unknown diagnostic type: " + d.getType());
        }
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String format(JCDiagnostic d, Locale locale) {
        this.allCaptured = List.nil();
        return formatDiagnostic(d, locale);
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String formatPosition(JCDiagnostic d, DiagnosticFormatter.PositionKind pk, Locale l) {
        Assert.check(d.getPosition() != -1);
        return String.valueOf(getPosition(d, pk));
    }

    private long getPosition(JCDiagnostic d, DiagnosticFormatter.PositionKind pk) {
        switch (pk) {
            case START:
                return d.getIntStartPosition();
            case END:
                return d.getIntEndPosition();
            case LINE:
                return d.getLineNumber();
            case COLUMN:
                return d.getColumnNumber();
            case OFFSET:
                return d.getIntPosition();
            default:
                throw new AssertionError("Unknown diagnostic position: " + pk);
        }
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public String formatSource(JCDiagnostic d, boolean fullname, Locale l) {
        JavaFileObject fo = d.getSource();
        if (fo == null) {
            throw new IllegalArgumentException();
        }
        if (fullname) {
            return fo.getName();
        }
        if (fo instanceof BaseFileObject) {
            return ((BaseFileObject) fo).getShortName();
        }
        return BaseFileObject.getSimpleName(fo);
    }

    protected Collection<String> formatArguments(JCDiagnostic d, Locale l) {
        ListBuffer<String> buf = new ListBuffer<>();
        for (Object o : d.getArgs()) {
            buf.append(formatArgument(d, o, l));
        }
        return buf.toList();
    }

    protected String formatArgument(JCDiagnostic d, Object arg, Locale l) {
        if (arg instanceof JCDiagnostic) {
            this.depth++;
            try {
                String s = formatMessage((JCDiagnostic) arg, l);
                return s;
            } finally {
                this.depth--;
            }
        }
        if (arg instanceof JCTree.JCExpression) {
            return expr2String((JCTree.JCExpression) arg);
        }
        if (arg instanceof Iterable) {
            return formatIterable(d, (Iterable) arg, l);
        }
        if (arg instanceof Type) {
            return this.printer.visit((Type) arg, l);
        }
        if (arg instanceof Symbol) {
            return this.printer.visit((Symbol) arg, l);
        }
        if (arg instanceof JavaFileObject) {
            return ((JavaFileObject) arg).getName();
        }
        if (arg instanceof Profile) {
            return ((Profile) arg).name;
        }
        if (arg instanceof Formattable) {
            return ((Formattable) arg).toString(l, this.messages);
        }
        return String.valueOf(arg);
    }

    private String expr2String(JCTree.JCExpression tree) {
        switch (tree.getTag()) {
            case PARENS:
                return expr2String(((JCTree.JCParens) tree).expr);
            case LAMBDA:
            case REFERENCE:
            case CONDEXPR:
                return Pretty.toSimpleString(tree);
            default:
                Assert.error("unexpected tree kind " + tree.getKind());
                return null;
        }
    }

    protected String formatIterable(JCDiagnostic d, Iterable<?> it, Locale l) {
        StringBuilder sbuf = new StringBuilder();
        String sep = "";
        for (Object o : it) {
            sbuf.append(sep);
            sbuf.append(formatArgument(d, o, l));
            sep = DocLint.TAGS_SEPARATOR;
        }
        return sbuf.toString();
    }

    protected List<String> formatSubdiagnostics(JCDiagnostic d, Locale l) {
        List<String> subdiagnostics = List.nil();
        int maxDepth = this.config.getMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit.DEPTH);
        if (maxDepth == -1 || this.depth < maxDepth) {
            this.depth++;
            try {
                int maxCount = this.config.getMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit.LENGTH);
                int count = 0;
                for (JCDiagnostic d2 : d.getSubdiagnostics()) {
                    if (maxCount != -1 && count >= maxCount) {
                        break;
                    }
                    subdiagnostics = subdiagnostics.append(formatSubdiagnostic(d, d2, l));
                    count++;
                }
            } finally {
                this.depth--;
            }
        }
        return subdiagnostics;
    }

    protected String formatSubdiagnostic(JCDiagnostic parent, JCDiagnostic sub, Locale l) {
        return formatMessage(sub, l);
    }

    protected String formatSourceLine(JCDiagnostic d, int nSpaces) {
        StringBuilder buf = new StringBuilder();
        DiagnosticSource source = d.getDiagnosticSource();
        int pos = d.getIntPosition();
        if (d.getIntPosition() == -1) {
            throw new AssertionError();
        }
        String line = source == null ? null : source.getLine(pos);
        if (line == null) {
            return "";
        }
        buf.append(indent(line, nSpaces));
        int col = source.getColumnNumber(pos, false);
        if (this.config.isCaretEnabled()) {
            buf.append("\n");
            for (int i = 0; i < col - 1; i++) {
                buf.append(line.charAt(i) == '\t' ? "\t" : " ");
            }
            buf.append(indent("^", nSpaces));
        }
        return buf.toString();
    }

    protected String formatLintCategory(JCDiagnostic d, Locale l) {
        Lint.LintCategory lc = d.getLintCategory();
        if (lc == null) {
            return "";
        }
        return localize(l, "compiler.warn.lintOption", lc.option);
    }

    protected String localize(Locale l, String key, Object... args) {
        return this.messages.getLocalizedString(l, key, args);
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public boolean displaySource(JCDiagnostic d) {
        return (!this.config.getVisible().contains(DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE) || d.getType() == JCDiagnostic.DiagnosticType.FRAGMENT || d.getIntPosition() == -1) ? false : true;
    }

    public boolean isRaw() {
        return false;
    }

    protected String indentString(int nSpaces) {
        if (nSpaces <= "                        ".length()) {
            return "                        ".substring(0, nSpaces);
        }
        StringBuilder buf = new StringBuilder();
        for (int i = 0; i < nSpaces; i++) {
            buf.append(" ");
        }
        return buf.toString();
    }

    protected String indent(String s, int nSpaces) {
        String indent = indentString(nSpaces);
        StringBuilder buf = new StringBuilder();
        String nl2 = "";
        for (String line : s.split("\n")) {
            buf.append(nl2);
            buf.append(indent + line);
            nl2 = "\n";
        }
        return buf.toString();
    }

    @Override // com.sun.tools.javac.api.DiagnosticFormatter
    public SimpleConfiguration getConfiguration() {
        return this.config;
    }

    public static class SimpleConfiguration implements DiagnosticFormatter.Configuration {
        protected boolean caretEnabled;
        protected Map<DiagnosticFormatter.Configuration.MultilineLimit, Integer> multilineLimits;
        protected EnumSet<DiagnosticFormatter.Configuration.DiagnosticPart> visibleParts;

        public SimpleConfiguration(Set<DiagnosticFormatter.Configuration.DiagnosticPart> parts) {
            this.multilineLimits = new HashMap();
            setVisible(parts);
            setMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit.DEPTH, -1);
            setMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit.LENGTH, -1);
            setCaretEnabled(true);
        }

        public SimpleConfiguration(Options options, Set<DiagnosticFormatter.Configuration.DiagnosticPart> parts) {
            this(parts);
            String showSource = options.get("showSource");
            if (showSource != null) {
                if (showSource.equals("true")) {
                    setVisiblePart(DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE, true);
                } else if (showSource.equals("false")) {
                    setVisiblePart(DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE, false);
                }
            }
            String diagOpts = options.get("diags");
            if (diagOpts != null) {
                Collection<String> args = Arrays.asList(diagOpts.split(DocLint.TAGS_SEPARATOR));
                if (args.contains("short")) {
                    setVisiblePart(DiagnosticFormatter.Configuration.DiagnosticPart.DETAILS, false);
                    setVisiblePart(DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS, false);
                }
                if (args.contains("source")) {
                    setVisiblePart(DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE, true);
                }
                if (args.contains("-source")) {
                    setVisiblePart(DiagnosticFormatter.Configuration.DiagnosticPart.SOURCE, false);
                }
            }
            String multiPolicy = options.get("multilinePolicy");
            if (multiPolicy != null) {
                if (multiPolicy.equals("disabled")) {
                    setVisiblePart(DiagnosticFormatter.Configuration.DiagnosticPart.SUBDIAGNOSTICS, false);
                } else if (multiPolicy.startsWith("limit:")) {
                    String limitString = multiPolicy.substring("limit:".length());
                    String[] limits = limitString.split(":");
                    try {
                        switch (limits.length) {
                            case 2:
                                if (!limits[1].equals(Marker.ANY_MARKER)) {
                                    setMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit.DEPTH, Integer.parseInt(limits[1]));
                                    break;
                                }
                            case 1:
                                if (!limits[0].equals(Marker.ANY_MARKER)) {
                                    setMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit.LENGTH, Integer.parseInt(limits[0]));
                                }
                                break;
                        }
                    } catch (NumberFormatException e) {
                        setMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit.DEPTH, -1);
                        setMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit.LENGTH, -1);
                    }
                }
            }
            String showCaret = options.get("showCaret");
            if (showCaret != null && showCaret.equals("false")) {
                setCaretEnabled(false);
            } else {
                setCaretEnabled(true);
            }
        }

        @Override // com.sun.tools.javac.api.DiagnosticFormatter.Configuration
        public int getMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit limit) {
            return this.multilineLimits.get(limit).intValue();
        }

        @Override // com.sun.tools.javac.api.DiagnosticFormatter.Configuration
        public EnumSet<DiagnosticFormatter.Configuration.DiagnosticPart> getVisible() {
            return EnumSet.copyOf((EnumSet) this.visibleParts);
        }

        @Override // com.sun.tools.javac.api.DiagnosticFormatter.Configuration
        public void setMultilineLimit(DiagnosticFormatter.Configuration.MultilineLimit limit, int value) {
            this.multilineLimits.put(limit, Integer.valueOf(value >= -1 ? value : -1));
        }

        @Override // com.sun.tools.javac.api.DiagnosticFormatter.Configuration
        public void setVisible(Set<DiagnosticFormatter.Configuration.DiagnosticPart> diagParts) {
            this.visibleParts = EnumSet.copyOf((Collection) diagParts);
        }

        public void setVisiblePart(DiagnosticFormatter.Configuration.DiagnosticPart diagParts, boolean enabled) {
            if (enabled) {
                this.visibleParts.add(diagParts);
            } else {
                this.visibleParts.remove(diagParts);
            }
        }

        public void setCaretEnabled(boolean caretEnabled) {
            this.caretEnabled = caretEnabled;
        }

        public boolean isCaretEnabled() {
            return this.caretEnabled;
        }
    }

    public Printer getPrinter() {
        return this.printer;
    }

    public void setPrinter(Printer printer) {
        this.printer = printer;
    }
}
