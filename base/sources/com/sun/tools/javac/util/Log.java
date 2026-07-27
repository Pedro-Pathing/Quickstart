package com.sun.tools.javac.util;

import com.sun.tools.javac.api.DiagnosticFormatter;
import com.sun.tools.javac.jvm.ByteCodes;
import com.sun.tools.javac.main.Main;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.tree.EndPosTable;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.JCDiagnostic;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Queue;
import java.util.Set;
import javax.tools.Diagnostic;
import javax.tools.DiagnosticListener;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class Log extends AbstractLog {
    public static final Context.Key<Log> logKey = new Context.Key<>();
    public static final Context.Key<PrintWriter> outKey = new Context.Key<>();
    private static boolean useRawMessages = false;
    protected int MaxErrors;
    protected int MaxWarnings;
    public boolean compressedOutput;
    private DiagnosticFormatter<JCDiagnostic> diagFormatter;
    protected DiagnosticListener<? super JavaFileObject> diagListener;
    private DiagnosticHandler diagnosticHandler;
    public boolean dumpOnError;
    public boolean emitWarnings;
    protected PrintWriter errWriter;
    public Set<String> expectDiagKeys;
    private JavacMessages messages;
    public boolean multipleErrors;
    public int nerrors;
    protected PrintWriter noticeWriter;
    public int nwarnings;
    public boolean promptOnError;
    private Set<Pair<JavaFileObject, Integer>> recorded;
    public boolean suppressNotes;
    protected PrintWriter warnWriter;

    public enum WriterKind {
        NOTICE,
        WARNING,
        ERROR
    }

    public enum PrefixKind {
        JAVAC("javac."),
        COMPILER_MISC("compiler.misc.");

        final String value;

        PrefixKind(String v) {
            this.value = v;
        }

        public String key(String k) {
            return this.value + k;
        }
    }

    public static abstract class DiagnosticHandler {
        protected DiagnosticHandler prev;

        public abstract void report(JCDiagnostic jCDiagnostic);

        protected void install(Log log) {
            this.prev = log.diagnosticHandler;
            log.diagnosticHandler = this;
        }
    }

    public static class DiscardDiagnosticHandler extends DiagnosticHandler {
        public DiscardDiagnosticHandler(Log log) {
            install(log);
        }

        @Override // com.sun.tools.javac.util.Log.DiagnosticHandler
        public void report(JCDiagnostic diag) {
        }
    }

    public static class DeferredDiagnosticHandler extends DiagnosticHandler {
        private Queue<JCDiagnostic> deferred;
        private final Filter<JCDiagnostic> filter;

        public DeferredDiagnosticHandler(Log log) {
            this(log, null);
        }

        public DeferredDiagnosticHandler(Log log, Filter<JCDiagnostic> filter) {
            this.deferred = new ListBuffer();
            this.filter = filter;
            install(log);
        }

        @Override // com.sun.tools.javac.util.Log.DiagnosticHandler
        public void report(JCDiagnostic diag) {
            if (!diag.isFlagSet(JCDiagnostic.DiagnosticFlag.NON_DEFERRABLE) && (this.filter == null || this.filter.accepts(diag))) {
                this.deferred.add(diag);
            } else {
                this.prev.report(diag);
            }
        }

        public Queue<JCDiagnostic> getDiagnostics() {
            return this.deferred;
        }

        public void reportDeferredDiagnostics() {
            reportDeferredDiagnostics(EnumSet.allOf(Diagnostic.Kind.class));
        }

        public void reportDeferredDiagnostics(Set<Diagnostic.Kind> kinds) {
            while (true) {
                JCDiagnostic d = this.deferred.poll();
                if (d != null) {
                    if (kinds.contains(d.getKind())) {
                        this.prev.report(d);
                    }
                } else {
                    this.deferred = null;
                    return;
                }
            }
        }
    }

    protected Log(Context context, PrintWriter errWriter, PrintWriter warnWriter, PrintWriter noticeWriter) {
        super(JCDiagnostic.Factory.instance(context));
        this.nerrors = 0;
        this.nwarnings = 0;
        this.recorded = new HashSet();
        context.put(logKey, this);
        this.errWriter = errWriter;
        this.warnWriter = warnWriter;
        this.noticeWriter = noticeWriter;
        DiagnosticListener<? super JavaFileObject> dl = (DiagnosticListener) context.get(DiagnosticListener.class);
        this.diagListener = dl;
        this.diagnosticHandler = new DefaultDiagnosticHandler();
        this.messages = JavacMessages.instance(context);
        this.messages.add(Main.javacBundleName);
        final Options options = Options.instance(context);
        initOptions(options);
        options.addListener(new Runnable() { // from class: com.sun.tools.javac.util.Log.1
            @Override // java.lang.Runnable
            public void run() {
                Log.this.initOptions(options);
            }
        });
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void initOptions(Options options) {
        this.dumpOnError = options.isSet(Option.DOE);
        this.promptOnError = options.isSet(Option.PROMPT);
        this.emitWarnings = options.isUnset(Option.XLINT_CUSTOM, "none");
        this.suppressNotes = options.isSet("suppressNotes");
        this.MaxErrors = getIntOption(options, Option.XMAXERRS, getDefaultMaxErrors());
        this.MaxWarnings = getIntOption(options, Option.XMAXWARNS, getDefaultMaxWarnings());
        boolean rawDiagnostics = options.isSet("rawDiagnostics");
        this.diagFormatter = rawDiagnostics ? new RawDiagnosticFormatter(options) : new BasicDiagnosticFormatter(options, this.messages);
        String ek = options.get("expectKeys");
        if (ek != null) {
            this.expectDiagKeys = new HashSet(Arrays.asList(ek.split(", *")));
        }
    }

    private int getIntOption(Options options, Option option, int defaultValue) {
        String s = options.get(option);
        if (s != null) {
            try {
                int n = Integer.parseInt(s);
                if (n <= 0) {
                    return Integer.MAX_VALUE;
                }
                return n;
            } catch (NumberFormatException e) {
            }
        }
        return defaultValue;
    }

    protected int getDefaultMaxErrors() {
        return 100;
    }

    protected int getDefaultMaxWarnings() {
        return 100;
    }

    static PrintWriter defaultWriter(Context context) {
        PrintWriter result = (PrintWriter) context.get(outKey);
        if (result == null) {
            Context.Key<PrintWriter> key = outKey;
            PrintWriter result2 = new PrintWriter(System.err);
            context.put(key, result2);
            return result2;
        }
        return result;
    }

    protected Log(Context context) {
        this(context, defaultWriter(context));
    }

    protected Log(Context context, PrintWriter defaultWriter) {
        this(context, defaultWriter, defaultWriter, defaultWriter);
    }

    public static Log instance(Context context) {
        Log instance = (Log) context.get(logKey);
        if (instance == null) {
            return new Log(context);
        }
        return instance;
    }

    public boolean hasDiagnosticListener() {
        return this.diagListener != null;
    }

    public void setEndPosTable(JavaFileObject name, EndPosTable endPosTable) {
        name.getClass();
        getSource(name).setEndPosTable(endPosTable);
    }

    public JavaFileObject currentSourceFile() {
        if (this.source == null) {
            return null;
        }
        return this.source.getFile();
    }

    public DiagnosticFormatter<JCDiagnostic> getDiagnosticFormatter() {
        return this.diagFormatter;
    }

    public void setDiagnosticFormatter(DiagnosticFormatter<JCDiagnostic> diagFormatter) {
        this.diagFormatter = diagFormatter;
    }

    public PrintWriter getWriter(WriterKind kind) {
        switch (kind) {
            case NOTICE:
                return this.noticeWriter;
            case WARNING:
                return this.warnWriter;
            case ERROR:
                return this.errWriter;
            default:
                throw new IllegalArgumentException();
        }
    }

    public void setWriter(WriterKind kind, PrintWriter pw) {
        pw.getClass();
        switch (kind) {
            case NOTICE:
                this.noticeWriter = pw;
                return;
            case WARNING:
                this.warnWriter = pw;
                return;
            case ERROR:
                this.errWriter = pw;
                return;
            default:
                throw new IllegalArgumentException();
        }
    }

    public void setWriters(PrintWriter pw) {
        pw.getClass();
        this.errWriter = pw;
        this.warnWriter = pw;
        this.noticeWriter = pw;
    }

    public void initRound(Log other) {
        this.noticeWriter = other.noticeWriter;
        this.warnWriter = other.warnWriter;
        this.errWriter = other.errWriter;
        this.sourceMap = other.sourceMap;
        this.recorded = other.recorded;
        this.nerrors = other.nerrors;
        this.nwarnings = other.nwarnings;
    }

    public void popDiagnosticHandler(DiagnosticHandler h) {
        Assert.check(this.diagnosticHandler == h);
        this.diagnosticHandler = h.prev;
    }

    public void flush() {
        this.errWriter.flush();
        this.warnWriter.flush();
        this.noticeWriter.flush();
    }

    public void flush(WriterKind kind) {
        getWriter(kind).flush();
    }

    protected boolean shouldReport(JavaFileObject file, int pos) {
        if (this.multipleErrors || file == null) {
            return true;
        }
        Pair<JavaFileObject, Integer> coords = new Pair<>(file, Integer.valueOf(pos));
        boolean shouldReport = true ^ this.recorded.contains(coords);
        if (shouldReport) {
            this.recorded.add(coords);
        }
        return shouldReport;
    }

    public void prompt() {
        if (this.promptOnError) {
            System.err.println(localize("resume.abort", new Object[0]));
            while (true) {
                try {
                    switch (System.in.read()) {
                        case 65:
                        case 97:
                            System.exit(-1);
                            return;
                        case 82:
                        case ByteCodes.fmod /* 114 */:
                            return;
                        case 88:
                        case 120:
                            throw new AssertionError("user abort");
                    }
                } catch (IOException e) {
                    return;
                }
            }
        }
    }

    private void printErrLine(int pos, PrintWriter writer) {
        String line = this.source == null ? null : this.source.getLine(pos);
        if (line == null) {
            return;
        }
        int col = this.source.getColumnNumber(pos, false);
        printRawLines(writer, line);
        for (int i = 0; i < col - 1; i++) {
            writer.print(line.charAt(i) == '\t' ? "\t" : " ");
        }
        writer.println("^");
        writer.flush();
    }

    public void printNewline() {
        this.noticeWriter.println();
    }

    public void printNewline(WriterKind wk) {
        getWriter(wk).println();
    }

    public void printLines(String key, Object... args) {
        printRawLines(this.noticeWriter, localize(key, args));
    }

    public void printLines(PrefixKind pk, String key, Object... args) {
        printRawLines(this.noticeWriter, localize(pk, key, args));
    }

    public void printLines(WriterKind wk, String key, Object... args) {
        printRawLines(getWriter(wk), localize(key, args));
    }

    public void printLines(WriterKind wk, PrefixKind pk, String key, Object... args) {
        printRawLines(getWriter(wk), localize(pk, key, args));
    }

    public void printRawLines(String msg) {
        printRawLines(this.noticeWriter, msg);
    }

    public void printRawLines(WriterKind kind, String msg) {
        printRawLines(getWriter(kind), msg);
    }

    public static void printRawLines(PrintWriter writer, String msg) {
        while (true) {
            int nl2 = msg.indexOf(10);
            if (nl2 == -1) {
                break;
            }
            writer.println(msg.substring(0, nl2));
            msg = msg.substring(nl2 + 1);
        }
        if (msg.length() != 0) {
            writer.println(msg);
        }
    }

    public void printVerbose(String key, Object... args) {
        printRawLines(this.noticeWriter, localize("verbose." + key, args));
    }

    @Override // com.sun.tools.javac.util.AbstractLog
    protected void directError(String key, Object... args) {
        printRawLines(this.errWriter, localize(key, args));
        this.errWriter.flush();
    }

    public void strictWarning(JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        writeDiagnostic(this.diags.warning(this.source, pos, key, args));
        this.nwarnings++;
    }

    @Override // com.sun.tools.javac.util.AbstractLog
    public void report(JCDiagnostic diagnostic) {
        this.diagnosticHandler.report(diagnostic);
    }

    private class DefaultDiagnosticHandler extends DiagnosticHandler {
        private DefaultDiagnosticHandler() {
        }

        @Override // com.sun.tools.javac.util.Log.DiagnosticHandler
        public void report(JCDiagnostic diagnostic) {
            if (Log.this.expectDiagKeys != null) {
                Log.this.expectDiagKeys.remove(diagnostic.getCode());
            }
            switch (diagnostic.getType()) {
                case FRAGMENT:
                    throw new IllegalArgumentException();
                case NOTE:
                    if ((Log.this.emitWarnings || diagnostic.isMandatory()) && !Log.this.suppressNotes) {
                        Log.this.writeDiagnostic(diagnostic);
                    }
                    break;
                case WARNING:
                    if ((Log.this.emitWarnings || diagnostic.isMandatory()) && Log.this.nwarnings < Log.this.MaxWarnings) {
                        Log.this.writeDiagnostic(diagnostic);
                        Log.this.nwarnings++;
                    }
                    break;
                case ERROR:
                    if (Log.this.nerrors < Log.this.MaxErrors && Log.this.shouldReport(diagnostic.getSource(), diagnostic.getIntPosition())) {
                        Log.this.writeDiagnostic(diagnostic);
                        Log.this.nerrors++;
                    }
                    break;
            }
            if (diagnostic.isFlagSet(JCDiagnostic.DiagnosticFlag.COMPRESSED)) {
                Log.this.compressedOutput = true;
            }
        }
    }

    protected void writeDiagnostic(JCDiagnostic diag) {
        if (this.diagListener != null) {
            this.diagListener.report(diag);
            return;
        }
        PrintWriter writer = getWriterForDiagnosticType(diag.getType());
        printRawLines(writer, this.diagFormatter.format(diag, this.messages.getCurrentLocale()));
        if (this.promptOnError) {
            switch (diag.getType()) {
                case WARNING:
                case ERROR:
                    prompt();
                    break;
            }
        }
        if (this.dumpOnError) {
            new RuntimeException().printStackTrace(writer);
        }
        writer.flush();
    }

    @Deprecated
    protected PrintWriter getWriterForDiagnosticType(JCDiagnostic.DiagnosticType dt) {
        switch (dt) {
            case FRAGMENT:
                throw new IllegalArgumentException();
            case NOTE:
                return this.noticeWriter;
            case WARNING:
                return this.warnWriter;
            case ERROR:
                return this.errWriter;
            default:
                throw new Error();
        }
    }

    public static String getLocalizedString(String key, Object... args) {
        return JavacMessages.getDefaultLocalizedString(PrefixKind.COMPILER_MISC.key(key), args);
    }

    public String localize(String key, Object... args) {
        return localize(PrefixKind.COMPILER_MISC, key, args);
    }

    public String localize(PrefixKind pk, String key, Object... args) {
        if (useRawMessages) {
            return pk.key(key);
        }
        return this.messages.getLocalizedString(pk.key(key), args);
    }

    private void printRawError(int pos, String msg) {
        if (this.source == null || pos == -1) {
            printRawLines(this.errWriter, "error: " + msg);
        } else {
            int line = this.source.getLineNumber(pos);
            JavaFileObject file = this.source.getFile();
            if (file != null) {
                printRawLines(this.errWriter, file.getName() + ":" + line + ": " + msg);
            }
            printErrLine(pos, this.errWriter);
        }
        this.errWriter.flush();
    }

    public void rawError(int pos, String msg) {
        if (this.nerrors < this.MaxErrors && shouldReport(currentSourceFile(), pos)) {
            printRawError(pos, msg);
            prompt();
            this.nerrors++;
        }
        this.errWriter.flush();
    }

    public void rawWarning(int pos, String msg) {
        if (this.nwarnings < this.MaxWarnings && this.emitWarnings) {
            printRawError(pos, "warning: " + msg);
        }
        prompt();
        this.nwarnings++;
        this.errWriter.flush();
    }

    public static String format(String fmt, Object... args) {
        return String.format(null, fmt, args);
    }
}
