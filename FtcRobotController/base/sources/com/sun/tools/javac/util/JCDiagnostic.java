package com.sun.tools.javac.util;

import androidx.core.app.NotificationCompat;
import com.sun.tools.javac.api.DiagnosticFormatter;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.tree.EndPosTable;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.Context;
import java.util.EnumSet;
import java.util.Locale;
import java.util.Set;
import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class JCDiagnostic implements Diagnostic<JavaFileObject> {

    @Deprecated
    private static DiagnosticFormatter<JCDiagnostic> fragmentFormatter;
    protected final Object[] args;
    private DiagnosticFormatter<JCDiagnostic> defaultFormatter;
    private final Set<DiagnosticFlag> flags;
    private final String key;
    private final Lint.LintCategory lintCategory;
    private final DiagnosticPosition position;
    private final DiagnosticSource source;
    private SourcePosition sourcePosition;
    private final DiagnosticType type;

    public enum DiagnosticFlag {
        MANDATORY,
        RESOLVE_ERROR,
        SYNTAX,
        RECOVERABLE,
        NON_DEFERRABLE,
        COMPRESSED
    }

    public interface DiagnosticPosition {
        int getEndPosition(EndPosTable endPosTable);

        int getPreferredPosition();

        int getStartPosition();

        JCTree getTree();
    }

    public static class Factory {
        protected static final Context.Key<Factory> diagnosticFactoryKey = new Context.Key<>();
        final Set<DiagnosticFlag> defaultErrorFlags;
        DiagnosticFormatter<JCDiagnostic> formatter;
        final String prefix;

        public static Factory instance(Context context) {
            Factory instance = (Factory) context.get(diagnosticFactoryKey);
            if (instance == null) {
                return new Factory(context);
            }
            return instance;
        }

        protected Factory(Context context) {
            this(JavacMessages.instance(context), "compiler");
            context.put(diagnosticFactoryKey, this);
            final Options options = Options.instance(context);
            initOptions(options);
            options.addListener(new Runnable() { // from class: com.sun.tools.javac.util.JCDiagnostic.Factory.1
                @Override // java.lang.Runnable
                public void run() {
                    Factory.this.initOptions(options);
                }
            });
        }

        /* JADX INFO: Access modifiers changed from: private */
        public void initOptions(Options options) {
            if (options.isSet("onlySyntaxErrorsUnrecoverable")) {
                this.defaultErrorFlags.add(DiagnosticFlag.RECOVERABLE);
            }
        }

        public Factory(JavacMessages messages, String prefix) {
            this.prefix = prefix;
            this.formatter = new BasicDiagnosticFormatter(messages);
            this.defaultErrorFlags = EnumSet.of(DiagnosticFlag.MANDATORY);
        }

        public JCDiagnostic error(DiagnosticSource source, DiagnosticPosition pos, String key, Object... args) {
            return create(DiagnosticType.ERROR, null, this.defaultErrorFlags, source, pos, key, args);
        }

        public JCDiagnostic mandatoryWarning(DiagnosticSource source, DiagnosticPosition pos, String key, Object... args) {
            return create(DiagnosticType.WARNING, null, EnumSet.of(DiagnosticFlag.MANDATORY), source, pos, key, args);
        }

        public JCDiagnostic mandatoryWarning(Lint.LintCategory lc, DiagnosticSource source, DiagnosticPosition pos, String key, Object... args) {
            return create(DiagnosticType.WARNING, lc, EnumSet.of(DiagnosticFlag.MANDATORY), source, pos, key, args);
        }

        public JCDiagnostic warning(Lint.LintCategory lc, String key, Object... args) {
            return create(DiagnosticType.WARNING, lc, EnumSet.noneOf(DiagnosticFlag.class), null, null, key, args);
        }

        public JCDiagnostic warning(DiagnosticSource source, DiagnosticPosition pos, String key, Object... args) {
            return create(DiagnosticType.WARNING, null, EnumSet.noneOf(DiagnosticFlag.class), source, pos, key, args);
        }

        public JCDiagnostic warning(Lint.LintCategory lc, DiagnosticSource source, DiagnosticPosition pos, String key, Object... args) {
            return create(DiagnosticType.WARNING, lc, EnumSet.noneOf(DiagnosticFlag.class), source, pos, key, args);
        }

        public JCDiagnostic mandatoryNote(DiagnosticSource source, String key, Object... args) {
            return create(DiagnosticType.NOTE, null, EnumSet.of(DiagnosticFlag.MANDATORY), source, null, key, args);
        }

        public JCDiagnostic note(String key, Object... args) {
            return create(DiagnosticType.NOTE, null, EnumSet.noneOf(DiagnosticFlag.class), null, null, key, args);
        }

        public JCDiagnostic note(DiagnosticSource source, DiagnosticPosition pos, String key, Object... args) {
            return create(DiagnosticType.NOTE, null, EnumSet.noneOf(DiagnosticFlag.class), source, pos, key, args);
        }

        public JCDiagnostic fragment(String key, Object... args) {
            return create(DiagnosticType.FRAGMENT, null, EnumSet.noneOf(DiagnosticFlag.class), null, null, key, args);
        }

        public JCDiagnostic create(DiagnosticType kind, DiagnosticSource source, DiagnosticPosition pos, String key, Object... args) {
            return create(kind, null, EnumSet.noneOf(DiagnosticFlag.class), source, pos, key, args);
        }

        public JCDiagnostic create(DiagnosticType kind, Lint.LintCategory lc, Set<DiagnosticFlag> flags, DiagnosticSource source, DiagnosticPosition pos, String key, Object... args) {
            return new JCDiagnostic(this.formatter, kind, lc, flags, source, pos, qualify(kind, key), args);
        }

        protected String qualify(DiagnosticType t, String key) {
            return this.prefix + "." + t.key + "." + key;
        }
    }

    @Deprecated
    public static JCDiagnostic fragment(String key, Object... args) {
        return new JCDiagnostic(getFragmentFormatter(), DiagnosticType.FRAGMENT, null, EnumSet.noneOf(DiagnosticFlag.class), null, null, "compiler." + DiagnosticType.FRAGMENT.key + "." + key, args);
    }

    @Deprecated
    public static DiagnosticFormatter<JCDiagnostic> getFragmentFormatter() {
        if (fragmentFormatter == null) {
            fragmentFormatter = new BasicDiagnosticFormatter(JavacMessages.getDefaultMessages());
        }
        return fragmentFormatter;
    }

    public enum DiagnosticType {
        FRAGMENT("misc"),
        NOTE("note"),
        WARNING("warn"),
        ERROR(NotificationCompat.CATEGORY_ERROR);

        final String key;

        DiagnosticType(String key) {
            this.key = key;
        }
    }

    public static class SimpleDiagnosticPosition implements DiagnosticPosition {
        private final int pos;

        public SimpleDiagnosticPosition(int pos) {
            this.pos = pos;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public JCTree getTree() {
            return null;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public int getStartPosition() {
            return this.pos;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public int getPreferredPosition() {
            return this.pos;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public int getEndPosition(EndPosTable endPosTable) {
            return this.pos;
        }
    }

    class SourcePosition {
        private final int column;
        private final int line;

        SourcePosition() {
            int n = JCDiagnostic.this.position == null ? -1 : JCDiagnostic.this.position.getPreferredPosition();
            if (n != -1 && JCDiagnostic.this.source != null) {
                this.line = JCDiagnostic.this.source.getLineNumber(n);
                this.column = JCDiagnostic.this.source.getColumnNumber(n, true);
            } else {
                this.column = -1;
                this.line = -1;
            }
        }

        public int getLineNumber() {
            return this.line;
        }

        public int getColumnNumber() {
            return this.column;
        }
    }

    protected JCDiagnostic(DiagnosticFormatter<JCDiagnostic> formatter, DiagnosticType dt, Lint.LintCategory lc, Set<DiagnosticFlag> flags, DiagnosticSource source, DiagnosticPosition pos, String key, Object... args) {
        if (source == null && pos != null && pos.getPreferredPosition() != -1) {
            throw new IllegalArgumentException();
        }
        this.defaultFormatter = formatter;
        this.type = dt;
        this.lintCategory = lc;
        this.flags = flags;
        this.source = source;
        this.position = pos;
        this.key = key;
        this.args = args;
    }

    public DiagnosticType getType() {
        return this.type;
    }

    public List<JCDiagnostic> getSubdiagnostics() {
        return List.nil();
    }

    public boolean isMultiline() {
        return false;
    }

    public boolean isMandatory() {
        return this.flags.contains(DiagnosticFlag.MANDATORY);
    }

    public boolean hasLintCategory() {
        return this.lintCategory != null;
    }

    public Lint.LintCategory getLintCategory() {
        return this.lintCategory;
    }

    @Override // javax.tools.Diagnostic
    public JavaFileObject getSource() {
        if (this.source == null) {
            return null;
        }
        return this.source.getFile();
    }

    public DiagnosticSource getDiagnosticSource() {
        return this.source;
    }

    protected int getIntStartPosition() {
        if (this.position == null) {
            return -1;
        }
        return this.position.getStartPosition();
    }

    protected int getIntPosition() {
        if (this.position == null) {
            return -1;
        }
        return this.position.getPreferredPosition();
    }

    protected int getIntEndPosition() {
        if (this.position == null) {
            return -1;
        }
        return this.position.getEndPosition(this.source.getEndPosTable());
    }

    @Override // javax.tools.Diagnostic
    public long getStartPosition() {
        return getIntStartPosition();
    }

    @Override // javax.tools.Diagnostic
    public long getPosition() {
        return getIntPosition();
    }

    @Override // javax.tools.Diagnostic
    public long getEndPosition() {
        return getIntEndPosition();
    }

    public DiagnosticPosition getDiagnosticPosition() {
        return this.position;
    }

    @Override // javax.tools.Diagnostic
    public long getLineNumber() {
        if (this.sourcePosition == null) {
            this.sourcePosition = new SourcePosition();
        }
        return this.sourcePosition.getLineNumber();
    }

    @Override // javax.tools.Diagnostic
    public long getColumnNumber() {
        if (this.sourcePosition == null) {
            this.sourcePosition = new SourcePosition();
        }
        return this.sourcePosition.getColumnNumber();
    }

    public Object[] getArgs() {
        return this.args;
    }

    public String getPrefix() {
        return getPrefix(this.type);
    }

    public String getPrefix(DiagnosticType dt) {
        return this.defaultFormatter.formatKind(this, Locale.getDefault());
    }

    public String toString() {
        return this.defaultFormatter.format(this, Locale.getDefault());
    }

    @Override // javax.tools.Diagnostic
    public Diagnostic.Kind getKind() {
        switch (this.type) {
            case NOTE:
                return Diagnostic.Kind.NOTE;
            case WARNING:
                return this.flags.contains(DiagnosticFlag.MANDATORY) ? Diagnostic.Kind.MANDATORY_WARNING : Diagnostic.Kind.WARNING;
            case ERROR:
                return Diagnostic.Kind.ERROR;
            default:
                return Diagnostic.Kind.OTHER;
        }
    }

    @Override // javax.tools.Diagnostic
    public String getCode() {
        return this.key;
    }

    @Override // javax.tools.Diagnostic
    public String getMessage(Locale locale) {
        return this.defaultFormatter.formatMessage(this, locale);
    }

    public void setFlag(DiagnosticFlag flag) {
        this.flags.add(flag);
        if (this.type == DiagnosticType.ERROR) {
            switch (flag) {
                case SYNTAX:
                    this.flags.remove(DiagnosticFlag.RECOVERABLE);
                    break;
                case RESOLVE_ERROR:
                    this.flags.add(DiagnosticFlag.RECOVERABLE);
                    break;
            }
        }
    }

    public boolean isFlagSet(DiagnosticFlag flag) {
        return this.flags.contains(flag);
    }

    public static class MultilineDiagnostic extends JCDiagnostic {
        private final List<JCDiagnostic> subdiagnostics;

        @Override // com.sun.tools.javac.util.JCDiagnostic, javax.tools.Diagnostic
        public /* bridge */ /* synthetic */ JavaFileObject getSource() {
            return super.getSource();
        }

        public MultilineDiagnostic(JCDiagnostic other, List<JCDiagnostic> subdiagnostics) {
            super(other.defaultFormatter, other.getType(), other.getLintCategory(), other.flags, other.getDiagnosticSource(), other.position, other.getCode(), other.getArgs());
            this.subdiagnostics = subdiagnostics;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic
        public List<JCDiagnostic> getSubdiagnostics() {
            return this.subdiagnostics;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic
        public boolean isMultiline() {
            return true;
        }
    }
}
