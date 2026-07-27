package com.sun.tools.javac.util;

import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.util.JCDiagnostic;
import java.util.HashMap;
import java.util.Map;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public abstract class AbstractLog {
    protected JCDiagnostic.Factory diags;
    protected DiagnosticSource source;
    protected Map<JavaFileObject, DiagnosticSource> sourceMap = new HashMap();

    protected abstract void directError(String str, Object... objArr);

    protected abstract void report(JCDiagnostic jCDiagnostic);

    AbstractLog(JCDiagnostic.Factory diags) {
        this.diags = diags;
    }

    public JavaFileObject useSource(JavaFileObject file) {
        JavaFileObject prev = this.source == null ? null : this.source.getFile();
        this.source = getSource(file);
        return prev;
    }

    protected DiagnosticSource getSource(JavaFileObject file) {
        if (file == null) {
            return DiagnosticSource.NO_SOURCE;
        }
        DiagnosticSource s = this.sourceMap.get(file);
        if (s == null) {
            DiagnosticSource s2 = new DiagnosticSource(file, this);
            this.sourceMap.put(file, s2);
            return s2;
        }
        return s;
    }

    public DiagnosticSource currentSource() {
        return this.source;
    }

    public void error(String key, Object... args) {
        report(this.diags.error(this.source, null, key, args));
    }

    public void error(JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        report(this.diags.error(this.source, pos, key, args));
    }

    public void error(JCDiagnostic.DiagnosticFlag flag, JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        JCDiagnostic d = this.diags.error(this.source, pos, key, args);
        d.setFlag(flag);
        report(d);
    }

    public void error(int pos, String key, Object... args) {
        report(this.diags.error(this.source, wrap(pos), key, args));
    }

    public void error(JCDiagnostic.DiagnosticFlag flag, int pos, String key, Object... args) {
        JCDiagnostic d = this.diags.error(this.source, wrap(pos), key, args);
        d.setFlag(flag);
        report(d);
    }

    public void warning(String key, Object... args) {
        report(this.diags.warning(this.source, null, key, args));
    }

    public void warning(Lint.LintCategory lc, String key, Object... args) {
        report(this.diags.warning(lc, key, args));
    }

    public void warning(JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        report(this.diags.warning(this.source, pos, key, args));
    }

    public void warning(Lint.LintCategory lc, JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        report(this.diags.warning(lc, this.source, pos, key, args));
    }

    public void warning(int pos, String key, Object... args) {
        report(this.diags.warning(this.source, wrap(pos), key, args));
    }

    public void mandatoryWarning(JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        report(this.diags.mandatoryWarning(this.source, pos, key, args));
    }

    public void mandatoryWarning(Lint.LintCategory lc, JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        report(this.diags.mandatoryWarning(lc, this.source, pos, key, args));
    }

    public void note(String key, Object... args) {
        report(this.diags.note(this.source, null, key, args));
    }

    public void note(JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        report(this.diags.note(this.source, pos, key, args));
    }

    public void note(int pos, String key, Object... args) {
        report(this.diags.note(this.source, wrap(pos), key, args));
    }

    public void note(JavaFileObject file, String key, Object... args) {
        report(this.diags.note(getSource(file), null, key, args));
    }

    public void mandatoryNote(JavaFileObject file, String key, Object... args) {
        report(this.diags.mandatoryNote(getSource(file), key, args));
    }

    private JCDiagnostic.DiagnosticPosition wrap(int pos) {
        if (pos == -1) {
            return null;
        }
        return new JCDiagnostic.SimpleDiagnosticPosition(pos);
    }
}
