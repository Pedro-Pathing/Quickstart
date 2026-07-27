package com.sun.tools.javac.util;

import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.util.JCDiagnostic;
import java.util.EnumSet;

/* JADX INFO: loaded from: classes.dex */
public class Warner {
    private EnumSet<Lint.LintCategory> nonSilentLintSet;
    private JCDiagnostic.DiagnosticPosition pos;
    private EnumSet<Lint.LintCategory> silentLintSet;
    protected boolean warned;

    public JCDiagnostic.DiagnosticPosition pos() {
        return this.pos;
    }

    public void warn(Lint.LintCategory lint) {
        this.nonSilentLintSet.add(lint);
    }

    public void silentWarn(Lint.LintCategory lint) {
        this.silentLintSet.add(lint);
    }

    public Warner(JCDiagnostic.DiagnosticPosition pos) {
        this.pos = null;
        this.warned = false;
        this.nonSilentLintSet = EnumSet.noneOf(Lint.LintCategory.class);
        this.silentLintSet = EnumSet.noneOf(Lint.LintCategory.class);
        this.pos = pos;
    }

    public boolean hasSilentLint(Lint.LintCategory lint) {
        return this.silentLintSet.contains(lint);
    }

    public boolean hasNonSilentLint(Lint.LintCategory lint) {
        return this.nonSilentLintSet.contains(lint);
    }

    public boolean hasLint(Lint.LintCategory lint) {
        return hasSilentLint(lint) || hasNonSilentLint(lint);
    }

    public void clear() {
        this.nonSilentLintSet.clear();
        this.silentLintSet.clear();
        this.warned = false;
    }

    public Warner() {
        this(null);
    }
}
