package com.sun.source.doctree;

import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface ErroneousTree extends TextTree {
    Diagnostic<JavaFileObject> getDiagnostic();
}
