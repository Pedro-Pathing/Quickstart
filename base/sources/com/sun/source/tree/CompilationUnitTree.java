package com.sun.source.tree;

import java.util.List;
import javax.tools.JavaFileObject;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface CompilationUnitTree extends Tree {
    List<? extends ImportTree> getImports();

    LineMap getLineMap();

    List<? extends AnnotationTree> getPackageAnnotations();

    ExpressionTree getPackageName();

    JavaFileObject getSourceFile();

    List<? extends Tree> getTypeDecls();
}
