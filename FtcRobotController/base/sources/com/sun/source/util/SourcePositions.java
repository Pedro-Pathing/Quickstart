package com.sun.source.util;

import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.Tree;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface SourcePositions {
    long getEndPosition(CompilationUnitTree compilationUnitTree, Tree tree);

    long getStartPosition(CompilationUnitTree compilationUnitTree, Tree tree);
}
