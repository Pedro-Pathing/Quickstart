package com.sun.source.util;

import com.sun.source.doctree.DocCommentTree;
import com.sun.source.doctree.DocTree;
import com.sun.source.tree.CompilationUnitTree;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface DocSourcePositions extends SourcePositions {
    long getEndPosition(CompilationUnitTree compilationUnitTree, DocCommentTree docCommentTree, DocTree docTree);

    long getStartPosition(CompilationUnitTree compilationUnitTree, DocCommentTree docCommentTree, DocTree docTree);
}
