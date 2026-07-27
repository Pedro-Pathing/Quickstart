package com.sun.tools.javac.tree;

import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.tree.DCTree;

/* JADX INFO: loaded from: classes.dex */
public interface DocCommentTable {
    Tokens.Comment getComment(JCTree jCTree);

    String getCommentText(JCTree jCTree);

    DCTree.DCDocComment getCommentTree(JCTree jCTree);

    boolean hasComment(JCTree jCTree);

    void putComment(JCTree jCTree, Tokens.Comment comment);
}
