package com.sun.tools.javac.tree;

/* JADX INFO: loaded from: classes.dex */
public interface EndPosTable {
    int getEndPos(JCTree jCTree);

    int replaceTree(JCTree jCTree, JCTree jCTree2);

    void storeEnd(JCTree jCTree, int i);
}
