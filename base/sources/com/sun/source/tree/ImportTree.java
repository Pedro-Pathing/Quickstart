package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface ImportTree extends Tree {
    Tree getQualifiedIdentifier();

    boolean isStatic();
}
