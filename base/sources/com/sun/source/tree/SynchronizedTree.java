package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface SynchronizedTree extends StatementTree {
    BlockTree getBlock();

    ExpressionTree getExpression();
}
