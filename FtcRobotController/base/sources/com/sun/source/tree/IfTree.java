package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface IfTree extends StatementTree {
    ExpressionTree getCondition();

    StatementTree getElseStatement();

    StatementTree getThenStatement();
}
