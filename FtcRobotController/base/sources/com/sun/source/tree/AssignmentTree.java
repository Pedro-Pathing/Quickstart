package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface AssignmentTree extends ExpressionTree {
    ExpressionTree getExpression();

    ExpressionTree getVariable();
}
