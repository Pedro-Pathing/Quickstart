package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface EnhancedForLoopTree extends StatementTree {
    ExpressionTree getExpression();

    StatementTree getStatement();

    VariableTree getVariable();
}
