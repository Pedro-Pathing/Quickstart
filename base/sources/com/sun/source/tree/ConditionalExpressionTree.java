package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface ConditionalExpressionTree extends ExpressionTree {
    ExpressionTree getCondition();

    ExpressionTree getFalseExpression();

    ExpressionTree getTrueExpression();
}
