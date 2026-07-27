package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface InstanceOfTree extends ExpressionTree {
    ExpressionTree getExpression();

    Tree getType();
}
