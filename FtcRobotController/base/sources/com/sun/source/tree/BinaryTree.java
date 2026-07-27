package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface BinaryTree extends ExpressionTree {
    ExpressionTree getLeftOperand();

    ExpressionTree getRightOperand();
}
