package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface NewClassTree extends ExpressionTree {
    List<? extends ExpressionTree> getArguments();

    ClassTree getClassBody();

    ExpressionTree getEnclosingExpression();

    ExpressionTree getIdentifier();

    List<? extends Tree> getTypeArguments();
}
