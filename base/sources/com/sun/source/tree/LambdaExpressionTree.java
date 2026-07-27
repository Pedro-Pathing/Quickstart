package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface LambdaExpressionTree extends ExpressionTree {

    @Exported
    public enum BodyKind {
        EXPRESSION,
        STATEMENT
    }

    Tree getBody();

    BodyKind getBodyKind();

    List<? extends VariableTree> getParameters();
}
