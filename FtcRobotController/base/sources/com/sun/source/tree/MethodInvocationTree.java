package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface MethodInvocationTree extends ExpressionTree {
    List<? extends ExpressionTree> getArguments();

    ExpressionTree getMethodSelect();

    List<? extends Tree> getTypeArguments();
}
