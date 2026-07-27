package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface AnnotationTree extends ExpressionTree {
    Tree getAnnotationType();

    List<? extends ExpressionTree> getArguments();
}
