package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface AnnotatedTypeTree extends ExpressionTree {
    List<? extends AnnotationTree> getAnnotations();

    ExpressionTree getUnderlyingType();
}
