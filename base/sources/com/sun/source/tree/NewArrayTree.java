package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface NewArrayTree extends ExpressionTree {
    List<? extends AnnotationTree> getAnnotations();

    List<? extends List<? extends AnnotationTree>> getDimAnnotations();

    List<? extends ExpressionTree> getDimensions();

    List<? extends ExpressionTree> getInitializers();

    Tree getType();
}
