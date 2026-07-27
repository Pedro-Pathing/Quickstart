package com.sun.source.tree;

import javax.lang.model.element.Name;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface MemberSelectTree extends ExpressionTree {
    ExpressionTree getExpression();

    Name getIdentifier();
}
