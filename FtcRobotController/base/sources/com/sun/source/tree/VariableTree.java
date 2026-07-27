package com.sun.source.tree;

import javax.lang.model.element.Name;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface VariableTree extends StatementTree {
    ExpressionTree getInitializer();

    ModifiersTree getModifiers();

    Name getName();

    ExpressionTree getNameExpression();

    Tree getType();
}
