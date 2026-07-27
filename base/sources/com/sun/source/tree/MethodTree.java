package com.sun.source.tree;

import java.util.List;
import javax.lang.model.element.Name;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface MethodTree extends Tree {
    BlockTree getBody();

    Tree getDefaultValue();

    ModifiersTree getModifiers();

    Name getName();

    List<? extends VariableTree> getParameters();

    VariableTree getReceiverParameter();

    Tree getReturnType();

    List<? extends ExpressionTree> getThrows();

    List<? extends TypeParameterTree> getTypeParameters();
}
