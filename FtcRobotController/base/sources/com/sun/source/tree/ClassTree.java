package com.sun.source.tree;

import java.util.List;
import javax.lang.model.element.Name;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface ClassTree extends StatementTree {
    Tree getExtendsClause();

    List<? extends Tree> getImplementsClause();

    List<? extends Tree> getMembers();

    ModifiersTree getModifiers();

    Name getSimpleName();

    List<? extends TypeParameterTree> getTypeParameters();
}
