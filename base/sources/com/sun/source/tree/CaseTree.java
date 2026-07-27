package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface CaseTree extends Tree {
    ExpressionTree getExpression();

    List<? extends StatementTree> getStatements();
}
