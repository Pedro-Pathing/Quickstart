package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface ForLoopTree extends StatementTree {
    ExpressionTree getCondition();

    List<? extends StatementTree> getInitializer();

    StatementTree getStatement();

    List<? extends ExpressionStatementTree> getUpdate();
}
