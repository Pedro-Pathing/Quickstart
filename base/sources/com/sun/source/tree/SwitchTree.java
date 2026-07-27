package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface SwitchTree extends StatementTree {
    List<? extends CaseTree> getCases();

    ExpressionTree getExpression();
}
