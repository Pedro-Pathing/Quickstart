package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface BlockTree extends StatementTree {
    List<? extends StatementTree> getStatements();

    boolean isStatic();
}
