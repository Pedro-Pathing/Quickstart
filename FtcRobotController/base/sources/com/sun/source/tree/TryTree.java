package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface TryTree extends StatementTree {
    BlockTree getBlock();

    List<? extends CatchTree> getCatches();

    BlockTree getFinallyBlock();

    List<? extends Tree> getResources();
}
