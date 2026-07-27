package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface UnionTypeTree extends Tree {
    List<? extends Tree> getTypeAlternatives();
}
