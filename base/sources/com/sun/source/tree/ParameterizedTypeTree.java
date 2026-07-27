package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface ParameterizedTypeTree extends Tree {
    Tree getType();

    List<? extends Tree> getTypeArguments();
}
