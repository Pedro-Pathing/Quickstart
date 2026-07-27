package com.sun.source.tree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface IntersectionTypeTree extends Tree {
    List<? extends Tree> getBounds();
}
