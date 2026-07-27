package com.sun.source.doctree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface ReturnTree extends BlockTagTree {
    List<? extends DocTree> getDescription();
}
