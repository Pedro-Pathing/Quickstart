package com.sun.source.doctree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface UnknownBlockTagTree extends BlockTagTree {
    List<? extends DocTree> getContent();
}
