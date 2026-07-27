package com.sun.source.doctree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface LinkTree extends InlineTagTree {
    List<? extends DocTree> getLabel();

    ReferenceTree getReference();
}
