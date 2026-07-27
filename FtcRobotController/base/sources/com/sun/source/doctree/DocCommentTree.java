package com.sun.source.doctree;

import java.util.List;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface DocCommentTree extends DocTree {
    List<? extends DocTree> getBlockTags();

    List<? extends DocTree> getBody();

    List<? extends DocTree> getFirstSentence();
}
