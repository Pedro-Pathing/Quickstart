package com.sun.source.doctree;

import java.util.List;
import javax.lang.model.element.Name;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface StartElementTree extends DocTree {
    List<? extends DocTree> getAttributes();

    Name getName();

    boolean isSelfClosing();
}
