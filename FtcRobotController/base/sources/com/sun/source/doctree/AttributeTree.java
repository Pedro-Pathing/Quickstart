package com.sun.source.doctree;

import java.util.List;
import javax.lang.model.element.Name;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface AttributeTree extends DocTree {

    @Exported
    public enum ValueKind {
        EMPTY,
        UNQUOTED,
        SINGLE,
        DOUBLE
    }

    Name getName();

    List<? extends DocTree> getValue();

    ValueKind getValueKind();
}
