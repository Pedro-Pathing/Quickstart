package com.sun.source.tree;

import javax.lang.model.type.TypeKind;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface PrimitiveTypeTree extends Tree {
    TypeKind getPrimitiveTypeKind();
}
