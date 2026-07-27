package com.sun.source.tree;

import java.util.List;
import javax.lang.model.element.Name;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface TypeParameterTree extends Tree {
    List<? extends AnnotationTree> getAnnotations();

    List<? extends Tree> getBounds();

    Name getName();
}
