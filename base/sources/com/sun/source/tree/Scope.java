package com.sun.source.tree;

import javax.lang.model.element.Element;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.TypeElement;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface Scope {
    TypeElement getEnclosingClass();

    ExecutableElement getEnclosingMethod();

    Scope getEnclosingScope();

    Iterable<? extends Element> getLocalElements();
}
