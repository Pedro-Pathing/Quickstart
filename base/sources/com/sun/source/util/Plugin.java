package com.sun.source.util;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface Plugin {
    String getName();

    void init(JavacTask javacTask, String... strArr);
}
