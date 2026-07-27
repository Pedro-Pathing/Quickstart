package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface LineMap {
    long getColumnNumber(long j);

    long getLineNumber(long j);

    long getPosition(long j, long j2);

    long getStartPosition(long j);
}
