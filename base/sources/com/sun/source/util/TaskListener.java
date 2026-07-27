package com.sun.source.util;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface TaskListener {
    void finished(TaskEvent taskEvent);

    void started(TaskEvent taskEvent);
}
