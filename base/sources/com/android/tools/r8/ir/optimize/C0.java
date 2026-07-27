package com.android.tools.r8.ir.optimize;

import java.util.List;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C0 {
    public static final /* synthetic */ boolean d = true;
    public final int a;
    public final List b;
    public final List c;

    public C0(int i, List list, List list2) {
        this.a = i;
        this.b = list;
        this.c = list2;
        if (d || !A0.b(i)) {
            return;
        }
        if (list.isEmpty() || list2.isEmpty()) {
            throw new AssertionError();
        }
    }
}
