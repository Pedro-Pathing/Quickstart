package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.AssertionsConfiguration;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.d, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract /* synthetic */ class AbstractC3664d {
    public static final /* synthetic */ int[] a;

    static {
        int[] iArr = new int[((AssertionsConfiguration.a[]) AssertionsConfiguration.a.e.clone()).length];
        a = iArr;
        try {
            iArr[AssertionsConfiguration.a.c.ordinal()] = 1;
        } catch (NoSuchFieldError e) {
        }
        try {
            a[AssertionsConfiguration.a.d.ordinal()] = 2;
        } catch (NoSuchFieldError e2) {
        }
        try {
            a[AssertionsConfiguration.a.b.ordinal()] = 3;
        } catch (NoSuchFieldError e3) {
        }
    }
}
