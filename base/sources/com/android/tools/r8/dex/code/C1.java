package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.C0365q5;
import java.nio.ShortBuffer;
import java.util.ArrayList;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public class C1 extends O {
    public A1[] a(ShortBuffer shortBuffer, int i, int i2, C0365q5 c0365q5) {
        B1 b1 = new B1(i, i2, shortBuffer);
        ArrayList arrayList = new ArrayList(i2);
        while (b1.a - b1.d > 0) {
            arrayList.add(O.a(b1.a(), b1.a(), b1, c0365q5));
        }
        return (A1[]) arrayList.toArray(A1.c);
    }
}
