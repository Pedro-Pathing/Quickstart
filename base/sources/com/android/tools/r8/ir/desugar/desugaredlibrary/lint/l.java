package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.S40;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class l {
    public static final l e = new l();
    public final boolean a;
    public final boolean b;
    public final List c;
    public final AbstractC0695Iu d;

    public l(boolean z, ArrayList arrayList, ArrayList arrayList2) {
        this.a = false;
        this.b = z;
        arrayList.sort(Comparator.naturalOrder());
        this.c = arrayList;
        arrayList2.sort(Comparator.naturalOrder());
        this.d = AbstractC0695Iu.a(arrayList2);
    }

    public final List a() {
        return this.c;
    }

    public final List b() {
        return this.d;
    }

    public final boolean c() {
        return this.a;
    }

    public final boolean d() {
        return this.b;
    }

    public l() {
        this.a = true;
        this.b = false;
        int i = AbstractC0695Iu.c;
        S40 s40 = S40.e;
        this.c = s40;
        this.d = s40;
    }
}
