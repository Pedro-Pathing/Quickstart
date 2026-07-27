package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.graph.B1;
import com.android.tools.r8.internal.C1064Vk;
import com.android.tools.r8.internal.C3030tM;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.C3384x50;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.InterfaceC1037Uk;
import com.android.tools.r8.t0;
import java.nio.file.Path;
import java.util.Collection;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class a {
    final C3200vB a;
    final InterfaceC1037Uk b;
    final t0 c;
    final Collection d;
    final Path e;
    final Collection f;
    static final /* synthetic */ boolean h = true;
    static final EnumC3471y2 g = EnumC3471y2.J;

    a(C3384x50 c3384x50, t0 t0Var, Collection collection, Path path, Collection collection2) {
        if (!h && collection2 == null) {
            throw new AssertionError();
        }
        C3200vB c3200vB = new C3200vB(new B1(), c3384x50).w().a(true).f;
        this.a = c3200vB;
        this.c = t0Var;
        this.f = collection2;
        this.b = t0Var == null ? C3030tM.g() : C1064Vk.a(t0Var, c3200vB.s(), c3200vB.i, false, EnumC3471y2.c.d());
        this.d = collection;
        this.e = path;
    }
}
