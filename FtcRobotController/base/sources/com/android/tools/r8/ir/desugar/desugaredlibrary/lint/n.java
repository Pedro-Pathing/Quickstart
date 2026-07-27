package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.E0;
import com.android.tools.r8.ir.desugar.desugaredlibrary.lint.m;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.function.BiConsumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class n {
    public static final /* synthetic */ boolean g = true;
    public final E0 a;
    public l b;
    public final IdentityHashMap c = new IdentityHashMap();
    public final IdentityHashMap d = new IdentityHashMap();
    public final HashMap e = new HashMap();
    public final HashMap f = new HashMap();

    public n(E0 e0) {
        this.a = e0;
    }

    public final void a(BiConsumer biConsumer) {
        Iterator it = this.d.values().iterator();
        while (it.hasNext()) {
            biConsumer.accept(this.a, (C0257g1) it.next());
        }
    }

    public final void b(BiConsumer biConsumer) {
        Iterator it = this.c.values().iterator();
        while (it.hasNext()) {
            biConsumer.accept(this.a, (C0291j1) it.next());
        }
    }

    public final void a(C0409w2 c0409w2, m.c cVar) {
        if (!g && c0409w2.w0() != this.a.e) {
            throw new AssertionError();
        }
        HashMap map = this.e;
        m.c cVar2 = m.c.i;
        m.c cVar3 = (m.c) map.getOrDefault(c0409w2, cVar2);
        HashMap map2 = this.e;
        if (cVar == cVar2) {
            cVar = cVar3;
        } else if (cVar3 != cVar2) {
            int iA = cVar.a(cVar3);
            cVar = new m.c(cVar.e || cVar3.e, cVar.f || cVar3.f, cVar.g || cVar3.g, cVar.a || cVar3.a, iA & 255, iA >> 16);
        }
        map2.put(c0409w2, cVar);
    }

    public final void a(C0309l1 c0309l1, m.a aVar) {
        if (!g && c0309l1.w0() != this.a.e) {
            throw new AssertionError();
        }
        HashMap map = this.f;
        m.a aVar2 = m.a.e;
        m.a aVar3 = (m.a) map.getOrDefault(c0309l1, aVar2);
        HashMap map2 = this.f;
        if (aVar == aVar2) {
            aVar = aVar3;
        } else if (aVar3 != aVar2) {
            int iA = aVar.a(aVar3);
            aVar = new m.a(iA & 255, iA >> 16, aVar.a || aVar3.a);
        }
        map2.put(c0309l1, aVar);
    }
}
