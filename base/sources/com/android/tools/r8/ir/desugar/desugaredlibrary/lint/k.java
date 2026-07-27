package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.E0;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.C3366wv;
import com.android.tools.r8.internal.InterfaceC2428mi0;
import com.android.tools.r8.internal.S40;
import com.android.tools.r8.ir.desugar.desugaredlibrary.lint.m;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class k {
    public static final /* synthetic */ boolean c = true;
    public final IdentityHashMap a = new IdentityHashMap();
    public List b;

    public k() {
        int i = AbstractC0695Iu.c;
        this.b = S40.e;
    }

    public static n a(E0 e0, I2 i2) {
        return new n(e0);
    }

    public static n b(E0 e0, I2 i2) {
        return new n(e0);
    }

    public static void a(InterfaceC2428mi0 interfaceC2428mi0, n nVar) {
        interfaceC2428mi0.a(nVar.a, nVar.d.values(), nVar.c.values());
    }

    public final void b(final BiConsumer biConsumer) {
        this.a.values().forEach(new Consumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.k$$ExternalSyntheticLambda1
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((n) obj).b(biConsumer);
            }
        });
    }

    public static void a(Map map, I2 i2, n nVar) {
        map.put(i2, new m.d(nVar.a, nVar.b, C3366wv.a(nVar.c), C3366wv.a(nVar.d), nVar.e, nVar.f));
    }

    public final void a(final InterfaceC2428mi0 interfaceC2428mi0) {
        this.a.values().forEach(new Consumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.k$$ExternalSyntheticLambda3
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                k.a(interfaceC2428mi0, (n) obj);
            }
        });
    }

    public final void a(final BiConsumer biConsumer) {
        this.a.values().forEach(new Consumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.k$$ExternalSyntheticLambda5
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                ((n) obj).a(biConsumer);
            }
        });
    }

    public final void a(final E0 e0, C0291j1 c0291j1) {
        n nVar = (n) this.a.computeIfAbsent(e0.e, new Function() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.k$$ExternalSyntheticLambda2
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return k.b(e0, (I2) obj);
            }
        });
        if (!n.g) {
            nVar.getClass();
            if (c0291j1.F0() != nVar.a.e) {
                throw new AssertionError();
            }
        }
        nVar.c.put(c0291j1.getReference(), c0291j1);
    }

    public final void a(final E0 e0, C0257g1 c0257g1) {
        n nVar = (n) this.a.computeIfAbsent(e0.e, new Function() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.k$$ExternalSyntheticLambda4
            @Override // java.util.function.Function
            public final Object apply(Object obj) {
                return k.a(e0, (I2) obj);
            }
        });
        if (!n.g) {
            nVar.getClass();
            if (c0257g1.F0() != nVar.a.e) {
                throw new AssertionError();
            }
        }
        nVar.d.put(c0257g1.getReference(), c0257g1);
    }

    public final void a(I2 i2, l lVar) {
        l lVar2;
        n nVar = (n) this.a.get(i2);
        if (!c && nVar == null) {
            throw new AssertionError();
        }
        nVar.getClass();
        boolean z = n.g;
        if (!z && lVar == null) {
            throw new AssertionError();
        }
        if (!z && (lVar2 = nVar.b) != null && lVar != lVar2) {
            throw new AssertionError();
        }
        nVar.b = lVar;
    }

    public final void a(C0409w2 c0409w2, m.c cVar) {
        n nVar = (n) this.a.get(c0409w2.w0());
        if (!c && nVar == null) {
            throw new AssertionError();
        }
        nVar.a(c0409w2, cVar);
    }

    public final void a(C0309l1 c0309l1, m.a aVar) {
        n nVar = (n) this.a.get(c0309l1.w0());
        if (!c && nVar == null) {
            throw new AssertionError();
        }
        nVar.a(c0309l1, aVar);
    }

    public final m a() {
        final IdentityHashMap identityHashMap = new IdentityHashMap();
        this.a.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.k$$ExternalSyntheticLambda0
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                k.a(identityHashMap, (I2) obj, (n) obj2);
            }
        });
        return new m(C3366wv.a(identityHashMap), this.b);
    }
}
