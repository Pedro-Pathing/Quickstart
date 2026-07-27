package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0325n1;
import com.android.tools.r8.graph.AbstractC0410w3;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.graph.H1;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.graph.M2;
import com.android.tools.r8.graph.N2;
import com.android.tools.r8.graph.U2;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C0795Mc;
import com.android.tools.r8.internal.C0806Ml;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1020Tt;
import com.android.tools.r8.internal.C1456cg;
import com.android.tools.r8.internal.C2026ig;
import com.android.tools.r8.internal.C2929sC;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3239ve0;
import com.android.tools.r8.internal.EnumC0765Lc;
import com.android.tools.r8.internal.InterfaceC1160Yw;
import com.android.tools.r8.internal.Nk0;
import com.android.tools.r8.internal.Y1;
import com.android.tools.r8.shaking.C3877i;
import com.android.tools.r8.shaking.C3884j;
import com.android.tools.r8.shaking.Y0;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.w, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3702w {
    public static final /* synthetic */ boolean e = true;
    public final C0421y a;
    public final C1020Tt b;
    public final B1 c;
    public C3701v d = null;

    public C3702w(C0421y c0421y, C1020Tt c1020Tt) {
        this.a = c0421y;
        this.b = c1020Tt;
        this.c = c0421y.a();
    }

    public final C3700u a(C0874Ot c0874Ot, com.android.tools.r8.ir.optimize.info.y yVar) {
        C0257g1 c0257g1Q;
        if (this.a.M().Z0) {
            return C3700u.b;
        }
        final A5 a5I = c0874Ot.i();
        if (a5I.a().f(this.a)) {
            return C3700u.b;
        }
        if (!a5I.e().n1()) {
            return C3700u.b;
        }
        final Set setC = AbstractC3424xb0.c();
        IdentityHashMap identityHashMap = (IdentityHashMap) a(a5I, c0874Ot, setC);
        if (identityHashMap.isEmpty()) {
            if (e || setC.isEmpty()) {
                return C3700u.b;
            }
            throw new AssertionError();
        }
        final IdentityHashMap identityHashMap2 = new IdentityHashMap();
        identityHashMap.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda0
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.a(setC, identityHashMap2, a5I, (C0257g1) obj, (C3239ve0) obj2);
            }
        });
        if (!setC.isEmpty()) {
            final Set setC2 = AbstractC3424xb0.c();
            InterfaceC1160Yw interfaceC1160YwR = c0874Ot.r();
            while (interfaceC1160YwR.hasNext()) {
                AbstractC1076Vw next = interfaceC1160YwR.next();
                if (next.e() && setC.contains(next.J0())) {
                    C3161ul0 c3161ul0Value = next.J0().value();
                    interfaceC1160YwR.p();
                    if (c3161ul0Value.U() <= 0) {
                        if (c3161ul0Value.I()) {
                            setC2.add(c3161ul0Value.c);
                        } else if (!c3161ul0Value.l() && c3161ul0Value.c.c2()) {
                            setC2.add(c3161ul0Value.c);
                        }
                    }
                }
            }
            if (setC2.size() > 0) {
                InterfaceC1160Yw interfaceC1160YwR2 = c0874Ot.r();
                Predicate predicate = new Predicate() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda5
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return setC2.contains((AbstractC1076Vw) obj);
                    }
                };
                while (interfaceC1160YwR2.hasNext()) {
                    if (predicate.test(interfaceC1160YwR2.next())) {
                        interfaceC1160YwR2.remove();
                    }
                }
            }
        }
        if (!this.a.o() || this.b.C == null) {
            identityHashMap2.forEach(new v$$ExternalSyntheticLambda0());
        } else if (this.a.g().i()) {
            final C3877i c3877i = (C3877i) this.a.V().g();
            Stream map = identityHashMap.values().stream().filter(new Predicate() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda6
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return setC.contains((C3239ve0) obj);
                }
            }).map(new Function() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda7
                @Override // java.util.function.Function
                public final Object apply(Object obj) {
                    return ((C3239ve0) obj).getField();
                }
            });
            Objects.requireNonNull(c3877i);
            final Set set = (Set) map.map(new C3712w$$ExternalSyntheticLambda8(c3877i)).map(new Function() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda9
                @Override // java.util.function.Function
                public final Object apply(Object obj) {
                    return ((AbstractC0410w3) obj).p();
                }
            }).filter(new Predicate() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda10
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return c3877i.d((com.android.tools.r8.graph.F0) obj);
                }
            }).map(new Function() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda11
                @Override // java.util.function.Function
                public final Object apply(Object obj) {
                    return ((com.android.tools.r8.graph.F0) obj).getReference();
                }
            }).collect(Collectors.toSet());
            for (AbstractC1076Vw abstractC1076Vw : c0874Ot.s()) {
                if (abstractC1076Vw.e() && (c0257g1Q = c3877i.c(abstractC1076Vw.J0().getField()).q()) != null) {
                    set.remove(c0257g1Q.getReference());
                }
            }
            yVar.a(new Consumer() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda12
                @Override // java.util.function.Consumer
                public final void accept(Object obj) {
                    C3702w.a(set, (C3884j) obj);
                }
            });
            synchronized (this) {
                C3701v c3701v = this.d;
                if (c3701v == null) {
                    this.d = new C3701v(identityHashMap2);
                    C1020Tt c1020Tt = this.b;
                    Y1 y1 = new Y1() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda1
                        @Override // com.android.tools.r8.internal.Y1
                        public final void b() {
                            this.f$0.a();
                        }
                    };
                    if (!c1020Tt.a.o()) {
                        throw new Nk0("addWaveDoneAction() should never be used in D8.");
                    }
                    List list = c1020Tt.C;
                    if (list == null) {
                        throw new Nk0("Attempt to call addWaveDoneAction() outside of wave.");
                    }
                    list.add(y1);
                } else {
                    c3701v.a(identityHashMap2);
                }
            }
        } else if (!e) {
            throw new AssertionError();
        }
        if (!identityHashMap2.isEmpty()) {
            c0874Ot.y();
        }
        return new C3700u(identityHashMap2);
    }

    public final void a(Set set, Map map, A5 a5, C0257g1 c0257g1, C3239ve0 c3239ve0) {
        EnumC0765Lc enumC0765Lc;
        I2 i2 = c0257g1.getReference().i;
        C3161ul0 c3161ul0I = c3239ve0.value().i();
        if (set.contains(c3239ve0)) {
            if (i2 == this.c.Y1) {
                D2 d2A = a5.a();
                AbstractC0325n1 kVar = null;
                if (!c3161ul0I.l()) {
                    if (c3161ul0I.J()) {
                        if (c3161ul0I.H()) {
                            if (!e && !c3161ul0I.Q()) {
                                throw new AssertionError();
                            }
                            kVar = U2.c;
                        } else if (c3161ul0I.I()) {
                            kVar = new N2.k(c3161ul0I.j().I().L2());
                        } else if (c3161ul0I.K()) {
                            C0806Ml c0806MlM = c3161ul0I.j().M();
                            if (!e && c0806MlM.k.d()) {
                                throw new AssertionError();
                            }
                            kVar = new M2(c0806MlM.j, c0806MlM.k);
                        } else if (!e) {
                            throw new AssertionError();
                        }
                    } else {
                        C0409w2 c0409w2U2 = c3161ul0I.i().c.i0().U2();
                        C0421y c0421y = this.a;
                        Y0 y0 = Y0.p;
                        if (c0421y.a(d2A).b(this.a.M())) {
                            H1 h1 = this.c.y4;
                            if (c0409w2U2 == h1.e) {
                                kVar = new M2(d2A.getType(), C0795Mc.a(EnumC0765Lc.c));
                            } else if (c0409w2U2 == h1.f) {
                                kVar = new M2(d2A.getType(), C0795Mc.a(EnumC0765Lc.e));
                            } else if (c0409w2U2 == h1.g) {
                                kVar = new M2(d2A.getType(), C0795Mc.a(EnumC0765Lc.f));
                            } else if (!e) {
                                throw new AssertionError();
                            }
                        } else {
                            H1 h12 = this.c.y4;
                            if (c0409w2U2 == h12.e) {
                                enumC0765Lc = EnumC0765Lc.c;
                            } else if (c0409w2U2 == h12.f) {
                                enumC0765Lc = EnumC0765Lc.e;
                            } else if (c0409w2U2 != h12.g) {
                                enumC0765Lc = null;
                            } else {
                                enumC0765Lc = EnumC0765Lc.f;
                            }
                            if (enumC0765Lc != null) {
                                kVar = new N2.k(enumC0765Lc.a(d2A.getType().Z0(), d2A, this.c, 0));
                            } else if (!e) {
                                throw new AssertionError();
                            }
                        }
                    }
                }
                map.put(c0257g1, kVar);
                return;
            }
            if (!i2.M0() && !i2.I0()) {
                C2026ig c2026igH = c3161ul0I.j().H();
                B1 b1 = this.c;
                if (i2 == b1.w1) {
                    map.put(c0257g1, N2.c.a(c2026igH.K2()));
                    return;
                }
                if (i2 == b1.x1) {
                    map.put(c0257g1, N2.d.a((byte) c2026igH.N2()));
                    return;
                }
                if (i2 == b1.D1) {
                    map.put(c0257g1, N2.j.a((short) c2026igH.N2()));
                    return;
                }
                if (i2 == b1.B1) {
                    map.put(c0257g1, N2.h.j(c2026igH.N2()));
                    return;
                }
                if (i2 == b1.C1) {
                    map.put(c0257g1, N2.i.a(c2026igH.O2()));
                    return;
                }
                if (i2 == b1.A1) {
                    map.put(c0257g1, N2.g.a(c2026igH.M2()));
                    return;
                } else if (i2 == b1.z1) {
                    map.put(c0257g1, N2.f.a(c2026igH.L2()));
                    return;
                } else {
                    if (i2 == b1.y1) {
                        map.put(c0257g1, N2.e.a((char) c2026igH.N2()));
                        return;
                    }
                    throw new Nk0("Unexpected field type " + i2 + ".");
                }
            }
            if (c3161ul0I.Q()) {
                map.put(c0257g1, U2.c);
                return;
            }
            throw new Nk0("Unexpected default value for field type " + i2 + ".");
        }
    }

    public static /* synthetic */ void a(Set set, final C3884j c3884j) {
        Objects.requireNonNull(c3884j);
        set.forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda4
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                c3884j.a((C0309l1) obj);
            }
        });
    }

    public final /* synthetic */ void a() {
        this.d.b();
        this.d = null;
    }

    /* JADX WARN: Removed duplicated region for block: B:147:0x018a A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:163:0x01a8 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:63:0x0137 A[Catch: all -> 0x0248, TryCatch #0 {all -> 0x0248, blocks: (B:3:0x001a, B:5:0x001f, B:7:0x0025, B:9:0x0030, B:10:0x003b, B:12:0x0041, B:14:0x004d, B:16:0x0059, B:19:0x0061, B:21:0x0065, B:23:0x0077, B:24:0x007e, B:27:0x0086, B:29:0x008c, B:31:0x009c, B:34:0x00a4, B:36:0x00be, B:39:0x00c6, B:41:0x00cc, B:44:0x00df, B:46:0x00e7, B:48:0x00ef, B:50:0x00f5, B:52:0x0101, B:54:0x0114, B:56:0x011c, B:58:0x0126, B:65:0x013f, B:67:0x014b, B:63:0x0137, B:88:0x01bc, B:70:0x0160, B:78:0x018a, B:80:0x0196, B:82:0x01a8, B:84:0x01ae, B:86:0x01b4, B:73:0x0173, B:76:0x0182, B:90:0x01cf, B:94:0x01d9, B:96:0x01e1, B:99:0x01e9, B:102:0x01f8, B:104:0x01fe, B:106:0x0204, B:108:0x020e, B:114:0x0222, B:116:0x022c), top: B:124:0x001a }] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final java.util.Map a(com.android.tools.r8.graph.A5 r19, com.android.tools.r8.internal.C0874Ot r20, java.util.Set r21) {
        /*
            Method dump skipped, instruction units count: 589
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.C3702w.a(com.android.tools.r8.graph.A5, com.android.tools.r8.internal.Ot, java.util.Set):java.util.Map");
    }

    public static Map a(final IdentityHashMap identityHashMap, IdentityHashMap identityHashMap2) {
        identityHashMap2.keySet().forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.w$$ExternalSyntheticLambda3
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                identityHashMap.remove((C0309l1) obj);
            }
        });
        return identityHashMap;
    }

    public final boolean a(D2 d2, AbstractC1076Vw abstractC1076Vw) {
        if (!abstractC1076Vw.c2()) {
            return false;
        }
        C2929sC c2929sCI0 = abstractC1076Vw.i0();
        if (!this.c.y4.p.contains(c2929sCI0.U2())) {
            return false;
        }
        C3161ul0 c3161ul0 = (C3161ul0) c2929sCI0.c.get(0);
        if (c3161ul0.l()) {
            return false;
        }
        AbstractC1076Vw abstractC1076Vw2 = c3161ul0.c;
        abstractC1076Vw2.getClass();
        return (abstractC1076Vw2 instanceof C1456cg) && c3161ul0.c.D().j == d2.e;
    }
}
