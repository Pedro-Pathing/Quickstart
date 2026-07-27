package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.ClassFileResourceProvider;
import com.android.tools.r8.ProgramResourceProvider;
import com.android.tools.r8.dex.C0034c;
import com.android.tools.r8.graph.AbstractC0264h1;
import com.android.tools.r8.graph.AbstractC0368r2;
import com.android.tools.r8.graph.AbstractC0410w3;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0203a3;
import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0262h;
import com.android.tools.r8.graph.C0285j;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.graph.C0350p4;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.D2;
import com.android.tools.r8.graph.E0;
import com.android.tools.r8.graph.E4;
import com.android.tools.r8.graph.F2;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.graph.S4;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.AbstractC1784g4;
import com.android.tools.r8.internal.AbstractC2990sv;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C1064Vk;
import com.android.tools.r8.internal.C1552dh0;
import com.android.tools.r8.internal.C1653en;
import com.android.tools.r8.internal.C2917s5;
import com.android.tools.r8.internal.C3030tM;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.Fh0;
import com.android.tools.r8.internal.InterfaceC2428mi0;
import com.android.tools.r8.internal.S40;
import com.android.tools.r8.ir.desugar.desugaredlibrary.lint.m;
import com.android.tools.r8.synthesis.E;
import com.android.tools.r8.t0;
import com.android.tools.r8.utils.i;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Predicate;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class o {
    public static final /* synthetic */ boolean j = true;
    public final AbstractC2990sv a;
    public final AbstractC2990sv b;
    public final C3200vB c;
    public final C0203a3 d;
    public final EnumC3471y2 e;
    public final k f;
    public final boolean g;
    public final boolean h;
    public final boolean i;

    public o(C3200vB c3200vB, Collection<ClassFileResourceProvider> collection, boolean z) throws IOException {
        this(c3200vB, collection, z, EnumC3471y2.c, false, false);
    }

    public final void a() {
        this.f.a(new InterfaceC2428mi0() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.o$$ExternalSyntheticLambda4
            @Override // com.android.tools.r8.internal.InterfaceC2428mi0
            public final void a(Object obj, Object obj2, Object obj3) {
                this.f$0.a((E0) obj, (Collection) obj2, (Collection) obj3);
            }
        });
    }

    public m b(Collection<ProgramResourceProvider> collection, t0 t0Var) throws IOException {
        a(collection, t0Var);
        b();
        for (C0409w2 c0409w2 : c()) {
            k kVar = this.f;
            m.c cVar = m.c.j;
            if (((n) kVar.a.get(c0409w2.w0())) != null) {
                kVar.a(c0409w2, cVar);
            }
        }
        a(t0Var);
        a();
        return this.f.a();
    }

    public final Set c() {
        Set setC = AbstractC3424xb0.c();
        B1 b1S = this.c.s();
        setC.add(b1S.a(b1S.R2, b1S.a(b1S.c(b1S.c("Ljava/util/stream/Stream;")), new I2[0]), b1S.c("parallelStream")));
        I2 i2C = b1S.c(b1S.c("Ljava/util/stream/BaseStream;"));
        String[] strArr = {"Base", "Double", "Int", "Long"};
        for (int i = 0; i < 4; i++) {
            I2 i2C2 = b1S.c(b1S.c("Ljava/util/stream/" + strArr[i] + "Stream;"));
            setC.add(b1S.a(i2C2, b1S.a(i2C2, new I2[0]), b1S.c("parallel")));
            setC.add(b1S.a(i2C2, b1S.a(i2C, new I2[0]), b1S.c("parallel")));
        }
        return setC;
    }

    public o(C3200vB c3200vB, Collection collection, boolean z, EnumC3471y2 enumC3471y2, boolean z2, boolean z3) throws IOException {
        this.f = new k();
        this.i = z;
        this.c = c3200vB;
        i.a aVarB = com.android.tools.r8.utils.i.b();
        Iterator it = collection.iterator();
        while (it.hasNext()) {
            aVarB.b((ClassFileResourceProvider) it.next());
        }
        C0034c c0034c = new C0034c(aVarB.a(), this.c, Fh0.a());
        ExecutorService executorServiceA = C1552dh0.a(this.c);
        if (!j && this.c.j0) {
            throw new AssertionError();
        }
        this.c.j0 = true;
        C0350p4 c0350p4A = c0034c.a(executorServiceA);
        C3200vB c3200vB2 = this.c;
        c3200vB2.j0 = false;
        if (c0350p4A.d(c3200vB2.s().e("Ljava/lang/invoke/VarHandle;")) == null && !this.i) {
            this.c.i.c("SupportedClassesGenerator expects library above or equal to T, it works below, but the modifiers are not correct which is fine for lint but not html doc generation.");
        }
        this.d = c0350p4A.i();
        this.e = enumC3471y2;
        this.g = z2;
        this.h = z3;
        B1 b1S = c3200vB.s();
        this.a = AbstractC2990sv.a(3, 3, b1S.l2, b1S.m2, b1S.i2);
        I2 i2 = b1S.b3;
        C0409w2 c0409w2A = b1S.a(i2, b1S.a(i2, b1S.a2), "ofNullable");
        I2 i22 = b1S.W1;
        this.b = AbstractC2990sv.a(2, 2, c0409w2A, b1S.a(i22, b1S.a(b1S.B1, i22, i22), "compare"));
    }

    public final void a(E0 e0, Collection collection, Collection collection2) {
        k kVar = this.f;
        n nVar = (n) kVar.a.get(e0.e);
        boolean z = k.c;
        if (!z && nVar == null) {
            throw new AssertionError();
        }
        l lVar = nVar.b;
        if (lVar == null || !lVar.a) {
            E0 e0D = this.d.d(e0.e);
            ArrayList arrayList = new ArrayList();
            ArrayList arrayList2 = new ArrayList();
            boolean zA = a(e0D.M0(), collection, arrayList) & a(e0D.C1(), collection2, arrayList2);
            n nVar2 = (n) this.f.a.get(e0.getType());
            if (!z && nVar2 == null) {
                throw new AssertionError();
            }
            boolean zIsEmpty = zA & nVar2.f.isEmpty();
            n nVar3 = (n) this.f.a.get(e0.getType());
            if (!z && nVar3 == null) {
                throw new AssertionError();
            }
            Iterator it = nVar3.e.values().iterator();
            while (it.hasNext()) {
                zIsEmpty &= ((m.c) it.next()).g;
            }
            this.f.a(e0.e, new l(zIsEmpty, arrayList, arrayList2));
        }
    }

    public final void b() {
        this.f.b(new BiConsumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.o$$ExternalSyntheticLambda2
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.a((E0) obj, (C0291j1) obj2);
            }
        });
    }

    public static boolean a(Iterable iterable, Collection collection, ArrayList arrayList) {
        Iterator it = iterable.iterator();
        boolean z = true;
        while (it.hasNext()) {
            final AbstractC0264h1 abstractC0264h1 = (AbstractC0264h1) it.next();
            if (abstractC0264h1.getAccessFlags().m() || abstractC0264h1.getAccessFlags().l()) {
                if (collection.stream().noneMatch(new Predicate() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.o$$ExternalSyntheticLambda3
                    @Override // java.util.function.Predicate
                    public final boolean test(Object obj) {
                        return o.a(abstractC0264h1, (AbstractC0264h1) obj);
                    }
                })) {
                    arrayList.add(abstractC0264h1.getReference());
                    z = false;
                }
            }
        }
        return z;
    }

    public static /* synthetic */ boolean a(AbstractC0264h1 abstractC0264h1, AbstractC0264h1 abstractC0264h12) {
        return abstractC0264h12.getReference() == abstractC0264h1.getReference();
    }

    public final void a(t0 t0Var) throws IOException {
        C3030tM c3030tMA;
        List listA;
        if (this.f.a.isEmpty()) {
            return;
        }
        for (int iD = EnumC3471y2.r.d(); iD <= a.g.d(); iD++) {
            EnumC3471y2 enumC3471y2B = EnumC3471y2.b(iD);
            if (t0Var == null) {
                c3030tMA = C3030tM.g();
            } else {
                C3200vB c3200vB = this.c;
                c3030tMA = C1064Vk.a(t0Var, c3200vB.a, c3200vB.i, false, enumC3471y2B.d()).a(this.d, Fh0.a());
            }
            this.c.c(enumC3471y2B);
            C3200vB c3200vB2 = this.c;
            c3200vB2.J1 = null;
            c3200vB2.K1 = C3030tM.g();
            this.c.a(c3030tMA);
            C0262h c0262hA = C0262h.a(this.d, E.d());
            final C0421y c0421y = new C0421y(c0262hA, AbstractC1784g4.a(c0262hA, c0262hA.j()), new C1653en(), 2, this.c.O(), Fh0.a());
            final C0285j c0285jH = c0421y.h();
            if (!this.g) {
                listA = C2917s5.a(this.d, this.c);
            } else {
                int i = AbstractC0695Iu.c;
                listA = S40.e;
            }
            final List list = listA;
            final C3030tM c3030tM = c3030tMA;
            final int i2 = iD;
            this.f.b(new BiConsumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.o$$ExternalSyntheticLambda5
                @Override // java.util.function.BiConsumer
                public final void accept(Object obj, Object obj2) {
                    this.f$0.a(c3030tM, list, c0285jH, c0421y, i2, (E0) obj, (C0291j1) obj2);
                }
            });
            final int i3 = iD;
            this.f.a(new BiConsumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.o$$ExternalSyntheticLambda6
                @Override // java.util.function.BiConsumer
                public final void accept(Object obj, Object obj2) {
                    this.f$0.a(c3030tM, c0285jH, c0421y, i3, (E0) obj, (C0257g1) obj2);
                }
            });
        }
    }

    public final /* synthetic */ void a(C3030tM c3030tM, List list, C0285j c0285j, C0421y c0421y, int i, E0 e0, C0291j1 c0291j1) {
        C0409w2 reference = c0291j1.getReference();
        if (!c3030tM.b(reference) && !list.contains(reference) && !c3030tM.i().containsKey(reference)) {
            if (c3030tM.k().containsKey(reference.w0()) && c0291j1.z0()) {
                return;
            }
            S4 s4A = c0285j.a(reference, c0285j.a(reference.w0()).isInterface());
            C0409w2 reference2 = s4A.w() ? s4A.q().getReference() : reference;
            com.android.tools.r8.androidapi.a aVarE = c0421y.e();
            int i2 = com.android.tools.r8.androidapi.f.a;
            com.android.tools.r8.androidapi.h hVar = com.android.tools.r8.androidapi.h.b;
            com.android.tools.r8.androidapi.f fVarA = aVarE.a((AbstractC0368r2) reference2);
            if (!fVarA.v()) {
                if (!s4A.w()) {
                    this.f.a(reference, m.c.a(i));
                    return;
                }
                throw new RuntimeException("API database does not recognize the method " + c0291j1.getReference().m0());
            }
            if (i < fVarA.Y().a().d()) {
                this.f.a(reference, m.c.a(i));
                return;
            }
            return;
        }
        if (c3030tM.i().containsKey(reference)) {
            this.f.a(reference, m.c.h);
        }
    }

    public final /* synthetic */ void a(C3030tM c3030tM, C0285j c0285j, C0421y c0421y, int i, E0 e0, C0257g1 c0257g1) {
        if (c3030tM.a((F2) c0257g1.F0()) || c3030tM.o().containsKey(c0257g1.getReference())) {
            return;
        }
        AbstractC0410w3 abstractC0410w3C = c0285j.c(c0257g1.getReference());
        if (abstractC0410w3C.y()) {
            C0257g1 c0257g1Q = abstractC0410w3C.q();
            com.android.tools.r8.androidapi.a aVarE = c0421y.e();
            C0309l1 reference = c0257g1Q.getReference();
            int i2 = com.android.tools.r8.androidapi.f.a;
            com.android.tools.r8.androidapi.h hVar = com.android.tools.r8.androidapi.h.b;
            com.android.tools.r8.androidapi.f fVarA = aVarE.a((AbstractC0368r2) reference);
            if (fVarA.v()) {
                if (i < fVarA.Y().a().d()) {
                    this.f.a(c0257g1.getReference(), m.a.a(i));
                    return;
                }
                return;
            }
            throw new RuntimeException("API database does not recognize the field " + c0257g1.getReference().m0());
        }
        if (!j && !abstractC0410w3C.h()) {
            throw new AssertionError();
        }
        this.f.a(c0257g1.getReference(), m.a.a(i));
    }

    public final /* synthetic */ void a(E0 e0, C0291j1 c0291j1) {
        E0 e0D = this.d.d(e0.e);
        if (!j && e0D == null) {
            throw new AssertionError();
        }
        if (e0D.c(c0291j1.getReference()) == null) {
            this.f.a(c0291j1.getReference(), m.c.k);
        }
    }

    public final void a(Collection collection, t0 t0Var) throws IOException {
        C3030tM c3030tMA;
        final List<C0409w2> listA;
        EnumC3471y2 enumC3471y2 = this.e;
        if (t0Var == null) {
            c3030tMA = C3030tM.g();
        } else {
            C3200vB c3200vB = this.c;
            c3030tMA = C1064Vk.a(t0Var, c3200vB.a, c3200vB.i, false, enumC3471y2.d()).a(this.d, Fh0.a());
        }
        this.c.c(this.e);
        C3200vB c3200vB2 = this.c;
        c3200vB2.J1 = null;
        c3200vB2.K1 = C3030tM.g();
        this.c.a(c3030tMA);
        i.a aVarB = com.android.tools.r8.utils.i.b();
        Iterator it = collection.iterator();
        while (it.hasNext()) {
            aVarB.a((ProgramResourceProvider) it.next());
        }
        final C0203a3 c0203a3I = new C0034c(aVarB.a(), this.c, Fh0.a()).a().i();
        if (this.g) {
            int i = AbstractC0695Iu.c;
            listA = S40.e;
        } else {
            listA = C2917s5.a(this.d, this.c);
        }
        for (D2 d2 : c0203a3I.d()) {
            if (c3030tMA.c.g().containsKey(d2.e)) {
                if (!j && !d2.isInterface()) {
                    throw new AssertionError();
                }
                for (C0291j1 c0291j1 : d2.C1()) {
                    if (c0291j1.o1() || c0291j1.z0()) {
                        if (!c0291j1.G0().c("lambda$") && !c0291j1.G0().toString().contains("$deserializeLambda$") && !c0291j1.getReference().m0().equals("void java.util.Collection.forEach(java.util.function.Consumer)")) {
                            this.f.a(d2, c0291j1);
                        }
                    }
                }
                for (C0409w2 c0409w2 : listA) {
                    if (d2.e == c0409w2.w0()) {
                        this.f.a(d2, a(this.d.d(d2.e), c0409w2));
                    }
                }
                this.f.a(d2.e, l.e);
            } else {
                if ((d2.f.m() || d2.f.l()) && c3030tMA.a((F2) d2.e) && this.d.d(d2.e) != null) {
                    for (C0291j1 c0291j12 : d2.C1()) {
                        if (!c0291j12.L0()) {
                            c0291j12.P0();
                            if (!c0291j12.g.l()) {
                            }
                        }
                        this.f.a(d2, c0291j12);
                    }
                    for (C0257g1 c0257g1 : d2.M0()) {
                        if (c0257g1.L0() || c0257g1.g.l()) {
                            this.f.a(d2, c0257g1);
                        }
                    }
                }
                for (C0409w2 c0409w22 : listA) {
                    if (d2.e == c0409w22.w0()) {
                        this.f.a(d2, a(this.d.d(d2.e), c0409w22));
                    }
                }
            }
        }
        c3030tMA.c.a(new Consumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.o$$ExternalSyntheticLambda0
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.a(c0203a3I, listA, (C0409w2) obj);
            }
        });
        c3030tMA.c.o().forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.o$$ExternalSyntheticLambda1
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.a(c0203a3I, (C0309l1) obj, (C0309l1) obj2);
            }
        });
        if (this.h) {
            ArrayList arrayList = new ArrayList();
            for (C0409w2 c0409w23 : listA) {
                if (c0203a3I.d(c0409w23.w0()) == null) {
                    arrayList.add(c0409w23);
                }
            }
            arrayList.sort(Comparator.naturalOrder());
            this.f.b = arrayList;
        }
    }

    public final void a(C0203a3 c0203a3, C0309l1 c0309l1, C0309l1 c0309l12) {
        C0257g1 c0257g1A;
        E0 e0D = c0203a3.d(c0309l1.w0());
        if (e0D != null && (c0257g1A = e0D.a(c0309l1)) != null) {
            this.f.a(e0D, c0257g1A);
            this.f.a(e0D.e, l.e);
            return;
        }
        E0 e0D2 = this.d.d(c0309l1.w0());
        C0257g1 c0257g1A2 = e0D2.a(c0309l1);
        if (!j && c0257g1A2 == null) {
            throw new AssertionError();
        }
        this.f.a(e0D2, c0257g1A2);
        this.f.a(e0D2.e, l.e);
    }

    public final void a(C0203a3 c0203a3, List list, C0409w2 c0409w2) {
        C0291j1 c0291j1C;
        E0 e0D = c0203a3.d(c0409w2.w0());
        if (e0D != null && (c0291j1C = e0D.c(c0409w2)) != null) {
            this.f.a(e0D, c0291j1C);
            this.f.a(e0D.e, l.e);
            return;
        }
        E0 e0D2 = this.d.d(c0409w2.w0());
        C0291j1 c0291j1A = a(e0D2, c0409w2);
        if (c0291j1A != null) {
            this.f.a(e0D2, c0291j1A);
            this.f.a(e0D2.getType(), l.e);
            list.remove(c0409w2);
        }
    }

    public final C0291j1 a(E0 e0, C0409w2 c0409w2) {
        if (e0 != null) {
            C0291j1 c0291j1C = e0.c(c0409w2);
            if (c0291j1C == null) {
                if (!this.i && !this.b.contains(c0409w2) && !this.a.contains(c0409w2.w0())) {
                    this.c.i.a("Backport missing from library: " + c0409w2);
                }
                c0291j1C = C0291j1.O0().a(c0409w2).a(E4.b(9, false)).a();
            }
            if (j || c0291j1C != null) {
                return c0291j1C;
            }
            throw new AssertionError();
        }
        throw new Error("Missing class from Android " + a.g + ": " + c0409w2.w0());
    }
}
