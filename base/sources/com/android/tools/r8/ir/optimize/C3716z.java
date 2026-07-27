package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.AbstractC0410w3;
import com.android.tools.r8.graph.C0236e3;
import com.android.tools.r8.graph.C0285j;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.C0425y3;
import com.android.tools.r8.graph.C0427y5;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2346lp;
import com.android.tools.r8.internal.AbstractC2584oX;
import com.android.tools.r8.internal.AbstractC2670pV;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C3010t7;
import com.android.tools.r8.internal.C3147ue0;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3201vC;
import com.android.tools.r8.internal.C3654zw;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.H5$$ExternalSyntheticLambda5;
import com.android.tools.r8.internal.InterfaceC1160Yw;
import com.android.tools.r8.internal.InterfaceC1187Zw;
import com.android.tools.r8.internal.InterfaceC2046ip;
import com.android.tools.r8.internal.JM;
import com.android.tools.r8.internal.L5;
import com.android.tools.r8.internal.LQ;
import com.android.tools.r8.internal.QT;
import com.android.tools.r8.internal.Vm0;
import com.android.tools.r8.internal.j3$$ExternalSyntheticLambda3;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.UnaryOperator;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.z, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3716z {
    public static final /* synthetic */ boolean g = true;
    public final C0421y a;
    public final C3010t7 b;
    public final A5 c;
    public final C0874Ot d;
    public AbstractC2670pV e = AbstractC2670pV.c;
    public Set f;

    /* JADX INFO: renamed from: $r8$lambda$wBZ5N9SAmxJLRX-vx8hIUeo_fgc, reason: not valid java name */
    public static /* synthetic */ ArrayList m111$r8$lambda$wBZ5N9SAmxJLRXvx8hIUeo_fgc() {
        return new ArrayList();
    }

    public C3716z(C0421y c0421y, A5 a5, C0874Ot c0874Ot) {
        if (!g && !c0421y.M().Z()) {
            throw new AssertionError();
        }
        this.a = c0421y;
        this.b = new C3010t7(c0421y);
        this.c = a5;
        this.d = c0874Ot;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public final boolean a(InterfaceC2046ip interfaceC2046ip, LQ lq) {
        if (this.e.e()) {
            this.e = AbstractC2670pV.a(d());
        }
        if (this.e.d()) {
            return false;
        }
        C0421y c0421yU = this.a.U();
        AbstractC0410w3.a<?> aVarL = ((C0285j) c0421yU.g()).c(interfaceC2046ip.getField()).l();
        if (aVarL == null || !(aVarL instanceof C0425y3)) {
            return false;
        }
        C0427y5 c0427y5R = aVarL.r();
        C0236e3 c0236e3W = c0427y5R.getAccessFlags();
        if (!g && c0236e3W.J()) {
            throw new AssertionError();
        }
        if (!c0427y5R.b(c0421yU)) {
            return false;
        }
        if ((!c0427y5R.getAccessFlags().f() && lq != null) || this.a.a(c0427y5R).d(this.a.M())) {
            return false;
        }
        if (this.c.e().q1()) {
            AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) interfaceC2046ip;
            if (this.c.getAccessFlags().n() == (abstractC1076Vw instanceof C3147ue0)) {
                if (this.c.a() == c0427y5R.a()) {
                    return false;
                }
                if ((abstractC1076Vw instanceof C3654zw) && ((C0285j) c0421yU.g()).c(this.c.a(), c0427y5R.a())) {
                    return false;
                }
            }
        }
        return aVarL.b.d(this.a);
    }

    public final Set b() {
        final Vm0 vm0 = new Vm0(2);
        this.d.d.forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.z$$ExternalSyntheticLambda2
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                C3716z.a(vm0, (H5) obj);
            }
        });
        while (vm0.b()) {
            vm0.b((Iterable) ((H5) vm0.d()).t());
        }
        return vm0.a();
    }

    public final Set c() {
        Set setC = AbstractC3424xb0.c();
        for (H5 h5 : b()) {
            for (C3654zw c3654zw : C3201vC.a(h5.k(), new j3$$ExternalSyntheticLambda3())) {
                C3161ul0 c3161ul0H = c3654zw.h();
                if (!c3161ul0H.c(new H5$$ExternalSyntheticLambda5()) && c3161ul0H.b().x()) {
                    setC.add(c3654zw);
                }
            }
        }
        return setC;
    }

    public final boolean d() {
        if (!this.a.g().h()) {
            return true;
        }
        C0421y c0421y = this.a;
        C0285j c0285jL = c0421y.g().h() ? c0421y.a.l() : null;
        Iterator it = this.d.b(new Predicate() { // from class: com.android.tools.r8.ir.optimize.z$$ExternalSyntheticLambda1
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return ((AbstractC1076Vw) obj).J1();
            }
        }).iterator();
        while (it.hasNext()) {
            AbstractC0410w3.a<?> aVarL = c0285jL.c(((AbstractC2346lp) it.next()).getField()).l();
            if (aVarL == null || aVarL.d.getAccessFlags().J()) {
                return true;
            }
        }
        return false;
    }

    public static void a(Vm0 vm0, H5 h5) {
        vm0.b((Iterable) h5.i().c);
    }

    /* JADX WARN: Removed duplicated region for block: B:100:0x023a  */
    /* JADX WARN: Removed duplicated region for block: B:107:0x0266  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void a() {
        /*
            Method dump skipped, instruction units count: 865
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.C3716z.a():void");
    }

    public static /* synthetic */ boolean a(QT qt) {
        return ((List) qt.getValue()).size() > 1;
    }

    public static void a(InterfaceC1187Zw interfaceC1187Zw, AbstractC1076Vw abstractC1076Vw, IdentityHashMap identityHashMap) {
        ((List) identityHashMap.computeIfAbsent(interfaceC1187Zw, JM.a(new z$$ExternalSyntheticLambda3()))).add(abstractC1076Vw);
    }

    public final InterfaceC1160Yw a(L5 l5, InterfaceC1160Yw interfaceC1160Yw, InterfaceC1187Zw interfaceC1187Zw, IdentityHashMap identityHashMap) {
        List listEmptyList = (List) identityHashMap.remove(interfaceC1187Zw);
        if (listEmptyList == null) {
            listEmptyList = Collections.emptyList();
        }
        if (listEmptyList.isEmpty()) {
            return interfaceC1160Yw;
        }
        Vm0 vm0 = new Vm0(2);
        vm0.b((Iterable) listEmptyList);
        while (vm0.b()) {
            AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) vm0.d();
            List listEmptyList2 = (List) identityHashMap.remove(abstractC1076Vw);
            if (listEmptyList2 == null) {
                listEmptyList2 = Collections.emptyList();
            }
            if (listEmptyList2.isEmpty()) {
                interfaceC1160Yw = a(l5, interfaceC1160Yw, interfaceC1187Zw, abstractC1076Vw);
            } else {
                vm0.b((Iterable) listEmptyList2);
                vm0.c(abstractC1076Vw);
            }
        }
        return interfaceC1160Yw;
    }

    /* JADX WARN: Code restructure failed: missing block: B:50:0x00c4, code lost:
    
        if (r0.b.b(r7.a.d(r1)) != null) goto L85;
     */
    /* JADX WARN: Removed duplicated region for block: B:35:0x006f  */
    /* JADX WARN: Removed duplicated region for block: B:64:0x00ff  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final boolean a(com.android.tools.r8.internal.AbstractC1076Vw r8) {
        /*
            Method dump skipped, instruction units count: 361
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.C3716z.a(com.android.tools.r8.internal.Vw):boolean");
    }

    public final InterfaceC1160Yw a(L5 l5, InterfaceC1160Yw interfaceC1160Yw, final InterfaceC1187Zw interfaceC1187Zw, AbstractC1076Vw abstractC1076Vw) {
        AbstractC2584oX position;
        boolean z = g;
        if (!z && interfaceC1187Zw.l() && interfaceC1160Yw.hasPrevious()) {
            throw new AssertionError();
        }
        if (!z && !interfaceC1187Zw.l() && interfaceC1160Yw.i() != interfaceC1187Zw) {
            throw new AssertionError();
        }
        if (interfaceC1187Zw.l()) {
            position = interfaceC1187Zw.b().r();
        } else {
            position = interfaceC1187Zw.n().getPosition();
        }
        if (abstractC1076Vw.i() && position.n()) {
            position = AbstractC2584oX.c.h;
        }
        abstractC1076Vw.b(position);
        if (abstractC1076Vw.i() && interfaceC1187Zw.b().x() && interfaceC1187Zw.b().a()) {
            H5 h5A = interfaceC1160Yw.a(this.d, l5, this.a.M(), new UnaryOperator() { // from class: com.android.tools.r8.ir.optimize.z$$ExternalSyntheticLambda0
                @Override // java.util.function.Function
                public final Object apply(Object obj) {
                    return interfaceC1187Zw.b();
                }
            });
            if (interfaceC1187Zw.l()) {
                if (!z && interfaceC1187Zw.b().k().size() != 1) {
                    throw new AssertionError();
                }
                interfaceC1160Yw.b(abstractC1076Vw);
                if (!z && interfaceC1160Yw.hasPrevious()) {
                    throw new AssertionError();
                }
            } else {
                if (interfaceC1187Zw.b().a()) {
                    if (!z && h5A.a()) {
                        throw new AssertionError();
                    }
                    h5A.a(this.d).add(abstractC1076Vw);
                } else {
                    if (!z && !h5A.a()) {
                        throw new AssertionError();
                    }
                    interfaceC1160Yw.b(abstractC1076Vw);
                }
                interfaceC1160Yw.a(interfaceC1187Zw.n());
            }
        } else {
            interfaceC1160Yw.d(abstractC1076Vw);
        }
        return interfaceC1160Yw;
    }
}
