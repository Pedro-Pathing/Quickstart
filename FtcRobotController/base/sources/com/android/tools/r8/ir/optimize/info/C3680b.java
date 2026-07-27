package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.AbstractC2627oz;
import com.android.tools.r8.internal.AbstractC2649pA;
import com.android.tools.r8.internal.B1;
import com.android.tools.r8.internal.C1105Wz;
import com.android.tools.r8.internal.C1284az;
import com.android.tools.r8.internal.Dk0;
import com.android.tools.r8.internal.InterfaceC1131Xx;
import com.android.tools.r8.internal.InterfaceC2450mz;
import com.android.tools.r8.internal.InterfaceC3508yU;
import com.android.tools.r8.internal.SR$$ExternalSyntheticLambda0;
import java.util.Collection;
import java.util.Objects;
import java.util.function.Predicate;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.info.b, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3680b extends AbstractC3679a {
    public static final /* synthetic */ boolean d = true;
    public final int a;
    public final InterfaceC2450mz b;
    public final InterfaceC2450mz c;

    public C3680b(int i, InterfaceC2450mz interfaceC2450mz, InterfaceC2450mz interfaceC2450mz2) {
        boolean z = d;
        if (!z && i <= 0) {
            throw new AssertionError();
        }
        if (!z && !interfaceC2450mz2.values().stream().noneMatch(new SR$$ExternalSyntheticLambda0())) {
            throw new AssertionError();
        }
        if (!z && !interfaceC2450mz.values().stream().noneMatch(new Predicate() { // from class: com.android.tools.r8.ir.optimize.info.b$$ExternalSyntheticLambda0
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return ((AbstractC0564Em) obj).l();
            }
        })) {
            throw new AssertionError();
        }
        this.a = i;
        this.b = (InterfaceC2450mz) Objects.requireNonNull(interfaceC2450mz);
        this.c = (InterfaceC2450mz) Objects.requireNonNull(interfaceC2450mz2);
    }

    @Override // com.android.tools.r8.ir.optimize.info.AbstractC3679a
    public final C3680b a() {
        return this;
    }

    @Override // com.android.tools.r8.ir.optimize.info.AbstractC3679a
    public final AbstractC0564Em b(int i) {
        if (d || (i >= 0 && i < this.a)) {
            return (AbstractC0564Em) this.b.getOrDefault(Integer.valueOf(i), AbstractC0564Em.m());
        }
        throw new AssertionError();
    }

    public final boolean equals(Object obj) {
        if (!(obj instanceof C3680b)) {
            return false;
        }
        C3680b c3680b = (C3680b) obj;
        return this.b.equals(c3680b.b) && this.c.equals(c3680b.c);
    }

    public final int hashCode() {
        return System.identityHashCode(this.c) + (System.identityHashCode(this.b) * 7);
    }

    public final String toString() {
        String str;
        String string = this.b.toString();
        if (this.c == null) {
            str = "";
        } else {
            str = System.lineSeparator() + this.c;
        }
        return string + str;
    }

    public final AbstractC3679a a(com.android.tools.r8.graph.proto.j jVar) {
        Collection collectionKeySet;
        if (jVar.g()) {
            return this;
        }
        com.android.tools.r8.graph.proto.c cVar = jVar.b;
        if (cVar.c()) {
            if (!jVar.e()) {
                return this;
            }
            return new C3680b(jVar.a.size() + this.a, this.b, this.c);
        }
        if (!d) {
            int iA = com.android.tools.r8.graph.proto.c.a(Integer.MAX_VALUE, cVar.a);
            if (iA == 0) {
                collectionKeySet = AbstractC2649pA.a;
            } else if (iA == cVar.a.size()) {
                collectionKeySet = cVar.a.keySet();
            } else {
                C1105Wz c1105Wz = new C1105Wz(iA);
                InterfaceC3508yU it = cVar.a.b().iterator();
                while (it.hasNext()) {
                    InterfaceC1131Xx interfaceC1131Xx = (InterfaceC1131Xx) it.next();
                    if (((com.android.tools.r8.graph.proto.b) interfaceC1131Xx.getValue()).c()) {
                        c1105Wz.add(interfaceC1131Xx.a());
                    }
                }
                collectionKeySet = c1105Wz;
            }
            if (!collectionKeySet.stream().allMatch(new Predicate() { // from class: com.android.tools.r8.ir.optimize.info.b$$ExternalSyntheticLambda1
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return this.f$0.a((Integer) obj);
                }
            })) {
                throw new AssertionError();
            }
        }
        int iA2 = this.a - com.android.tools.r8.graph.proto.c.a(Integer.MAX_VALUE, cVar.a);
        if (iA2 == 0) {
            return E.a;
        }
        InterfaceC2450mz c1284az = new C1284az(iA2);
        InterfaceC2450mz c1284az2 = new C1284az(iA2);
        int i = 0;
        for (int i2 = 0; i2 < this.a; i2++) {
            if (!cVar.a(i2).c()) {
                com.android.tools.r8.graph.proto.k kVarB = cVar.a(i2).b();
                if (kVarB == null || !kVarB.g().U0() || !kVarB.f().T0()) {
                    B1 b1 = (B1) this.c.getOrDefault(Integer.valueOf(i2), Dk0.a);
                    if (!b1.isUnknown()) {
                        c1284az.a(i, b1);
                    }
                    AbstractC0564Em abstractC0564Em = (AbstractC0564Em) this.b.get(i2);
                    if (abstractC0564Em != null) {
                        c1284az2.a(i, abstractC0564Em);
                    }
                }
                i++;
            }
        }
        int size = jVar.a.size() + iA2;
        if (c1284az.isEmpty() && c1284az2.isEmpty()) {
            return E.a;
        }
        if (c1284az2.isEmpty()) {
            c1284az2 = AbstractC2627oz.a;
        }
        if (c1284az.isEmpty()) {
            c1284az = AbstractC2627oz.a;
        }
        return new C3680b(size, c1284az2, c1284az);
    }

    public final /* synthetic */ boolean a(Integer num) {
        return num.intValue() < this.a;
    }

    @Override // com.android.tools.r8.ir.optimize.info.AbstractC3679a
    public final B1 a(int i) {
        if (d || (i >= 0 && i < this.a)) {
            return (B1) this.c.getOrDefault(Integer.valueOf(i), Dk0.a);
        }
        throw new AssertionError();
    }
}
