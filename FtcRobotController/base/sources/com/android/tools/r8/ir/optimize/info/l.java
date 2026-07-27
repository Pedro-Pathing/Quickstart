package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.internal.AbstractC0548Dw;
import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.AbstractC0722Jn;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.AbstractC2670pV;
import com.android.tools.r8.internal.B1;
import com.android.tools.r8.internal.B7;
import com.android.tools.r8.internal.C2700pk0;
import com.android.tools.r8.internal.InterfaceC2591oc;
import com.android.tools.r8.internal.Jb0;
import com.android.tools.r8.internal.Nk0;
import com.android.tools.r8.internal.WB;
import java.util.BitSet;
import java.util.Set;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class l extends h {
    public static final /* synthetic */ int j = 0;
    public final B1 b;
    public final AbstractC0564Em c;
    public final boolean d;
    public final boolean e;
    public final BitSet f;
    public final BitSet g;
    public final int h;
    public final boolean i;

    public l(B1 b1, AbstractC0564Em abstractC0564Em, boolean z, boolean z2, BitSet bitSet, BitSet bitSet2, int i, boolean z3) {
        this.b = b1;
        this.c = abstractC0564Em;
        this.d = z;
        this.e = z2;
        this.f = bitSet;
        this.g = bitSet2;
        this.h = i;
        this.i = z3;
    }

    public static h a(final w wVar) {
        k kVar = new k();
        kVar.a = wVar.f;
        kVar.b = wVar.j;
        kVar.c = wVar.a(8);
        kVar.d = wVar.a(32);
        kVar.e = wVar.o;
        kVar.f = wVar.n;
        k kVarA = kVar.a(wVar.G(), new Consumer() { // from class: com.android.tools.r8.ir.optimize.info.l$$ExternalSyntheticLambda0
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                l.a(wVar, (k) obj);
            }
        });
        kVarA.h = wVar.a(16);
        return (kVarA.b.l() && kVarA.a.isUnknown() && kVarA.g < 0 && kVarA.e == null && kVarA.f == null && kVarA.c && !kVarA.d && !kVarA.h) ? C3682d.b : new l(kVarA.a, kVarA.b, kVarA.c, kVarA.d, kVarA.e, kVarA.f, kVarA.g, kVarA.h);
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean A() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC2670pV B() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean C() {
        return this.d;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean D() {
        return this.e;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean E() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean F() {
        return this.i;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean G() {
        return this.h >= 0;
    }

    @Override // com.android.tools.r8.ir.optimize.info.g
    public final g b() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean e() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean f() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean g() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final com.android.tools.r8.internal.r h() {
        return C2700pk0.a;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final B1 i() {
        return this.b;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC3679a j() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final B7 k() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final InterfaceC2591oc l() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC0548Dw m() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC0564Em n() {
        return this.c;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC0722Jn o() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final Set p() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final int q() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final BitSet r() {
        return this.f;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final BitSet s() {
        return this.g;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final int t() {
        return this.h;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final Jb0 u() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final BitSet v() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean w() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean y() {
        throw new Nk0();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean z() {
        throw new Nk0();
    }

    public static void a(h hVar, k kVar) {
        kVar.g = hVar.t();
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final boolean a(AbstractC1308bC abstractC1308bC) {
        return this.d;
    }

    @Override // com.android.tools.r8.ir.optimize.info.h
    public final AbstractC0548Dw a(WB wb) {
        throw new Nk0();
    }
}
