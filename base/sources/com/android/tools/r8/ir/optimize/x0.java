package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.AbstractC0029c;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2125ji0;
import com.android.tools.r8.internal.AbstractC2972si;
import com.android.tools.r8.internal.C1411cA;
import com.android.tools.r8.internal.C1850gi0;
import com.android.tools.r8.internal.C2031ii0;
import com.android.tools.r8.internal.C2784qi;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.S40;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Function;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class x0 extends AbstractC2972si {
    public static final /* synthetic */ boolean i = true;
    public final /* synthetic */ C1411cA f;
    public final /* synthetic */ y0 g;
    public final /* synthetic */ D0 h;

    public x0(C1411cA c1411cA, y0 y0Var, D0 d0) {
        this.f = c1411cA;
        this.g = y0Var;
        this.h = d0;
    }

    @Override // com.android.tools.r8.internal.AbstractC2972si
    public final C2031ii0 a(C2784qi c2784qi, List list) {
        B0 b0 = (B0) c2784qi.d;
        if (!A0.a(b0.a)) {
            boolean z = i;
            if (!z) {
                int i2 = b0.a;
                if (i2 == 0) {
                    throw null;
                }
                if (i2 != 2 && i2 != 3) {
                    throw new AssertionError();
                }
            }
            if (!z && !list.isEmpty()) {
                throw new AssertionError();
            }
        } else {
            if (!B0.c && !A0.a(b0.a)) {
                throw new AssertionError();
            }
            int i3 = list.isEmpty() ? 3 : 4;
            Iterator it = list.iterator();
            while (it.hasNext()) {
                B0 b02 = (B0) ((C2784qi) it.next()).d;
                if (!B0.c && A0.a(b02.a)) {
                    throw new AssertionError();
                }
                int i4 = b02.a;
                if (!A0.b(i3) && !A0.a(i4)) {
                    if (A0.a(i3) || A0.b(i4)) {
                        i3 = i4;
                    } else if (i3 != i4) {
                        i3 = 1;
                    }
                }
            }
            if (!B0.c && A0.a(i3)) {
                throw new AssertionError();
            }
            ArrayList arrayList = new ArrayList();
            if (A0.b(i3)) {
                Iterator it2 = list.iterator();
                while (it2.hasNext()) {
                    C2784qi c2784qi2 = (C2784qi) it2.next();
                    B0 b03 = (B0) c2784qi2.d;
                    if (b03.a == 3) {
                        arrayList.add((H5) c2784qi2.a);
                    } else if (A0.b(b03.a)) {
                        arrayList.addAll(((B0) c2784qi2.d).b);
                    }
                }
            }
            b0 = new B0(i3, arrayList);
        }
        c2784qi.d = b0;
        return new C2031ii0(b0);
    }

    @Override // com.android.tools.r8.internal.AbstractC2972si
    public final AbstractC2125ji0 a(C2784qi c2784qi, Function function) {
        int i2;
        C2784qi c2784qi2 = c2784qi;
        Iterator<AbstractC1076Vw> it = ((H5) c2784qi2.a).k().iterator();
        int i3 = 1;
        while (true) {
            i2 = 2;
            if (!it.hasNext()) {
                break;
            }
            AbstractC1076Vw next = it.next();
            if (this.f.b() > this.g.a()) {
                return C1850gi0.c;
            }
            int iA = this.g.a(next);
            if (iA == 1) {
                i3 = iA;
            } else {
                if (iA == 2) {
                    this.h.a.add(next);
                }
                i3 = iA;
            }
        }
        if (i3 == 1) {
            Iterator it2 = this.g.a((H5) c2784qi2.a).iterator();
            while (it2.hasNext()) {
                if (((C2784qi) function.apply((H5) it2.next())).d != null) {
                    return C1850gi0.c;
                }
            }
        }
        int iB = AbstractC0029c.b(i3);
        if (iB == 0) {
            i2 = 4;
        } else if (iB != 1) {
            i2 = 3;
            if (!z0.a && i3 != 3) {
                throw new AssertionError();
            }
        }
        int i4 = AbstractC0695Iu.c;
        c2784qi.d = new B0(i2, S40.e);
        return C2031ii0.c;
    }
}
