package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3337we0;
import com.android.tools.r8.internal.C5;
import com.android.tools.r8.internal.FL;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.HW;
import com.android.tools.r8.internal.J5;
import com.android.tools.r8.internal.SW;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.f0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class AbstractC3672f0 {
    public static boolean a(C0874Ot c0874Ot) {
        boolean z;
        boolean z2;
        boolean z3 = false;
        for (H5 h5 : c0874Ot.d) {
            HashSet hashSet = new HashSet(h5.s());
            for (boolean z4 = true; z4; z4 = z) {
                z = false;
                for (SW sw : h5.q()) {
                    Iterator<H5> it = h5.s().iterator();
                    while (true) {
                        if (!it.hasNext()) {
                            if (sw.U() != 1 || sw.W() != 1) {
                                break;
                            }
                            Iterator<C3161ul0> it2 = sw.c0().iterator();
                            while (true) {
                                if (it2.hasNext()) {
                                    if (it2.next().U() != 1) {
                                        break;
                                    }
                                } else {
                                    FL flK0 = sw.Z().k0();
                                    if (flK0 == null || ((C3161ul0) flK0.c.get(0)).y()) {
                                        break;
                                    }
                                    HashSet hashSet2 = new HashSet();
                                    int iB = 0;
                                    H5 h5A = h5;
                                    while (h5A != null) {
                                        hashSet2.add(h5A);
                                        Iterator<AbstractC1076Vw> it3 = h5A.k().iterator();
                                        while (it3.hasNext()) {
                                            if (flK0 == it3.next()) {
                                                break;
                                            }
                                            int iA = iB - HW.a(flK0);
                                            if (iA < 0) {
                                                break;
                                            }
                                            iB = iA + HW.b(flK0);
                                        }
                                        h5A = (!h5A.h().L1() || hashSet2.contains(h5A.h().S().L2())) ? null : C5.a(h5A);
                                    }
                                    iB = Integer.MIN_VALUE;
                                    if (iB != 0) {
                                        break;
                                    }
                                    for (C3161ul0 c3161ul0 : sw.c0()) {
                                        AbstractC1076Vw abstractC1076Vw = c3161ul0.c;
                                        if (abstractC1076Vw != null && (abstractC1076Vw instanceof C3337we0) && hashSet.contains(abstractC1076Vw.b())) {
                                            AbstractC1076Vw abstractC1076Vw2 = c3161ul0.c;
                                            H5 h5B = abstractC1076Vw2.b();
                                            J5 j5 = new J5(h5B, h5B.k().size() - 1);
                                            int iA2 = 0;
                                            while (true) {
                                                if (!j5.b.hasPrevious() || ((AbstractC1076Vw) j5.b.previous()) == abstractC1076Vw2) {
                                                    break;
                                                }
                                                int iB2 = iA2 - HW.b(abstractC1076Vw2);
                                                if (iB2 < 0) {
                                                    iA2 = Integer.MIN_VALUE;
                                                    break;
                                                }
                                                iA2 = iB2 + HW.a(abstractC1076Vw2);
                                            }
                                            if (iA2 != 0) {
                                            }
                                        }
                                    }
                                    ArrayList arrayList = new ArrayList();
                                    Iterator<C3161ul0> it4 = sw.c0().iterator();
                                    while (it4.hasNext()) {
                                        arrayList.add(it4.next().c.K0());
                                    }
                                    for (int i = 0; i < arrayList.size(); i++) {
                                        AbstractC1076Vw abstractC1076Vw3 = (C3337we0) arrayList.get(i);
                                        sw.a(i, (C3161ul0) abstractC1076Vw3.c.get(0), (C3658a) null);
                                        C3161ul0 c3161ul02 = (C3161ul0) abstractC1076Vw3.c.get(0);
                                        c3161ul02.d.remove(abstractC1076Vw3);
                                        c3161ul02.e = null;
                                        abstractC1076Vw3.b().e(abstractC1076Vw3);
                                    }
                                    flK0.d().f(sw);
                                    C3161ul0 c3161ul03 = (C3161ul0) flK0.c.get(0);
                                    c3161ul03.d.remove(flK0);
                                    c3161ul03.e = null;
                                    flK0.b().e(flK0);
                                    sw.u = true;
                                    z2 = true;
                                }
                            }
                        } else {
                            H5 next = it.next();
                            if (!next.h().L1() || C5.a(next) != h5) {
                                break;
                            }
                        }
                    }
                    z2 = false;
                    z |= z2;
                }
                if (z) {
                    z3 = true;
                }
            }
        }
        return z3;
    }
}
