package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2346lp;
import com.android.tools.r8.internal.C0956Rv;
import com.android.tools.r8.internal.C3147ue0;
import com.android.tools.r8.internal.C3654zw;
import com.android.tools.r8.internal.H5;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class k0 {
    public static final /* synthetic */ boolean c = true;
    public final LinkedHashMap a = new LinkedHashMap();
    public int b = 10000;

    public final C3685j0 a(H5 h5, int i) {
        AbstractC1076Vw abstractC1076VwG;
        LinkedHashMap linkedHashMap;
        LinkedHashMap linkedHashMap2;
        LinkedHashSet linkedHashSet;
        LinkedHashMap linkedHashMap3;
        LinkedHashMap linkedHashMap4;
        LinkedHashMap linkedHashMap5;
        if (h5.s().isEmpty()) {
            return new C3685j0(i);
        }
        List<H5> listS = h5.s();
        Iterator<H5> it = listS.iterator();
        C3685j0 c3685j0 = new C3685j0(i, (C3685j0) this.a.get(it.next()));
        while (it.hasNext()) {
            C3685j0 c3685j02 = (C3685j0) this.a.get(it.next());
            if (c3685j02 == null) {
                return new C3685j0(i);
            }
            LinkedHashMap linkedHashMap6 = c3685j0.a;
            if (linkedHashMap6 == null || (linkedHashMap5 = c3685j02.a) == null) {
                c3685j0.a = null;
            } else {
                C3685j0.a(linkedHashMap6, linkedHashMap5);
            }
            LinkedHashMap linkedHashMap7 = c3685j0.b;
            if (linkedHashMap7 == null || (linkedHashMap4 = c3685j02.b) == null) {
                c3685j0.b = null;
            } else {
                C3685j0.a(linkedHashMap7, linkedHashMap4);
            }
            LinkedHashMap linkedHashMap8 = c3685j0.c;
            if (linkedHashMap8 == null || (linkedHashMap3 = c3685j02.c) == null) {
                c3685j0.c = null;
            } else {
                C3685j0.a(linkedHashMap8, linkedHashMap3);
            }
            LinkedHashSet linkedHashSet2 = c3685j0.d;
            if (linkedHashSet2 == null || (linkedHashSet = c3685j02.d) == null) {
                c3685j0.d = null;
            } else {
                C3685j0.a(linkedHashSet2, linkedHashSet);
            }
            LinkedHashMap linkedHashMap9 = c3685j0.e;
            if (linkedHashMap9 == null || (linkedHashMap2 = c3685j02.e) == null) {
                c3685j0.e = null;
            } else {
                C3685j0.a(linkedHashMap9, linkedHashMap2);
            }
            LinkedHashMap linkedHashMap10 = c3685j0.f;
            if (linkedHashMap10 == null || (linkedHashMap = c3685j02.f) == null) {
                c3685j0.f = null;
            } else {
                C3685j0.a(linkedHashMap10, linkedHashMap);
            }
            boolean z = C3685j0.k;
            if (!z && c3685j0.g != null) {
                throw new AssertionError();
            }
            if (!z && c3685j0.h != null) {
                throw new AssertionError();
            }
            if (!z && c3685j0.i != null) {
                throw new AssertionError();
            }
        }
        for (H5 h52 : listS) {
            if (h52.c(h5) && (abstractC1076VwG = h52.g()) != null) {
                if (abstractC1076VwG.J1()) {
                    AbstractC2346lp abstractC2346lpQ = abstractC1076VwG.Q();
                    C0309l1 field = abstractC2346lpQ.getField();
                    if (abstractC2346lpQ instanceof C3654zw) {
                        l0 l0Var = new l0(field, abstractC2346lpQ.c().h().i());
                        LinkedHashMap linkedHashMap11 = c3685j0.b;
                        if (linkedHashMap11 != null) {
                            linkedHashMap11.remove(l0Var);
                        }
                        LinkedHashMap linkedHashMap12 = c3685j0.e;
                        if (linkedHashMap12 != null) {
                            linkedHashMap12.remove(l0Var);
                        }
                        LinkedHashMap linkedHashMap13 = c3685j0.h;
                        if (linkedHashMap13 != null) {
                            linkedHashMap13.remove(l0Var);
                        }
                    } else if (abstractC2346lpQ instanceof C3147ue0) {
                        LinkedHashMap linkedHashMap14 = c3685j0.c;
                        if (linkedHashMap14 != null) {
                            linkedHashMap14.remove(field);
                        }
                        LinkedHashMap linkedHashMap15 = c3685j0.f;
                        if (linkedHashMap15 != null) {
                            linkedHashMap15.remove(field);
                        }
                        LinkedHashMap linkedHashMap16 = c3685j0.i;
                        if (linkedHashMap16 != null) {
                            linkedHashMap16.remove(field);
                        }
                    }
                } else if (abstractC1076VwG.N1()) {
                    C0956Rv c0956RvU = abstractC1076VwG.U();
                    LinkedHashSet linkedHashSet3 = c3685j0.d;
                    if (linkedHashSet3 != null) {
                        linkedHashSet3.remove(c0956RvU.i);
                    }
                }
            }
        }
        return c3685j0;
    }

    /* JADX WARN: Code restructure failed: missing block: B:46:0x0008, code lost:
    
        continue;
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void a(com.android.tools.r8.internal.H5 r5, com.android.tools.r8.internal.C3098u30 r6) {
        /*
            r4 = this;
            java.util.List r0 = r5.s()
            java.util.Iterator r0 = r0.iterator()
        L8:
            boolean r1 = r0.hasNext()
            if (r1 == 0) goto L89
            java.lang.Object r1 = r0.next()
            com.android.tools.r8.internal.H5 r1 = (com.android.tools.r8.internal.H5) r1
            boolean r2 = r1.C()
            if (r2 == 0) goto L3b
            java.util.LinkedHashMap r2 = r4.a
            java.lang.Object r1 = r2.remove(r1)
            com.android.tools.r8.ir.optimize.j0 r1 = (com.android.tools.r8.ir.optimize.C3685j0) r1
            if (r1 == 0) goto L8
            int r1 = r1.d()
            boolean r2 = com.android.tools.r8.ir.optimize.k0.c
            if (r2 != 0) goto L35
            if (r1 <= 0) goto L2f
            goto L35
        L2f:
            java.lang.AssertionError r5 = new java.lang.AssertionError
            r5.<init>()
            throw r5
        L35:
            int r2 = r4.b
            int r2 = r2 + r1
            r4.b = r2
            goto L8
        L3b:
            java.util.ArrayList r2 = r1.b
            int r2 = r2.size()
            int r3 = r1.L()
            int r2 = r2 - r3
        L46:
            java.util.ArrayList r3 = r1.b
            int r3 = r3.size()
            if (r2 >= r3) goto L8
            java.util.ArrayList r3 = r1.b
            java.lang.Object r3 = r3.get(r2)
            if (r3 != r5) goto L86
            int r2 = r6.b(r1)
            int r2 = r2 + (-1)
            if (r2 != 0) goto L82
            r6.c(r1)
            java.util.LinkedHashMap r2 = r4.a
            java.lang.Object r1 = r2.remove(r1)
            com.android.tools.r8.ir.optimize.j0 r1 = (com.android.tools.r8.ir.optimize.C3685j0) r1
            if (r1 == 0) goto L8
            int r1 = r1.d()
            boolean r2 = com.android.tools.r8.ir.optimize.k0.c
            if (r2 != 0) goto L7c
            if (r1 <= 0) goto L76
            goto L7c
        L76:
            java.lang.AssertionError r5 = new java.lang.AssertionError
            r5.<init>()
            throw r5
        L7c:
            int r2 = r4.b
            int r2 = r2 + r1
            r4.b = r2
            goto L8
        L82:
            r6.b(r2, r1)
            goto L8
        L86:
            int r2 = r2 + 1
            goto L46
        L89:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.k0.a(com.android.tools.r8.internal.H5, com.android.tools.r8.internal.u30):void");
    }

    /* JADX WARN: Code restructure failed: missing block: B:57:0x00ca, code lost:
    
        if (r1 <= 0) goto L59;
     */
    /* JADX WARN: Code restructure failed: missing block: B:58:0x00cc, code lost:
    
        r8.a(r1);
     */
    /* JADX WARN: Code restructure failed: missing block: B:60:0x00d1, code lost:
    
        if (com.android.tools.r8.ir.optimize.k0.c != false) goto L76;
     */
    /* JADX WARN: Code restructure failed: missing block: B:61:0x00d3, code lost:
    
        r0 = r6.b;
        r1 = r6.a.values().iterator();
     */
    /* JADX WARN: Code restructure failed: missing block: B:63:0x00e3, code lost:
    
        if (r1.hasNext() == false) goto L92;
     */
    /* JADX WARN: Code restructure failed: missing block: B:64:0x00e5, code lost:
    
        r2 = ((com.android.tools.r8.ir.optimize.C3685j0) r1.next()).d();
     */
    /* JADX WARN: Code restructure failed: missing block: B:65:0x00f1, code lost:
    
        if (com.android.tools.r8.ir.optimize.k0.c != false) goto L93;
     */
    /* JADX WARN: Code restructure failed: missing block: B:66:0x00f3, code lost:
    
        if (r2 <= 0) goto L91;
     */
    /* JADX WARN: Code restructure failed: missing block: B:69:0x00fb, code lost:
    
        throw new java.lang.AssertionError();
     */
    /* JADX WARN: Code restructure failed: missing block: B:70:0x00fc, code lost:
    
        r3 = r3 + r2;
     */
    /* JADX WARN: Code restructure failed: missing block: B:72:0x0100, code lost:
    
        if (r0 != (10000 - r3)) goto L74;
     */
    /* JADX WARN: Code restructure failed: missing block: B:75:0x0108, code lost:
    
        throw new java.lang.AssertionError();
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final void a(com.android.tools.r8.internal.H5 r7, com.android.tools.r8.ir.optimize.C3685j0 r8) {
        /*
            Method dump skipped, instruction units count: 293
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.k0.a(com.android.tools.r8.internal.H5, com.android.tools.r8.ir.optimize.j0):void");
    }
}
