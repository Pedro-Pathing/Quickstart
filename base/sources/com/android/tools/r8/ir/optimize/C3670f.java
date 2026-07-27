package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.AssertionsConfiguration;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC2346lp;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C1938hh0;
import com.android.tools.r8.internal.C2435mm;
import com.android.tools.r8.internal.C2445mu;
import com.android.tools.r8.internal.C2937sJ;
import com.android.tools.r8.internal.C3147ue0;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.C3200vB;
import com.android.tools.r8.internal.C3381x4;
import com.android.tools.r8.internal.Ef0$$ExternalSyntheticLambda0;
import com.android.tools.r8.internal.Fh0;
import com.android.tools.r8.internal.H5;
import com.android.tools.r8.internal.Y6;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.f, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3670f {
    public static final /* synthetic */ boolean g = true;
    public final C0421y a;
    public final B1 b;
    public final C3666e c;
    public final List d;
    public final C3666e e;
    public final boolean f;

    public C3670f(final C0421y c0421y) {
        this.a = c0421y;
        B1 b1A = c0421y.a();
        this.b = b1A;
        boolean zA = a(c0421y.M());
        this.f = zA;
        if (zA) {
            this.c = new C3666e(c0421y.M().W0.a, b1A);
            this.d = (List) c0421y.M().W0.b.stream().map(new Function() { // from class: com.android.tools.r8.ir.optimize.f$$ExternalSyntheticLambda0
                @Override // java.util.function.Function
                public final Object apply(Object obj) {
                    return C3670f.a(c0421y, (AssertionsConfiguration) obj);
                }
            }).collect(Collectors.toList());
            this.e = a(c0421y.a().J4.e.a);
        } else {
            this.c = null;
            this.d = null;
            this.e = null;
        }
    }

    public static /* synthetic */ C3666e a(C0421y c0421y, AssertionsConfiguration assertionsConfiguration) {
        return new C3666e(assertionsConfiguration, c0421y.a());
    }

    public static boolean a(C3200vB c3200vB) {
        C3381x4 c3381x4 = c3200vB.W0;
        if (c3381x4 != null) {
            return !(c3381x4.b.size() == 0 ? c3381x4.a.isPassthrough() : c3381x4.b.size() == 1 && ((AssertionsConfiguration) c3381x4.b.get(0)).getScope() == AssertionsConfiguration.a.b && ((AssertionsConfiguration) c3381x4.b.get(0)).isPassthrough());
        }
        return false;
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Removed duplicated region for block: B:27:0x008a  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final boolean a(com.android.tools.r8.graph.C0291j1 r18, final com.android.tools.r8.internal.C0874Ot r19) {
        /*
            Method dump skipped, instruction units count: 781
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.C3670f.a(com.android.tools.r8.graph.j1, com.android.tools.r8.internal.Ot):boolean");
    }

    /* JADX WARN: Code restructure failed: missing block: B:100:0x0008, code lost:
    
        continue;
     */
    /* JADX WARN: Code restructure failed: missing block: B:38:0x0081, code lost:
    
        if (com.android.tools.r8.ir.optimize.C3670f.g != false) goto L100;
     */
    /* JADX WARN: Code restructure failed: missing block: B:40:0x0087, code lost:
    
        if (r3.a() == false) goto L85;
     */
    /* JADX WARN: Code restructure failed: missing block: B:43:0x0090, code lost:
    
        throw new java.lang.AssertionError();
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public final com.android.tools.r8.ir.optimize.C3666e a(com.android.tools.r8.graph.I2 r13) {
        /*
            Method dump skipped, instruction units count: 230
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.android.tools.r8.ir.optimize.C3670f.a(com.android.tools.r8.graph.I2):com.android.tools.r8.ir.optimize.e");
    }

    public final void a(C0291j1 c0291j1, C0874Ot c0874Ot, F f, Fh0 fh0) {
        if (this.f) {
            fh0.a("Rewrite assertions");
            boolean zA = a(c0291j1, c0874Ot);
            c0874Ot.y();
            if (zA) {
                f.a(c0874Ot, fh0);
            }
            if (!g && !c0874Ot.b(this.a)) {
                throw new AssertionError();
            }
            fh0.b();
        }
    }

    public static C2435mm a(C0874Ot c0874Ot) {
        return new C2435mm(c0874Ot, 1);
    }

    public final void a(Set set, C2937sJ c2937sJ, Map map, Map map2, final Map map3, H5 h5) {
        C2445mu c2445muA;
        if (set.contains(h5) || (c2445muA = a(h5)) == null) {
            return;
        }
        boolean zA = a(((C3161ul0) c2445muA.c.get(0)).m().I0());
        boolean z = !zA;
        if (!C2445mu.k && !c2445muA.P2()) {
            throw new AssertionError();
        }
        H5 h5B = c2445muA.b(Y6.a(z));
        C2435mm c2435mm = (C2435mm) c2937sJ.a(c2937sJ.b);
        c2435mm.getClass();
        List list = (List) c2435mm.a(h5B, new ArrayList());
        Iterator it = list.iterator();
        C1938hh0 c1938hh0 = null;
        C1938hh0 c1938hh0O0 = null;
        while (true) {
            if (!it.hasNext()) {
                c1938hh0 = c1938hh0O0;
                break;
            }
            H5 h52 = (H5) it.next();
            if (h52.h().v2()) {
                break;
            }
            if (h52.h().C2()) {
                if (c1938hh0O0 != null) {
                    break;
                } else {
                    c1938hh0O0 = h52.h().O0();
                }
            }
        }
        if (c1938hh0 != null) {
            map.put(c2445muA, Boolean.valueOf(z));
            if (!C2445mu.k && !c2445muA.P2()) {
                throw new AssertionError();
            }
            map2.put(c1938hh0, c2445muA.b(Y6.a(zA)));
            list.forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.f$$ExternalSyntheticLambda3
                @Override // java.util.function.Consumer
                public final void accept(Object obj) {
                    this.f$0.a(map3, (H5) obj);
                }
            });
            set.addAll(list);
        }
    }

    public final void a(Map map, H5 h5) {
        C2445mu c2445muA = a(h5);
        if (c2445muA != null) {
            map.put(c2445muA, Boolean.valueOf(!a(((C3161ul0) c2445muA.c.get(0)).m().I0())));
        }
    }

    public final boolean a(AbstractC2346lp abstractC2346lp) {
        return abstractC2346lp.getField().x0() == this.b.E0 && abstractC2346lp.getField().getType() == this.b.w1;
    }

    public final C2445mu a(H5 h5) {
        if (!h5.h().M1()) {
            return null;
        }
        C2445mu c2445muT = h5.h().T();
        if (!c2445muT.P2() || !((C3161ul0) c2445muT.c.get(0)).c(new Ef0$$ExternalSyntheticLambda0())) {
            return null;
        }
        C3147ue0 c3147ue0I0 = ((C3161ul0) c2445muT.c.get(0)).m().I0();
        if ((a(c3147ue0I0) || c3147ue0I0.getField() == this.b.J4.e.b) && c3147ue0I0.b.B() && !c3147ue0I0.b.A()) {
            return c2445muT;
        }
        return null;
    }
}
