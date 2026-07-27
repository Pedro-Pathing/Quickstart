package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.internal.C0956Rv;
import com.android.tools.r8.internal.JX;
import com.android.tools.r8.internal.LN;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Predicate;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.j0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3685j0 {
    public static final /* synthetic */ boolean k = true;
    public LinkedHashMap a;
    public LinkedHashMap b;
    public LinkedHashMap c;
    public LinkedHashSet d;
    public LinkedHashMap e;
    public LinkedHashMap f;
    public C0956Rv g;
    public LinkedHashMap h;
    public LinkedHashMap i;
    public final int j;

    public C3685j0(int i) {
        this.j = i;
    }

    public static boolean a(LN ln, AbstractC3674g0 abstractC3674g0) {
        return abstractC3674g0.b == ln;
    }

    public final void b(final C0309l1 c0309l1) {
        LinkedHashMap linkedHashMap = this.e;
        if (linkedHashMap != null) {
            linkedHashMap.keySet().removeIf(new Predicate() { // from class: com.android.tools.r8.ir.optimize.j0$$ExternalSyntheticLambda3
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return C3685j0.b(c0309l1, (l0) obj);
                }
            });
        }
    }

    public final void c() {
        int iD = d();
        if (!k && iD > this.j) {
            throw new AssertionError();
        }
        if (iD == this.j) {
            a(1);
        }
    }

    public final int d() {
        int iB = b(this.c) + b(this.b) + b(this.a);
        LinkedHashSet linkedHashSet = this.d;
        return b(this.i) + b(this.h) + b(this.f) + b(this.e) + iB + (linkedHashSet != null ? linkedHashSet.size() : 0);
    }

    public static boolean a(LN ln, int i, AbstractC3674g0 abstractC3674g0) {
        return abstractC3674g0.b == ln && abstractC3674g0.a(i);
    }

    public C3685j0(int i, C3685j0 c3685j0) {
        this.j = i;
        if (c3685j0 != null) {
            LinkedHashMap linkedHashMap = c3685j0.a;
            if (linkedHashMap != null && !linkedHashMap.isEmpty()) {
                LinkedHashMap linkedHashMap2 = new LinkedHashMap();
                this.a = linkedHashMap2;
                linkedHashMap2.putAll(c3685j0.a);
            }
            LinkedHashMap linkedHashMap3 = c3685j0.b;
            if (linkedHashMap3 != null && !linkedHashMap3.isEmpty()) {
                LinkedHashMap linkedHashMap4 = new LinkedHashMap();
                this.b = linkedHashMap4;
                linkedHashMap4.putAll(c3685j0.b);
            }
            LinkedHashMap linkedHashMap5 = c3685j0.c;
            if (linkedHashMap5 != null && !linkedHashMap5.isEmpty()) {
                LinkedHashMap linkedHashMap6 = new LinkedHashMap();
                this.c = linkedHashMap6;
                linkedHashMap6.putAll(c3685j0.c);
            }
            LinkedHashSet linkedHashSet = c3685j0.d;
            if (linkedHashSet != null && !linkedHashSet.isEmpty()) {
                LinkedHashSet linkedHashSet2 = new LinkedHashSet();
                this.d = linkedHashSet2;
                linkedHashSet2.addAll(c3685j0.d);
            }
            LinkedHashMap linkedHashMap7 = c3685j0.e;
            if (linkedHashMap7 != null && !linkedHashMap7.isEmpty()) {
                LinkedHashMap linkedHashMap8 = new LinkedHashMap();
                this.e = linkedHashMap8;
                linkedHashMap8.putAll(c3685j0.e);
            }
            LinkedHashMap linkedHashMap9 = c3685j0.f;
            if (linkedHashMap9 != null && !linkedHashMap9.isEmpty()) {
                LinkedHashMap linkedHashMap10 = new LinkedHashMap();
                this.f = linkedHashMap10;
                linkedHashMap10.putAll(c3685j0.f);
            }
            this.g = c3685j0.g;
            LinkedHashMap linkedHashMap11 = c3685j0.h;
            if (linkedHashMap11 != null && !linkedHashMap11.isEmpty()) {
                LinkedHashMap linkedHashMap12 = new LinkedHashMap();
                this.h = linkedHashMap12;
                linkedHashMap12.putAll(c3685j0.h);
            }
            LinkedHashMap linkedHashMap13 = c3685j0.i;
            if (linkedHashMap13 == null || linkedHashMap13.isEmpty()) {
                return;
            }
            LinkedHashMap linkedHashMap14 = new LinkedHashMap();
            this.i = linkedHashMap14;
            linkedHashMap14.putAll(c3685j0.i);
        }
    }

    public static /* synthetic */ boolean b(C0309l1 c0309l1, l0 l0Var) {
        return l0Var.a == c0309l1;
    }

    public final void a(final C0309l1 c0309l1) {
        LinkedHashMap linkedHashMap = this.h;
        if (linkedHashMap != null) {
            linkedHashMap.keySet().removeIf(new Predicate() { // from class: com.android.tools.r8.ir.optimize.j0$$ExternalSyntheticLambda4
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return C3685j0.a(c0309l1, (l0) obj);
                }
            });
        }
    }

    public final void b(C0309l1 c0309l1, n0 n0Var) {
        LinkedHashMap linkedHashMap;
        c();
        if (!k && (linkedHashMap = this.f) != null && linkedHashMap.containsKey(c0309l1)) {
            throw new AssertionError();
        }
        if (this.f == null) {
            this.f = new LinkedHashMap();
        }
        this.f.put(c0309l1, n0Var);
    }

    public static /* synthetic */ boolean a(C0309l1 c0309l1, l0 l0Var) {
        return l0Var.a == c0309l1;
    }

    public final void a() {
        this.h = null;
        this.i = null;
    }

    public static void a(LinkedHashMap linkedHashMap, final LinkedHashMap linkedHashMap2) {
        linkedHashMap.entrySet().removeIf(new Predicate() { // from class: com.android.tools.r8.ir.optimize.j0$$ExternalSyntheticLambda0
            @Override // java.util.function.Predicate
            public final boolean test(Object obj) {
                return C3685j0.a(linkedHashMap2, (Map.Entry) obj);
            }
        });
    }

    public static /* synthetic */ boolean a(Map map, Map.Entry entry) {
        return map.get(entry.getKey()) != entry.getValue();
    }

    public final void b() {
        this.g = null;
    }

    public static void a(LinkedHashSet linkedHashSet, LinkedHashSet linkedHashSet2) {
        Objects.requireNonNull(linkedHashSet2);
        linkedHashSet.removeIf(JX.a(new j0$$ExternalSyntheticLambda1(linkedHashSet2)));
    }

    public static int b(Map map) {
        if (map != null) {
            return map.size();
        }
        return 0;
    }

    public static boolean a(Map map) {
        return map == null || map.isEmpty();
    }

    public final void a(int i) {
        boolean z = k;
        if (!z && i <= 0) {
            throw new AssertionError();
        }
        if (!z && i >= d()) {
            throw new AssertionError();
        }
        int iA = a(i, this.a);
        LinkedHashSet linkedHashSet = this.d;
        if (linkedHashSet != null && iA != 0) {
            Iterator it = linkedHashSet.iterator();
            while (it.hasNext() && iA > 0) {
                it.next();
                it.remove();
                iA--;
            }
        }
        int iA2 = a(a(a(a(a(a(iA, this.e), this.f), this.b), this.c), this.h), this.i);
        if (!z && iA2 != 0) {
            throw new AssertionError();
        }
    }

    public static int a(int i, Map map) {
        Set setKeySet = map != null ? map.keySet() : null;
        if (setKeySet != null && i != 0) {
            Iterator it = setKeySet.iterator();
            while (it.hasNext() && i > 0) {
                it.next();
                it.remove();
                i--;
            }
        }
        return i;
    }

    public final void a(final LN ln) {
        LinkedHashMap linkedHashMap = this.a;
        if (linkedHashMap != null) {
            linkedHashMap.keySet().removeIf(new Predicate() { // from class: com.android.tools.r8.ir.optimize.j0$$ExternalSyntheticLambda5
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return C3685j0.a(ln, (AbstractC3674g0) obj);
                }
            });
        }
    }

    public final void a(final LN ln, final int i) {
        LinkedHashMap linkedHashMap = this.a;
        if (linkedHashMap != null) {
            linkedHashMap.keySet().removeIf(new Predicate() { // from class: com.android.tools.r8.ir.optimize.j0$$ExternalSyntheticLambda2
                @Override // java.util.function.Predicate
                public final boolean test(Object obj) {
                    return C3685j0.a(ln, i, (AbstractC3674g0) obj);
                }
            });
        }
    }

    public final void a(AbstractC3674g0 abstractC3674g0, n0 n0Var) {
        c();
        if (this.a == null) {
            this.a = new LinkedHashMap();
        }
        this.a.put(abstractC3674g0, n0Var);
    }

    public final void a(C0309l1 c0309l1, n0 n0Var) {
        c();
        if (this.c == null) {
            this.c = new LinkedHashMap();
        }
        this.c.put(c0309l1, n0Var);
    }

    public final void a(l0 l0Var, m0 m0Var) {
        LinkedHashMap linkedHashMap;
        c();
        if (!k && (linkedHashMap = this.b) != null && linkedHashMap.containsKey(l0Var)) {
            throw new AssertionError();
        }
        if (this.e == null) {
            this.e = new LinkedHashMap();
        }
        this.e.put(l0Var, m0Var);
    }
}
