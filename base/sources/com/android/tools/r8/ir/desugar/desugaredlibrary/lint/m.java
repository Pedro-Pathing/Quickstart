package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.E0;
import com.android.tools.r8.internal.C3366wv;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class m {
    public final Map a;
    public final List b;

    /* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
    public static class a extends b {
        public static final a e = new a(-1, -1, false);

        public a(int i, int i2, boolean z) {
            super(i, i2, z);
        }

        public static a a(int i) {
            return new a(i, i, true);
        }
    }

    /* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
    public static abstract class b {
        public static final /* synthetic */ boolean d = true;
        public final boolean a;
        public final int b;
        public final int c;

        public b(int i, int i2, boolean z) {
            this.a = z;
            this.b = i;
            this.c = i2;
        }

        public int a() {
            return this.c;
        }

        public int b() {
            return this.b;
        }

        public boolean c() {
            return this.a;
        }

        public final int a(b bVar) {
            int i;
            int i2;
            boolean z = this.a;
            if (!z && !bVar.a) {
                i2 = -1;
                i = -1;
            } else if (z && bVar.a) {
                int i3 = this.c;
                i = bVar.b;
                if (i3 == i - 1) {
                    int i4 = this.b;
                    i2 = bVar.c;
                    i = i4;
                } else {
                    i2 = bVar.c;
                    int i5 = this.b;
                    if (i2 == i5 - 1) {
                        i2 = i3;
                    } else if (i3 == 19 && i == 21) {
                        i = i5;
                    } else {
                        if (i2 != 19 || i5 != 21) {
                            throw new RuntimeException("Cannot merge ranges.");
                        }
                        i2 = i3;
                    }
                }
            } else {
                i = z ? this.b : bVar.b;
                i2 = z ? this.c : bVar.c;
            }
            if (d || (i2 < 32768 && i < 32768)) {
                return (i2 << 16) + i;
            }
            throw new AssertionError();
        }
    }

    /* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
    public static class c extends b {
        public static final c h = new c(false, false, true, false, -1, -1);
        public static final c i = new c(false, false, false, false, -1, -1);
        public static final c j = new c(true, false, false, false, -1, -1);
        public static final c k = new c(false, true, false, false, -1, -1);
        public final boolean e;
        public final boolean f;
        public final boolean g;

        public c(boolean z, boolean z2, boolean z3, boolean z4, int i2, int i3) {
            super(i2, i3, z4);
            this.e = z;
            this.f = z2;
            this.g = z3;
        }

        public static c a(int i2) {
            return new c(false, false, false, true, i2, i2);
        }
    }

    /* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
    public static class d {
        public final E0 a;
        public final l b;
        public final SortedMap c;
        public final SortedMap d;
        public final Map e;
        public final Map f;

        public d(E0 e0, l lVar, C3366wv c3366wv, C3366wv c3366wv2, HashMap map, HashMap map2) {
            this.a = e0;
            this.b = lVar;
            this.c = c3366wv;
            this.d = c3366wv2;
            this.e = map;
            this.f = map2;
        }

        public void a(BiConsumer<C0257g1, a> biConsumer) {
            for (C0257g1 c0257g1 : this.d.values()) {
                biConsumer.accept(c0257g1, (a) this.f.get(c0257g1.getReference()));
            }
        }

        public void b(BiConsumer<C0291j1, c> biConsumer) {
            for (C0291j1 c0291j1 : this.c.values()) {
                biConsumer.accept(c0291j1, (c) this.e.get(c0291j1.getReference()));
            }
        }
    }

    public m(C3366wv c3366wv, List list) {
        this.a = c3366wv;
        this.b = list;
    }

    public void a(Consumer<d> consumer) {
        this.a.values().forEach(consumer);
    }

    public final List a() {
        return this.b;
    }
}
