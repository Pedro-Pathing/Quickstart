package com.android.tools.r8.ir.regalloc;

import com.android.tools.r8.internal.AbstractC3250vj0;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class m {
    public static final /* synthetic */ boolean d = true;
    public final AbstractC3250vj0 a;
    public c b;
    public final c c;

    public m(AbstractC3250vj0 abstractC3250vj0, c cVar, c cVar2) {
        this.a = abstractC3250vj0;
        this.c = cVar;
        this.b = cVar2;
        boolean z = d;
        if (!z && cVar.l == Integer.MIN_VALUE) {
            throw new AssertionError();
        }
        if (!z && cVar2.l == Integer.MIN_VALUE) {
            throw new AssertionError();
        }
    }

    public final boolean equals(Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof m)) {
            return false;
        }
        m mVar = (m) obj;
        if (this.a == mVar.a) {
            c cVar = this.b;
            int i = cVar.l;
            c cVar2 = mVar.b;
            if (i == cVar2.l) {
                c cVar3 = this.c;
                int i2 = cVar3.l;
                c cVar4 = mVar.c;
                if (i2 == cVar4.l && cVar.e == cVar2.e && cVar3.e == cVar4.e) {
                    return true;
                }
            }
        }
        return false;
    }

    public final int hashCode() {
        return (this.c.l * 5) + (this.b.l * 3) + this.a.hashCode();
    }

    public final String toString() {
        return this.c.d() + " <- " + this.b.d() + " (" + this.a + ")";
    }
}
