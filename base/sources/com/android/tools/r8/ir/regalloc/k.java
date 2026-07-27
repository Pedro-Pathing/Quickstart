package com.android.tools.r8.ir.regalloc;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.Nk0;
import java.util.Arrays;
import java.util.BitSet;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class k extends j {
    public static final /* synthetic */ boolean g = true;
    public final int a;
    public int[] b = new int[16];
    public final BitSet c;
    public final BitSet d;
    public final BitSet e;
    public final BitSet f;

    public k(int i) {
        this.a = i;
        for (int i2 = 0; i2 < 16; i2++) {
            this.b[i2] = Integer.MAX_VALUE;
        }
        this.c = new BitSet(i);
        this.d = new BitSet(i);
        this.e = new BitSet(i);
        this.f = new BitSet(i);
    }

    @Override // com.android.tools.r8.ir.regalloc.j
    public final boolean a(int i, i iVar) {
        if (!g && this.f.get(i)) {
            throw new AssertionError();
        }
        switch (iVar.ordinal()) {
            case 0:
                return this.d.get(i);
            case 1:
                return this.c.get(i);
            case 2:
                return (this.d.get(i) || this.c.get(i) || this.e.get(i)) ? false : true;
            case 3:
                return true;
            default:
                throw new Nk0("Unexpected register position type: " + iVar);
        }
    }

    @Override // com.android.tools.r8.ir.regalloc.j
    public final boolean b(int i) {
        return this.f.get(i);
    }

    public final void a(int i, int i2, c cVar) {
        int[] iArr = this.b;
        if (i >= iArr.length) {
            int i3 = i + 1;
            int length = iArr.length;
            while (length < i3) {
                length *= 2;
            }
            int iMin = Math.min(length, this.a);
            int[] iArr2 = this.b;
            this.b = Arrays.copyOf(iArr2, iMin);
            for (int length2 = iArr2.length; length2 < iMin; length2++) {
                this.b[length2] = Integer.MAX_VALUE;
            }
        }
        this.b[i] = i2;
        BitSet bitSet = this.c;
        C3161ul0 c3161ul0 = cVar.b;
        boolean z = false;
        bitSet.set(i, c3161ul0.c != null && c3161ul0.H());
        this.d.set(i, cVar.o);
        BitSet bitSet2 = this.e;
        AbstractC1076Vw abstractC1076Vw = cVar.b.c;
        if (abstractC1076Vw != null && abstractC1076Vw.o2() && !cVar.b.c.u0().j) {
            z = true;
        }
        bitSet2.set(i, z);
    }

    @Override // com.android.tools.r8.ir.regalloc.j
    public final int a(int i) {
        boolean z = g;
        if (!z && this.f.get(i)) {
            throw new AssertionError();
        }
        int[] iArr = this.b;
        if (i < iArr.length) {
            return iArr[i];
        }
        if (z || i < this.a) {
            return Integer.MAX_VALUE;
        }
        throw new AssertionError();
    }
}
