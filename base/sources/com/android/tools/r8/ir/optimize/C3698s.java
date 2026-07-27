package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.AbstractC2575oO;
import com.android.tools.r8.internal.AbstractC2614oo;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.H5;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.s, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3698s extends AbstractC2614oo {
    public static final /* synthetic */ boolean d = true;
    public final com.android.tools.r8.ir.regalloc.f a;
    public final AbstractC2575oO.a b;
    public final int[] c;

    public C3698s(C0874Ot c0874Ot, com.android.tools.r8.ir.regalloc.f fVar) {
        this.a = fVar;
        this.b = c0874Ot.b;
        int[] iArr = new int[c0874Ot.f.b() + 1];
        this.c = iArr;
        Arrays.fill(iArr, -1);
    }

    @Override // com.android.tools.r8.internal.AbstractC2614oo
    public final boolean a(Object obj, Object obj2) {
        H5 h5 = (H5) obj;
        H5 h52 = (H5) obj2;
        LinkedList<AbstractC1076Vw> linkedListK = h5.k();
        LinkedList<AbstractC1076Vw> linkedListK2 = h52.k();
        if (linkedListK.size() != linkedListK2.size()) {
            return false;
        }
        for (int i = 0; i < linkedListK.size(); i++) {
            if (!linkedListK.get(i).a(linkedListK2.get(i), this.a, this.b)) {
                return false;
            }
        }
        if (!this.a.a(h5, h52) || !h5.i().equals(h52.i())) {
            return false;
        }
        if (!d) {
            List<H5> listT = h5.t();
            List<H5> listT2 = h52.t();
            if (listT.size() == listT2.size()) {
                for (int i2 = 0; i2 < listT.size(); i2++) {
                    if (listT.get(i2) == listT2.get(i2)) {
                    }
                }
            }
            throw new AssertionError();
        }
        return true;
    }

    @Override // com.android.tools.r8.internal.AbstractC2614oo
    public final int a(Object obj) {
        H5 h5 = (H5) obj;
        int i = this.c[h5.o()];
        if (i != -1) {
            if (d || i == a(h5)) {
                return i;
            }
            throw new AssertionError();
        }
        int iA = a(h5);
        this.c[h5.o()] = iA;
        return iA;
    }

    public final int a(H5 h5) {
        int iA;
        LinkedList<AbstractC1076Vw> linkedListK = h5.k();
        int size = linkedListK.size();
        for (int i = 0; i < linkedListK.size() && i < 5; i++) {
            AbstractC1076Vw abstractC1076Vw = linkedListK.get(i);
            if (abstractC1076Vw.d() != null && abstractC1076Vw.d().T()) {
                iA = this.a.a(abstractC1076Vw.d(), abstractC1076Vw.e);
            } else {
                iA = 0;
            }
            for (C3161ul0 c3161ul0 : abstractC1076Vw.c) {
                iA <<= 4;
                if (c3161ul0.T()) {
                    iA += this.a.a(c3161ul0, abstractC1076Vw.e);
                }
            }
            size = (size * 3) + iA;
        }
        return size;
    }
}
