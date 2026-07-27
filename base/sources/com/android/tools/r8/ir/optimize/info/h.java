package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.internal.AbstractC0548Dw;
import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.AbstractC0722Jn;
import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.AbstractC2670pV;
import com.android.tools.r8.internal.B1;
import com.android.tools.r8.internal.B7;
import com.android.tools.r8.internal.InterfaceC2591oc;
import com.android.tools.r8.internal.Jb0;
import com.android.tools.r8.internal.WB;
import java.util.BitSet;
import java.util.Set;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class h implements g {
    public static final /* synthetic */ boolean a = true;

    public abstract boolean A();

    public abstract AbstractC2670pV B();

    public abstract boolean C();

    public abstract boolean D();

    public abstract boolean E();

    public abstract boolean F();

    public abstract boolean G();

    public abstract AbstractC0548Dw a(WB wb);

    public abstract boolean a(AbstractC1308bC abstractC1308bC);

    public abstract boolean e();

    public abstract boolean f();

    public abstract boolean g();

    public abstract com.android.tools.r8.internal.r h();

    public abstract B1 i();

    public abstract AbstractC3679a j();

    public abstract B7 k();

    public abstract InterfaceC2591oc l();

    public abstract AbstractC0548Dw m();

    public abstract AbstractC0564Em n();

    public abstract AbstractC0722Jn o();

    public abstract Set p();

    public abstract int q();

    public abstract BitSet r();

    public abstract BitSet s();

    public abstract int t();

    public abstract Jb0 u();

    public abstract BitSet v();

    public abstract boolean w();

    public final boolean x() {
        if (a || v() == null || !v().isEmpty()) {
            return v() != null;
        }
        throw new AssertionError();
    }

    public abstract boolean y();

    public abstract boolean z();
}
