package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.C2199kS;
import com.android.tools.r8.internal.C2947sS;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.i, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3677i {
    public static final /* synthetic */ boolean c = true;
    public AbstractC3675h a;
    public AbstractC0564Em b = AbstractC0564Em.m();

    public C3677i(AbstractC3675h abstractC3675h) {
        this.a = abstractC3675h;
    }

    public final boolean a() {
        return this.b.h() && !this.b.l();
    }

    public final boolean b() {
        return this.b.d().d();
    }

    public final void c() {
        if (!this.b.l()) {
            this.b = this.b.a(C2947sS.b());
        } else {
            boolean z = AbstractC0564Em.a;
            this.b = C2199kS.b;
        }
    }

    public final void a(AbstractC0564Em abstractC0564Em) {
        boolean z = c;
        if (!z && abstractC0564Em == null) {
            throw new AssertionError();
        }
        if (!z && (!this.b.j() ? this.b.l() : abstractC0564Em.d().d())) {
            throw new AssertionError();
        }
        this.b = abstractC0564Em;
    }
}
