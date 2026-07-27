package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.C3654zw;
import com.android.tools.r8.internal.InterfaceC0693Is;
import com.android.tools.r8.internal.Nk0;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.y, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3715y implements InterfaceC0693Is {
    public static final /* synthetic */ boolean a = true;

    @Override // com.android.tools.r8.internal.InterfaceC0693Is
    public final boolean a(Object obj, Object obj2) {
        AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) obj;
        AbstractC1076Vw abstractC1076Vw2 = (AbstractC1076Vw) obj2;
        if (abstractC1076Vw == abstractC1076Vw2) {
            return true;
        }
        if (abstractC1076Vw == null || abstractC1076Vw2 == null || abstractC1076Vw.getClass() != abstractC1076Vw2.getClass() || ((abstractC1076Vw instanceof C3654zw) && abstractC1076Vw.U0() != abstractC1076Vw2.U0())) {
            return false;
        }
        return abstractC1076Vw.b(abstractC1076Vw2);
    }

    @Override // com.android.tools.r8.internal.InterfaceC0693Is
    public final int a(Object obj) {
        AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) obj;
        if (!a && !abstractC1076Vw.h1()) {
            throw new AssertionError();
        }
        int iH2 = abstractC1076Vw.H2();
        if (iH2 == 12) {
            return abstractC1076Vw.D().j.hashCode();
        }
        if (iH2 == 20) {
            return abstractC1076Vw.M().j.hashCode();
        }
        if (iH2 == 28 || iH2 == 59) {
            return abstractC1076Vw.P().getField().hashCode();
        }
        if (iH2 == 71) {
            return Integer.hashCode(abstractC1076Vw.D0().j);
        }
        if (iH2 == 15) {
            return (abstractC1076Vw.I2().hashCode() * 13) + Long.hashCode(abstractC1076Vw.H().P2());
        }
        if (iH2 == 16) {
            return abstractC1076Vw.I().L2().hashCode();
        }
        throw new Nk0();
    }
}
