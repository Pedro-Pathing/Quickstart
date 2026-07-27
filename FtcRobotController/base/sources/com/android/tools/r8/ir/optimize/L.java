package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.AbstractC1308bC;
import com.android.tools.r8.internal.InterfaceC0693Is;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class L implements InterfaceC0693Is {
    public static final /* synthetic */ boolean a = true;

    @Override // com.android.tools.r8.internal.InterfaceC0693Is
    public final boolean a(Object obj, Object obj2) {
        AbstractC1308bC abstractC1308bC = (AbstractC1308bC) obj;
        AbstractC1308bC abstractC1308bC2 = (AbstractC1308bC) obj2;
        boolean z = a;
        if (!z && abstractC1308bC != null && abstractC1308bC.d().y()) {
            throw new AssertionError();
        }
        if (z || abstractC1308bC2 == null || !abstractC1308bC2.d().y()) {
            return abstractC1308bC == abstractC1308bC2 || (abstractC1308bC != null && abstractC1308bC2 != null && abstractC1308bC.b(abstractC1308bC2) && abstractC1308bC.c.equals(abstractC1308bC2.c));
        }
        throw new AssertionError();
    }

    @Override // com.android.tools.r8.internal.InterfaceC0693Is
    public final int a(Object obj) {
        AbstractC1308bC abstractC1308bC = (AbstractC1308bC) obj;
        return abstractC1308bC.c.hashCode() + (abstractC1308bC.U2().hashCode() * 31);
    }
}
