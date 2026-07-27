package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.I2;
import com.android.tools.r8.graph.InterfaceC0227d1;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.d0, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class AbstractC3665d0 {
    public static final /* synthetic */ boolean a = true;

    public static boolean a(I2 i2, I2 i22, InterfaceC0227d1 interfaceC0227d1) {
        com.android.tools.r8.graph.E0 e0D;
        if (i2 == i22) {
            return true;
        }
        com.android.tools.r8.graph.E0 e0D2 = interfaceC0227d1.d(i2);
        return e0D2 != null && e0D2.t1() && (e0D = interfaceC0227d1.d(i22)) != null && e0D2.W0() == e0D.W0();
    }
}
