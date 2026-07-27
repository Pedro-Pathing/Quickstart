package com.android.tools.r8.ir.optimize.info;

import com.android.tools.r8.internal.AbstractC0564Em;
import com.android.tools.r8.internal.B1;
import com.android.tools.r8.internal.Dk0;
import java.util.BitSet;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class k {
    public B1 a = Dk0.a;
    public AbstractC0564Em b = AbstractC0564Em.m();
    public boolean c = true;
    public boolean d = false;
    public BitSet e = null;
    public BitSet f = null;
    public int g = -1;
    public boolean h = false;

    public final k a(boolean z, Consumer consumer) {
        if (z) {
            consumer.accept(this);
        }
        return this;
    }
}
