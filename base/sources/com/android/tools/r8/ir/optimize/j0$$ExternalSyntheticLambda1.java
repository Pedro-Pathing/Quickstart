package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.I2;
import java.util.Set;
import java.util.function.Predicate;

/* JADX INFO: compiled from: D8$$SyntheticClass */
/* JADX INFO: loaded from: classes8.dex */
public final /* synthetic */ class j0$$ExternalSyntheticLambda1 implements Predicate {
    public final /* synthetic */ Set f$0;

    public /* synthetic */ j0$$ExternalSyntheticLambda1(Set set) {
        this.f$0 = set;
    }

    @Override // java.util.function.Predicate
    public final boolean test(Object obj) {
        return this.f$0.contains((I2) obj);
    }
}
