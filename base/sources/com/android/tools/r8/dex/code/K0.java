package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.W3;
import com.android.tools.r8.internal.C2108ja0;
import com.android.tools.r8.utils.structural.AbstractC4012a;
import java.util.function.ToIntFunction;

/* JADX INFO: Access modifiers changed from: package-private */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public abstract class K0<T extends com.android.tools.r8.graph.W3> extends K {
    public static final /* synthetic */ boolean h = true;
    public final short f;
    public T g;

    public K0(int i, InterfaceC0037a interfaceC0037a, com.android.tools.r8.graph.W3[] w3Arr) {
        super(interfaceC0037a);
        this.f = (short) i;
        this.g = (T) w3Arr[A1.a(interfaceC0037a)];
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int a(A1 a1, AbstractC4012a abstractC4012a) {
        return abstractC4012a.a(this, (K0) a1, (com.android.tools.r8.utils.structural.y<K0>) new com.android.tools.r8.utils.structural.y() { // from class: com.android.tools.r8.dex.code.K0$$ExternalSyntheticLambda2
            @Override // com.android.tools.r8.utils.structural.y
            public final void a(com.android.tools.r8.utils.structural.A a) {
                this.f$0.b(a);
            }
        });
    }

    public abstract void a(com.android.tools.r8.utils.structural.A a);

    public final void b(com.android.tools.r8.utils.structural.A a) {
        com.android.tools.r8.utils.structural.A a2 = a.a(new ToIntFunction() { // from class: com.android.tools.r8.dex.code.K0$$ExternalSyntheticLambda0
            @Override // java.util.function.ToIntFunction
            public final int applyAsInt(Object obj) {
                return ((K0) obj).f;
            }
        });
        K0$$ExternalSyntheticLambda1 k0$$ExternalSyntheticLambda1 = new K0$$ExternalSyntheticLambda1(this);
        a2.getClass();
        k0$$ExternalSyntheticLambda1.a(a2);
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int hashCode() {
        return ((this.g.hashCode() << 8) | this.f) ^ getClass().hashCode();
    }

    @Override // com.android.tools.r8.dex.code.A1
    public String a(C2108ja0 c2108ja0) {
        return a("v" + ((int) this.f) + ", " + this.g.l0());
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void b(com.android.tools.r8.utils.structural.o oVar) {
        com.android.tools.r8.utils.structural.q qVar = (com.android.tools.r8.utils.structural.q) oVar;
        qVar.a.a((int) this.f);
        new K0$$ExternalSyntheticLambda1(this).a(new com.android.tools.r8.utils.structural.p(this, qVar));
    }

    /* JADX WARN: Multi-variable type inference failed */
    public K0(int i, com.android.tools.r8.graph.W3 w3) {
        if (!h && (i < 0 || i > 255)) {
            throw new AssertionError();
        }
        this.f = (short) i;
        this.g = w3;
    }

    @Override // com.android.tools.r8.dex.code.A1
    public String b(C2108ja0 c2108ja0) {
        return b("v" + ((int) this.f) + ", " + c2108ja0.a(this.g));
    }
}
