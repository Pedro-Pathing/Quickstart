package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0357p5;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C2108ja0;
import com.android.tools.r8.internal.SJ;
import com.android.tools.r8.utils.structural.AbstractC4012a;
import java.nio.ShortBuffer;
import java.util.function.ToIntFunction;

/* JADX INFO: Access modifiers changed from: package-private */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public abstract class G0 extends J {
    public static final /* synthetic */ boolean h = true;
    public final byte f;
    public final byte g;

    public G0(int i, B1 b1) {
        super(b1);
        this.f = (byte) (i & 15);
        int i2 = i >> 4;
        if ((i2 & 8) != 0) {
            this.g = (byte) (~((~i2) & 15));
        } else {
            this.g = (byte) (i2 & 15);
        }
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int hashCode() {
        return ((this.f << 4) | this.g) ^ getClass().hashCode();
    }

    public static void a(com.android.tools.r8.utils.structural.A a) {
        a.a(new ToIntFunction() { // from class: com.android.tools.r8.dex.code.G0$$ExternalSyntheticLambda1
            @Override // java.util.function.ToIntFunction
            public final int applyAsInt(Object obj) {
                return ((G0) obj).f;
            }
        }).a(new ToIntFunction() { // from class: com.android.tools.r8.dex.code.G0$$ExternalSyntheticLambda2
            @Override // java.util.function.ToIntFunction
            public final int applyAsInt(Object obj) {
                return ((G0) obj).g;
            }
        });
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final void b(com.android.tools.r8.utils.structural.o oVar) {
        G0$$ExternalSyntheticLambda0 g0$$ExternalSyntheticLambda0 = new G0$$ExternalSyntheticLambda0();
        com.android.tools.r8.utils.structural.q qVar = (com.android.tools.r8.utils.structural.q) oVar;
        qVar.getClass();
        g0$$ExternalSyntheticLambda0.a(new com.android.tools.r8.utils.structural.p(this, qVar));
    }

    @Override // com.android.tools.r8.dex.code.A1
    public void a(C0357p5 c0357p5, A5 a5, AbstractC3650zs abstractC3650zs, AbstractC3650zs abstractC3650zs2, SJ sj, ShortBuffer shortBuffer) {
        A1.a(this.g, this.f, shortBuffer, p());
    }

    @Override // com.android.tools.r8.dex.code.A1
    public final int a(A1 a1, AbstractC4012a abstractC4012a) {
        return abstractC4012a.a(this, (G0) a1, new G0$$ExternalSyntheticLambda0());
    }

    @Override // com.android.tools.r8.dex.code.A1
    public String b(C2108ja0 c2108ja0) {
        return b("v" + ((int) this.f) + ", #" + ((int) this.g));
    }

    @Override // com.android.tools.r8.dex.code.A1
    public void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj) {
    }

    public G0(int i, int i2) {
        boolean z = h;
        if (!z && (i < 0 || i > 15)) {
            throw new AssertionError();
        }
        if (!z && (-8 > i2 || i2 > 7)) {
            throw new AssertionError();
        }
        this.f = (byte) i;
        this.g = (byte) i2;
    }
}
