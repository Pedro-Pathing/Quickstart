package com.android.tools.r8.dex.code;

import androidx.core.internal.view.SupportMenu;
import com.android.tools.r8.graph.A5;
import com.android.tools.r8.graph.C0309l1;
import com.android.tools.r8.graph.C0357p5;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.graph.Y5;
import com.android.tools.r8.internal.AbstractC3581z9;
import com.android.tools.r8.internal.AbstractC3650zs;
import com.android.tools.r8.internal.C0756Kt;
import com.android.tools.r8.internal.C2079jB;
import com.android.tools.r8.internal.C2108ja0;
import com.android.tools.r8.internal.Nk0;
import com.android.tools.r8.internal.SJ;
import com.android.tools.r8.internal.Zf0;
import com.android.tools.r8.utils.structural.AbstractC4012a;
import java.nio.ShortBuffer;
import org.slf4j.Marker;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public abstract class A1 implements InterfaceC0047c, com.android.tools.r8.utils.structural.x<A1> {
    public int b;
    public static final /* synthetic */ boolean e = true;
    public static final A1[] c = new A1[0];
    public static final int[] d = new int[0];

    public A1(InterfaceC0037a interfaceC0037a) {
        this.b = ((B1) interfaceC0037a).d - 1;
    }

    public boolean A() {
        return false;
    }

    public boolean B() {
        return this instanceof C0070g2;
    }

    public boolean C() {
        return false;
    }

    public boolean D() {
        return this instanceof C0157y0;
    }

    public boolean F() {
        return !D() && (this instanceof Q2);
    }

    public boolean H() {
        return false;
    }

    @Override // com.android.tools.r8.utils.structural.x
    public final com.android.tools.r8.utils.structural.x R() {
        return this;
    }

    public abstract int a(A1 a1, AbstractC4012a abstractC4012a);

    public abstract String a(C2108ja0 c2108ja0);

    public void a(Y5 y5) {
    }

    public abstract void a(C0357p5 c0357p5, A5 a5, AbstractC3650zs abstractC3650zs, AbstractC3650zs abstractC3650zs2, SJ sj, ShortBuffer shortBuffer);

    public abstract void a(C0421y c0421y, AbstractC3650zs abstractC3650zs, com.android.tools.r8.dex.M m, A5 a5, SJ sj);

    public abstract void a(C0756Kt c0756Kt);

    public P b() {
        return null;
    }

    public abstract String b(C2108ja0 c2108ja0);

    public abstract void b(com.android.tools.r8.utils.structural.o oVar);

    public C0038a0 c() {
        return null;
    }

    public C0058e0 d() {
        return null;
    }

    public C0063f0 e() {
        return null;
    }

    public final boolean equals(Object obj) {
        return com.android.tools.r8.utils.structural.k.a(this, obj);
    }

    public Y f() {
        return null;
    }

    public C0165z1 g() {
        return null;
    }

    public C0309l1 getField() {
        return null;
    }

    public boolean h() {
        return false;
    }

    public abstract int hashCode();

    @Override // com.android.tools.r8.internal.H
    public final boolean i() {
        return h();
    }

    public com.android.tools.r8.graph.D0 j() {
        return null;
    }

    public int k() {
        return p();
    }

    public C0409w2 l() {
        return null;
    }

    public abstract String m();

    public int n() {
        return this.b;
    }

    @Override // com.android.tools.r8.utils.structural.x
    public final com.android.tools.r8.utils.structural.y o() {
        throw new Nk0();
    }

    public abstract int p();

    @Override // com.android.tools.r8.dex.code.InterfaceC0047c
    public final AbstractC3581z9 q() {
        return null;
    }

    @Override // com.android.tools.r8.dex.code.InterfaceC0047c
    public final A1 r() {
        return this;
    }

    public int s() {
        return 0;
    }

    public abstract int t();

    public String toString() {
        return b(C2108ja0.b);
    }

    public abstract String u();

    public int[] w() {
        return null;
    }

    public boolean x() {
        return false;
    }

    public boolean y() {
        return false;
    }

    public boolean z() {
        return false;
    }

    public static String b(int i) {
        return i >= 0 ? Marker.ANY_NON_NULL_MARKER + i : Integer.toString(i);
    }

    public static String c(int i) {
        return Zf0.a(i, 2);
    }

    public static short d(int i, int i2) {
        return (short) (((i & 255) << 8) | (i2 & 255));
    }

    public static int e(int i, int i2) {
        return ((i & 15) << 4) | (i2 & 15);
    }

    @Override // com.android.tools.r8.utils.structural.x
    public final int a(com.android.tools.r8.utils.structural.x xVar, AbstractC4012a abstractC4012a) {
        A1 a1 = (A1) xVar;
        int iA = abstractC4012a.a(k(), a1.k());
        if (iA != 0) {
            return iA;
        }
        int iA2 = abstractC4012a.a(n(), a1.n());
        return iA2 != 0 ? iA2 : a(a1, abstractC4012a);
    }

    public void f(int i) {
        this.b = i;
    }

    public final String b(String str) {
        StringBuilder sb = new StringBuilder();
        Zf0.a(6, Zf0.a(n(), 2), sb);
        sb.append(": ");
        Zf0.b(20, m(), sb);
        if (str == null) {
            str = "";
        }
        sb.append(str);
        return sb.toString();
    }

    public final String e(int i) {
        return Zf0.a(n() + i, 2) + " (" + b(i) + ")";
    }

    public A1() {
        this.b = -1;
    }

    public static char a(InterfaceC0037a interfaceC0037a) {
        return (char) (((B1) interfaceC0037a).b() & 65535);
    }

    public static int a(B1 b1) {
        return ((((char) (b1.b() & 65535)) << 16) & SupportMenu.CATEGORY_MASK) | (((char) (b1.b() & 65535)) & 65535);
    }

    public static void a(int i, int i2, ShortBuffer shortBuffer) {
        shortBuffer.put((short) (((i & 255) << 8) | (i2 & 255)));
    }

    public static void a(int i, int i2, ShortBuffer shortBuffer, int i3) {
        shortBuffer.put((short) (((i & 15) << 12) | ((i2 & 15) << 8) | (i3 & 255)));
    }

    public static void a(long j, ShortBuffer shortBuffer) {
        shortBuffer.put((short) (j & 65535));
        shortBuffer.put((short) ((j >> 16) & 65535));
    }

    public static void a(com.android.tools.r8.graph.W3 w3, ShortBuffer shortBuffer, C0357p5 c0357p5) {
        int iA = w3.a(c0357p5);
        if (!e && iA != (65535 & iA)) {
            throw new AssertionError();
        }
        shortBuffer.put((short) iA);
    }

    public final String a(String str) {
        StringBuilder sb = new StringBuilder();
        sb.append("    ");
        if (str != null) {
            Zf0.b(20, u(), sb);
            sb.append(str);
        } else {
            sb.append(u());
        }
        return sb.toString();
    }

    @Override // com.android.tools.r8.utils.structural.x
    public final void a(com.android.tools.r8.utils.structural.o oVar) {
        com.android.tools.r8.utils.structural.q qVar = (com.android.tools.r8.utils.structural.q) oVar;
        qVar.a.a(k());
        qVar.a.a(n());
        b(oVar);
    }

    public String a(A1 a1) {
        throw new C2079jB("Instruction " + a1 + " is not a payload user");
    }

    public String a(C2108ja0 c2108ja0, A1 a1) {
        throw new C2079jB("Instruction " + a1 + " is not a payload user");
    }
}
