package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.ExtractMarkerCommand$$ExternalSyntheticBackport0;
import com.android.tools.r8.graph.C0236e3;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.E2;
import com.android.tools.r8.graph.E4;
import com.android.tools.r8.graph.G;
import com.android.tools.r8.graph.G4$$ExternalSyntheticLambda1;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.graph.g1$$ExternalSyntheticLambda0;
import com.android.tools.r8.internal.C3306wL;
import com.android.tools.r8.internal.Zf0;
import dk.sgjesse.r8api.DescriptorUtils;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.TreeMap;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public abstract class h {
    public static final /* synthetic */ boolean f = true;
    public final TreeMap a = new TreeMap(Comparator.comparing(new g1$$ExternalSyntheticLambda0()));
    public final TreeMap b = new TreeMap(Comparator.comparing(new G4$$ExternalSyntheticLambda1()));
    public final TreeMap c = new TreeMap(Comparator.comparing(new G4$$ExternalSyntheticLambda1()));
    public final String d;
    public final String e;

    public h(I2 i2) {
        String strM0 = i2.m0();
        this.d = strM0;
        int iLastIndexOf = strM0.lastIndexOf(46);
        this.e = iLastIndexOf > 0 ? strM0.substring(0, iLastIndexOf) : "";
    }

    public static String a(String str, String str2) {
        if (str.startsWith(str2) && str.length() > str2.length() && str.charAt(str2.length()) == '.') {
            return str.substring(str.lastIndexOf(46) + 1);
        }
        return null;
    }

    public final String a(String str) {
        String str2 = this.e;
        String strA = a(str, str2);
        if (strA == null) {
            if (Zf0.a(str2, DescriptorUtils.JAVA_PACKAGE_SEPARATOR).size() > 2) {
                strA = a(str, str2.substring(0, (str2.length() - ((String) C3306wL.b(r1)).length()) - 1));
            } else {
                strA = null;
            }
        }
        if (strA == null) {
            strA = a(str, "java.lang");
        }
        if (strA == null) {
            strA = a(str, "java.util.function");
        }
        if (strA != null) {
            str = strA;
        }
        return str.replace('$', DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
    }

    public final String a(I2 i2) {
        if (i2.T0()) {
            return i2.m0();
        }
        return a(i2.m0());
    }

    public static String a(C0236e3 c0236e3) {
        ArrayList arrayList = new ArrayList();
        if (c0236e3.m()) {
            arrayList.add("public");
        }
        if (c0236e3.l()) {
            arrayList.add("protected");
        }
        if (c0236e3.i()) {
            if (f) {
                arrayList.add("private");
            } else {
                throw new AssertionError();
            }
        }
        if (c0236e3.g()) {
            if (f) {
                arrayList.add("/* package */");
            } else {
                throw new AssertionError();
            }
        }
        if (c0236e3.n()) {
            arrayList.add("static");
        }
        if (c0236e3.f()) {
            arrayList.add("final");
        }
        return ExtractMarkerCommand$$ExternalSyntheticBackport0.m(" ", arrayList);
    }

    public static String a(E4 e4) {
        ArrayList arrayList = new ArrayList();
        if (e4.m()) {
            arrayList.add("public");
        }
        if (e4.l()) {
            arrayList.add("protected");
        }
        if (e4.i()) {
            if (f) {
                arrayList.add("private");
            } else {
                throw new AssertionError();
            }
        }
        if (e4.g()) {
            if (f) {
                arrayList.add("/* package */");
            } else {
                throw new AssertionError();
            }
        }
        if (e4.J()) {
            arrayList.add("abstract");
        }
        if (e4.n()) {
            arrayList.add("static");
        }
        if (e4.f()) {
            arrayList.add("final");
        }
        return ExtractMarkerCommand$$ExternalSyntheticBackport0.m(" ", arrayList);
    }

    public final String a(C0291j1 c0291j1) {
        E2 e2 = c0291j1.getReference().i;
        StringBuilder sb = new StringBuilder();
        int i = (c0291j1.C1() || c0291j1.g.L()) ? 1 : 0;
        sb.append("(");
        I2[] i2Arr = e2.f.b;
        int length = i2Arr.length;
        int i2 = 0;
        int i3 = 0;
        boolean z = true;
        while (i2 < length) {
            I2 i22 = i2Arr[i2];
            if (!z) {
                sb.append(", ");
            }
            if (c0291j1.j1()) {
                String string = "p" + i3;
                for (G.a aVar : c0291j1.V0().H().I0()) {
                    if (aVar.b() == i) {
                        if (!f && aVar.c().b.toString().equals("this")) {
                            throw new AssertionError();
                        }
                        string = aVar.c().b.toString();
                    }
                }
                sb.append(a(i22)).append(" ").append(string);
            } else {
                sb.append(a(i22)).append(" p").append(i3);
            }
            i += i22.X0() ? 2 : 1;
            i3++;
            i2++;
            z = false;
        }
        sb.append(")");
        return sb.toString();
    }
}
