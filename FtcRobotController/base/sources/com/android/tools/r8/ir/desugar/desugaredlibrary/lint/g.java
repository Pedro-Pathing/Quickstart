package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.H2;
import com.android.tools.r8.graph.I2;
import com.android.tools.r8.ir.desugar.desugaredlibrary.lint.m;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class g extends h {
    public final l g;
    public boolean h;
    public boolean i;
    public boolean j;
    public boolean k;

    public g(I2 i2, l lVar) {
        super(i2);
        this.h = false;
        this.i = false;
        this.j = false;
        this.k = false;
        this.g = lVar;
    }

    public final String a(m.c cVar) {
        if (cVar == null) {
            return "";
        }
        StringBuilder sb = new StringBuilder();
        if (cVar.e) {
            sb.append("<sup>1</sup>");
            this.h = true;
        }
        if (cVar.f) {
            sb.append("<sup>2</sup>");
            this.i = true;
        }
        if (cVar.a) {
            sb.append("<sup>3</sup>");
            this.j = true;
        }
        if (cVar.g) {
            sb.append("<sup>4</sup>");
            this.k = true;
        }
        return sb.toString();
    }

    public final String toString() {
        String string;
        f fVar = new f();
        fVar.k("tr");
        if (this.e.length() > 0) {
            fVar.i(this.e);
        }
        fVar.g(a(this.d));
        fVar.k("td").k("ul style=\"list-style-position:inside; list-style-type: none !important; margin-left:0px;padding-left:0px !important;\"");
        if (!this.a.isEmpty()) {
            for (C0257g1 c0257g1 : this.a.keySet()) {
                String strA = h.a(c0257g1.g);
                String strA2 = a(c0257g1.getReference().i);
                H2 h2 = c0257g1.getReference().g;
                m.a aVar = (m.a) this.a.get(c0257g1);
                if (aVar == null) {
                    string = "";
                } else {
                    StringBuilder sb = new StringBuilder();
                    if (aVar.a) {
                        sb.append("<sup>3</sup>");
                        this.j = true;
                    }
                    string = sb.toString();
                }
                fVar.e(strA + " " + strA2 + " " + h2 + string);
            }
        }
        if (!this.b.isEmpty()) {
            for (C0291j1 c0291j1 : this.b.keySet()) {
                fVar.f(h.a(c0291j1.g) + " " + a(this.d) + a(c0291j1) + a((m.c) this.b.get(c0291j1)));
            }
        }
        if (!this.c.isEmpty()) {
            for (C0291j1 c0291j12 : this.c.keySet()) {
                fVar.f(h.a(c0291j12.g) + " " + a(c0291j12.getReference().i.e) + " " + c0291j12.getReference().g + a(c0291j12) + a((m.c) this.c.get(c0291j12)));
            }
        }
        fVar.j("ul").j("td");
        StringBuilder sb2 = new StringBuilder();
        if (this.g.d()) {
            sb2.append("Fully implemented class.<br>&nbsp;");
        }
        if (this.g.c()) {
            sb2.append("Additional methods on existing class.<br>&nbsp;");
        }
        if (this.h) {
            sb2.append("<sup>1</sup> Supported only on devices which API level is 21 or higher.<br>&nbsp;");
        }
        if (this.i) {
            sb2.append("<sup>2</sup> Not present in Android ").append(a.g).append(" (May not resolve at compilation).<br>&nbsp;");
        }
        if (this.j) {
            sb2.append("<sup>3</sup> Not supported at all minSDK levels.<br>&nbsp;");
        }
        if (this.k) {
            sb2.append("<sup>4</sup> Also supported with covariant return type.<br>&nbsp;");
        }
        if (!this.g.a().isEmpty()) {
            sb2.append("Some fields (").append(this.g.a().size()).append(") present in Android ").append(a.g).append(" are not supported.<br>&nbsp;");
        }
        if (!this.g.b().isEmpty()) {
            sb2.append("Some methods (").append(this.g.b().size()).append(") present in Android ").append(a.g).append(" are not supported.");
        }
        fVar.h(sb2.toString());
        fVar.j("tr");
        return fVar.toString();
    }
}
