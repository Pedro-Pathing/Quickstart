package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.AbstractC0026a;
import com.android.tools.r8.internal.Zf0;
import dk.sgjesse.r8api.DescriptorUtils;
import java.util.List;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class f extends i {
    public final void a() {
        this.c += "  ";
    }

    public final f e(String str) {
        b("<li class=\"java8_table\"><code>" + str + "</code></li>");
        return this;
    }

    public final void f(String str) {
        if (str.length() < 53 || str.contains("()")) {
            e(str);
            return;
        }
        StringBuilder sb = new StringBuilder();
        List<String> listA = Zf0.a(str, '(');
        sb.append(listA.get(0)).append("(<br>&nbsp;");
        if (listA.get(1).length() < 51) {
            sb.append(listA.get(1));
            e(sb.toString());
            return;
        }
        List<String> listA2 = Zf0.a(listA.get(1), ',');
        sb.append("&nbsp;");
        for (int i = 0; i < listA2.size(); i++) {
            sb.append(listA2.get(i));
            if (i != listA2.size() - 1) {
                sb.append(",<br>&nbsp;");
            }
        }
        e(sb.toString());
    }

    public final void g(String str) {
        c("<code><br><br><div style=\"font-size:small;font-weight:bold;\">&nbsp;" + a(2, str) + "</div></code><br><br></td>");
    }

    public final void h(String str) {
        b("<td><p>" + str + "</p></td>");
    }

    public final void i(String str) {
        d("<td><code><em>" + a(4, str) + "</em></code><br>");
        if (str.startsWith("java.time")) {
            a("<a href=\"#java-time-customizations\">See customizations</a><br");
        } else if (str.startsWith("java.nio")) {
            a("<a href=\"#java-nio-customizations\">See customizations</a><br");
        }
    }

    public final f j(String str) {
        this.c = AbstractC0026a.a(2, 0, this.c);
        b("</" + str + ">");
        return this;
    }

    public final f k(String str) {
        b("<" + str + ">");
        a();
        return this;
    }

    public static String a(int i, String str) {
        List<String> listA = Zf0.a(str, DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
        if (listA.size() < i) {
            return str;
        }
        int i2 = i / 2;
        int length = 0;
        for (int i3 = 0; i3 < i2; i3++) {
            length += listA.get(i3).length();
        }
        int i4 = length + i2;
        return str.substring(0, i4) + "<br>&nbsp;" + str.substring(i4);
    }
}
