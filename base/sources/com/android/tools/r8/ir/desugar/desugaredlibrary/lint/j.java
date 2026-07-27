package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.ArchiveClassFileProvider;
import com.android.tools.r8.ArchiveProgramResourceProvider;
import com.android.tools.r8.ClassFileResourceProvider;
import com.android.tools.r8.ProgramResourceProvider;
import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.C3384x50;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.Zf0;
import com.android.tools.r8.ir.desugar.desugaredlibrary.lint.m;
import com.android.tools.r8.t0;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.OpenOption;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class j extends a {
    public j(t0 t0Var, Collection<ProgramResourceProvider> collection, Path path, Collection<ClassFileResourceProvider> collection2) {
        super(new C3384x50(), t0Var, collection, path, collection2);
    }

    public static void a(PrintStream printStream, m.d dVar) {
        final g gVar = new g(dVar.a.e, dVar.b);
        dVar.a(new BiConsumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.j$$ExternalSyntheticLambda0
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                j.a(gVar, (C0257g1) obj, (m.a) obj2);
            }
        });
        dVar.b(new BiConsumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.j$$ExternalSyntheticLambda1
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                j.a(gVar, (C0291j1) obj, (m.c) obj2);
            }
        });
        printStream.println(gVar);
    }

    public static void main(String[] strArr) throws Exception {
        if (!strArr[0].equals("--generate-api-docs") || strArr.length != 5) {
            throw new RuntimeException(Zf0.a("Invalid invocation.", "Usage: GenerateHtmlDoc --generate-api-docs <desugar configuration> <desugar implementation> <output directory> <android jar path for Android " + a.g + " or higher>"));
        }
        new j(t0.a(Paths.get(strArr[1], new String[0])), AbstractC0695Iu.c(ArchiveProgramResourceProvider.fromArchive(Paths.get(strArr[2], new String[0]))), Paths.get(strArr[3], new String[0]), AbstractC0695Iu.c(new ArchiveClassFileProvider(Paths.get(strArr[4], new String[0])))).a("apis.html");
    }

    public final /* synthetic */ void b(PrintStream printStream, m.d dVar) {
        a(printStream, dVar);
    }

    public static void a(h hVar, C0257g1 c0257g1, m.a aVar) {
        hVar.a.put(c0257g1, aVar);
    }

    public static void a(h hVar, C0291j1 c0291j1, m.c cVar) {
        if (c0291j1.g.K()) {
            return;
        }
        hVar.getClass();
        if (!h.f && c0291j1.n1()) {
            throw new AssertionError();
        }
        if (c0291j1.q1()) {
            hVar.b.put(c0291j1, cVar);
        } else {
            hVar.c.put(c0291j1, cVar);
        }
    }

    public EnumC3471y2 a(String str) throws Exception {
        final PrintStream printStream = new PrintStream(Files.newOutputStream(this.e.resolve(str), new OpenOption[0]));
        new o(this.a, this.f, false).b(this.d, this.c).a(new Consumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.j$$ExternalSyntheticLambda2
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.b(printStream, (m.d) obj);
            }
        });
        return a.g;
    }
}
