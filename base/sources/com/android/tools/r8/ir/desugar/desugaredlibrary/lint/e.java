package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.ArchiveClassFileProvider;
import com.android.tools.r8.ArchiveProgramResourceProvider;
import com.android.tools.r8.ClassFileResourceProvider;
import com.android.tools.r8.ProgramResourceProvider;
import com.android.tools.r8.graph.C0257g1;
import com.android.tools.r8.graph.C0291j1;
import com.android.tools.r8.graph.C0409w2;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.C1015Tp;
import com.android.tools.r8.internal.C1116Xj;
import com.android.tools.r8.internal.C3384x50;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.Zf0;
import com.android.tools.r8.ir.desugar.desugaredlibrary.lint.m;
import com.android.tools.r8.t0;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.FileAttribute;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class e extends a {
    static final /* synthetic */ boolean i = true;

    public e(C3384x50 c3384x50, t0 t0Var, Collection<ProgramResourceProvider> collection, Path path, Collection<ClassFileResourceProvider> collection2) {
        super(c3384x50, t0Var, collection, path, collection2);
    }

    private static String a(EnumC3471y2 enumC3471y2, EnumC3471y2 enumC3471y22) {
        return "desugared_apis_" + enumC3471y2.d() + "_" + enumC3471y22.d();
    }

    private Path b(EnumC3471y2 enumC3471y2, EnumC3471y2 enumC3471y22) throws IOException {
        Path pathResolve = this.e.resolve("compile_api_level_" + enumC3471y2.d());
        Files.createDirectories(pathResolve, new FileAttribute[0]);
        return Paths.get(pathResolve + File.separator + a(enumC3471y2, enumC3471y22) + ".txt", new String[0]);
    }

    public static void main(String[] strArr) throws Exception {
        if (strArr.length != 4) {
            throw new RuntimeException(Zf0.a("Invalid invocation.", "Usage: GenerateDesugaredLibraryLintFiles <desugar configuration> <desugar implementation> <output directory> <android jar path for Android " + a.g + " or higher>"));
        }
        new e(new C3384x50(), t0.a(Paths.get(strArr[0], new String[0])), AbstractC0695Iu.c(ArchiveProgramResourceProvider.fromArchive(Paths.get(strArr[1], new String[0]))), Paths.get(strArr[2], new String[0]), AbstractC0695Iu.c(new ArchiveClassFileProvider(Paths.get(strArr[3], new String[0])))).run();
    }

    public EnumC3471y2 run() throws Exception {
        EnumC3471y2 enumC3471y2C = this.b.c();
        m mVarB = new o(this.a, this.f, true).b(this.d, this.c);
        a(enumC3471y2C, EnumC3471y2.c, mVarB);
        a(enumC3471y2C, EnumC3471y2.w, mVarB);
        return enumC3471y2C;
    }

    final void a(EnumC3471y2 enumC3471y2, final EnumC3471y2 enumC3471y22, m mVar) throws IOException {
        final ArrayList arrayList = new ArrayList();
        mVar.a(new Consumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.e$$ExternalSyntheticLambda2
            @Override // java.util.function.Consumer
            public final void accept(Object obj) {
                this.f$0.a(enumC3471y22, arrayList, (m.d) obj);
            }
        });
        for (C0409w2 c0409w2 : mVar.a()) {
            arrayList.add(C1116Xj.i(c0409w2.w0().f.toString()) + "#" + c0409w2.g + c0409w2.i.s0());
        }
        arrayList.sort(Comparator.naturalOrder());
        a(enumC3471y2, enumC3471y22, arrayList);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void a(EnumC3471y2 enumC3471y2, List list, String str, C0291j1 c0291j1, m.c cVar) {
        if (c0291j1.r1() || c0291j1.n1()) {
            return;
        }
        if (cVar != null) {
            if (cVar.a) {
                return;
            }
            if (cVar.e) {
                if (enumC3471y2 != EnumC3471y2.w) {
                    return;
                }
            } else if (!i && !cVar.f) {
                throw new AssertionError();
            }
        }
        list.add(str + "#" + c0291j1.getReference().g + c0291j1.getReference().i.s0());
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void a(final EnumC3471y2 enumC3471y2, final List list, m.d dVar) {
        final String strI = C1116Xj.i(dVar.a.e.f.toString());
        if (!dVar.b.b) {
            dVar.b(new BiConsumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.e$$ExternalSyntheticLambda0
                @Override // java.util.function.BiConsumer
                public final void accept(Object obj, Object obj2) {
                    this.f$0.a(enumC3471y2, list, strI, (C0291j1) obj, (m.c) obj2);
                }
            });
            dVar.a(new BiConsumer() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.e$$ExternalSyntheticLambda1
                @Override // java.util.function.BiConsumer
                public final void accept(Object obj, Object obj2) {
                    e.a(list, strI, (C0257g1) obj, (m.a) obj2);
                }
            });
        } else {
            list.add(strI);
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static /* synthetic */ void a(List list, String str, C0257g1 c0257g1, m.a aVar) {
        if (aVar == null || !aVar.a) {
            list.add(str + "#" + c0257g1.getReference().g);
        }
    }

    void a(EnumC3471y2 enumC3471y2, EnumC3471y2 enumC3471y22, ArrayList arrayList) throws IOException {
        C1015Tp.a(b(enumC3471y2, enumC3471y22), arrayList);
    }
}
