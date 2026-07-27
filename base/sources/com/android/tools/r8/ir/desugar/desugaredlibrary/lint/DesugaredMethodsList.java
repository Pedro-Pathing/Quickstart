package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.CompilationFailedException;
import com.android.tools.r8.StringConsumer;
import com.android.tools.r8.Version;
import com.android.tools.r8.internal.AbstractC3072to;
import com.android.tools.r8.internal.C0767Le;
import com.android.tools.r8.internal.C3384x50;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.InterfaceC3166uo;
import com.android.tools.r8.t0;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class DesugaredMethodsList extends e {
    private final EnumC3471y2 j;
    private final boolean k;
    private final StringConsumer l;

    DesugaredMethodsList(int i, boolean z, C3384x50 c3384x50, t0 t0Var, Collection collection, StringConsumer stringConsumer, Collection collection2) {
        super(c3384x50, t0Var, collection, null, collection2);
        this.j = EnumC3471y2.b(i);
        this.k = z;
        this.l = stringConsumer;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static /* synthetic */ void a(DesugaredMethodsListCommand desugaredMethodsListCommand) throws IOException {
        new DesugaredMethodsList(desugaredMethodsListCommand.getMinApi(), desugaredMethodsListCommand.isAndroidPlatformBuild(), desugaredMethodsListCommand.getReporter(), desugaredMethodsListCommand.getDesugarLibrarySpecification(), desugaredMethodsListCommand.getDesugarLibraryImplementation(), desugaredMethodsListCommand.getOutputConsumer(), desugaredMethodsListCommand.getLibrary()).run();
    }

    public static void main(final String[] strArr) {
        AbstractC3072to.a(new InterfaceC3166uo() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.DesugaredMethodsList$$ExternalSyntheticLambda1
            @Override // com.android.tools.r8.internal.InterfaceC3166uo
            public final void run() throws CompilationFailedException {
                DesugaredMethodsList.a(strArr);
            }
        });
    }

    public static void run(final DesugaredMethodsListCommand desugaredMethodsListCommand) throws CompilationFailedException {
        if (desugaredMethodsListCommand.isHelp()) {
            System.out.println(DesugaredMethodsListCommand.getUsageMessage());
        } else {
            if (!desugaredMethodsListCommand.isVersion()) {
                AbstractC3072to.b(desugaredMethodsListCommand.getReporter(), new AbstractC3072to.a() { // from class: com.android.tools.r8.ir.desugar.desugaredlibrary.lint.DesugaredMethodsList$$ExternalSyntheticLambda0
                    @Override // com.android.tools.r8.internal.AbstractC3072to.a
                    public final void run() throws IOException {
                        DesugaredMethodsList.a(desugaredMethodsListCommand);
                    }
                });
                return;
            }
            System.out.println("DesugaredMethodsList " + Version.getVersionString());
        }
    }

    @Override // com.android.tools.r8.ir.desugar.desugaredlibrary.lint.e
    final void a(EnumC3471y2 enumC3471y2, EnumC3471y2 enumC3471y22, ArrayList arrayList) {
        Iterator it = arrayList.iterator();
        while (it.hasNext()) {
            this.l.accept((String) it.next(), this.a.i);
        }
        this.l.finished(this.a.i);
    }

    @Override // com.android.tools.r8.ir.desugar.desugaredlibrary.lint.e
    public EnumC3471y2 run() throws IOException {
        EnumC3471y2 enumC3471y2C = this.b.c();
        a(enumC3471y2C, this.j, new o(this.a, this.f, true, this.j, this.k, true).b(this.d, this.c));
        return enumC3471y2C;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static /* synthetic */ void a(String[] strArr) throws CompilationFailedException {
        try {
            run(strArr);
        } catch (IOException e) {
            throw new C0767Le(e.getMessage(), e);
        }
    }

    public static void run(String[] strArr) throws IOException, CompilationFailedException {
        run(DesugaredMethodsListCommand.parse(strArr));
    }
}
