package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.ClassFileResourceProvider;
import com.android.tools.r8.DiagnosticsHandler;
import com.android.tools.r8.ParseFlagPrinter;
import com.android.tools.r8.ProgramResourceProvider;
import com.android.tools.r8.StringConsumer;
import com.android.tools.r8.a0;
import com.android.tools.r8.internal.AbstractC0695Iu;
import com.android.tools.r8.internal.C3384x50;
import com.android.tools.r8.internal.EnumC3471y2;
import com.android.tools.r8.internal.Zf0;
import com.android.tools.r8.t0;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class DesugaredMethodsListCommand {
    private final boolean a;
    private final boolean b;
    private final int c;
    private final C3384x50 d;
    private final t0 e;
    private final Collection f;
    private final StringConsumer g;
    private final Collection h;
    private final boolean i;

    /* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
    public static class Builder {
        private final C3384x50 b;
        private StringConsumer e;
        private int a = EnumC3471y2.c.d();
        private t0 c = null;
        private final ArrayList d = new ArrayList();
        private final ArrayList f = new ArrayList();
        private boolean g = false;
        private boolean h = false;
        private boolean i = false;

        public Builder(DiagnosticsHandler diagnosticsHandler) {
            this.b = new C3384x50(diagnosticsHandler);
        }

        public Builder addDesugarLibraryImplementation(ProgramResourceProvider programResourceProvider) {
            this.d.add(programResourceProvider);
            return this;
        }

        public Builder addLibrary(ClassFileResourceProvider classFileResourceProvider) {
            this.f.add(classFileResourceProvider);
            return this;
        }

        public DesugaredMethodsListCommand build() {
            boolean z = this.g;
            if (z || this.h) {
                return new DesugaredMethodsListCommand(z, this.h);
            }
            if (this.i && !this.d.isEmpty()) {
                this.b.a("With platform build desugared library is not allowed.");
            }
            if (this.c != null && this.f.isEmpty()) {
                this.b.a("With desugared library specification a library is required.");
            }
            if (!this.d.isEmpty() && this.c == null) {
                this.b.a("The desugar library specification is required when desugared library implementation is present.");
            }
            if (this.e == null) {
                this.e = new c();
            }
            return new DesugaredMethodsListCommand(this.a, this.b, this.c, this.d, this.e, this.f, this.i);
        }

        public Builder setAndroidPlatformBuild() {
            this.i = true;
            return this;
        }

        public Builder setDesugarLibrarySpecification(t0 t0Var) {
            this.c = t0Var;
            return this;
        }

        public Builder setHelp() {
            this.g = true;
            return this;
        }

        public Builder setMinApi(int i) {
            this.a = i;
            return this;
        }

        public Builder setOutputConsumer(StringConsumer stringConsumer) {
            this.e = stringConsumer;
            return this;
        }

        public Builder setOutputPath(Path path) {
            this.e = new b(path);
            return this;
        }

        public Builder setVersion() {
            this.h = true;
            return this;
        }
    }

    DesugaredMethodsListCommand(int i, C3384x50 c3384x50, t0 t0Var, ArrayList arrayList, StringConsumer stringConsumer, ArrayList arrayList2, boolean z) {
        this.a = false;
        this.b = false;
        this.c = i;
        this.d = c3384x50;
        this.e = t0Var;
        this.f = arrayList;
        this.g = stringConsumer;
        this.h = arrayList2;
        this.i = z;
    }

    public static Builder builder(DiagnosticsHandler diagnosticsHandler) {
        return new Builder(diagnosticsHandler);
    }

    public static String getUsageMessage() {
        StringBuilder sb = new StringBuilder();
        Zf0.a(sb, "Usage: desugaredmethods [options] where  options are:");
        new ParseFlagPrinter().addFlags(AbstractC0695Iu.a(AbstractC0695Iu.g().a(a0.a("--output", Collections.singletonList("<file>"), Arrays.asList("Output result in <file>.", "<file> must be an existing directory or a zip file."))).a(a0.a("--lib", Collections.singletonList("<file|jdk-home>"), Arrays.asList("Add <file|jdk-home> as a library resource."))).a(a0.e()).a(a0.a("DesugaredMethods")).a(a0.c()).a(a0.a("--desugared-lib", Collections.singletonList("<file>"), Arrays.asList("Specify desugared library configuration.", "<file> is a desugared library configuration (json)."))).a(a0.a("--android-platform-build", Collections.emptyList(), Arrays.asList("Compile as a platform build where the runtime/bootclasspath", "is assumed to be the version specified by --min-api."))).a(a0.a("--desugared-lib-jar", Collections.singletonList("<file>"), Arrays.asList("Specify desugared library jar."))).a())).appendLinesToBuilder(sb);
        return sb.toString();
    }

    public static DesugaredMethodsListCommand parse(String[] strArr) throws IOException {
        return parse(strArr, new C3384x50());
    }

    public Collection<ProgramResourceProvider> getDesugarLibraryImplementation() {
        return this.f;
    }

    public t0 getDesugarLibrarySpecification() {
        return this.e;
    }

    public Collection<ClassFileResourceProvider> getLibrary() {
        return this.h;
    }

    public int getMinApi() {
        return this.c;
    }

    public StringConsumer getOutputConsumer() {
        return this.g;
    }

    public C3384x50 getReporter() {
        return this.d;
    }

    public boolean isAndroidPlatformBuild() {
        return this.i;
    }

    public boolean isHelp() {
        return this.a;
    }

    public boolean isVersion() {
        return this.b;
    }

    public static DesugaredMethodsListCommand parse(String[] strArr, C3384x50 c3384x50) throws IOException {
        return d.a(strArr, c3384x50);
    }

    DesugaredMethodsListCommand(boolean z, boolean z2) {
        this.a = z;
        this.b = z2;
        this.c = -1;
        this.d = null;
        this.e = null;
        this.f = null;
        this.g = null;
        this.h = null;
        this.i = false;
    }
}
