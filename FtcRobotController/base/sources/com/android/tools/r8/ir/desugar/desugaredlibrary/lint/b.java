package com.android.tools.r8.ir.desugar.desugaredlibrary.lint;

import com.android.tools.r8.DiagnosticsHandler;
import com.android.tools.r8.StringConsumer;
import java.nio.file.Path;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class b extends StringConsumer.FileConsumer {
    public b(Path path) {
        super(path);
    }

    @Override // com.android.tools.r8.StringConsumer.FileConsumer, com.android.tools.r8.StringConsumer.ForwardingConsumer, com.android.tools.r8.StringConsumer
    public final void accept(String str, DiagnosticsHandler diagnosticsHandler) {
        super.accept(str, diagnosticsHandler);
        super.accept(System.lineSeparator(), diagnosticsHandler);
    }
}
