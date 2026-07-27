package dk.sgjesse.d8onandroid;

import com.android.tools.r8.Diagnostic;
import com.android.tools.r8.DiagnosticsHandler;

/* JADX INFO: loaded from: classes.dex */
public class D8DiagnosticsHandler implements DiagnosticsHandler {
    @Override // com.android.tools.r8.DiagnosticsHandler
    public void error(Diagnostic diagnostic) {
        System.out.println("ERROR: " + diagnostic.getDiagnosticMessage());
    }

    @Override // com.android.tools.r8.DiagnosticsHandler
    public void warning(Diagnostic diagnostic) {
        System.out.println("WARNING: " + diagnostic.getDiagnosticMessage());
    }

    @Override // com.android.tools.r8.DiagnosticsHandler
    public void info(Diagnostic diagnostic) {
        System.out.println("INFO: " + diagnostic.getDiagnosticMessage());
    }
}
