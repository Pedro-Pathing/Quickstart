package com.android.tools.r8.ir.optimize.inliner;

import com.android.tools.r8.Diagnostic;
import com.android.tools.r8.origin.Origin;
import com.android.tools.r8.position.Position;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class WhyAreYouNotInliningDiagnostic implements Diagnostic {
    private final Origin b;
    private final String c;

    public WhyAreYouNotInliningDiagnostic(Origin origin, String str) {
        this.b = origin;
        this.c = str;
    }

    @Override // com.android.tools.r8.Diagnostic
    public String getDiagnosticMessage() {
        return this.c;
    }

    @Override // com.android.tools.r8.Diagnostic
    public Origin getOrigin() {
        return this.b;
    }

    @Override // com.android.tools.r8.Diagnostic
    public Position getPosition() {
        return Position.UNKNOWN;
    }
}
