package dk.sgjesse.r8api;

import com.android.tools.r8.origin.Origin;
import java.io.File;

/* JADX INFO: loaded from: classes.dex */
public class FileOrigin extends Origin {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private final File file;

    public FileOrigin(File file) {
        super(root());
        if (file == null) {
            throw new AssertionError();
        }
        this.file = file;
    }

    @Override // com.android.tools.r8.origin.Origin
    public String part() {
        return this.file.toString();
    }
}
