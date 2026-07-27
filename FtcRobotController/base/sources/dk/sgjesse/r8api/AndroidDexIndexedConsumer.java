package dk.sgjesse.r8api;

import com.android.tools.r8.ByteDataView;
import com.android.tools.r8.DexIndexedConsumer;
import com.android.tools.r8.DiagnosticsHandler;
import com.android.tools.r8.utils.ExceptionDiagnostic;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Map;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

/* JADX INFO: loaded from: classes.dex */
public class AndroidDexIndexedConsumer extends DexIndexedConsumer.ForwardingConsumer {
    private final File destination;
    private final SortedMap<String, ByteDataView> dexFiles;

    public AndroidDexIndexedConsumer(File destination) {
        super(null);
        this.dexFiles = new TreeMap();
        this.destination = destination;
    }

    @Override // com.android.tools.r8.DexIndexedConsumer.ForwardingConsumer, com.android.tools.r8.ProgramConsumer, com.android.tools.r8.DataResourceConsumer
    public void finished(DiagnosticsHandler handler) {
        try {
            ZipOutputStream stream = new ZipOutputStream(new BufferedOutputStream(new FileOutputStream(this.destination)));
            if (this.dexFiles.isEmpty()) {
                stream.putNextEntry(new ZipEntry("README.txt"));
                stream.write("Created by FTC SDK.".getBytes());
                stream.closeEntry();
            }
            for (Map.Entry<String, ByteDataView> entry : this.dexFiles.entrySet()) {
                ZipUtils.writeToZipStream(stream, entry.getKey(), entry.getValue(), 8);
            }
            stream.close();
        } catch (IOException e) {
            handler.error(new ExceptionDiagnostic(e));
        }
    }

    @Override // com.android.tools.r8.DexIndexedConsumer.ForwardingConsumer, com.android.tools.r8.DexIndexedConsumer
    public void accept(int fileIndex, ByteDataView data, Set<String> descriptors, DiagnosticsHandler handler) {
        String name = getDefaultDexFileName(fileIndex);
        this.dexFiles.put(name, ByteDataView.of(data.copyByteData()));
    }

    private String getDefaultDexFileName(int fileIndex) {
        return "classes" + (fileIndex == 0 ? "" : Integer.valueOf(fileIndex + 1)) + FileUtils.DEX_EXTENSION;
    }
}
