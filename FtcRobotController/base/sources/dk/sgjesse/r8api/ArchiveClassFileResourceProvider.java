package dk.sgjesse.r8api;

import com.android.tools.r8.ClassFileResourceProvider;
import com.android.tools.r8.ProgramResource;
import com.android.tools.r8.origin.ArchiveEntryOrigin;
import com.android.tools.r8.origin.Origin;
import java.io.Closeable;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashSet;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

/* JADX INFO: loaded from: classes.dex */
public class ArchiveClassFileResourceProvider implements ClassFileResourceProvider, Closeable {
    private final File archive;
    private volatile boolean closed;
    private final Set<String> descriptors = new HashSet();
    private final Origin origin;
    private final ZipFile zipFile;

    public ArchiveClassFileResourceProvider(File archive) throws IOException {
        this.archive = archive;
        this.origin = new FileOrigin(archive);
        this.zipFile = new ZipFile(archive);
        Enumeration<? extends ZipEntry> entries = this.zipFile.entries();
        while (entries.hasMoreElements()) {
            ZipEntry entry = entries.nextElement();
            String entryName = entry.getName();
            if (ZipUtils.isClassFile(entryName)) {
                this.descriptors.add(DescriptorUtils.guessTypeDescriptor(entryName));
            }
        }
    }

    @Override // com.android.tools.r8.ClassFileResourceProvider
    public Set<String> getClassDescriptors() {
        return Collections.unmodifiableSet(this.descriptors);
    }

    @Override // com.android.tools.r8.ClassFileResourceProvider
    public ProgramResource getProgramResource(String descriptor) {
        if (!this.descriptors.contains(descriptor)) {
            return null;
        }
        ZipEntry entry = this.zipFile.getEntry(DescriptorUtils.getPathFromDescriptor(descriptor));
        String name = entry.getName();
        Origin entryOrigin = new ArchiveEntryOrigin(name, this.origin);
        try {
            InputStream stream = this.zipFile.getInputStream(entry);
            try {
                ProgramResource programResourceFromBytes = ProgramResource.fromBytes(entryOrigin, ProgramResource.Kind.CF, ZipUtils.toByteArray(stream), Collections.singleton(descriptor));
                if (stream != null) {
                    stream.close();
                }
                return programResourceFromBytes;
            } finally {
            }
        } catch (IOException e) {
            throw new RuntimeException("Failed to read '" + descriptor + " from " + this.archive.getAbsolutePath(), e);
        }
    }

    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }

    @Override // java.io.Closeable, java.lang.AutoCloseable
    public void close() throws IOException {
        if (!this.closed) {
            this.closed = true;
            this.descriptors.clear();
            this.zipFile.close();
        }
    }
}
