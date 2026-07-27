package dk.sgjesse.r8api;

import com.android.tools.r8.ProgramResource;
import com.android.tools.r8.ProgramResourceProvider;
import com.android.tools.r8.ResourceException;
import com.android.tools.r8.origin.ArchiveEntryOrigin;
import com.android.tools.r8.origin.Origin;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Enumeration;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

/* JADX INFO: loaded from: classes.dex */
public class ArchiveProgramResourceProvider implements ProgramResourceProvider {
    private final File archive;
    private final Origin origin;

    public ArchiveProgramResourceProvider(File archive) {
        this.archive = archive;
        this.origin = new FileOrigin(archive);
    }

    @Override // com.android.tools.r8.ProgramResourceProvider
    public Collection<ProgramResource> getProgramResources() throws ResourceException {
        List<ProgramResource> programResources = new ArrayList<>();
        try {
            ZipFile zipFile = new ZipFile(this.archive);
            try {
                Enumeration<? extends ZipEntry> entries = zipFile.entries();
                while (entries.hasMoreElements()) {
                    ZipEntry entry = entries.nextElement();
                    String name = entry.getName();
                    if (ZipUtils.isClassFile(name)) {
                        Origin entryOrigin = new ArchiveEntryOrigin(name, this.origin);
                        InputStream stream = zipFile.getInputStream(entry);
                        try {
                            String descriptor = DescriptorUtils.guessTypeDescriptor(name);
                            programResources.add(ProgramResource.fromBytes(entryOrigin, ProgramResource.Kind.CF, ZipUtils.toByteArray(stream), Collections.singleton(descriptor)));
                            if (stream != null) {
                                stream.close();
                            }
                        } finally {
                        }
                    }
                }
                zipFile.close();
                return programResources;
            } finally {
            }
        } catch (IOException e) {
            throw new ResourceException(this.origin, e);
        }
    }
}
