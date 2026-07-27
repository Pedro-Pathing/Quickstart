package dk.sgjesse.r8api;

import com.android.tools.r8.ClassFileResourceProvider;
import com.android.tools.r8.ProgramResource;
import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/* JADX INFO: loaded from: classes.dex */
public class OrderedClassFileResourceProvider implements ClassFileResourceProvider, Closeable {
    private volatile boolean closed;
    private final List<ClassFileResourceProvider> providers = new ArrayList();
    private final Set<String> descriptors = new HashSet();
    private boolean empty = true;

    public void addClassFileResourceProvider(ClassFileResourceProvider provider) {
        this.providers.add(provider);
        this.descriptors.addAll(provider.getClassDescriptors());
        this.empty = false;
    }

    public boolean isEmpty() {
        return this.empty;
    }

    @Override // com.android.tools.r8.ClassFileResourceProvider
    public Set<String> getClassDescriptors() {
        return this.descriptors;
    }

    @Override // com.android.tools.r8.ClassFileResourceProvider
    public ProgramResource getProgramResource(String descriptor) {
        for (ClassFileResourceProvider provider : this.providers) {
            if (provider.getClassDescriptors().contains(descriptor)) {
                return provider.getProgramResource(descriptor);
            }
        }
        return null;
    }

    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }

    @Override // java.io.Closeable, java.lang.AutoCloseable
    public void close() throws IOException {
        if (!this.closed) {
            this.closed = true;
            for (ClassFileResourceProvider provider : this.providers) {
                if (provider instanceof Closeable) {
                    ((Closeable) provider).close();
                }
            }
            this.providers.clear();
            this.descriptors.clear();
        }
    }
}
