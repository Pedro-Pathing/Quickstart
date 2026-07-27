package com.sun.tools.javac.file;

import com.sun.tools.javac.util.Context;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/* JADX INFO: loaded from: classes.dex */
public class CacheFSInfo extends FSInfo {
    private Map<File, Entry> cache = new ConcurrentHashMap();

    public static void preRegister(Context context) {
        context.put(FSInfo.class, (Context.Factory) new Context.Factory<FSInfo>() { // from class: com.sun.tools.javac.file.CacheFSInfo.1
            /* JADX WARN: Can't rename method to resolve collision */
            @Override // com.sun.tools.javac.util.Context.Factory
            public FSInfo make(Context c) {
                CacheFSInfo cacheFSInfo = new CacheFSInfo();
                c.put((Class<CacheFSInfo>) FSInfo.class, cacheFSInfo);
                return cacheFSInfo;
            }
        });
    }

    public void clearCache() {
        this.cache.clear();
    }

    @Override // com.sun.tools.javac.file.FSInfo
    public File getCanonicalFile(File file) {
        Entry e = getEntry(file);
        return e.canonicalFile;
    }

    @Override // com.sun.tools.javac.file.FSInfo
    public boolean exists(File file) {
        Entry e = getEntry(file);
        return e.exists;
    }

    @Override // com.sun.tools.javac.file.FSInfo
    public boolean isDirectory(File file) {
        Entry e = getEntry(file);
        return e.isDirectory;
    }

    @Override // com.sun.tools.javac.file.FSInfo
    public boolean isFile(File file) {
        Entry e = getEntry(file);
        return e.isFile;
    }

    @Override // com.sun.tools.javac.file.FSInfo
    public List<File> getJarClassPath(File file) throws IOException {
        Entry e = getEntry(file);
        if (e.jarClassPath == null) {
            e.jarClassPath = super.getJarClassPath(file);
        }
        return e.jarClassPath;
    }

    private Entry getEntry(File file) {
        Entry e = this.cache.get(file);
        if (e == null) {
            Entry e2 = new Entry();
            e2.canonicalFile = super.getCanonicalFile(file);
            e2.exists = super.exists(file);
            e2.isDirectory = super.isDirectory(file);
            e2.isFile = super.isFile(file);
            this.cache.put(file, e2);
            return e2;
        }
        return e;
    }

    private static class Entry {
        File canonicalFile;
        boolean exists;
        boolean isDirectory;
        boolean isFile;
        List<File> jarClassPath;

        private Entry() {
        }
    }
}
