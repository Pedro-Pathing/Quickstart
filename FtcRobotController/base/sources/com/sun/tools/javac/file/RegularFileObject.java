package com.sun.tools.javac.file;

import dk.sgjesse.r8api.DescriptorUtils;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.lang.ref.Reference;
import java.lang.ref.SoftReference;
import java.net.URI;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.charset.CharsetDecoder;
import java.text.Normalizer;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
class RegularFileObject extends BaseFileObject {
    static final boolean isMacOS = System.getProperty("os.name", "").contains("OS X");
    private Reference<File> absFileRef;
    final File file;
    private boolean hasParents;
    private String name;

    public RegularFileObject(JavacFileManager fileManager, File f) {
        this(fileManager, f.getName(), f);
    }

    public RegularFileObject(JavacFileManager fileManager, String name, File f) {
        super(fileManager);
        this.hasParents = false;
        if (f.isDirectory()) {
            throw new IllegalArgumentException("directories not supported");
        }
        this.name = name;
        this.file = f;
    }

    @Override // javax.tools.FileObject
    public URI toUri() {
        return this.file.toURI().normalize();
    }

    @Override // javax.tools.FileObject
    public String getName() {
        return this.file.getPath();
    }

    @Override // com.sun.tools.javac.file.BaseFileObject
    public String getShortName() {
        return this.name;
    }

    @Override // javax.tools.JavaFileObject
    public JavaFileObject.Kind getKind() {
        return getKind(this.name);
    }

    @Override // javax.tools.FileObject
    public InputStream openInputStream() throws IOException {
        return new FileInputStream(this.file);
    }

    @Override // javax.tools.FileObject
    public OutputStream openOutputStream() throws IOException {
        this.fileManager.flushCache(this);
        ensureParentDirectoriesExist();
        return new FileOutputStream(this.file);
    }

    @Override // javax.tools.FileObject
    public CharBuffer getCharContent(boolean ignoreEncodingErrors) throws IOException {
        CharBuffer cb = this.fileManager.getCachedContent(this);
        if (cb == null) {
            InputStream in = new FileInputStream(this.file);
            try {
                ByteBuffer bb = this.fileManager.makeByteBuffer(in);
                JavaFileObject prev = this.fileManager.log.useSource(this);
                try {
                    cb = this.fileManager.decode(bb, ignoreEncodingErrors);
                    this.fileManager.log.useSource(prev);
                    this.fileManager.recycleByteBuffer(bb);
                    if (!ignoreEncodingErrors) {
                        this.fileManager.cache(this, cb);
                    }
                } catch (Throwable th) {
                    this.fileManager.log.useSource(prev);
                    throw th;
                }
            } finally {
                in.close();
            }
        }
        return cb;
    }

    @Override // javax.tools.FileObject
    public Writer openWriter() throws IOException {
        this.fileManager.flushCache(this);
        ensureParentDirectoriesExist();
        return new OutputStreamWriter(new FileOutputStream(this.file), this.fileManager.getEncodingName());
    }

    @Override // javax.tools.FileObject
    public long getLastModified() {
        return this.file.lastModified();
    }

    @Override // javax.tools.FileObject
    public boolean delete() {
        return this.file.delete();
    }

    @Override // com.sun.tools.javac.file.BaseFileObject
    protected CharsetDecoder getDecoder(boolean ignoreEncodingErrors) {
        return this.fileManager.getDecoder(this.fileManager.getEncodingName(), ignoreEncodingErrors);
    }

    @Override // com.sun.tools.javac.file.BaseFileObject
    protected String inferBinaryName(Iterable<? extends File> path) {
        String dPath;
        String fPath = this.file.getPath();
        for (File dir : path) {
            String dPath2 = dir.getPath();
            if (dPath2.length() == 0) {
                dPath2 = System.getProperty("user.dir");
            }
            if (dPath2.endsWith(File.separator)) {
                dPath = dPath2;
            } else {
                dPath = dPath2 + File.separator;
            }
            if (fPath.regionMatches(true, 0, dPath, 0, dPath.length()) && new File(fPath.substring(0, dPath.length())).equals(new File(dPath))) {
                String relativeName = fPath.substring(dPath.length());
                return removeExtension(relativeName).replace(File.separatorChar, DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
            }
        }
        return null;
    }

    @Override // javax.tools.JavaFileObject
    public boolean isNameCompatible(String cn, JavaFileObject.Kind kind) {
        cn.getClass();
        if (kind == JavaFileObject.Kind.OTHER && getKind() != kind) {
            return false;
        }
        String n = cn + kind.extension;
        if (this.name.equals(n)) {
            return true;
        }
        if (isMacOS && Normalizer.isNormalized(this.name, Normalizer.Form.NFD) && Normalizer.isNormalized(n, Normalizer.Form.NFC)) {
            String normName = Normalizer.normalize(this.name, Normalizer.Form.NFC);
            if (normName.equals(n)) {
                this.name = normName;
                return true;
            }
        }
        if (this.name.equalsIgnoreCase(n)) {
            try {
                return this.file.getCanonicalFile().getName().equals(n);
            } catch (IOException e) {
            }
        }
        return false;
    }

    private void ensureParentDirectoriesExist() throws IOException {
        if (!this.hasParents) {
            File parent = this.file.getParentFile();
            if (parent != null && !parent.exists() && !parent.mkdirs() && (!parent.exists() || !parent.isDirectory())) {
                throw new IOException("could not create parent directories");
            }
            this.hasParents = true;
        }
    }

    @Override // com.sun.tools.javac.file.BaseFileObject
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (!(other instanceof RegularFileObject)) {
            return false;
        }
        RegularFileObject o = (RegularFileObject) other;
        return getAbsoluteFile().equals(o.getAbsoluteFile());
    }

    @Override // com.sun.tools.javac.file.BaseFileObject
    public int hashCode() {
        return getAbsoluteFile().hashCode();
    }

    private File getAbsoluteFile() {
        File absFile = this.absFileRef == null ? null : this.absFileRef.get();
        if (absFile == null) {
            File absFile2 = this.file.getAbsoluteFile();
            this.absFileRef = new SoftReference(absFile2);
            return absFile2;
        }
        return absFile;
    }
}
