package com.sun.tools.javac.file;

import com.android.tools.r8.DataResource;
import com.sun.tools.javac.file.JavacFileManager;
import com.sun.tools.javac.file.RelativePath;
import com.sun.tools.javac.util.List;
import dk.sgjesse.r8api.DescriptorUtils;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.Writer;
import java.lang.ref.Reference;
import java.lang.ref.SoftReference;
import java.net.URI;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.charset.CharsetDecoder;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class ZipArchive implements JavacFileManager.Archive {
    protected Reference<File> absFileRef;
    protected JavacFileManager fileManager;
    protected final Map<RelativePath.RelativeDirectory, List<String>> map;
    protected final ZipFile zfile;

    public ZipArchive(JavacFileManager fm, ZipFile zfile) throws IOException {
        this(fm, zfile, true);
    }

    protected ZipArchive(JavacFileManager fm, ZipFile zfile, boolean initMap) throws IOException {
        this.fileManager = fm;
        this.zfile = zfile;
        this.map = new HashMap();
        if (initMap) {
            initMap();
        }
    }

    protected void initMap() throws IOException {
        Enumeration<? extends ZipEntry> e = this.zfile.entries();
        while (e.hasMoreElements()) {
            try {
                ZipEntry entry = e.nextElement();
                addZipEntry(entry);
            } catch (InternalError ex) {
                IOException io = new IOException();
                io.initCause(ex);
                throw io;
            }
        }
    }

    void addZipEntry(ZipEntry entry) {
        String name = entry.getName();
        int i = name.lastIndexOf(47);
        RelativePath.RelativeDirectory dirname = new RelativePath.RelativeDirectory(name.substring(0, i + 1));
        String basename = name.substring(i + 1);
        if (basename.length() == 0) {
            return;
        }
        List<String> list = this.map.get(dirname);
        if (list == null) {
            list = List.nil();
        }
        this.map.put(dirname, list.prepend(basename));
    }

    @Override // com.sun.tools.javac.file.JavacFileManager.Archive
    public boolean contains(RelativePath name) {
        List<String> list;
        RelativePath.RelativeDirectory dirname = name.dirname();
        String basename = name.basename();
        return (basename.length() == 0 || (list = this.map.get(dirname)) == null || !list.contains(basename)) ? false : true;
    }

    @Override // com.sun.tools.javac.file.JavacFileManager.Archive
    public List<String> getFiles(RelativePath.RelativeDirectory subdirectory) {
        return this.map.get(subdirectory);
    }

    @Override // com.sun.tools.javac.file.JavacFileManager.Archive
    public JavaFileObject getFileObject(RelativePath.RelativeDirectory subdirectory, String file) {
        ZipEntry ze = new RelativePath.RelativeFile(subdirectory, file).getZipEntry(this.zfile);
        return new ZipFileObject(this, file, ze);
    }

    @Override // com.sun.tools.javac.file.JavacFileManager.Archive
    public Set<RelativePath.RelativeDirectory> getSubdirectories() {
        return this.map.keySet();
    }

    @Override // com.sun.tools.javac.file.JavacFileManager.Archive
    public void close() throws IOException {
        this.zfile.close();
    }

    public String toString() {
        return "ZipArchive[" + this.zfile.getName() + "]";
    }

    /* JADX INFO: Access modifiers changed from: private */
    public File getAbsoluteFile() {
        File absFile = this.absFileRef == null ? null : this.absFileRef.get();
        if (absFile == null) {
            File absFile2 = new File(this.zfile.getName()).getAbsoluteFile();
            this.absFileRef = new SoftReference(absFile2);
            return absFile2;
        }
        return absFile;
    }

    public static class ZipFileObject extends BaseFileObject {
        ZipEntry entry;
        private String name;
        ZipArchive zarch;

        protected ZipFileObject(ZipArchive zarch, String name, ZipEntry entry) {
            super(zarch.fileManager);
            this.zarch = zarch;
            this.name = name;
            this.entry = entry;
        }

        @Override // javax.tools.FileObject
        public URI toUri() {
            File zipFile = new File(this.zarch.zfile.getName());
            return createJarUri(zipFile, this.entry.getName());
        }

        @Override // javax.tools.FileObject
        public String getName() {
            return this.zarch.zfile.getName() + "(" + this.entry.getName() + ")";
        }

        @Override // com.sun.tools.javac.file.BaseFileObject
        public String getShortName() {
            return new File(this.zarch.zfile.getName()).getName() + "(" + this.entry + ")";
        }

        @Override // javax.tools.JavaFileObject
        public JavaFileObject.Kind getKind() {
            return getKind(this.entry.getName());
        }

        @Override // javax.tools.FileObject
        public InputStream openInputStream() throws IOException {
            return this.zarch.zfile.getInputStream(this.entry);
        }

        @Override // javax.tools.FileObject
        public OutputStream openOutputStream() throws IOException {
            throw new UnsupportedOperationException();
        }

        @Override // javax.tools.FileObject
        public CharBuffer getCharContent(boolean ignoreEncodingErrors) throws IOException {
            CharBuffer cb = this.fileManager.getCachedContent(this);
            if (cb == null) {
                InputStream in = this.zarch.zfile.getInputStream(this.entry);
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
            throw new UnsupportedOperationException();
        }

        @Override // javax.tools.FileObject
        public long getLastModified() {
            return this.entry.getTime();
        }

        @Override // javax.tools.FileObject
        public boolean delete() {
            throw new UnsupportedOperationException();
        }

        @Override // com.sun.tools.javac.file.BaseFileObject
        protected CharsetDecoder getDecoder(boolean ignoreEncodingErrors) {
            return this.fileManager.getDecoder(this.fileManager.getEncodingName(), ignoreEncodingErrors);
        }

        @Override // com.sun.tools.javac.file.BaseFileObject
        protected String inferBinaryName(Iterable<? extends File> path) {
            String entryName = this.entry.getName();
            return removeExtension(entryName).replace(DataResource.SEPARATOR, DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
        }

        @Override // javax.tools.JavaFileObject
        public boolean isNameCompatible(String cn, JavaFileObject.Kind k) {
            cn.getClass();
            if (k == JavaFileObject.Kind.OTHER && getKind() != k) {
                return false;
            }
            return this.name.equals(cn + k.extension);
        }

        @Override // com.sun.tools.javac.file.BaseFileObject
        public boolean equals(Object other) {
            if (this == other) {
                return true;
            }
            if (!(other instanceof ZipFileObject)) {
                return false;
            }
            ZipFileObject o = (ZipFileObject) other;
            return this.zarch.getAbsoluteFile().equals(o.zarch.getAbsoluteFile()) && this.name.equals(o.name);
        }

        @Override // com.sun.tools.javac.file.BaseFileObject
        public int hashCode() {
            return this.zarch.getAbsoluteFile().hashCode() + this.name.hashCode();
        }
    }
}
