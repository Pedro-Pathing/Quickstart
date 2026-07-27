package com.sun.tools.javac.nio;

import com.sun.tools.javac.util.BaseFileManager;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Reader;
import java.io.Writer;
import java.net.URI;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.charset.CharsetDecoder;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.OpenOption;
import java.nio.file.Path;
import java.nio.file.attribute.FileAttribute;
import java.util.Iterator;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.NestingKind;
import javax.tools.JavaFileObject;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
abstract class PathFileObject implements JavaFileObject {
    private JavacPathFileManager fileManager;
    private Path path;

    abstract String inferBinaryName(Iterable<? extends Path> iterable);

    static PathFileObject createDirectoryPathFileObject(JavacPathFileManager fileManager, final Path path, final Path dir) {
        return new PathFileObject(fileManager, path) { // from class: com.sun.tools.javac.nio.PathFileObject.1
            @Override // com.sun.tools.javac.nio.PathFileObject
            String inferBinaryName(Iterable<? extends Path> paths) {
                return toBinaryName(dir.relativize(path));
            }
        };
    }

    static PathFileObject createJarPathFileObject(JavacPathFileManager fileManager, final Path path) {
        return new PathFileObject(fileManager, path) { // from class: com.sun.tools.javac.nio.PathFileObject.2
            @Override // com.sun.tools.javac.nio.PathFileObject
            String inferBinaryName(Iterable<? extends Path> paths) {
                return toBinaryName(path);
            }
        };
    }

    static PathFileObject createSiblingPathFileObject(JavacPathFileManager fileManager, Path path, final String relativePath) {
        return new PathFileObject(fileManager, path) { // from class: com.sun.tools.javac.nio.PathFileObject.3
            @Override // com.sun.tools.javac.nio.PathFileObject
            String inferBinaryName(Iterable<? extends Path> paths) {
                return toBinaryName(relativePath, OnBotJavaFileSystemUtils.PATH_SEPARATOR);
            }
        };
    }

    static PathFileObject createSimplePathFileObject(JavacPathFileManager fileManager, final Path path) {
        return new PathFileObject(fileManager, path) { // from class: com.sun.tools.javac.nio.PathFileObject.4
            @Override // com.sun.tools.javac.nio.PathFileObject
            String inferBinaryName(Iterable<? extends Path> paths) {
                Path absPath = path.toAbsolutePath();
                Iterator<? extends Path> it = paths.iterator();
                while (it.hasNext()) {
                    Path p = it.next();
                    Path ap = p.toAbsolutePath();
                    if (absPath.startsWith(ap)) {
                        try {
                            Path rp = ap.relativize(absPath);
                            if (rp != null) {
                                return toBinaryName(rp);
                            }
                            continue;
                        } catch (IllegalArgumentException e) {
                        }
                    }
                }
                return null;
            }
        };
    }

    protected PathFileObject(JavacPathFileManager fileManager, Path path) {
        fileManager.getClass();
        path.getClass();
        this.fileManager = fileManager;
        this.path = path;
    }

    Path getPath() {
        return this.path;
    }

    @Override // javax.tools.JavaFileObject
    public JavaFileObject.Kind getKind() {
        return BaseFileManager.getKind(this.path.getFileName().toString());
    }

    @Override // javax.tools.JavaFileObject
    public boolean isNameCompatible(String simpleName, JavaFileObject.Kind kind) {
        simpleName.getClass();
        if (kind == JavaFileObject.Kind.OTHER && getKind() != kind) {
            return false;
        }
        String sn = simpleName + kind.extension;
        String pn = this.path.getFileName().toString();
        if (pn.equals(sn)) {
            return true;
        }
        if (pn.equalsIgnoreCase(sn)) {
            try {
                return this.path.toRealPath(LinkOption.NOFOLLOW_LINKS).getFileName().toString().equals(sn);
            } catch (IOException e) {
            }
        }
        return false;
    }

    @Override // javax.tools.JavaFileObject
    public NestingKind getNestingKind() {
        return null;
    }

    @Override // javax.tools.JavaFileObject
    public Modifier getAccessLevel() {
        return null;
    }

    @Override // javax.tools.FileObject
    public URI toUri() {
        return this.path.toUri();
    }

    @Override // javax.tools.FileObject
    public String getName() {
        return this.path.toString();
    }

    @Override // javax.tools.FileObject
    public InputStream openInputStream() throws IOException {
        return Files.newInputStream(this.path, new OpenOption[0]);
    }

    @Override // javax.tools.FileObject
    public OutputStream openOutputStream() throws IOException {
        this.fileManager.flushCache(this);
        ensureParentDirectoriesExist();
        return Files.newOutputStream(this.path, new OpenOption[0]);
    }

    @Override // javax.tools.FileObject
    public Reader openReader(boolean ignoreEncodingErrors) throws IOException {
        CharsetDecoder decoder = this.fileManager.getDecoder(this.fileManager.getEncodingName(), ignoreEncodingErrors);
        return new InputStreamReader(openInputStream(), decoder);
    }

    @Override // javax.tools.FileObject
    public CharSequence getCharContent(boolean ignoreEncodingErrors) throws IOException {
        CharBuffer cb = this.fileManager.getCachedContent(this);
        if (cb == null) {
            InputStream in = openInputStream();
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
        return new OutputStreamWriter(Files.newOutputStream(this.path, new OpenOption[0]), this.fileManager.getEncodingName());
    }

    @Override // javax.tools.FileObject
    public long getLastModified() {
        try {
            return Files.getLastModifiedTime(this.path, new LinkOption[0]).toMillis();
        } catch (IOException e) {
            return -1L;
        }
    }

    @Override // javax.tools.FileObject
    public boolean delete() {
        try {
            Files.delete(this.path);
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    public boolean isSameFile(PathFileObject other) {
        try {
            return Files.isSameFile(this.path, other.path);
        } catch (IOException e) {
            return false;
        }
    }

    public boolean equals(Object other) {
        return (other instanceof PathFileObject) && this.path.equals(((PathFileObject) other).path);
    }

    public int hashCode() {
        return this.path.hashCode();
    }

    public String toString() {
        return getClass().getSimpleName() + "[" + this.path + "]";
    }

    private void ensureParentDirectoriesExist() throws IOException {
        Path parent = this.path.getParent();
        if (parent != null) {
            Files.createDirectories(parent, new FileAttribute[0]);
        }
    }

    private long size() {
        try {
            return Files.size(this.path);
        } catch (IOException e) {
            return -1L;
        }
    }

    protected static String toBinaryName(Path relativePath) {
        return toBinaryName(relativePath.toString(), relativePath.getFileSystem().getSeparator());
    }

    protected static String toBinaryName(String relativePath, String sep) {
        return removeExtension(relativePath).replace(sep, ".");
    }

    protected static String removeExtension(String fileName) {
        int lastDot = fileName.lastIndexOf(".");
        return lastDot == -1 ? fileName : fileName.substring(0, lastDot);
    }
}
