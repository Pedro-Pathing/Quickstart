package com.sun.tools.javac.nio;

import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.util.BaseFileManager;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import java.io.File;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.nio.charset.Charset;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.FileVisitOption;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import javax.lang.model.SourceVersion;
import javax.tools.FileObject;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.StandardLocation;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class JavacPathFileManager extends BaseFileManager implements PathFileManager {
    protected FileSystem defaultFileSystem;
    private Map<Path, FileSystem> fileSystems;
    private boolean inited;
    private Map<JavaFileManager.Location, PathsForLocation> pathsForLocation;

    public JavacPathFileManager(Context context, boolean register, Charset charset) {
        super(charset);
        this.inited = false;
        if (register) {
            context.put((Class<JavacPathFileManager>) JavaFileManager.class, this);
        }
        this.pathsForLocation = new HashMap();
        this.fileSystems = new HashMap();
        setContext(context);
    }

    @Override // com.sun.tools.javac.util.BaseFileManager
    public void setContext(Context context) {
        super.setContext(context);
    }

    @Override // com.sun.tools.javac.nio.PathFileManager
    public FileSystem getDefaultFileSystem() {
        if (this.defaultFileSystem == null) {
            this.defaultFileSystem = FileSystems.getDefault();
        }
        return this.defaultFileSystem;
    }

    @Override // com.sun.tools.javac.nio.PathFileManager
    public void setDefaultFileSystem(FileSystem fs) {
        this.defaultFileSystem = fs;
    }

    @Override // javax.tools.JavaFileManager, java.io.Flushable
    public void flush() throws IOException {
        this.contentCache.clear();
    }

    @Override // javax.tools.JavaFileManager, java.io.Closeable, java.lang.AutoCloseable
    public void close() throws IOException {
        for (FileSystem fs : this.fileSystems.values()) {
            fs.close();
        }
    }

    @Override // javax.tools.JavaFileManager
    public ClassLoader getClassLoader(JavaFileManager.Location location) {
        nullCheck(location);
        Iterable<? extends Path> path = getLocation(location);
        if (path == null) {
            return null;
        }
        ListBuffer<URL> lb = new ListBuffer<>();
        for (Path p : path) {
            try {
                lb.append(p.toUri().toURL());
            } catch (MalformedURLException e) {
                throw new AssertionError(e);
            }
        }
        return getClassLoader((URL[]) lb.toArray(new URL[lb.size()]));
    }

    @Override // com.sun.tools.javac.util.BaseFileManager
    public boolean isDefaultBootClassPath() {
        return this.locations.isDefaultBootClassPath();
    }

    @Override // javax.tools.JavaFileManager
    public boolean hasLocation(JavaFileManager.Location location) {
        return getLocation(location) != null;
    }

    @Override // com.sun.tools.javac.nio.PathFileManager
    public Iterable<? extends Path> getLocation(JavaFileManager.Location location) {
        nullCheck(location);
        lazyInitSearchPaths();
        PathsForLocation path = this.pathsForLocation.get(location);
        if (path == null && !this.pathsForLocation.containsKey(location)) {
            setDefaultForLocation(location);
            return this.pathsForLocation.get(location);
        }
        return path;
    }

    private Path getOutputLocation(JavaFileManager.Location location) {
        Iterable<? extends Path> paths = getLocation(location);
        if (paths == null) {
            return null;
        }
        return paths.iterator().next();
    }

    @Override // com.sun.tools.javac.nio.PathFileManager
    public void setLocation(JavaFileManager.Location location, Iterable<? extends Path> searchPath) throws IOException {
        nullCheck(location);
        lazyInitSearchPaths();
        if (searchPath == null) {
            setDefaultForLocation(location);
            return;
        }
        if (location.isOutputLocation()) {
            checkOutputPath(searchPath);
        }
        PathsForLocation pl = new PathsForLocation();
        for (Path p : searchPath) {
            pl.add(p);
        }
        this.pathsForLocation.put(location, pl);
    }

    private void checkOutputPath(Iterable<? extends Path> searchPath) throws IOException {
        Iterator<? extends Path> pathIter = searchPath.iterator();
        if (!pathIter.hasNext()) {
            throw new IllegalArgumentException("empty path for directory");
        }
        Path path = pathIter.next();
        if (pathIter.hasNext()) {
            throw new IllegalArgumentException("path too long for directory");
        }
        if (!isDirectory(path)) {
            throw new IOException(path + ": not a directory");
        }
    }

    private void setDefaultForLocation(JavaFileManager.Location locn) {
        Collection<File> files = null;
        if (locn instanceof StandardLocation) {
            switch ((StandardLocation) locn) {
                case CLASS_PATH:
                    files = this.locations.userClassPath();
                    break;
                case PLATFORM_CLASS_PATH:
                    files = this.locations.bootClassPath();
                    break;
                case SOURCE_PATH:
                    files = this.locations.sourcePath();
                    break;
                case CLASS_OUTPUT:
                    String arg = this.options.get(Option.D);
                    files = arg == null ? null : Collections.singleton(new File(arg));
                    break;
                case SOURCE_OUTPUT:
                    String arg2 = this.options.get(Option.S);
                    files = arg2 == null ? null : Collections.singleton(new File(arg2));
                    break;
            }
        }
        PathsForLocation pl = new PathsForLocation();
        if (files != null) {
            for (File f : files) {
                pl.add(f.toPath());
            }
        }
        if (!pl.isEmpty()) {
            this.pathsForLocation.put(locn, pl);
        }
    }

    private void lazyInitSearchPaths() {
        if (!this.inited) {
            setDefaultForLocation(StandardLocation.PLATFORM_CLASS_PATH);
            setDefaultForLocation(StandardLocation.CLASS_PATH);
            setDefaultForLocation(StandardLocation.SOURCE_PATH);
            this.inited = true;
        }
    }

    private static class PathsForLocation extends LinkedHashSet<Path> {
        private static final long serialVersionUID = 6788510222394486733L;

        private PathsForLocation() {
        }
    }

    @Override // com.sun.tools.javac.nio.PathFileManager
    public Path getPath(FileObject fo) {
        nullCheck(fo);
        if (!(fo instanceof PathFileObject)) {
            throw new IllegalArgumentException();
        }
        return ((PathFileObject) fo).getPath();
    }

    @Override // javax.tools.JavaFileManager
    public boolean isSameFile(FileObject a, FileObject b) {
        nullCheck(a);
        nullCheck(b);
        if (!(a instanceof PathFileObject)) {
            throw new IllegalArgumentException("Not supported: " + a);
        }
        if (!(b instanceof PathFileObject)) {
            throw new IllegalArgumentException("Not supported: " + b);
        }
        return ((PathFileObject) a).isSameFile((PathFileObject) b);
    }

    @Override // javax.tools.JavaFileManager
    public Iterable<JavaFileObject> list(JavaFileManager.Location location, String packageName, Set<JavaFileObject.Kind> kinds, boolean recurse) throws IOException {
        nullCheck(packageName);
        nullCheck((Collection) kinds);
        Iterable<? extends Path> paths = getLocation(location);
        if (paths == null) {
            return List.nil();
        }
        ListBuffer<JavaFileObject> results = new ListBuffer<>();
        for (Path path : paths) {
            list(path, packageName, kinds, recurse, results);
        }
        return results.toList();
    }

    private void list(Path path, String packageName, final Set<JavaFileObject.Kind> kinds, boolean recurse, final ListBuffer<JavaFileObject> results) throws IOException {
        final Path pathDir;
        if (!Files.exists(path, new LinkOption[0])) {
            return;
        }
        if (isDirectory(path)) {
            pathDir = path;
        } else {
            FileSystem fs = getFileSystem(path);
            if (fs == null) {
                return;
            } else {
                pathDir = fs.getRootDirectories().iterator().next();
            }
        }
        String sep = path.getFileSystem().getSeparator();
        Path packageDir = packageName.isEmpty() ? pathDir : pathDir.resolve(packageName.replace(".", sep));
        if (!Files.exists(packageDir, new LinkOption[0])) {
            return;
        }
        int maxDepth = recurse ? Integer.MAX_VALUE : 1;
        Set<FileVisitOption> opts = EnumSet.of(FileVisitOption.FOLLOW_LINKS);
        Files.walkFileTree(packageDir, opts, maxDepth, new SimpleFileVisitor<Path>() { // from class: com.sun.tools.javac.nio.JavacPathFileManager.1
            @Override // java.nio.file.SimpleFileVisitor, java.nio.file.FileVisitor
            public FileVisitResult preVisitDirectory(Path dir, BasicFileAttributes attrs) {
                Path name = dir.getFileName();
                if (name == null || SourceVersion.isIdentifier(name.toString())) {
                    return FileVisitResult.CONTINUE;
                }
                return FileVisitResult.SKIP_SUBTREE;
            }

            @Override // java.nio.file.SimpleFileVisitor, java.nio.file.FileVisitor
            public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) {
                if (attrs.isRegularFile() && kinds.contains(BaseFileManager.getKind(file.getFileName().toString()))) {
                    results.append(PathFileObject.createDirectoryPathFileObject(JavacPathFileManager.this, file, pathDir));
                }
                return FileVisitResult.CONTINUE;
            }
        });
    }

    @Override // com.sun.tools.javac.nio.PathFileManager
    public Iterable<? extends JavaFileObject> getJavaFileObjectsFromPaths(Iterable<? extends Path> paths) {
        ArrayList<PathFileObject> result;
        if (paths instanceof Collection) {
            result = new ArrayList<>(((Collection) paths).size());
        } else {
            result = new ArrayList<>();
        }
        for (Path p : paths) {
            result.add(PathFileObject.createSimplePathFileObject(this, (Path) nullCheck(p)));
        }
        return result;
    }

    @Override // com.sun.tools.javac.nio.PathFileManager
    public Iterable<? extends JavaFileObject> getJavaFileObjects(Path... paths) {
        return getJavaFileObjectsFromPaths(Arrays.asList((Object[]) nullCheck(paths)));
    }

    @Override // javax.tools.JavaFileManager
    public JavaFileObject getJavaFileForInput(JavaFileManager.Location location, String className, JavaFileObject.Kind kind) throws IOException {
        return getFileForInput(location, getRelativePath(className, kind));
    }

    @Override // javax.tools.JavaFileManager
    public FileObject getFileForInput(JavaFileManager.Location location, String packageName, String relativeName) throws IOException {
        return getFileForInput(location, getRelativePath(packageName, relativeName));
    }

    private JavaFileObject getFileForInput(JavaFileManager.Location location, String relativePath) throws IOException {
        for (Path p : getLocation(location)) {
            if (isDirectory(p)) {
                Path f = resolve(p, relativePath);
                if (Files.exists(f, new LinkOption[0])) {
                    return PathFileObject.createDirectoryPathFileObject(this, f, p);
                }
            } else {
                FileSystem fs = getFileSystem(p);
                if (fs != null) {
                    Path file = getPath(fs, relativePath);
                    if (Files.exists(file, new LinkOption[0])) {
                        return PathFileObject.createJarPathFileObject(this, file);
                    }
                } else {
                    continue;
                }
            }
        }
        return null;
    }

    @Override // javax.tools.JavaFileManager
    public JavaFileObject getJavaFileForOutput(JavaFileManager.Location location, String className, JavaFileObject.Kind kind, FileObject sibling) throws IOException {
        return getFileForOutput(location, getRelativePath(className, kind), sibling);
    }

    @Override // javax.tools.JavaFileManager
    public FileObject getFileForOutput(JavaFileManager.Location location, String packageName, String relativeName, FileObject sibling) throws IOException {
        return getFileForOutput(location, getRelativePath(packageName, relativeName), sibling);
    }

    private JavaFileObject getFileForOutput(JavaFileManager.Location location, String relativePath, FileObject sibling) {
        Path dir = getOutputLocation(location);
        if (dir == null) {
            if (location == StandardLocation.CLASS_OUTPUT) {
                Path siblingDir = null;
                if (sibling != null && (sibling instanceof PathFileObject)) {
                    siblingDir = ((PathFileObject) sibling).getPath().getParent();
                }
                return PathFileObject.createSiblingPathFileObject(this, siblingDir.resolve(getBaseName(relativePath)), relativePath);
            }
            if (location == StandardLocation.SOURCE_OUTPUT) {
                dir = getOutputLocation(StandardLocation.CLASS_OUTPUT);
            }
        }
        if (dir != null) {
            Path file = resolve(dir, relativePath);
            return PathFileObject.createDirectoryPathFileObject(this, file, dir);
        }
        Path file2 = getPath(getDefaultFileSystem(), relativePath);
        return PathFileObject.createSimplePathFileObject(this, file2);
    }

    @Override // javax.tools.JavaFileManager
    public String inferBinaryName(JavaFileManager.Location location, JavaFileObject fo) {
        nullCheck(fo);
        Iterable<? extends Path> paths = getLocation(location);
        if (paths == null) {
            return null;
        }
        if (!(fo instanceof PathFileObject)) {
            throw new IllegalArgumentException(fo.getClass().getName());
        }
        return ((PathFileObject) fo).inferBinaryName(paths);
    }

    private FileSystem getFileSystem(Path p) throws IOException {
        FileSystem fs = this.fileSystems.get(p);
        if (fs == null) {
            FileSystem fs2 = FileSystems.newFileSystem(p, (ClassLoader) null);
            this.fileSystems.put(p, fs2);
            return fs2;
        }
        return fs;
    }

    private static String getRelativePath(String className, JavaFileObject.Kind kind) {
        return className.replace(".", OnBotJavaFileSystemUtils.PATH_SEPARATOR) + kind.extension;
    }

    private static String getRelativePath(String packageName, String relativeName) {
        return packageName.isEmpty() ? relativeName : packageName.replace(".", OnBotJavaFileSystemUtils.PATH_SEPARATOR) + OnBotJavaFileSystemUtils.PATH_SEPARATOR + relativeName;
    }

    private static String getBaseName(String relativePath) {
        int lastSep = relativePath.lastIndexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
        return relativePath.substring(lastSep + 1);
    }

    private static boolean isDirectory(Path path) throws IOException {
        BasicFileAttributes attrs = Files.readAttributes(path, (Class<BasicFileAttributes>) BasicFileAttributes.class, new LinkOption[0]);
        return attrs.isDirectory();
    }

    private static Path getPath(FileSystem fs, String relativePath) {
        return fs.getPath(relativePath.replace(OnBotJavaFileSystemUtils.PATH_SEPARATOR, fs.getSeparator()), new String[0]);
    }

    private static Path resolve(Path base, String relativePath) {
        FileSystem fs = base.getFileSystem();
        Path rp = fs.getPath(relativePath.replace(OnBotJavaFileSystemUtils.PATH_SEPARATOR, fs.getSeparator()), new String[0]);
        return base.resolve(rp);
    }
}
