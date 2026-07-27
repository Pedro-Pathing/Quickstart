package com.sun.tools.javac.file;

import com.android.tools.r8.DataResource;
import com.sun.tools.javac.file.RelativePath;
import com.sun.tools.javac.file.ZipFileIndex;
import com.sun.tools.javac.util.BaseFileManager;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.UnsupportedEncodingException;
import java.net.MalformedURLException;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.CharBuffer;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import javax.lang.model.SourceVersion;
import javax.tools.FileObject;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.StandardLocation;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class JavacFileManager extends BaseFileManager implements StandardJavaFileManager {
    private static final boolean fileSystemIsCaseSensitive;
    private static final String[] symbolFileLocation;
    private static final RelativePath.RelativeDirectory symbolFilePrefix;
    Map<File, Archive> archives;
    private boolean contextUseOptimizedZip;
    private String defaultEncodingName;
    private FSInfo fsInfo;
    protected boolean mmappedIO;
    protected SortFiles sortFiles;
    private final Set<JavaFileObject.Kind> sourceOrClass;
    protected boolean symbolFileEnabled;
    private ZipFileIndexCache zipFileIndexCache;

    public interface Archive {
        void close() throws IOException;

        boolean contains(RelativePath relativePath);

        JavaFileObject getFileObject(RelativePath.RelativeDirectory relativeDirectory, String str);

        List<String> getFiles(RelativePath.RelativeDirectory relativeDirectory);

        Set<RelativePath.RelativeDirectory> getSubdirectories();
    }

    protected enum SortFiles implements Comparator<File> {
        FORWARD { // from class: com.sun.tools.javac.file.JavacFileManager.SortFiles.1
            @Override // java.util.Comparator
            public int compare(File f1, File f2) {
                return f1.getName().compareTo(f2.getName());
            }
        },
        REVERSE { // from class: com.sun.tools.javac.file.JavacFileManager.SortFiles.2
            @Override // java.util.Comparator
            public int compare(File f1, File f2) {
                return -f1.getName().compareTo(f2.getName());
            }
        }
    }

    public static char[] toArray(CharBuffer buffer) {
        if (buffer.hasArray()) {
            return ((CharBuffer) buffer.compact().flip()).array();
        }
        return buffer.toString().toCharArray();
    }

    public static void preRegister(Context context) {
        context.put(JavaFileManager.class, (Context.Factory) new Context.Factory<JavaFileManager>() { // from class: com.sun.tools.javac.file.JavacFileManager.1
            @Override // com.sun.tools.javac.util.Context.Factory
            public JavaFileManager make(Context c) {
                return new JavacFileManager(c, true, null);
            }
        });
    }

    public JavacFileManager(Context context, boolean register, Charset charset) {
        super(charset);
        this.sourceOrClass = EnumSet.of(JavaFileObject.Kind.SOURCE, JavaFileObject.Kind.CLASS);
        this.archives = new HashMap();
        if (register) {
            context.put((Class<JavacFileManager>) JavaFileManager.class, this);
        }
        setContext(context);
    }

    @Override // com.sun.tools.javac.util.BaseFileManager
    public void setContext(Context context) {
        super.setContext(context);
        this.fsInfo = FSInfo.instance(context);
        this.contextUseOptimizedZip = this.options.getBoolean("useOptimizedZip", true);
        if (this.contextUseOptimizedZip) {
            this.zipFileIndexCache = ZipFileIndexCache.getSharedInstance();
        }
        this.mmappedIO = this.options.isSet("mmappedIO");
        this.symbolFileEnabled = !this.options.isSet("ignore.symbol.file");
        String sf = this.options.get("sortFiles");
        if (sf != null) {
            this.sortFiles = sf.equals("reverse") ? SortFiles.REVERSE : SortFiles.FORWARD;
        }
    }

    public void setSymbolFileEnabled(boolean b) {
        this.symbolFileEnabled = b;
    }

    @Override // com.sun.tools.javac.util.BaseFileManager
    public boolean isDefaultBootClassPath() {
        return this.locations.isDefaultBootClassPath();
    }

    public JavaFileObject getFileForInput(String name) {
        return getRegularFile(new File(name));
    }

    public JavaFileObject getRegularFile(File file) {
        return new RegularFileObject(this, file);
    }

    public JavaFileObject getFileForOutput(String classname, JavaFileObject.Kind kind, JavaFileObject sibling) throws IOException {
        return getJavaFileForOutput(StandardLocation.CLASS_OUTPUT, classname, kind, sibling);
    }

    @Override // javax.tools.StandardJavaFileManager
    public Iterable<? extends JavaFileObject> getJavaFileObjectsFromStrings(Iterable<String> names) {
        ListBuffer<File> files = new ListBuffer<>();
        for (String name : names) {
            files.append(new File((String) nullCheck(name)));
        }
        return getJavaFileObjectsFromFiles(files.toList());
    }

    @Override // javax.tools.StandardJavaFileManager
    public Iterable<? extends JavaFileObject> getJavaFileObjects(String... names) {
        return getJavaFileObjectsFromStrings(Arrays.asList((Object[]) nullCheck(names)));
    }

    private static boolean isValidName(String name) {
        for (String s : name.split("\\.", -1)) {
            if (!SourceVersion.isIdentifier(s)) {
                return false;
            }
        }
        return true;
    }

    private static void validateClassName(String className) {
        if (!isValidName(className)) {
            throw new IllegalArgumentException("Invalid class name: " + className);
        }
    }

    private static void validatePackageName(String packageName) {
        if (packageName.length() > 0 && !isValidName(packageName)) {
            throw new IllegalArgumentException("Invalid packageName name: " + packageName);
        }
    }

    public static void testName(String name, boolean isValidPackageName, boolean isValidClassName) {
        try {
            validatePackageName(name);
        } catch (IllegalArgumentException e) {
            if (isValidPackageName) {
                throw new AssertionError("Valid package name rejected: " + name);
            }
            printAscii("Invalid package name: \"%s\"", name);
        }
        if (!isValidPackageName) {
            throw new AssertionError("Invalid package name accepted: " + name);
        }
        printAscii("Valid package name: \"%s\"", name);
        try {
            validateClassName(name);
            if (!isValidClassName) {
                throw new AssertionError("Invalid class name accepted: " + name);
            }
            printAscii("Valid class name: \"%s\"", name);
        } catch (IllegalArgumentException e2) {
            if (isValidClassName) {
                throw new AssertionError("Valid class name rejected: " + name);
            }
            printAscii("Invalid class name: \"%s\"", name);
        }
    }

    private static void printAscii(String format, Object... args) {
        try {
            String message = new String(String.format(null, format, args).getBytes("US-ASCII"), "US-ASCII");
            System.out.println(message);
        } catch (UnsupportedEncodingException ex) {
            throw new AssertionError(ex);
        }
    }

    private void listDirectory(File directory, RelativePath.RelativeDirectory subdirectory, Set<JavaFileObject.Kind> fileKinds, boolean recurse, ListBuffer<JavaFileObject> resultList) {
        File[] files;
        File d = subdirectory.getFile(directory);
        if (!caseMapCheck(d, subdirectory) || (files = d.listFiles()) == null) {
            return;
        }
        if (this.sortFiles != null) {
            Arrays.sort(files, this.sortFiles);
        }
        for (File f : files) {
            String fname = f.getName();
            if (f.isDirectory()) {
                if (recurse && SourceVersion.isIdentifier(fname)) {
                    listDirectory(directory, new RelativePath.RelativeDirectory(subdirectory, fname), fileKinds, recurse, resultList);
                }
            } else if (isValidFile(fname, fileKinds)) {
                JavaFileObject fe = new RegularFileObject(this, fname, new File(d, fname));
                resultList.append(fe);
            }
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    private void listArchive(Archive archive, RelativePath.RelativeDirectory subdirectory, Set<JavaFileObject.Kind> fileKinds, boolean recurse, ListBuffer<JavaFileObject> resultList) {
        List files = archive.getFiles(subdirectory);
        if (files != null) {
            while (!files.isEmpty()) {
                String file = (String) files.head;
                if (isValidFile(file, fileKinds)) {
                    resultList.append(archive.getFileObject(subdirectory, file));
                }
                files = files.tail;
            }
        }
        if (recurse) {
            for (RelativePath.RelativeDirectory s : archive.getSubdirectories()) {
                if (subdirectory.contains(s)) {
                    listArchive(archive, s, fileKinds, false, resultList);
                }
            }
        }
    }

    private void listContainer(File container, RelativePath.RelativeDirectory subdirectory, Set<JavaFileObject.Kind> fileKinds, boolean recurse, ListBuffer<JavaFileObject> resultList) {
        Archive archive;
        Archive archive2 = this.archives.get(container);
        if (archive2 != null) {
            archive = archive2;
        } else {
            if (this.fsInfo.isDirectory(container)) {
                listDirectory(container, subdirectory, fileKinds, recurse, resultList);
                return;
            }
            try {
                archive = openArchive(container);
            } catch (IOException ex) {
                this.log.error("error.reading.file", container, getMessage(ex));
                return;
            }
        }
        listArchive(archive, subdirectory, fileKinds, recurse, resultList);
    }

    private boolean isValidFile(String s, Set<JavaFileObject.Kind> fileKinds) {
        JavaFileObject.Kind kind = getKind(s);
        return fileKinds.contains(kind);
    }

    static {
        fileSystemIsCaseSensitive = File.separatorChar == '/';
        symbolFileLocation = new String[]{"lib", "ct.sym"};
        symbolFilePrefix = new RelativePath.RelativeDirectory("META-INF/sym/rt.jar/");
    }

    private boolean caseMapCheck(File f, RelativePath name) {
        if (fileSystemIsCaseSensitive) {
            return true;
        }
        try {
            String path = f.getCanonicalPath();
            char[] pcs = path.toCharArray();
            char[] ncs = name.path.toCharArray();
            int i = pcs.length - 1;
            int j = ncs.length - 1;
            while (i >= 0 && j >= 0) {
                while (i >= 0 && pcs[i] == File.separatorChar) {
                    i--;
                }
                while (j >= 0 && ncs[j] == '/') {
                    j--;
                }
                if (i >= 0 && j >= 0) {
                    if (pcs[i] != ncs[j]) {
                        return false;
                    }
                    i--;
                    j--;
                }
            }
            return j < 0;
        } catch (IOException e) {
            return false;
        }
    }

    public class MissingArchive implements Archive {
        final File zipFileName;

        public MissingArchive(File name) {
            this.zipFileName = name;
        }

        @Override // com.sun.tools.javac.file.JavacFileManager.Archive
        public boolean contains(RelativePath name) {
            return false;
        }

        @Override // com.sun.tools.javac.file.JavacFileManager.Archive
        public void close() {
        }

        @Override // com.sun.tools.javac.file.JavacFileManager.Archive
        public JavaFileObject getFileObject(RelativePath.RelativeDirectory subdirectory, String file) {
            return null;
        }

        @Override // com.sun.tools.javac.file.JavacFileManager.Archive
        public List<String> getFiles(RelativePath.RelativeDirectory subdirectory) {
            return List.nil();
        }

        @Override // com.sun.tools.javac.file.JavacFileManager.Archive
        public Set<RelativePath.RelativeDirectory> getSubdirectories() {
            return Collections.emptySet();
        }

        public String toString() {
            return "MissingArchive[" + this.zipFileName + "]";
        }
    }

    protected Archive openArchive(File zipFilename) throws IOException {
        try {
            return openArchive(zipFilename, this.contextUseOptimizedZip);
        } catch (IOException ioe) {
            if (ioe instanceof ZipFileIndex.ZipFormatException) {
                return openArchive(zipFilename, false);
            }
            throw ioe;
        }
    }

    /* JADX WARN: Removed duplicated region for block: B:38:0x00d1 A[PHI: r4
      0x00d1: PHI (r4v3 'preindexCacheLocation' java.lang.String) = 
      (r4v2 'preindexCacheLocation' java.lang.String)
      (r4v2 'preindexCacheLocation' java.lang.String)
      (r4v2 'preindexCacheLocation' java.lang.String)
      (r4v2 'preindexCacheLocation' java.lang.String)
      (r4v4 'preindexCacheLocation' java.lang.String)
      (r4v4 'preindexCacheLocation' java.lang.String)
     binds: [B:19:0x0073, B:21:0x0079, B:29:0x00a1, B:31:0x00a7, B:33:0x00b0, B:35:0x00b8] A[DONT_GENERATE, DONT_INLINE]] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private com.sun.tools.javac.file.JavacFileManager.Archive openArchive(java.io.File r12, boolean r13) throws java.io.IOException {
        /*
            Method dump skipped, instruction units count: 329
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.file.JavacFileManager.openArchive(java.io.File, boolean):com.sun.tools.javac.file.JavacFileManager$Archive");
    }

    @Override // javax.tools.JavaFileManager, java.io.Flushable
    public void flush() {
        this.contentCache.clear();
    }

    @Override // javax.tools.JavaFileManager, java.io.Closeable, java.lang.AutoCloseable
    public void close() {
        Iterator<Archive> i = this.archives.values().iterator();
        while (i.hasNext()) {
            Archive a = i.next();
            i.remove();
            try {
                a.close();
            } catch (IOException e) {
            }
        }
    }

    private String getDefaultEncodingName() {
        if (this.defaultEncodingName == null) {
            this.defaultEncodingName = new OutputStreamWriter(new ByteArrayOutputStream()).getEncoding();
        }
        return this.defaultEncodingName;
    }

    @Override // javax.tools.JavaFileManager
    public ClassLoader getClassLoader(JavaFileManager.Location location) {
        nullCheck(location);
        Iterable<? extends File> path = getLocation(location);
        if (path == null) {
            return null;
        }
        ListBuffer<URL> lb = new ListBuffer<>();
        for (File f : path) {
            try {
                lb.append(f.toURI().toURL());
            } catch (MalformedURLException e) {
                throw new AssertionError(e);
            }
        }
        return getClassLoader((URL[]) lb.toArray(new URL[lb.size()]));
    }

    @Override // javax.tools.JavaFileManager
    public Iterable<JavaFileObject> list(JavaFileManager.Location location, String packageName, Set<JavaFileObject.Kind> kinds, boolean recurse) throws IOException {
        nullCheck(packageName);
        nullCheck((Collection) kinds);
        Iterable<? extends File> path = getLocation(location);
        if (path == null) {
            return List.nil();
        }
        RelativePath.RelativeDirectory subdirectory = RelativePath.RelativeDirectory.forPackage(packageName);
        ListBuffer<JavaFileObject> results = new ListBuffer<>();
        for (File directory : path) {
            listContainer(directory, subdirectory, kinds, recurse, results);
        }
        return results.toList();
    }

    @Override // javax.tools.JavaFileManager
    public String inferBinaryName(JavaFileManager.Location location, JavaFileObject file) {
        file.getClass();
        location.getClass();
        Iterable<? extends File> path = getLocation(location);
        if (path == null) {
            return null;
        }
        if (file instanceof BaseFileObject) {
            return ((BaseFileObject) file).inferBinaryName(path);
        }
        throw new IllegalArgumentException(file.getClass().getName());
    }

    @Override // javax.tools.StandardJavaFileManager, javax.tools.JavaFileManager
    public boolean isSameFile(FileObject a, FileObject b) {
        nullCheck(a);
        nullCheck(b);
        if (!(a instanceof BaseFileObject)) {
            throw new IllegalArgumentException("Not supported: " + a);
        }
        if (!(b instanceof BaseFileObject)) {
            throw new IllegalArgumentException("Not supported: " + b);
        }
        return a.equals(b);
    }

    @Override // javax.tools.JavaFileManager
    public boolean hasLocation(JavaFileManager.Location location) {
        return getLocation(location) != null;
    }

    @Override // javax.tools.JavaFileManager
    public JavaFileObject getJavaFileForInput(JavaFileManager.Location location, String className, JavaFileObject.Kind kind) throws IOException {
        nullCheck(location);
        nullCheck(className);
        nullCheck(kind);
        if (!this.sourceOrClass.contains(kind)) {
            throw new IllegalArgumentException("Invalid kind: " + kind);
        }
        return getFileForInput(location, RelativePath.RelativeFile.forClass(className, kind));
    }

    @Override // javax.tools.JavaFileManager
    public FileObject getFileForInput(JavaFileManager.Location location, String packageName, String relativeName) throws IOException {
        nullCheck(location);
        nullCheck(packageName);
        if (!isRelativeUri(relativeName)) {
            throw new IllegalArgumentException("Invalid relative name: " + relativeName);
        }
        RelativePath.RelativeFile name = packageName.length() == 0 ? new RelativePath.RelativeFile(relativeName) : new RelativePath.RelativeFile(RelativePath.RelativeDirectory.forPackage(packageName), relativeName);
        return getFileForInput(location, name);
    }

    private JavaFileObject getFileForInput(JavaFileManager.Location location, RelativePath.RelativeFile name) throws IOException {
        Iterable<? extends File> path = getLocation(location);
        if (path == null) {
            return null;
        }
        for (File dir : path) {
            Archive a = this.archives.get(dir);
            if (a == null) {
                if (this.fsInfo.isDirectory(dir)) {
                    File f = name.getFile(dir);
                    if (f.exists()) {
                        return new RegularFileObject(this, f);
                    }
                } else {
                    a = openArchive(dir);
                }
            }
            if (a.contains(name)) {
                return a.getFileObject(name.dirname(), name.basename());
            }
        }
        return null;
    }

    @Override // javax.tools.JavaFileManager
    public JavaFileObject getJavaFileForOutput(JavaFileManager.Location location, String className, JavaFileObject.Kind kind, FileObject sibling) throws IOException {
        nullCheck(location);
        nullCheck(className);
        nullCheck(kind);
        if (!this.sourceOrClass.contains(kind)) {
            throw new IllegalArgumentException("Invalid kind: " + kind);
        }
        return getFileForOutput(location, RelativePath.RelativeFile.forClass(className, kind), sibling);
    }

    @Override // javax.tools.JavaFileManager
    public FileObject getFileForOutput(JavaFileManager.Location location, String packageName, String relativeName, FileObject sibling) throws IOException {
        nullCheck(location);
        nullCheck(packageName);
        if (!isRelativeUri(relativeName)) {
            throw new IllegalArgumentException("Invalid relative name: " + relativeName);
        }
        RelativePath.RelativeFile name = packageName.length() == 0 ? new RelativePath.RelativeFile(relativeName) : new RelativePath.RelativeFile(RelativePath.RelativeDirectory.forPackage(packageName), relativeName);
        return getFileForOutput(location, name, sibling);
    }

    private JavaFileObject getFileForOutput(JavaFileManager.Location location, RelativePath.RelativeFile fileName, FileObject sibling) throws IOException {
        File dir;
        if (location == StandardLocation.CLASS_OUTPUT) {
            if (getClassOutDir() != null) {
                dir = getClassOutDir();
            } else {
                File siblingDir = null;
                if (sibling != null && (sibling instanceof RegularFileObject)) {
                    siblingDir = ((RegularFileObject) sibling).file.getParentFile();
                }
                return new RegularFileObject(this, new File(siblingDir, fileName.basename()));
            }
        } else if (location == StandardLocation.SOURCE_OUTPUT) {
            dir = getSourceOutDir() != null ? getSourceOutDir() : getClassOutDir();
        } else {
            Iterable<? extends File> path = this.locations.getLocation(location);
            Iterator<? extends File> it = path.iterator();
            if (!it.hasNext()) {
                dir = null;
            } else {
                File f = (File) it.next();
                dir = f;
            }
        }
        File file = fileName.getFile(dir);
        return new RegularFileObject(this, file);
    }

    @Override // javax.tools.StandardJavaFileManager
    public Iterable<? extends JavaFileObject> getJavaFileObjectsFromFiles(Iterable<? extends File> files) {
        ArrayList<RegularFileObject> result;
        if (files instanceof Collection) {
            result = new ArrayList<>(((Collection) files).size());
        } else {
            result = new ArrayList<>();
        }
        for (File f : files) {
            result.add(new RegularFileObject(this, (File) nullCheck(f)));
        }
        return result;
    }

    @Override // javax.tools.StandardJavaFileManager
    public Iterable<? extends JavaFileObject> getJavaFileObjects(File... files) {
        return getJavaFileObjectsFromFiles(Arrays.asList((Object[]) nullCheck(files)));
    }

    @Override // javax.tools.StandardJavaFileManager
    public void setLocation(JavaFileManager.Location location, Iterable<? extends File> path) throws IOException {
        nullCheck(location);
        this.locations.setLocation(location, path);
    }

    @Override // javax.tools.StandardJavaFileManager
    public Iterable<? extends File> getLocation(JavaFileManager.Location location) {
        nullCheck(location);
        return this.locations.getLocation(location);
    }

    private File getClassOutDir() {
        return this.locations.getOutputLocation(StandardLocation.CLASS_OUTPUT);
    }

    private File getSourceOutDir() {
        return this.locations.getOutputLocation(StandardLocation.SOURCE_OUTPUT);
    }

    protected static boolean isRelativeUri(URI uri) {
        if (uri.isAbsolute()) {
            return false;
        }
        String path = uri.normalize().getPath();
        return (path.length() == 0 || !path.equals(uri.getPath()) || path.startsWith(OnBotJavaFileSystemUtils.PATH_SEPARATOR) || path.startsWith("./") || path.startsWith("../")) ? false : true;
    }

    protected static boolean isRelativeUri(String u) {
        try {
            return isRelativeUri(new URI(u));
        } catch (URISyntaxException e) {
            return false;
        }
    }

    public static String getRelativeName(File file) {
        if (!file.isAbsolute()) {
            String result = file.getPath().replace(File.separatorChar, DataResource.SEPARATOR);
            if (isRelativeUri(result)) {
                return result;
            }
        }
        throw new IllegalArgumentException("Invalid relative path: " + file);
    }

    public static String getMessage(IOException e) {
        String s = e.getLocalizedMessage();
        if (s != null) {
            return s;
        }
        String s2 = e.getMessage();
        if (s2 != null) {
            return s2;
        }
        return e.toString();
    }
}
