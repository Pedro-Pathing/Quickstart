package com.sun.tools.javac.file;

import com.android.tools.r8.DataResource;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.StringUtils;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.StringTokenizer;
import java.util.zip.ZipFile;
import javax.tools.JavaFileManager;
import javax.tools.StandardLocation;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class Locations {
    private FSInfo fsInfo;
    Map<JavaFileManager.Location, LocationHandler> handlersForLocation;
    Map<Option, LocationHandler> handlersForOption;
    private boolean inited = false;
    private Lint lint;
    private Log log;
    private Options options;
    private boolean warn;

    public Locations() {
        initHandlers();
    }

    public void update(Log log, Options options, Lint lint, FSInfo fsInfo) {
        this.log = log;
        this.options = options;
        this.lint = lint;
        this.fsInfo = fsInfo;
    }

    public Collection<File> bootClassPath() {
        return getLocation(StandardLocation.PLATFORM_CLASS_PATH);
    }

    public boolean isDefaultBootClassPath() {
        BootClassPathLocationHandler h = (BootClassPathLocationHandler) getHandler(StandardLocation.PLATFORM_CLASS_PATH);
        return h.isDefault();
    }

    boolean isDefaultBootClassPathRtJar(File file) {
        BootClassPathLocationHandler h = (BootClassPathLocationHandler) getHandler(StandardLocation.PLATFORM_CLASS_PATH);
        return h.isDefaultRtJar(file);
    }

    public Collection<File> userClassPath() {
        return getLocation(StandardLocation.CLASS_PATH);
    }

    public Collection<File> sourcePath() {
        Collection<File> p = getLocation(StandardLocation.SOURCE_PATH);
        if (p == null || p.isEmpty()) {
            return null;
        }
        return p;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static Iterable<File> getPathEntries(String path) {
        return getPathEntries(path, null);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static Iterable<File> getPathEntries(String path, File emptyPathDefault) {
        ListBuffer<File> entries = new ListBuffer<>();
        int start = 0;
        while (start <= path.length()) {
            int sep = path.indexOf(File.pathSeparatorChar, start);
            if (sep == -1) {
                sep = path.length();
            }
            if (start < sep) {
                entries.add(new File(path.substring(start, sep)));
            } else if (emptyPathDefault != null) {
                entries.add(emptyPathDefault);
            }
            start = sep + 1;
        }
        return entries;
    }

    private class Path extends LinkedHashSet<File> {
        private static final long serialVersionUID = 0;
        private boolean expandJarClassPaths = false;
        private Set<File> canonicalValues = new HashSet();
        private File emptyPathDefault = null;

        public Path expandJarClassPaths(boolean x) {
            this.expandJarClassPaths = x;
            return this;
        }

        public Path emptyPathDefault(File x) {
            this.emptyPathDefault = x;
            return this;
        }

        public Path() {
        }

        public Path addDirectories(String dirs, boolean warn) {
            boolean prev = this.expandJarClassPaths;
            this.expandJarClassPaths = true;
            if (dirs != null) {
                try {
                    for (File dir : Locations.getPathEntries(dirs)) {
                        addDirectory(dir, warn);
                    }
                } finally {
                    this.expandJarClassPaths = prev;
                }
            }
            return this;
        }

        public Path addDirectories(String dirs) {
            return addDirectories(dirs, Locations.this.warn);
        }

        private void addDirectory(File dir, boolean warn) {
            if (!dir.isDirectory()) {
                if (warn) {
                    Locations.this.log.warning(Lint.LintCategory.PATH, "dir.path.element.not.found", dir);
                    return;
                }
                return;
            }
            File[] files = dir.listFiles();
            if (files == null) {
                return;
            }
            for (File direntry : files) {
                if (Locations.this.isArchive(direntry)) {
                    addFile(direntry, warn);
                }
            }
        }

        public Path addFiles(String files, boolean warn) {
            if (files != null) {
                addFiles(Locations.getPathEntries(files, this.emptyPathDefault), warn);
            }
            return this;
        }

        public Path addFiles(String files) {
            return addFiles(files, Locations.this.warn);
        }

        public Path addFiles(Iterable<? extends File> files, boolean warn) {
            if (files != null) {
                for (File file : files) {
                    addFile(file, warn);
                }
            }
            return this;
        }

        public Path addFiles(Iterable<? extends File> files) {
            return addFiles(files, Locations.this.warn);
        }

        public void addFile(File file, boolean warn) {
            if (!contains(file)) {
                if (Locations.this.fsInfo.exists(file)) {
                    File canonFile = Locations.this.fsInfo.getCanonicalFile(file);
                    if (!this.canonicalValues.contains(canonFile)) {
                        if (Locations.this.fsInfo.isFile(file) && !Locations.this.isArchive(file)) {
                            try {
                                ZipFile z = new ZipFile(file);
                                z.close();
                                if (warn) {
                                    Locations.this.log.warning(Lint.LintCategory.PATH, "unexpected.archive.file", file);
                                }
                            } catch (IOException e) {
                                if (warn) {
                                    Locations.this.log.warning(Lint.LintCategory.PATH, "invalid.archive.file", file);
                                    return;
                                }
                                return;
                            }
                        }
                        super.add(file);
                        this.canonicalValues.add(canonFile);
                        if (this.expandJarClassPaths && Locations.this.fsInfo.isFile(file)) {
                            addJarClassPath(file, warn);
                            return;
                        }
                        return;
                    }
                    return;
                }
                if (warn) {
                    Locations.this.log.warning(Lint.LintCategory.PATH, "path.element.not.found", file);
                }
                super.add(file);
            }
        }

        private void addJarClassPath(File jarFile, boolean warn) {
            try {
                for (File f : Locations.this.fsInfo.getJarClassPath(jarFile)) {
                    addFile(f, warn);
                }
            } catch (IOException e) {
                Locations.this.log.error("error.reading.file", jarFile, JavacFileManager.getMessage(e));
            }
        }
    }

    protected abstract class LocationHandler {
        final JavaFileManager.Location location;
        final Set<Option> options;

        abstract Collection<File> getLocation();

        abstract boolean handleOption(Option option, String str);

        abstract void setLocation(Iterable<? extends File> iterable) throws IOException;

        protected LocationHandler(JavaFileManager.Location location, Option... options) {
            EnumSet enumSetCopyOf;
            this.location = location;
            if (options.length == 0) {
                enumSetCopyOf = EnumSet.noneOf(Option.class);
            } else {
                enumSetCopyOf = EnumSet.copyOf((Collection) Arrays.asList(options));
            }
            this.options = enumSetCopyOf;
        }

        void update(Options optionTable) {
            for (Option o : this.options) {
                String v = optionTable.get(o);
                if (v != null) {
                    handleOption(o, v);
                }
            }
        }
    }

    private class OutputLocationHandler extends LocationHandler {
        private File outputDir;

        OutputLocationHandler(JavaFileManager.Location location, Option... options) {
            super(location, options);
        }

        @Override // com.sun.tools.javac.file.Locations.LocationHandler
        boolean handleOption(Option option, String value) {
            if (!this.options.contains(option)) {
                return false;
            }
            this.outputDir = new File(value);
            return true;
        }

        @Override // com.sun.tools.javac.file.Locations.LocationHandler
        Collection<File> getLocation() {
            if (this.outputDir == null) {
                return null;
            }
            return Collections.singleton(this.outputDir);
        }

        @Override // com.sun.tools.javac.file.Locations.LocationHandler
        void setLocation(Iterable<? extends File> files) throws IOException {
            if (files == null) {
                this.outputDir = null;
                return;
            }
            Iterator<? extends File> pathIter = files.iterator();
            if (!pathIter.hasNext()) {
                throw new IllegalArgumentException("empty path for directory");
            }
            File dir = pathIter.next();
            if (pathIter.hasNext()) {
                throw new IllegalArgumentException("path too long for directory");
            }
            if (!dir.exists()) {
                throw new FileNotFoundException(dir + ": does not exist");
            }
            if (!dir.isDirectory()) {
                throw new IOException(dir + ": not a directory");
            }
            this.outputDir = dir;
        }
    }

    private class SimpleLocationHandler extends LocationHandler {
        protected Collection<File> searchPath;

        SimpleLocationHandler(JavaFileManager.Location location, Option... options) {
            super(location, options);
        }

        @Override // com.sun.tools.javac.file.Locations.LocationHandler
        boolean handleOption(Option option, String value) {
            if (!this.options.contains(option)) {
                return false;
            }
            this.searchPath = value == null ? null : Collections.unmodifiableCollection(createPath().addFiles(value));
            return true;
        }

        @Override // com.sun.tools.javac.file.Locations.LocationHandler
        Collection<File> getLocation() {
            return this.searchPath;
        }

        @Override // com.sun.tools.javac.file.Locations.LocationHandler
        void setLocation(Iterable<? extends File> files) {
            Path p;
            if (files == null) {
                p = computePath(null);
            } else {
                Path p2 = createPath();
                p = p2.addFiles(files);
            }
            this.searchPath = Collections.unmodifiableCollection(p);
        }

        protected Path computePath(String value) {
            return createPath().addFiles(value);
        }

        protected Path createPath() {
            return Locations.this.new Path();
        }
    }

    private class ClassPathLocationHandler extends SimpleLocationHandler {
        ClassPathLocationHandler() {
            super(StandardLocation.CLASS_PATH, Option.CLASSPATH, Option.CP);
        }

        @Override // com.sun.tools.javac.file.Locations.SimpleLocationHandler, com.sun.tools.javac.file.Locations.LocationHandler
        Collection<File> getLocation() {
            lazy();
            return this.searchPath;
        }

        @Override // com.sun.tools.javac.file.Locations.SimpleLocationHandler
        protected Path computePath(String value) {
            String cp = value;
            if (cp == null) {
                cp = System.getProperty("env.class.path");
            }
            if (cp == null && System.getProperty("application.home") == null) {
                cp = System.getProperty("java.class.path");
            }
            if (cp == null) {
                cp = ".";
            }
            return createPath().addFiles(cp);
        }

        @Override // com.sun.tools.javac.file.Locations.SimpleLocationHandler
        protected Path createPath() {
            return Locations.this.new Path().expandJarClassPaths(true).emptyPathDefault(new File("."));
        }

        private void lazy() {
            if (this.searchPath == null) {
                setLocation(null);
            }
        }
    }

    private class BootClassPathLocationHandler extends LocationHandler {
        private File defaultBootClassPathRtJar;
        private boolean isDefaultBootClassPath;
        final Map<Option, String> optionValues;
        private Collection<File> searchPath;

        BootClassPathLocationHandler() {
            super(StandardLocation.PLATFORM_CLASS_PATH, Option.BOOTCLASSPATH, Option.XBOOTCLASSPATH, Option.XBOOTCLASSPATH_PREPEND, Option.XBOOTCLASSPATH_APPEND, Option.ENDORSEDDIRS, Option.DJAVA_ENDORSED_DIRS, Option.EXTDIRS, Option.DJAVA_EXT_DIRS);
            this.optionValues = new EnumMap(Option.class);
            this.defaultBootClassPathRtJar = null;
        }

        boolean isDefault() {
            lazy();
            return this.isDefaultBootClassPath;
        }

        boolean isDefaultRtJar(File file) {
            lazy();
            return file.equals(this.defaultBootClassPathRtJar);
        }

        @Override // com.sun.tools.javac.file.Locations.LocationHandler
        boolean handleOption(Option option, String value) {
            if (!this.options.contains(option)) {
                return false;
            }
            Option option2 = canonicalize(option);
            this.optionValues.put(option2, value);
            if (option2 == Option.BOOTCLASSPATH) {
                this.optionValues.remove(Option.XBOOTCLASSPATH_PREPEND);
                this.optionValues.remove(Option.XBOOTCLASSPATH_APPEND);
            }
            this.searchPath = null;
            return true;
        }

        private Option canonicalize(Option option) {
            switch (option) {
                case XBOOTCLASSPATH:
                    return Option.BOOTCLASSPATH;
                case DJAVA_ENDORSED_DIRS:
                    return Option.ENDORSEDDIRS;
                case DJAVA_EXT_DIRS:
                    return Option.EXTDIRS;
                default:
                    return option;
            }
        }

        @Override // com.sun.tools.javac.file.Locations.LocationHandler
        Collection<File> getLocation() {
            lazy();
            return this.searchPath;
        }

        @Override // com.sun.tools.javac.file.Locations.LocationHandler
        void setLocation(Iterable<? extends File> files) {
            if (files == null) {
                this.searchPath = null;
                return;
            }
            this.defaultBootClassPathRtJar = null;
            this.isDefaultBootClassPath = false;
            Path p = Locations.this.new Path().addFiles(files, false);
            this.searchPath = Collections.unmodifiableCollection(p);
            this.optionValues.clear();
        }

        Path computePath() {
            this.defaultBootClassPathRtJar = null;
            Path path = Locations.this.new Path();
            String bootclasspathOpt = this.optionValues.get(Option.BOOTCLASSPATH);
            String endorseddirsOpt = this.optionValues.get(Option.ENDORSEDDIRS);
            String extdirsOpt = this.optionValues.get(Option.EXTDIRS);
            String xbootclasspathPrependOpt = this.optionValues.get(Option.XBOOTCLASSPATH_PREPEND);
            String xbootclasspathAppendOpt = this.optionValues.get(Option.XBOOTCLASSPATH_APPEND);
            path.addFiles(xbootclasspathPrependOpt);
            boolean z = false;
            if (endorseddirsOpt == null) {
                path.addDirectories(System.getProperty("java.endorsed.dirs"), false);
            } else {
                path.addDirectories(endorseddirsOpt);
            }
            if (bootclasspathOpt != null) {
                path.addFiles(bootclasspathOpt);
            } else {
                String files = System.getProperty("sun.boot.class.path");
                path.addFiles(files, false);
                File rt_jar = new File("rt.jar");
                for (File file : Locations.getPathEntries(files)) {
                    if (new File(file.getName()).equals(rt_jar)) {
                        this.defaultBootClassPathRtJar = file;
                    }
                }
            }
            path.addFiles(xbootclasspathAppendOpt);
            if (extdirsOpt == null) {
                path.addDirectories(System.getProperty("java.ext.dirs"), false);
            } else {
                path.addDirectories(extdirsOpt);
            }
            if (xbootclasspathPrependOpt == null && bootclasspathOpt == null && xbootclasspathAppendOpt == null) {
                z = true;
            }
            this.isDefaultBootClassPath = z;
            return path;
        }

        private void lazy() {
            if (this.searchPath == null) {
                this.searchPath = Collections.unmodifiableCollection(computePath());
            }
        }
    }

    void initHandlers() {
        this.handlersForLocation = new HashMap();
        this.handlersForOption = new EnumMap(Option.class);
        LocationHandler[] handlers = {new BootClassPathLocationHandler(), new ClassPathLocationHandler(), new SimpleLocationHandler(StandardLocation.SOURCE_PATH, Option.SOURCEPATH), new SimpleLocationHandler(StandardLocation.ANNOTATION_PROCESSOR_PATH, Option.PROCESSORPATH), new OutputLocationHandler(StandardLocation.CLASS_OUTPUT, Option.D), new OutputLocationHandler(StandardLocation.SOURCE_OUTPUT, Option.S), new OutputLocationHandler(StandardLocation.NATIVE_HEADER_OUTPUT, Option.H)};
        for (LocationHandler h : handlers) {
            this.handlersForLocation.put(h.location, h);
            for (Option o : h.options) {
                this.handlersForOption.put(o, h);
            }
        }
    }

    boolean handleOption(Option option, String value) {
        LocationHandler h = this.handlersForOption.get(option);
        if (h == null) {
            return false;
        }
        return h.handleOption(option, value);
    }

    Collection<File> getLocation(JavaFileManager.Location location) {
        LocationHandler h = getHandler(location);
        if (h == null) {
            return null;
        }
        return h.getLocation();
    }

    File getOutputLocation(JavaFileManager.Location location) {
        if (!location.isOutputLocation()) {
            throw new IllegalArgumentException();
        }
        LocationHandler h = getHandler(location);
        return ((OutputLocationHandler) h).outputDir;
    }

    void setLocation(JavaFileManager.Location location, Iterable<? extends File> files) throws IOException {
        LocationHandler h = getHandler(location);
        if (h == null) {
            if (location.isOutputLocation()) {
                h = new OutputLocationHandler(location, new Option[0]);
            } else {
                h = new SimpleLocationHandler(location, new Option[0]);
            }
            this.handlersForLocation.put(location, h);
        }
        h.setLocation(files);
    }

    protected LocationHandler getHandler(JavaFileManager.Location location) {
        location.getClass();
        lazy();
        return this.handlersForLocation.get(location);
    }

    protected void lazy() {
        if (!this.inited) {
            this.warn = this.lint.isEnabled(Lint.LintCategory.PATH);
            for (LocationHandler h : this.handlersForLocation.values()) {
                h.update(this.options);
            }
            this.inited = true;
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public boolean isArchive(File file) {
        String n = StringUtils.toLowerCase(file.getName());
        return this.fsInfo.isFile(file) && (n.endsWith(".jar") || n.endsWith(OnBotJavaFileSystemUtils.EXT_ZIP_FILE));
    }

    public static URL[] pathToURLs(String path) {
        StringTokenizer st = new StringTokenizer(path, File.pathSeparator);
        URL[] urls = new URL[st.countTokens()];
        int count = 0;
        while (st.hasMoreTokens()) {
            URL url = fileToURL(new File(st.nextToken()));
            if (url != null) {
                urls[count] = url;
                count++;
            }
        }
        return (URL[]) Arrays.copyOf(urls, count);
    }

    private static URL fileToURL(File file) {
        String name;
        try {
            name = file.getCanonicalPath();
        } catch (IOException e) {
            name = file.getAbsolutePath();
        }
        String name2 = name.replace(File.separatorChar, DataResource.SEPARATOR);
        if (!name2.startsWith(OnBotJavaFileSystemUtils.PATH_SEPARATOR)) {
            name2 = OnBotJavaFileSystemUtils.PATH_SEPARATOR + name2;
        }
        if (!file.isFile()) {
            name2 = name2 + OnBotJavaFileSystemUtils.PATH_SEPARATOR;
        }
        try {
            return new URL("file", "", name2);
        } catch (MalformedURLException e2) {
            throw new IllegalArgumentException(file.toString());
        }
    }
}
