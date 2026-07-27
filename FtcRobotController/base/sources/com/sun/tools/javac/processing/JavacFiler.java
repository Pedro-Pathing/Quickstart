package com.sun.tools.javac.processing;

import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Log;
import dk.sgjesse.r8api.FileUtils;
import java.io.Closeable;
import java.io.FileNotFoundException;
import java.io.FilterOutputStream;
import java.io.FilterWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.Reader;
import java.io.Writer;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import javax.annotation.processing.Filer;
import javax.annotation.processing.FilerException;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.NestingKind;
import javax.tools.FileObject;
import javax.tools.ForwardingFileObject;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.StandardLocation;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class JavacFiler implements Filer, Closeable {
    private static final String ALREADY_OPENED = "Output stream or writer has already been opened.";
    private static final String NOT_FOR_READING = "FileObject was not opened for reading.";
    private static final String NOT_FOR_WRITING = "FileObject was not opened for writing.";
    Context context;
    JavaFileManager fileManager;
    boolean lastRound;
    private final boolean lint;
    Log log;
    private final Set<FileObject> fileObjectHistory = Collections.synchronizedSet(new LinkedHashSet());
    private Set<String> generatedSourceNames = Collections.synchronizedSet(new LinkedHashSet());
    private Set<JavaFileObject> generatedSourceFileObjects = Collections.synchronizedSet(new LinkedHashSet());
    private final Map<String, JavaFileObject> generatedClasses = Collections.synchronizedMap(new LinkedHashMap());
    private final Set<String> openTypeNames = Collections.synchronizedSet(new LinkedHashSet());
    private final Set<String> aggregateGeneratedSourceNames = new LinkedHashSet();
    private final Set<String> aggregateGeneratedClassNames = new LinkedHashSet();

    private class FilerOutputFileObject extends ForwardingFileObject<FileObject> {
        private String name;
        private boolean opened;

        FilerOutputFileObject(String name, FileObject fileObject) {
            super(fileObject);
            this.opened = false;
            this.name = name;
        }

        @Override // javax.tools.ForwardingFileObject, javax.tools.FileObject
        public synchronized OutputStream openOutputStream() throws IOException {
            if (this.opened) {
                throw new IOException(JavacFiler.ALREADY_OPENED);
            }
            this.opened = true;
            return JavacFiler.this.new FilerOutputStream(this.name, this.fileObject);
        }

        @Override // javax.tools.ForwardingFileObject, javax.tools.FileObject
        public synchronized Writer openWriter() throws IOException {
            if (this.opened) {
                throw new IOException(JavacFiler.ALREADY_OPENED);
            }
            this.opened = true;
            return JavacFiler.this.new FilerWriter(this.name, this.fileObject);
        }

        @Override // javax.tools.ForwardingFileObject, javax.tools.FileObject
        public InputStream openInputStream() throws IOException {
            throw new IllegalStateException(JavacFiler.NOT_FOR_READING);
        }

        @Override // javax.tools.ForwardingFileObject, javax.tools.FileObject
        public Reader openReader(boolean ignoreEncodingErrors) throws IOException {
            throw new IllegalStateException(JavacFiler.NOT_FOR_READING);
        }

        @Override // javax.tools.ForwardingFileObject, javax.tools.FileObject
        public CharSequence getCharContent(boolean ignoreEncodingErrors) throws IOException {
            throw new IllegalStateException(JavacFiler.NOT_FOR_READING);
        }

        @Override // javax.tools.ForwardingFileObject, javax.tools.FileObject
        public boolean delete() {
            return false;
        }
    }

    private class FilerOutputJavaFileObject extends FilerOutputFileObject implements JavaFileObject {
        private final JavaFileObject javaFileObject;

        FilerOutputJavaFileObject(String name, JavaFileObject javaFileObject) {
            super(name, javaFileObject);
            this.javaFileObject = javaFileObject;
        }

        @Override // javax.tools.JavaFileObject
        public JavaFileObject.Kind getKind() {
            return this.javaFileObject.getKind();
        }

        @Override // javax.tools.JavaFileObject
        public boolean isNameCompatible(String simpleName, JavaFileObject.Kind kind) {
            return this.javaFileObject.isNameCompatible(simpleName, kind);
        }

        @Override // javax.tools.JavaFileObject
        public NestingKind getNestingKind() {
            return this.javaFileObject.getNestingKind();
        }

        @Override // javax.tools.JavaFileObject
        public Modifier getAccessLevel() {
            return this.javaFileObject.getAccessLevel();
        }
    }

    private class FilerInputFileObject extends ForwardingFileObject<FileObject> {
        FilerInputFileObject(FileObject fileObject) {
            super(fileObject);
        }

        @Override // javax.tools.ForwardingFileObject, javax.tools.FileObject
        public OutputStream openOutputStream() throws IOException {
            throw new IllegalStateException(JavacFiler.NOT_FOR_WRITING);
        }

        @Override // javax.tools.ForwardingFileObject, javax.tools.FileObject
        public Writer openWriter() throws IOException {
            throw new IllegalStateException(JavacFiler.NOT_FOR_WRITING);
        }

        @Override // javax.tools.ForwardingFileObject, javax.tools.FileObject
        public boolean delete() {
            return false;
        }
    }

    private class FilerInputJavaFileObject extends FilerInputFileObject implements JavaFileObject {
        private final JavaFileObject javaFileObject;

        FilerInputJavaFileObject(JavaFileObject javaFileObject) {
            super(javaFileObject);
            this.javaFileObject = javaFileObject;
        }

        @Override // javax.tools.JavaFileObject
        public JavaFileObject.Kind getKind() {
            return this.javaFileObject.getKind();
        }

        @Override // javax.tools.JavaFileObject
        public boolean isNameCompatible(String simpleName, JavaFileObject.Kind kind) {
            return this.javaFileObject.isNameCompatible(simpleName, kind);
        }

        @Override // javax.tools.JavaFileObject
        public NestingKind getNestingKind() {
            return this.javaFileObject.getNestingKind();
        }

        @Override // javax.tools.JavaFileObject
        public Modifier getAccessLevel() {
            return this.javaFileObject.getAccessLevel();
        }
    }

    private class FilerOutputStream extends FilterOutputStream {
        boolean closed;
        FileObject fileObject;
        String typeName;

        FilerOutputStream(String typeName, FileObject fileObject) throws IOException {
            super(fileObject.openOutputStream());
            this.closed = false;
            this.typeName = typeName;
            this.fileObject = fileObject;
        }

        @Override // java.io.FilterOutputStream, java.io.OutputStream, java.io.Closeable, java.lang.AutoCloseable
        public synchronized void close() throws IOException {
            if (!this.closed) {
                this.closed = true;
                JavacFiler.this.closeFileObject(this.typeName, this.fileObject);
                this.out.close();
            }
        }
    }

    private class FilerWriter extends FilterWriter {
        boolean closed;
        FileObject fileObject;
        String typeName;

        FilerWriter(String typeName, FileObject fileObject) throws IOException {
            super(fileObject.openWriter());
            this.closed = false;
            this.typeName = typeName;
            this.fileObject = fileObject;
        }

        @Override // java.io.FilterWriter, java.io.Writer, java.io.Closeable, java.lang.AutoCloseable
        public synchronized void close() throws IOException {
            if (!this.closed) {
                this.closed = true;
                JavacFiler.this.closeFileObject(this.typeName, this.fileObject);
                this.out.close();
            }
        }
    }

    JavacFiler(Context context) {
        this.context = context;
        this.fileManager = (JavaFileManager) context.get(JavaFileManager.class);
        this.log = Log.instance(context);
        this.lint = Lint.instance(context).isEnabled(Lint.LintCategory.PROCESSING);
    }

    @Override // javax.annotation.processing.Filer
    public JavaFileObject createSourceFile(CharSequence name, Element... originatingElements) throws IOException {
        return createSourceOrClassFile(true, name.toString());
    }

    @Override // javax.annotation.processing.Filer
    public JavaFileObject createClassFile(CharSequence name, Element... originatingElements) throws IOException {
        return createSourceOrClassFile(false, name.toString());
    }

    private JavaFileObject createSourceOrClassFile(boolean isSourceFile, String name) throws IOException {
        int periodIndex;
        if (this.lint && (periodIndex = name.lastIndexOf(".")) != -1) {
            String base = name.substring(periodIndex);
            String extn = isSourceFile ? OnBotJavaFileSystemUtils.EXT_JAVA_FILE : FileUtils.CLASS_EXTENSION;
            if (base.equals(extn)) {
                this.log.warning("proc.suspicious.class.name", name, extn);
            }
        }
        checkNameAndExistence(name, isSourceFile);
        JavaFileManager.Location loc = isSourceFile ? StandardLocation.SOURCE_OUTPUT : StandardLocation.CLASS_OUTPUT;
        JavaFileObject.Kind kind = isSourceFile ? JavaFileObject.Kind.SOURCE : JavaFileObject.Kind.CLASS;
        JavaFileObject fileObject = this.fileManager.getJavaFileForOutput(loc, name, kind, null);
        checkFileReopening(fileObject, true);
        if (this.lastRound) {
            this.log.warning("proc.file.create.last.round", name);
        }
        if (isSourceFile) {
            this.aggregateGeneratedSourceNames.add(name);
        } else {
            this.aggregateGeneratedClassNames.add(name);
        }
        this.openTypeNames.add(name);
        return new FilerOutputJavaFileObject(name, fileObject);
    }

    @Override // javax.annotation.processing.Filer
    public FileObject createResource(JavaFileManager.Location location, CharSequence pkg, CharSequence relativeName, Element... originatingElements) throws IOException {
        locationCheck(location);
        String strPkg = pkg.toString();
        if (strPkg.length() > 0) {
            checkName(strPkg);
        }
        FileObject fileObject = this.fileManager.getFileForOutput(location, strPkg, relativeName.toString(), null);
        checkFileReopening(fileObject, true);
        if (fileObject instanceof JavaFileObject) {
            return new FilerOutputJavaFileObject(null, (JavaFileObject) fileObject);
        }
        return new FilerOutputFileObject(null, fileObject);
    }

    private void locationCheck(JavaFileManager.Location location) {
        if (location instanceof StandardLocation) {
            StandardLocation stdLoc = (StandardLocation) location;
            if (!stdLoc.isOutputLocation()) {
                throw new IllegalArgumentException("Resource creation not supported in location " + stdLoc);
            }
        }
    }

    @Override // javax.annotation.processing.Filer
    public FileObject getResource(JavaFileManager.Location location, CharSequence pkg, CharSequence relativeName) throws IOException {
        FileObject fileObject;
        String strPkg = pkg.toString();
        if (strPkg.length() > 0) {
            checkName(strPkg);
        }
        if (location.isOutputLocation()) {
            fileObject = this.fileManager.getFileForOutput(location, pkg.toString(), relativeName.toString(), null);
        } else {
            fileObject = this.fileManager.getFileForInput(location, pkg.toString(), relativeName.toString());
        }
        if (fileObject == null) {
            String name = pkg.length() == 0 ? relativeName.toString() : ((Object) pkg) + OnBotJavaFileSystemUtils.PATH_SEPARATOR + ((Object) relativeName);
            throw new FileNotFoundException(name);
        }
        checkFileReopening(fileObject, false);
        return new FilerInputFileObject(fileObject);
    }

    private void checkName(String name) throws FilerException {
        checkName(name, false);
    }

    private void checkName(String name, boolean allowUnnamedPackageInfo) throws FilerException {
        if (!SourceVersion.isName(name) && !isPackageInfo(name, allowUnnamedPackageInfo)) {
            if (this.lint) {
                this.log.warning("proc.illegal.file.name", name);
            }
            throw new FilerException("Illegal name " + name);
        }
    }

    private boolean isPackageInfo(String name, boolean allowUnnamedPackageInfo) {
        int periodIndex = name.lastIndexOf(".");
        if (periodIndex == -1) {
            if (allowUnnamedPackageInfo) {
                return name.equals("package-info");
            }
            return false;
        }
        String prefix = name.substring(0, periodIndex);
        String simple = name.substring(periodIndex + 1);
        return SourceVersion.isName(prefix) && simple.equals("package-info");
    }

    private void checkNameAndExistence(String typename, boolean allowUnnamedPackageInfo) throws FilerException {
        checkName(typename, allowUnnamedPackageInfo);
        if (this.aggregateGeneratedSourceNames.contains(typename) || this.aggregateGeneratedClassNames.contains(typename)) {
            if (this.lint) {
                this.log.warning("proc.type.recreate", typename);
            }
            throw new FilerException("Attempt to recreate a file for type " + typename);
        }
    }

    private void checkFileReopening(FileObject fileObject, boolean addToHistory) throws FilerException {
        for (FileObject veteran : this.fileObjectHistory) {
            if (this.fileManager.isSameFile(veteran, fileObject)) {
                if (this.lint) {
                    this.log.warning("proc.file.reopening", fileObject.getName());
                }
                throw new FilerException("Attempt to reopen a file for path " + fileObject.getName());
            }
        }
        if (addToHistory) {
            this.fileObjectHistory.add(fileObject);
        }
    }

    public boolean newFiles() {
        return (this.generatedSourceNames.isEmpty() && this.generatedClasses.isEmpty()) ? false : true;
    }

    public Set<String> getGeneratedSourceNames() {
        return this.generatedSourceNames;
    }

    public Set<JavaFileObject> getGeneratedSourceFileObjects() {
        return this.generatedSourceFileObjects;
    }

    public Map<String, JavaFileObject> getGeneratedClasses() {
        return this.generatedClasses;
    }

    public void warnIfUnclosedFiles() {
        if (!this.openTypeNames.isEmpty()) {
            this.log.warning("proc.unclosed.type.files", this.openTypeNames.toString());
        }
    }

    public void newRound(Context context) {
        this.context = context;
        this.log = Log.instance(context);
        clearRoundState();
    }

    void setLastRound(boolean lastRound) {
        this.lastRound = lastRound;
    }

    @Override // java.io.Closeable, java.lang.AutoCloseable
    public void close() {
        clearRoundState();
        this.fileObjectHistory.clear();
        this.openTypeNames.clear();
        this.aggregateGeneratedSourceNames.clear();
        this.aggregateGeneratedClassNames.clear();
    }

    private void clearRoundState() {
        this.generatedSourceNames.clear();
        this.generatedSourceFileObjects.clear();
        this.generatedClasses.clear();
    }

    public void displayState() {
        PrintWriter xout = (PrintWriter) this.context.get(Log.outKey);
        xout.println("File Object History : " + this.fileObjectHistory);
        xout.println("Open Type Names     : " + this.openTypeNames);
        xout.println("Gen. Src Names      : " + this.generatedSourceNames);
        xout.println("Gen. Cls Names      : " + this.generatedClasses.keySet());
        xout.println("Agg. Gen. Src Names : " + this.aggregateGeneratedSourceNames);
        xout.println("Agg. Gen. Cls Names : " + this.aggregateGeneratedClassNames);
    }

    public String toString() {
        return "javac Filer";
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void closeFileObject(String typeName, FileObject fileObject) {
        if (typeName != null) {
            if (!(fileObject instanceof JavaFileObject)) {
                throw new AssertionError("JavaFileOject not found for " + fileObject);
            }
            JavaFileObject javaFileObject = (JavaFileObject) fileObject;
            switch (javaFileObject.getKind()) {
                case SOURCE:
                    this.generatedSourceNames.add(typeName);
                    this.generatedSourceFileObjects.add(javaFileObject);
                    this.openTypeNames.remove(typeName);
                    return;
                case CLASS:
                    this.generatedClasses.put(typeName, javaFileObject);
                    this.openTypeNames.remove(typeName);
                    return;
                default:
                    return;
            }
        }
    }
}
