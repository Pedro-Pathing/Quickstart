package com.sun.tools.javac.api;

import com.sun.source.util.TaskEvent;
import com.sun.source.util.TaskListener;
import com.sun.tools.javac.util.ClientCodeException;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.JCDiagnostic;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.Reader;
import java.io.Writer;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.net.URI;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.NestingKind;
import javax.tools.Diagnostic;
import javax.tools.DiagnosticListener;
import javax.tools.FileObject;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class ClientCodeWrapper {
    Map<Class<?>, Boolean> trustedClasses = new HashMap();

    @Target({ElementType.TYPE})
    @Retention(RetentionPolicy.RUNTIME)
    public @interface Trusted {
    }

    public static ClientCodeWrapper instance(Context context) {
        ClientCodeWrapper instance = (ClientCodeWrapper) context.get(ClientCodeWrapper.class);
        if (instance == null) {
            return new ClientCodeWrapper(context);
        }
        return instance;
    }

    protected ClientCodeWrapper(Context context) {
    }

    public JavaFileManager wrap(JavaFileManager fm) {
        if (isTrusted(fm)) {
            return fm;
        }
        return new WrappedJavaFileManager(fm);
    }

    public FileObject wrap(FileObject fo) {
        if (isTrusted(fo)) {
            return fo;
        }
        return new WrappedFileObject(fo);
    }

    FileObject unwrap(FileObject fo) {
        if (fo instanceof WrappedFileObject) {
            return ((WrappedFileObject) fo).clientFileObject;
        }
        return fo;
    }

    public JavaFileObject wrap(JavaFileObject fo) {
        if (isTrusted(fo)) {
            return fo;
        }
        return new WrappedJavaFileObject(fo);
    }

    public Iterable<JavaFileObject> wrapJavaFileObjects(Iterable<? extends JavaFileObject> list) {
        List<JavaFileObject> wrapped = new ArrayList<>();
        for (JavaFileObject fo : list) {
            wrapped.add(wrap(fo));
        }
        return Collections.unmodifiableList(wrapped);
    }

    JavaFileObject unwrap(JavaFileObject fo) {
        if (fo instanceof WrappedJavaFileObject) {
            return (JavaFileObject) ((WrappedJavaFileObject) fo).clientFileObject;
        }
        return fo;
    }

    public <T> DiagnosticListener<T> wrap(DiagnosticListener<T> dl) {
        if (isTrusted(dl)) {
            return dl;
        }
        return new WrappedDiagnosticListener(dl);
    }

    TaskListener wrap(TaskListener tl) {
        if (isTrusted(tl)) {
            return tl;
        }
        return new WrappedTaskListener(tl);
    }

    TaskListener unwrap(TaskListener l) {
        if (l instanceof WrappedTaskListener) {
            return ((WrappedTaskListener) l).clientTaskListener;
        }
        return l;
    }

    Collection<TaskListener> unwrap(Collection<? extends TaskListener> listeners) {
        Collection<TaskListener> c = new ArrayList<>(listeners.size());
        for (TaskListener l : listeners) {
            c.add(unwrap(l));
        }
        return c;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public <T> Diagnostic<T> unwrap(Diagnostic<T> diagnostic) {
        if (diagnostic instanceof JCDiagnostic) {
            JCDiagnostic d = (JCDiagnostic) diagnostic;
            return new DiagnosticSourceUnwrapper(d);
        }
        return diagnostic;
    }

    protected boolean isTrusted(Object o) {
        Class<?> c = o.getClass();
        Boolean trusted = this.trustedClasses.get(c);
        if (trusted == null) {
            trusted = Boolean.valueOf(c.getName().startsWith("com.sun.tools.javac.") || c.isAnnotationPresent(Trusted.class));
            this.trustedClasses.put(c, trusted);
        }
        return trusted.booleanValue();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public String wrappedToString(Class<?> wrapperClass, Object wrapped) {
        return wrapperClass.getSimpleName() + "[" + wrapped + "]";
    }

    protected class WrappedJavaFileManager implements JavaFileManager {
        protected JavaFileManager clientJavaFileManager;

        WrappedJavaFileManager(JavaFileManager clientJavaFileManager) {
            clientJavaFileManager.getClass();
            this.clientJavaFileManager = clientJavaFileManager;
        }

        @Override // javax.tools.JavaFileManager
        public ClassLoader getClassLoader(JavaFileManager.Location location) {
            try {
                return this.clientJavaFileManager.getClassLoader(location);
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager
        public Iterable<JavaFileObject> list(JavaFileManager.Location location, String packageName, Set<JavaFileObject.Kind> kinds, boolean recurse) throws IOException {
            try {
                return ClientCodeWrapper.this.wrapJavaFileObjects(this.clientJavaFileManager.list(location, packageName, kinds, recurse));
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager
        public String inferBinaryName(JavaFileManager.Location location, JavaFileObject file) {
            try {
                return this.clientJavaFileManager.inferBinaryName(location, ClientCodeWrapper.this.unwrap(file));
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager
        public boolean isSameFile(FileObject a, FileObject b) {
            try {
                return this.clientJavaFileManager.isSameFile(ClientCodeWrapper.this.unwrap(a), ClientCodeWrapper.this.unwrap(b));
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager
        public boolean handleOption(String current, Iterator<String> remaining) {
            try {
                return this.clientJavaFileManager.handleOption(current, remaining);
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager
        public boolean hasLocation(JavaFileManager.Location location) {
            try {
                return this.clientJavaFileManager.hasLocation(location);
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager
        public JavaFileObject getJavaFileForInput(JavaFileManager.Location location, String className, JavaFileObject.Kind kind) throws IOException {
            try {
                return ClientCodeWrapper.this.wrap(this.clientJavaFileManager.getJavaFileForInput(location, className, kind));
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager
        public JavaFileObject getJavaFileForOutput(JavaFileManager.Location location, String className, JavaFileObject.Kind kind, FileObject sibling) throws IOException {
            try {
                return ClientCodeWrapper.this.wrap(this.clientJavaFileManager.getJavaFileForOutput(location, className, kind, ClientCodeWrapper.this.unwrap(sibling)));
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager
        public FileObject getFileForInput(JavaFileManager.Location location, String packageName, String relativeName) throws IOException {
            try {
                return ClientCodeWrapper.this.wrap(this.clientJavaFileManager.getFileForInput(location, packageName, relativeName));
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager
        public FileObject getFileForOutput(JavaFileManager.Location location, String packageName, String relativeName, FileObject sibling) throws IOException {
            try {
                return ClientCodeWrapper.this.wrap(this.clientJavaFileManager.getFileForOutput(location, packageName, relativeName, ClientCodeWrapper.this.unwrap(sibling)));
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager, java.io.Flushable
        public void flush() throws IOException {
            try {
                this.clientJavaFileManager.flush();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileManager, java.io.Closeable, java.lang.AutoCloseable
        public void close() throws IOException {
            try {
                this.clientJavaFileManager.close();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.OptionChecker
        public int isSupportedOption(String option) {
            try {
                return this.clientJavaFileManager.isSupportedOption(option);
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        public String toString() {
            return ClientCodeWrapper.this.wrappedToString(getClass(), this.clientJavaFileManager);
        }
    }

    protected class WrappedFileObject implements FileObject {
        protected FileObject clientFileObject;

        WrappedFileObject(FileObject clientFileObject) {
            clientFileObject.getClass();
            this.clientFileObject = clientFileObject;
        }

        @Override // javax.tools.FileObject
        public URI toUri() {
            try {
                return this.clientFileObject.toUri();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.FileObject
        public String getName() {
            try {
                return this.clientFileObject.getName();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.FileObject
        public InputStream openInputStream() throws IOException {
            try {
                return this.clientFileObject.openInputStream();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.FileObject
        public OutputStream openOutputStream() throws IOException {
            try {
                return this.clientFileObject.openOutputStream();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.FileObject
        public Reader openReader(boolean ignoreEncodingErrors) throws IOException {
            try {
                return this.clientFileObject.openReader(ignoreEncodingErrors);
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.FileObject
        public CharSequence getCharContent(boolean ignoreEncodingErrors) throws IOException {
            try {
                return this.clientFileObject.getCharContent(ignoreEncodingErrors);
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.FileObject
        public Writer openWriter() throws IOException {
            try {
                return this.clientFileObject.openWriter();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.FileObject
        public long getLastModified() {
            try {
                return this.clientFileObject.getLastModified();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.FileObject
        public boolean delete() {
            try {
                return this.clientFileObject.delete();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        public String toString() {
            return ClientCodeWrapper.this.wrappedToString(getClass(), this.clientFileObject);
        }
    }

    protected class WrappedJavaFileObject extends WrappedFileObject implements JavaFileObject {
        WrappedJavaFileObject(JavaFileObject clientJavaFileObject) {
            super(clientJavaFileObject);
        }

        @Override // javax.tools.JavaFileObject
        public JavaFileObject.Kind getKind() {
            try {
                return ((JavaFileObject) this.clientFileObject).getKind();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileObject
        public boolean isNameCompatible(String simpleName, JavaFileObject.Kind kind) {
            try {
                return ((JavaFileObject) this.clientFileObject).isNameCompatible(simpleName, kind);
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileObject
        public NestingKind getNestingKind() {
            try {
                return ((JavaFileObject) this.clientFileObject).getNestingKind();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // javax.tools.JavaFileObject
        public Modifier getAccessLevel() {
            try {
                return ((JavaFileObject) this.clientFileObject).getAccessLevel();
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // com.sun.tools.javac.api.ClientCodeWrapper.WrappedFileObject
        public String toString() {
            return ClientCodeWrapper.this.wrappedToString(getClass(), this.clientFileObject);
        }
    }

    protected class WrappedDiagnosticListener<T> implements DiagnosticListener<T> {
        protected DiagnosticListener<T> clientDiagnosticListener;

        WrappedDiagnosticListener(DiagnosticListener<T> clientDiagnosticListener) {
            clientDiagnosticListener.getClass();
            this.clientDiagnosticListener = clientDiagnosticListener;
        }

        @Override // javax.tools.DiagnosticListener
        public void report(Diagnostic<? extends T> diagnostic) {
            try {
                this.clientDiagnosticListener.report(ClientCodeWrapper.this.unwrap(diagnostic));
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        public String toString() {
            return ClientCodeWrapper.this.wrappedToString(getClass(), this.clientDiagnosticListener);
        }
    }

    public class DiagnosticSourceUnwrapper implements Diagnostic<JavaFileObject> {
        public final JCDiagnostic d;

        DiagnosticSourceUnwrapper(JCDiagnostic d) {
            this.d = d;
        }

        @Override // javax.tools.Diagnostic
        public Diagnostic.Kind getKind() {
            return this.d.getKind();
        }

        @Override // javax.tools.Diagnostic
        public JavaFileObject getSource() {
            return ClientCodeWrapper.this.unwrap(this.d.getSource());
        }

        @Override // javax.tools.Diagnostic
        public long getPosition() {
            return this.d.getPosition();
        }

        @Override // javax.tools.Diagnostic
        public long getStartPosition() {
            return this.d.getStartPosition();
        }

        @Override // javax.tools.Diagnostic
        public long getEndPosition() {
            return this.d.getEndPosition();
        }

        @Override // javax.tools.Diagnostic
        public long getLineNumber() {
            return this.d.getLineNumber();
        }

        @Override // javax.tools.Diagnostic
        public long getColumnNumber() {
            return this.d.getColumnNumber();
        }

        @Override // javax.tools.Diagnostic
        public String getCode() {
            return this.d.getCode();
        }

        @Override // javax.tools.Diagnostic
        public String getMessage(Locale locale) {
            return this.d.getMessage(locale);
        }

        public String toString() {
            return this.d.toString();
        }
    }

    protected class WrappedTaskListener implements TaskListener {
        protected TaskListener clientTaskListener;

        WrappedTaskListener(TaskListener clientTaskListener) {
            clientTaskListener.getClass();
            this.clientTaskListener = clientTaskListener;
        }

        @Override // com.sun.source.util.TaskListener
        public void started(TaskEvent ev) {
            try {
                this.clientTaskListener.started(ev);
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        @Override // com.sun.source.util.TaskListener
        public void finished(TaskEvent ev) {
            try {
                this.clientTaskListener.finished(ev);
            } catch (ClientCodeException e) {
                throw e;
            } catch (Error e2) {
                throw new ClientCodeException(e2);
            } catch (RuntimeException e3) {
                throw new ClientCodeException(e3);
            }
        }

        public String toString() {
            return ClientCodeWrapper.this.wrappedToString(getClass(), this.clientTaskListener);
        }
    }
}
