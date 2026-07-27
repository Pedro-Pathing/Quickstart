package com.sun.tools.javac.api;

import com.sun.source.util.JavacTask;
import com.sun.tools.javac.file.JavacFileManager;
import com.sun.tools.javac.main.Main;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.main.OptionHelper;
import com.sun.tools.javac.util.ClientCodeException;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Options;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.util.Collections;
import java.util.EnumSet;
import java.util.Iterator;
import java.util.Locale;
import java.util.Set;
import javax.lang.model.SourceVersion;
import javax.tools.DiagnosticListener;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;

/* JADX INFO: loaded from: classes.dex */
public final class JavacTool implements JavaCompiler {
    @Override // javax.tools.JavaCompiler
    public /* bridge */ /* synthetic */ StandardJavaFileManager getStandardFileManager(DiagnosticListener diagnosticListener, Locale locale, Charset charset) {
        return getStandardFileManager((DiagnosticListener<? super JavaFileObject>) diagnosticListener, locale, charset);
    }

    @Override // javax.tools.JavaCompiler
    public /* bridge */ /* synthetic */ JavaCompiler.CompilationTask getTask(Writer writer, JavaFileManager javaFileManager, DiagnosticListener diagnosticListener, Iterable iterable, Iterable iterable2, Iterable iterable3) {
        return getTask(writer, javaFileManager, (DiagnosticListener<? super JavaFileObject>) diagnosticListener, (Iterable<String>) iterable, (Iterable<String>) iterable2, (Iterable<? extends JavaFileObject>) iterable3);
    }

    @Deprecated
    public JavacTool() {
    }

    public static JavacTool create() {
        return new JavacTool();
    }

    @Override // javax.tools.JavaCompiler
    public JavacFileManager getStandardFileManager(DiagnosticListener<? super JavaFileObject> diagnosticListener, Locale locale, Charset charset) {
        Context context = new Context();
        context.put((Class<Locale>) Locale.class, locale);
        if (diagnosticListener != null) {
            context.put((Class<DiagnosticListener<? super JavaFileObject>>) DiagnosticListener.class, diagnosticListener);
        }
        PrintWriter pw = charset == null ? new PrintWriter((OutputStream) System.err, true) : new PrintWriter((Writer) new OutputStreamWriter(System.err, charset), true);
        context.put(Log.outKey, pw);
        return new JavacFileManager(context, true, charset);
    }

    @Override // javax.tools.JavaCompiler
    public JavacTask getTask(Writer out, JavaFileManager fileManager, DiagnosticListener<? super JavaFileObject> diagnosticListener, Iterable<String> options, Iterable<String> classes, Iterable<? extends JavaFileObject> compilationUnits) {
        Context context = new Context();
        return getTask(out, fileManager, diagnosticListener, options, classes, compilationUnits, context);
    }

    public JavacTask getTask(Writer out, JavaFileManager fileManager, DiagnosticListener<? super JavaFileObject> diagnosticListener, Iterable<String> options, Iterable<String> classes, Iterable<? extends JavaFileObject> compilationUnits, Context context) {
        try {
            ClientCodeWrapper ccw = ClientCodeWrapper.instance(context);
            if (options != null) {
                for (String option : options) {
                    option.getClass();
                }
            }
            if (classes != null) {
                for (String cls : classes) {
                    if (!SourceVersion.isName(cls)) {
                        throw new IllegalArgumentException("Not a valid class name: " + cls);
                    }
                }
            }
            if (compilationUnits != null) {
                compilationUnits = ccw.wrapJavaFileObjects(compilationUnits);
                for (JavaFileObject cu : compilationUnits) {
                    if (cu.getKind() != JavaFileObject.Kind.SOURCE) {
                        String kindMsg = "Compilation unit is not of SOURCE kind: \"" + cu.getName() + "\"";
                        throw new IllegalArgumentException(kindMsg);
                    }
                }
            }
            if (diagnosticListener != null) {
                context.put((Class<DiagnosticListener>) DiagnosticListener.class, ccw.wrap(diagnosticListener));
            }
            if (out == null) {
                context.put(Log.outKey, new PrintWriter((OutputStream) System.err, true));
            } else {
                context.put(Log.outKey, new PrintWriter(out, true));
            }
            if (fileManager == null) {
                fileManager = getStandardFileManager(diagnosticListener, (Locale) null, (Charset) null);
            }
            JavaFileManager fileManager2 = ccw.wrap(fileManager);
            context.put((Class<JavaFileManager>) JavaFileManager.class, fileManager2);
            processOptions(context, fileManager2, options);
            Main compiler = new Main("javacTask", (PrintWriter) context.get(Log.outKey));
            return new JavacTaskImpl(compiler, options, context, classes, compilationUnits);
        } catch (ClientCodeException ex) {
            throw new RuntimeException(ex.getCause());
        }
    }

    public static void processOptions(Context context, JavaFileManager fileManager, Iterable<String> options) {
        if (options == null) {
            return;
        }
        final Options optionTable = Options.instance(context);
        Log log = Log.instance(context);
        Option[] recognizedOptions = (Option[]) Option.getJavacToolOptions().toArray(new Option[0]);
        OptionHelper optionHelper = new OptionHelper.GrumpyHelper(log) { // from class: com.sun.tools.javac.api.JavacTool.1
            @Override // com.sun.tools.javac.main.OptionHelper.GrumpyHelper, com.sun.tools.javac.main.OptionHelper
            public String get(Option option) {
                return optionTable.get(option.getText());
            }

            @Override // com.sun.tools.javac.main.OptionHelper.GrumpyHelper, com.sun.tools.javac.main.OptionHelper
            public void put(String name, String value) {
                optionTable.put(name, value);
            }

            @Override // com.sun.tools.javac.main.OptionHelper.GrumpyHelper, com.sun.tools.javac.main.OptionHelper
            public void remove(String name) {
                optionTable.remove(name);
            }
        };
        Iterator<String> flags = options.iterator();
        while (flags.hasNext()) {
            String flag = flags.next();
            int j = 0;
            while (j < recognizedOptions.length && !recognizedOptions[j].matches(flag)) {
                j++;
            }
            if (j == recognizedOptions.length) {
                if (!fileManager.handleOption(flag, flags)) {
                    String msg = log.localize(Log.PrefixKind.JAVAC, "err.invalid.flag", flag);
                    throw new IllegalArgumentException(msg);
                }
            } else {
                Option option = recognizedOptions[j];
                if (option.hasArg()) {
                    if (!flags.hasNext()) {
                        String msg2 = log.localize(Log.PrefixKind.JAVAC, "err.req.arg", flag);
                        throw new IllegalArgumentException(msg2);
                    }
                    String operand = flags.next();
                    if (option.process(optionHelper, flag, operand)) {
                        throw new IllegalArgumentException(flag + " " + operand);
                    }
                } else if (option.process(optionHelper, flag)) {
                    throw new IllegalArgumentException(flag);
                }
            }
        }
        optionTable.notifyListeners();
    }

    @Override // javax.tools.Tool
    public int run(InputStream in, OutputStream out, OutputStream err, String... arguments) {
        if (err == null) {
            err = System.err;
        }
        for (String argument : arguments) {
            argument.getClass();
        }
        return com.sun.tools.javac.Main.compile(arguments, new PrintWriter(err, true));
    }

    @Override // javax.tools.Tool
    public Set<SourceVersion> getSourceVersions() {
        return Collections.unmodifiableSet(EnumSet.range(SourceVersion.RELEASE_3, SourceVersion.latest()));
    }

    @Override // javax.tools.OptionChecker
    public int isSupportedOption(String str) {
        for (Option option : Option.getJavacToolOptions()) {
            if (option.matches(str)) {
                return option.hasArg() ? 1 : 0;
            }
        }
        return -1;
    }
}
