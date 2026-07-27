package com.sun.tools.javac.main;

import com.android.tools.r8.DataResource;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.file.JavacFileManager;
import com.sun.tools.javac.jvm.Profile;
import com.sun.tools.javac.jvm.Target;
import com.sun.tools.javac.processing.AnnotationProcessingError;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.PropagatedException;
import dk.sgjesse.r8api.DescriptorUtils;
import dk.sgjesse.r8api.FileUtils;
import java.io.File;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.URL;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.util.Arrays;
import java.util.Collection;
import java.util.Set;
import javax.annotation.processing.Processor;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class Main {
    public static final String javacBundleName = "com.sun.tools.javac.resources.javac";
    boolean apiMode;
    public ListBuffer<String> classnames;
    private JavaFileManager fileManager;
    public Set<File> filenames;
    public Log log;
    private OptionHelper optionHelper;
    private Options options;
    PrintWriter out;
    String ownName;
    private Option[] recognizedOptions;

    public enum Result {
        OK(0),
        ERROR(1),
        CMDERR(2),
        SYSERR(3),
        ABNORMAL(4);

        public final int exitCode;

        Result(int exitCode) {
            this.exitCode = exitCode;
        }

        public boolean isOK() {
            return this.exitCode == 0;
        }
    }

    public Main(String name) {
        this(name, new PrintWriter((OutputStream) System.err, true));
    }

    public Main(String name, PrintWriter out) {
        this.recognizedOptions = (Option[]) Option.getJavaCompilerOptions().toArray(new Option[0]);
        this.optionHelper = new OptionHelper() { // from class: com.sun.tools.javac.main.Main.1
            @Override // com.sun.tools.javac.main.OptionHelper
            public String get(Option option) {
                return Main.this.options.get(option);
            }

            @Override // com.sun.tools.javac.main.OptionHelper
            public void put(String name2, String value) {
                Main.this.options.put(name2, value);
            }

            @Override // com.sun.tools.javac.main.OptionHelper
            public void remove(String name2) {
                Main.this.options.remove(name2);
            }

            @Override // com.sun.tools.javac.main.OptionHelper
            public Log getLog() {
                return Main.this.log;
            }

            @Override // com.sun.tools.javac.main.OptionHelper
            public String getOwnName() {
                return Main.this.ownName;
            }

            @Override // com.sun.tools.javac.main.OptionHelper
            public void error(String key, Object... args) {
                Main.this.error(key, args);
            }

            @Override // com.sun.tools.javac.main.OptionHelper
            public void addFile(File f) {
                Main.this.filenames.add(f);
            }

            @Override // com.sun.tools.javac.main.OptionHelper
            public void addClassName(String s) {
                Main.this.classnames.append(s);
            }
        };
        this.options = null;
        this.filenames = null;
        this.classnames = null;
        this.ownName = name;
        this.out = out;
    }

    void error(String key, Object... args) {
        if (this.apiMode) {
            String msg = this.log.localize(Log.PrefixKind.JAVAC, key, args);
            throw new PropagatedException(new IllegalStateException(msg));
        }
        warning(key, args);
        this.log.printLines(Log.PrefixKind.JAVAC, "msg.usage", this.ownName);
    }

    void warning(String key, Object... args) {
        this.log.printRawLines(this.ownName + ": " + this.log.localize(Log.PrefixKind.JAVAC, key, args));
    }

    public Option getOption(String flag) {
        for (Option option : this.recognizedOptions) {
            if (option.matches(flag)) {
                return option;
            }
        }
        return null;
    }

    public void setOptions(Options options) {
        if (options == null) {
            throw new NullPointerException();
        }
        this.options = options;
    }

    public void setAPIMode(boolean apiMode) {
        this.apiMode = apiMode;
    }

    public Collection<File> processArgs(String[] flags) {
        return processArgs(flags, null);
    }

    public Collection<File> processArgs(String[] flags, String[] classNames) {
        int ac = 0;
        while (true) {
            if (ac < flags.length) {
                String flag = flags[ac];
                ac++;
                Option option = null;
                if (flag.length() > 0) {
                    int firstOptionToCheck = flag.charAt(0) != '-' ? this.recognizedOptions.length - 1 : 0;
                    int j = firstOptionToCheck;
                    while (true) {
                        if (j >= this.recognizedOptions.length) {
                            break;
                        }
                        if (!this.recognizedOptions[j].matches(flag)) {
                            j++;
                        } else {
                            option = this.recognizedOptions[j];
                            break;
                        }
                    }
                }
                if (option == null) {
                    error("err.invalid.flag", flag);
                    return null;
                }
                if (option.hasArg()) {
                    if (ac == flags.length) {
                        error("err.req.arg", flag);
                        return null;
                    }
                    String operand = flags[ac];
                    ac++;
                    if (option.process(this.optionHelper, flag, operand)) {
                        return null;
                    }
                } else if (option.process(this.optionHelper, flag)) {
                    return null;
                }
            } else {
                if (this.options.get(Option.PROFILE) != null && this.options.get(Option.BOOTCLASSPATH) != null) {
                    error("err.profile.bootclasspath.conflict", new Object[0]);
                    return null;
                }
                if (this.classnames != null && classNames != null) {
                    this.classnames.addAll(Arrays.asList(classNames));
                }
                if (!checkDirectory(Option.D) || !checkDirectory(Option.S)) {
                    return null;
                }
                String sourceString = this.options.get(Option.SOURCE);
                Source source = sourceString != null ? Source.lookup(sourceString) : Source.DEFAULT;
                String targetString = this.options.get(Option.TARGET);
                Target target = targetString != null ? Target.lookup(targetString) : Target.DEFAULT;
                if (Character.isDigit(target.name.charAt(0))) {
                    if (target.compareTo(source.requiredTarget()) < 0) {
                        if (targetString != null) {
                            if (sourceString == null) {
                                warning("warn.target.default.source.conflict", targetString, source.requiredTarget().name);
                            } else {
                                warning("warn.source.target.conflict", sourceString, source.requiredTarget().name);
                            }
                            return null;
                        }
                        target = source.requiredTarget();
                        this.options.put("-target", target.name);
                    } else if (targetString == null && !source.allowGenerics()) {
                        target = Target.JDK1_4;
                        this.options.put("-target", target.name);
                    }
                }
                String profileString = this.options.get(Option.PROFILE);
                if (profileString != null) {
                    Profile profile = Profile.lookup(profileString);
                    if (!profile.isValid(target)) {
                        warning("warn.profile.target.conflict", profileString, target.name);
                        return null;
                    }
                }
                String showClass = this.options.get("showClass");
                if (showClass != null) {
                    if (showClass.equals("showClass")) {
                        showClass = "com.sun.tools.javac.Main";
                    }
                    showClass(showClass);
                }
                this.options.notifyListeners();
                return this.filenames;
            }
        }
    }

    private boolean checkDirectory(Option option) {
        String value = this.options.get(option);
        if (value == null) {
            return true;
        }
        File file = new File(value);
        if (!file.exists()) {
            error("err.dir.not.found", value);
            return false;
        }
        if (file.isDirectory()) {
            return true;
        }
        error("err.file.not.directory", value);
        return false;
    }

    public Result compile(String[] args) {
        Context context = new Context();
        JavacFileManager.preRegister(context);
        Result result = compile(args, context);
        if (this.fileManager instanceof JavacFileManager) {
            ((JavacFileManager) this.fileManager).close();
        }
        return result;
    }

    public Result compile(String[] args, Context context) {
        return compile(args, context, List.nil(), null);
    }

    public Result compile(String[] args, Context context, List<JavaFileObject> fileObjects, Iterable<? extends Processor> processors) {
        return compile(args, null, context, fileObjects, processors);
    }

    /* JADX WARN: Removed duplicated region for block: B:282:0x04c7 A[Catch: all -> 0x05d0, TRY_ENTER, TryCatch #25 {all -> 0x05d0, blocks: (B:282:0x04c7, B:284:0x04cd, B:286:0x04d1, B:289:0x04de, B:288:0x04db, B:300:0x04fe, B:301:0x0502, B:304:0x0509, B:305:0x0512, B:308:0x0519, B:310:0x051d, B:319:0x053a, B:320:0x0543, B:323:0x054a, B:334:0x056d, B:345:0x0590, B:356:0x05b3), top: B:390:0x0031 }] */
    /* JADX WARN: Removed duplicated region for block: B:288:0x04db A[Catch: all -> 0x05d0, TryCatch #25 {all -> 0x05d0, blocks: (B:282:0x04c7, B:284:0x04cd, B:286:0x04d1, B:289:0x04de, B:288:0x04db, B:300:0x04fe, B:301:0x0502, B:304:0x0509, B:305:0x0512, B:308:0x0519, B:310:0x051d, B:319:0x053a, B:320:0x0543, B:323:0x054a, B:334:0x056d, B:345:0x0590, B:356:0x05b3), top: B:390:0x0031 }] */
    /* JADX WARN: Removed duplicated region for block: B:382:0x04e2 A[EXC_TOP_SPLITTER, SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:386:0x0551 A[EXC_TOP_SPLITTER, SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:391:0x05ba A[EXC_TOP_SPLITTER, SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:401:0x0597 A[EXC_TOP_SPLITTER, SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:409:0x0574 A[EXC_TOP_SPLITTER, SYNTHETIC] */
    /* JADX WARN: Type inference fix 'apply assigned field type' failed
    java.lang.UnsupportedOperationException: ArgType.getObject(), call class: class jadx.core.dex.instructions.args.ArgType$ArrayArg
    	at jadx.core.dex.instructions.args.ArgType.getObject(ArgType.java:593)
    	at jadx.core.dex.attributes.nodes.ClassTypeVarsAttr.getTypeVarsMapFor(ClassTypeVarsAttr.java:35)
    	at jadx.core.dex.nodes.utils.TypeUtils.replaceClassGenerics(TypeUtils.java:177)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.insertExplicitUseCast(FixTypesVisitor.java:397)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.tryFieldTypeWithNewCasts(FixTypesVisitor.java:359)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.applyFieldType(FixTypesVisitor.java:309)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.visit(FixTypesVisitor.java:94)
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public com.sun.tools.javac.main.Main.Result compile(java.lang.String[] r24, java.lang.String[] r25, com.sun.tools.javac.util.Context r26, com.sun.tools.javac.util.List<javax.tools.JavaFileObject> r27, java.lang.Iterable<? extends javax.annotation.processing.Processor> r28) {
        /*
            Method dump skipped, instruction units count: 1514
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.main.Main.compile(java.lang.String[], java.lang.String[], com.sun.tools.javac.util.Context, com.sun.tools.javac.util.List, java.lang.Iterable):com.sun.tools.javac.main.Main$Result");
    }

    void bugMessage(Throwable ex) {
        this.log.printLines(Log.PrefixKind.JAVAC, "msg.bug", JavaCompiler.version());
        ex.printStackTrace(this.log.getWriter(Log.WriterKind.NOTICE));
    }

    void feMessage(Throwable ex) {
        this.log.printRawLines(ex.getMessage());
        if (ex.getCause() != null && this.options.isSet("dev")) {
            ex.getCause().printStackTrace(this.log.getWriter(Log.WriterKind.NOTICE));
        }
    }

    void ioMessage(Throwable ex) {
        this.log.printLines(Log.PrefixKind.JAVAC, "msg.io", new Object[0]);
        ex.printStackTrace(this.log.getWriter(Log.WriterKind.NOTICE));
    }

    void resourceMessage(Throwable ex) {
        this.log.printLines(Log.PrefixKind.JAVAC, "msg.resource", new Object[0]);
        ex.printStackTrace(this.log.getWriter(Log.WriterKind.NOTICE));
    }

    void apMessage(AnnotationProcessingError ex) {
        this.log.printLines(Log.PrefixKind.JAVAC, "msg.proc.annotation.uncaught.exception", new Object[0]);
        ex.getCause().printStackTrace(this.log.getWriter(Log.WriterKind.NOTICE));
    }

    void pluginMessage(Throwable ex) {
        this.log.printLines(Log.PrefixKind.JAVAC, "msg.plugin.uncaught.exception", new Object[0]);
        ex.printStackTrace(this.log.getWriter(Log.WriterKind.NOTICE));
    }

    void showClass(String className) {
        int n;
        PrintWriter pw = this.log.getWriter(Log.WriterKind.NOTICE);
        pw.println("javac: show class: " + className);
        URL url = getClass().getResource(DataResource.SEPARATOR + className.replace(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, DataResource.SEPARATOR) + FileUtils.CLASS_EXTENSION);
        if (url == null) {
            pw.println("  class not found");
            return;
        }
        pw.println("  " + url);
        try {
            MessageDigest md = MessageDigest.getInstance("MD5");
            DigestInputStream in = new DigestInputStream(url.openStream(), md);
            try {
                byte[] buf = new byte[8192];
                do {
                    n = in.read(buf);
                } while (n > 0);
                byte[] digest = md.digest();
                in.close();
                StringBuilder sb = new StringBuilder();
                for (byte b : digest) {
                    sb.append(String.format("%02x", Byte.valueOf(b)));
                }
                pw.println("  MD5 checksum: " + ((Object) sb));
            } catch (Throwable th) {
                in.close();
                throw th;
            }
        } catch (Exception e) {
            pw.println("  cannot compute digest: " + e);
        }
    }
}
