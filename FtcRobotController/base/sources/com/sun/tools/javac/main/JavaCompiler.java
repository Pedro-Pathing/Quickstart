package com.sun.tools.javac.main;

import com.sun.source.util.TaskEvent;
import com.sun.tools.javac.api.MultiTaskListener;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.Annotate;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Check;
import com.sun.tools.javac.comp.CompileStates;
import com.sun.tools.javac.comp.Enter;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.comp.Flow;
import com.sun.tools.javac.comp.LambdaToMethod;
import com.sun.tools.javac.comp.Lower;
import com.sun.tools.javac.comp.Todo;
import com.sun.tools.javac.comp.TransTypes;
import com.sun.tools.javac.file.JavacFileManager;
import com.sun.tools.javac.jvm.ClassReader;
import com.sun.tools.javac.jvm.ClassReader.BadClassFile;
import com.sun.tools.javac.jvm.ClassWriter;
import com.sun.tools.javac.jvm.Gen;
import com.sun.tools.javac.jvm.JNIWriter;
import com.sun.tools.javac.jvm.Target;
import com.sun.tools.javac.parser.Parser;
import com.sun.tools.javac.parser.ParserFactory;
import com.sun.tools.javac.processing.JavacProcessingEnvironment;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.Pretty;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.tree.TreeTranslator;
import com.sun.tools.javac.util.Abort;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.BaseFileManager;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.FatalError;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.Pair;
import com.sun.tools.javac.util.RichDiagnosticFormatter;
import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.IOException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.MissingResourceException;
import java.util.Queue;
import java.util.ResourceBundle;
import java.util.Set;
import javax.annotation.processing.Processor;
import javax.lang.model.SourceVersion;
import javax.tools.Diagnostic;
import javax.tools.DiagnosticListener;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.StandardLocation;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class JavaCompiler {
    private static ResourceBundle versionRB = null;
    private static final String versionRBName = "com.sun.tools.javac.resources.version";
    protected Annotate annotate;
    protected boolean annotationProcessingOccurred;
    protected Attr attr;
    public boolean attrParseOnly;
    protected Check chk;
    protected CompilePolicy compilePolicy;
    protected CompileStates compileStates;
    protected final Name completionFailureName;
    protected Context context;
    Log.DeferredDiagnosticHandler deferredDiagnosticHandler;
    protected JavaCompiler delegateCompiler;
    protected boolean devVerbose;
    JCDiagnostic.Factory diagFactory;
    public String encoding;
    protected Enter enter;
    protected JavaFileManager fileManager;
    protected Flow flow;
    protected Gen gen;
    public boolean genEndPos;
    protected boolean implicitSourceFilesRead;
    protected ImplicitSourcePolicy implicitSourcePolicy;
    protected JNIWriter jniWriter;
    public boolean lineDebugInfo;
    public Log log;
    protected Lower lower;
    protected TreeMaker make;
    protected Names names;
    protected Options options;
    protected ParserFactory parserFactory;
    public boolean printFlat;
    protected boolean processPcks;
    protected ClassReader reader;
    boolean relax;
    private List<JCTree.JCClassDecl> rootClasses;
    public CompileStates.CompileState shouldStopPolicyIfError;
    public CompileStates.CompileState shouldStopPolicyIfNoError;
    protected Source source;
    public boolean sourceOutput;
    public boolean stubOutput;
    protected Symtab syms;
    protected MultiTaskListener taskListener;
    public Todo todo;
    protected TransTypes transTypes;
    protected Types types;
    public boolean verbose;
    public boolean verboseCompilePolicy;
    protected boolean werror;
    protected ClassWriter writer;
    protected static final Context.Key<JavaCompiler> compilerKey = new Context.Key<>();
    private static final CompilePolicy DEFAULT_COMPILE_POLICY = CompilePolicy.BY_TODO;
    protected final ClassReader.SourceCompleter thisCompleter = new ClassReader.SourceCompleter() { // from class: com.sun.tools.javac.main.JavaCompiler.1
        @Override // com.sun.tools.javac.jvm.ClassReader.SourceCompleter
        public void complete(Symbol.ClassSymbol sym) throws Symbol.CompletionFailure {
            JavaCompiler.this.complete(sym);
        }
    };
    protected boolean explicitAnnotationProcessingRequested = false;
    public List<Closeable> closeables = List.nil();
    protected Set<JavaFileObject> inputFiles = new HashSet();
    public boolean keepComments = false;
    private boolean hasBeenUsed = false;
    private long start_msec = 0;
    public long elapsed_msec = 0;
    protected boolean needRootClasses = false;
    boolean processAnnotations = false;
    private JavacProcessingEnvironment procEnvImpl = null;
    HashMap<Env<AttrContext>, Queue<Pair<Env<AttrContext>, JCTree.JCClassDecl>>> desugaredEnvs = new HashMap<>();

    public static JavaCompiler instance(Context context) {
        JavaCompiler instance = (JavaCompiler) context.get(compilerKey);
        if (instance == null) {
            return new JavaCompiler(context);
        }
        return instance;
    }

    public static String version() {
        return version("release");
    }

    public static String fullVersion() {
        return version("full");
    }

    private static String version(String key) {
        if (versionRB == null) {
            try {
                versionRB = ResourceBundle.getBundle(versionRBName);
            } catch (MissingResourceException e) {
                return Log.getLocalizedString("version.not.available", new Object[0]);
            }
        }
        try {
            return versionRB.getString(key);
        } catch (MissingResourceException e2) {
            return Log.getLocalizedString("version.not.available", new Object[0]);
        }
    }

    protected enum CompilePolicy {
        ATTR_ONLY,
        CHECK_ONLY,
        SIMPLE,
        BY_FILE,
        BY_TODO;

        static CompilePolicy decode(String option) {
            if (option == null) {
                return JavaCompiler.DEFAULT_COMPILE_POLICY;
            }
            if (option.equals("attr")) {
                return ATTR_ONLY;
            }
            if (option.equals("check")) {
                return CHECK_ONLY;
            }
            if (option.equals("simple")) {
                return SIMPLE;
            }
            if (option.equals("byfile")) {
                return BY_FILE;
            }
            if (!option.equals("bytodo")) {
                return JavaCompiler.DEFAULT_COMPILE_POLICY;
            }
            return BY_TODO;
        }
    }

    protected enum ImplicitSourcePolicy {
        NONE,
        CLASS,
        UNSET;

        static ImplicitSourcePolicy decode(String option) {
            if (option == null) {
                return UNSET;
            }
            if (option.equals("none")) {
                return NONE;
            }
            if (option.equals("class")) {
                return CLASS;
            }
            return UNSET;
        }
    }

    public JavaCompiler(Context context) {
        CompileStates.CompileState compileStateValueOf;
        this.context = context;
        context.put(compilerKey, this);
        if (context.get(JavaFileManager.class) == null) {
            JavacFileManager.preRegister(context);
        }
        this.names = Names.instance(context);
        this.log = Log.instance(context);
        this.diagFactory = JCDiagnostic.Factory.instance(context);
        this.reader = ClassReader.instance(context);
        this.make = TreeMaker.instance(context);
        this.writer = ClassWriter.instance(context);
        this.jniWriter = JNIWriter.instance(context);
        this.enter = Enter.instance(context);
        this.todo = Todo.instance(context);
        this.fileManager = (JavaFileManager) context.get(JavaFileManager.class);
        this.parserFactory = ParserFactory.instance(context);
        this.compileStates = CompileStates.instance(context);
        try {
            this.syms = Symtab.instance(context);
        } catch (Symbol.CompletionFailure ex) {
            this.log.error("cant.access", ex.sym, ex.getDetailValue());
            if (ex instanceof ClassReader.BadClassFile) {
                throw new Abort();
            }
        }
        this.source = Source.instance(context);
        Target target = Target.instance(context);
        this.attr = Attr.instance(context);
        this.chk = Check.instance(context);
        this.gen = Gen.instance(context);
        this.flow = Flow.instance(context);
        this.transTypes = TransTypes.instance(context);
        this.lower = Lower.instance(context);
        this.annotate = Annotate.instance(context);
        this.types = Types.instance(context);
        this.taskListener = MultiTaskListener.instance(context);
        this.reader.sourceCompleter = this.thisCompleter;
        this.options = Options.instance(context);
        this.verbose = this.options.isSet(Option.VERBOSE);
        this.sourceOutput = this.options.isSet(Option.PRINTSOURCE);
        this.stubOutput = this.options.isSet("-stubs");
        this.relax = this.options.isSet("-relax");
        this.printFlat = this.options.isSet("-printflat");
        this.attrParseOnly = this.options.isSet("-attrparseonly");
        this.encoding = this.options.get(Option.ENCODING);
        this.lineDebugInfo = this.options.isUnset(Option.G_CUSTOM) || this.options.isSet(Option.G_CUSTOM, "lines");
        this.genEndPos = this.options.isSet(Option.XJCOV) || context.get(DiagnosticListener.class) != null;
        this.devVerbose = this.options.isSet("dev");
        this.processPcks = this.options.isSet("process.packages");
        this.werror = this.options.isSet(Option.WERROR);
        if (this.source.compareTo(Source.DEFAULT) < 0 && this.options.isUnset(Option.XLINT_CUSTOM, "-" + Lint.LintCategory.OPTIONS.option) && (this.fileManager instanceof BaseFileManager) && ((BaseFileManager) this.fileManager).isDefaultBootClassPath()) {
            this.log.warning(Lint.LintCategory.OPTIONS, "source.no.bootclasspath", this.source.name);
        }
        checkForObsoleteOptions(target);
        this.verboseCompilePolicy = this.options.isSet("verboseCompilePolicy");
        if (this.attrParseOnly) {
            this.compilePolicy = CompilePolicy.ATTR_ONLY;
        } else {
            this.compilePolicy = CompilePolicy.decode(this.options.get("compilePolicy"));
        }
        this.implicitSourcePolicy = ImplicitSourcePolicy.decode(this.options.get("-implicit"));
        this.completionFailureName = this.options.isSet("failcomplete") ? this.names.fromString(this.options.get("failcomplete")) : null;
        if (this.options.isSet("shouldStopPolicy")) {
            compileStateValueOf = CompileStates.CompileState.valueOf(this.options.get("shouldStopPolicy"));
        } else {
            compileStateValueOf = this.options.isSet("shouldStopPolicyIfError") ? CompileStates.CompileState.valueOf(this.options.get("shouldStopPolicyIfError")) : CompileStates.CompileState.INIT;
        }
        this.shouldStopPolicyIfError = compileStateValueOf;
        this.shouldStopPolicyIfNoError = this.options.isSet("shouldStopPolicyIfNoError") ? CompileStates.CompileState.valueOf(this.options.get("shouldStopPolicyIfNoError")) : CompileStates.CompileState.GENERATE;
        if (this.options.isUnset("oldDiags")) {
            this.log.setDiagnosticFormatter(RichDiagnosticFormatter.instance(context));
        }
    }

    private void checkForObsoleteOptions(Target target) {
        boolean obsoleteOptionFound = false;
        if (this.options.isUnset(Option.XLINT_CUSTOM, "-" + Lint.LintCategory.OPTIONS.option)) {
            if (this.source.compareTo(Source.JDK1_5) <= 0) {
                this.log.warning(Lint.LintCategory.OPTIONS, "option.obsolete.source", this.source.name);
                obsoleteOptionFound = true;
            }
            if (target.compareTo(Target.JDK1_5) <= 0) {
                this.log.warning(Lint.LintCategory.OPTIONS, "option.obsolete.target", target.name);
                obsoleteOptionFound = true;
            }
            if (obsoleteOptionFound) {
                this.log.warning(Lint.LintCategory.OPTIONS, "option.obsolete.suppression", new Object[0]);
            }
        }
    }

    protected boolean shouldStop(CompileStates.CompileState cs) {
        CompileStates.CompileState shouldStopPolicy = (errorCount() > 0 || unrecoverableError()) ? this.shouldStopPolicyIfError : this.shouldStopPolicyIfNoError;
        return cs.isAfter(shouldStopPolicy);
    }

    public int errorCount() {
        if (this.delegateCompiler != null && this.delegateCompiler != this) {
            return this.delegateCompiler.errorCount();
        }
        if (this.werror && this.log.nerrors == 0 && this.log.nwarnings > 0) {
            this.log.error("warnings.and.werror", new Object[0]);
        }
        return this.log.nerrors;
    }

    protected final <T> Queue<T> stopIfError(CompileStates.CompileState cs, Queue<T> queue) {
        return shouldStop(cs) ? new ListBuffer() : queue;
    }

    protected final <T> List<T> stopIfError(CompileStates.CompileState cs, List<T> list) {
        return shouldStop(cs) ? List.nil() : list;
    }

    public int warningCount() {
        if (this.delegateCompiler != null && this.delegateCompiler != this) {
            return this.delegateCompiler.warningCount();
        }
        return this.log.nwarnings;
    }

    public CharSequence readSource(JavaFileObject filename) {
        try {
            this.inputFiles.add(filename);
            return filename.getCharContent(false);
        } catch (IOException e) {
            this.log.error("error.reading.file", filename, JavacFileManager.getMessage(e));
            return null;
        }
    }

    protected JCTree.JCCompilationUnit parse(JavaFileObject filename, CharSequence content) {
        long msec = now();
        JCTree.JCCompilationUnit tree = this.make.TopLevel(List.nil(), null, List.nil());
        if (content != null) {
            if (this.verbose) {
                this.log.printVerbose("parsing.started", filename);
            }
            if (!this.taskListener.isEmpty()) {
                TaskEvent e = new TaskEvent(TaskEvent.Kind.PARSE, filename);
                this.taskListener.started(e);
                this.keepComments = true;
                this.genEndPos = true;
            }
            Parser parser = this.parserFactory.newParser(content, keepComments(), this.genEndPos, this.lineDebugInfo);
            tree = parser.parseCompilationUnit();
            if (this.verbose) {
                this.log.printVerbose("parsing.done", Long.toString(elapsed(msec)));
            }
        }
        tree.sourcefile = filename;
        if (content != null && !this.taskListener.isEmpty()) {
            TaskEvent e2 = new TaskEvent(TaskEvent.Kind.PARSE, tree);
            this.taskListener.finished(e2);
        }
        return tree;
    }

    protected boolean keepComments() {
        return this.keepComments || this.sourceOutput || this.stubOutput;
    }

    @Deprecated
    public JCTree.JCCompilationUnit parse(String filename) {
        JavacFileManager fm = (JavacFileManager) this.fileManager;
        return parse(fm.getJavaFileObjectsFromStrings(List.of(filename)).iterator().next());
    }

    public JCTree.JCCompilationUnit parse(JavaFileObject filename) {
        JavaFileObject prev = this.log.useSource(filename);
        try {
            JCTree.JCCompilationUnit t = parse(filename, readSource(filename));
            if (t.endPositions != null) {
                this.log.setEndPosTable(filename, t.endPositions);
            }
            return t;
        } finally {
            this.log.useSource(prev);
        }
    }

    public Symbol resolveBinaryNameOrIdent(String name) {
        try {
            Name flatname = this.names.fromString(name.replace(OnBotJavaFileSystemUtils.PATH_SEPARATOR, "."));
            return this.reader.loadClass(flatname);
        } catch (Symbol.CompletionFailure e) {
            return resolveIdent(name);
        }
    }

    public Symbol resolveIdent(String name) {
        if (name.equals("")) {
            return this.syms.errSymbol;
        }
        JavaFileObject prev = this.log.useSource(null);
        JCTree.JCExpression tree = null;
        try {
            for (String s : name.split("\\.", -1)) {
                if (!SourceVersion.isIdentifier(s)) {
                    return this.syms.errSymbol;
                }
                tree = tree == null ? this.make.Ident(this.names.fromString(s)) : this.make.Select(tree, this.names.fromString(s));
            }
            JCTree.JCCompilationUnit toplevel = this.make.TopLevel(List.nil(), null, List.nil());
            toplevel.packge = this.syms.unnamedPackage;
            return this.attr.attribIdent(tree, toplevel);
        } finally {
            this.log.useSource(prev);
        }
    }

    JavaFileObject printSource(Env<AttrContext> env, JCTree.JCClassDecl cdef) throws IOException {
        JavaFileObject outFile = this.fileManager.getJavaFileForOutput(StandardLocation.CLASS_OUTPUT, cdef.sym.flatname.toString(), JavaFileObject.Kind.SOURCE, null);
        if (this.inputFiles.contains(outFile)) {
            this.log.error(cdef.pos(), "source.cant.overwrite.input.file", outFile);
            return null;
        }
        BufferedWriter out = new BufferedWriter(outFile.openWriter());
        try {
            new Pretty(out, true).printUnit(env.toplevel, cdef);
            if (this.verbose) {
                this.log.printVerbose("wrote.file", outFile);
            }
            return outFile;
        } finally {
            out.close();
        }
    }

    JavaFileObject genCode(Env<AttrContext> env, JCTree.JCClassDecl cdef) throws IOException {
        try {
            if (this.gen.genClass(env, cdef) && errorCount() == 0) {
                return this.writer.writeClass(cdef.sym);
            }
            return null;
        } catch (Symbol.CompletionFailure ex) {
            this.chk.completionError(cdef.pos(), ex);
            return null;
        } catch (ClassWriter.PoolOverflow e) {
            this.log.error(cdef.pos(), "limit.pool", new Object[0]);
            return null;
        } catch (ClassWriter.StringOverflow ex2) {
            this.log.error(cdef.pos(), "limit.string.overflow", ex2.value.substring(0, 20));
            return null;
        }
    }

    public void complete(Symbol.ClassSymbol c) throws Symbol.CompletionFailure {
        JCTree.JCCompilationUnit tree;
        if (this.completionFailureName == c.fullname) {
            throw new Symbol.CompletionFailure(c, "user-selected completion failure by class name");
        }
        JavaFileObject filename = c.classfile;
        JavaFileObject prev = this.log.useSource(filename);
        try {
            try {
                tree = parse(filename, filename.getCharContent(false));
            } catch (IOException e) {
                this.log.error("error.reading.file", filename, JavacFileManager.getMessage(e));
                tree = this.make.TopLevel(List.nil(), null, List.nil());
            }
            if (!this.taskListener.isEmpty()) {
                TaskEvent e2 = new TaskEvent(TaskEvent.Kind.ENTER, tree);
                this.taskListener.started(e2);
            }
            this.enter.complete(List.of(tree), c);
            if (!this.taskListener.isEmpty()) {
                TaskEvent e3 = new TaskEvent(TaskEvent.Kind.ENTER, tree);
                this.taskListener.finished(e3);
            }
            if (this.enter.getEnv(c) == null) {
                boolean isPkgInfo = tree.sourcefile.isNameCompatible("package-info", JavaFileObject.Kind.SOURCE);
                if (isPkgInfo) {
                    if (this.enter.getEnv(tree.packge) == null) {
                        JCDiagnostic diag = this.diagFactory.fragment("file.does.not.contain.package", c.location());
                        ClassReader classReader = this.reader;
                        classReader.getClass();
                        throw classReader.new BadClassFile(c, filename, diag);
                    }
                } else {
                    JCDiagnostic diag2 = this.diagFactory.fragment("file.doesnt.contain.class", c.getQualifiedName());
                    ClassReader classReader2 = this.reader;
                    classReader2.getClass();
                    throw classReader2.new BadClassFile(c, filename, diag2);
                }
            }
            this.implicitSourceFilesRead = true;
        } finally {
            this.log.useSource(prev);
        }
    }

    public void compile(List<JavaFileObject> sourceFileObject) throws Throwable {
        compile(sourceFileObject, List.nil(), null);
    }

    public void compile(List<JavaFileObject> sourceFileObjects, List<String> classnames, Iterable<? extends Processor> processors) {
        if (processors != null && processors.iterator().hasNext()) {
            this.explicitAnnotationProcessingRequested = true;
        }
        if (this.hasBeenUsed) {
            throw new AssertionError("attempt to reuse JavaCompiler");
        }
        this.hasBeenUsed = true;
        this.options.put(Option.XLINT_CUSTOM.text + "-" + Lint.LintCategory.OPTIONS.option, "true");
        this.options.remove(Option.XLINT_CUSTOM.text + Lint.LintCategory.OPTIONS.option);
        this.start_msec = now();
        try {
            try {
                initProcessAnnotations(processors);
                this.delegateCompiler = processAnnotations(enterTrees(stopIfError(CompileStates.CompileState.PARSE, parseFiles(sourceFileObjects))), classnames);
                this.delegateCompiler.compile2();
                this.delegateCompiler.close();
                this.elapsed_msec = this.delegateCompiler.elapsed_msec;
                if (this.procEnvImpl == null) {
                    return;
                }
            } catch (Abort ex) {
                if (this.devVerbose) {
                    ex.printStackTrace(System.err);
                }
                if (this.procEnvImpl == null) {
                    return;
                }
            }
            this.procEnvImpl.close();
        } catch (Throwable th) {
            if (this.procEnvImpl != null) {
                this.procEnvImpl.close();
            }
            throw th;
        }
    }

    /* JADX WARN: Can't fix incorrect switch cases order, some code will duplicate */
    private void compile2() {
        try {
            switch (this.compilePolicy) {
                case ATTR_ONLY:
                    attribute(this.todo);
                    break;
                case CHECK_ONLY:
                    flow(attribute(this.todo));
                    break;
                case SIMPLE:
                    generate(desugar(flow(attribute(this.todo))));
                    break;
                case BY_FILE:
                    Queue<Queue<Env<AttrContext>>> q = this.todo.groupByFile();
                    while (!q.isEmpty() && !shouldStop(CompileStates.CompileState.ATTR)) {
                        generate(desugar(flow(attribute(q.remove()))));
                    }
                    break;
                case BY_TODO:
                    while (!this.todo.isEmpty()) {
                        generate(desugar(flow(attribute(this.todo.remove()))));
                    }
                    break;
                default:
                    Assert.error("unknown compile policy");
                    break;
            }
        } catch (Abort ex) {
            if (this.devVerbose) {
                ex.printStackTrace(System.err);
            }
        }
        if (this.verbose) {
            this.elapsed_msec = elapsed(this.start_msec);
            this.log.printVerbose("total", Long.toString(this.elapsed_msec));
        }
        reportDeferredDiagnostics();
        if (!this.log.hasDiagnosticListener()) {
            printCount("error", errorCount());
            printCount("warn", warningCount());
        }
    }

    public List<JCTree.JCCompilationUnit> parseFiles(Iterable<JavaFileObject> fileObjects) {
        if (shouldStop(CompileStates.CompileState.PARSE)) {
            return List.nil();
        }
        ListBuffer<JCTree.JCCompilationUnit> trees = new ListBuffer<>();
        Set<JavaFileObject> filesSoFar = new HashSet<>();
        for (JavaFileObject fileObject : fileObjects) {
            if (!filesSoFar.contains(fileObject)) {
                filesSoFar.add(fileObject);
                trees.append(parse(fileObject));
            }
        }
        return trees.toList();
    }

    public List<JCTree.JCCompilationUnit> enterTreesIfNeeded(List<JCTree.JCCompilationUnit> roots) {
        if (shouldStop(CompileStates.CompileState.ATTR)) {
            return List.nil();
        }
        return enterTrees(roots);
    }

    /* JADX WARN: Multi-variable type inference failed */
    public List<JCTree.JCCompilationUnit> enterTrees(List<JCTree.JCCompilationUnit> roots) {
        if (!this.taskListener.isEmpty()) {
            for (JCTree.JCCompilationUnit unit : roots) {
                TaskEvent e = new TaskEvent(TaskEvent.Kind.ENTER, unit);
                this.taskListener.started(e);
            }
        }
        this.enter.main(roots);
        if (!this.taskListener.isEmpty()) {
            for (JCTree.JCCompilationUnit unit2 : roots) {
                TaskEvent e2 = new TaskEvent(TaskEvent.Kind.ENTER, unit2);
                this.taskListener.finished(e2);
            }
        }
        if (this.needRootClasses || this.sourceOutput || this.stubOutput) {
            ListBuffer<JCTree.JCClassDecl> cdefs = new ListBuffer<>();
            for (JCTree.JCCompilationUnit unit3 : roots) {
                for (List list = unit3.defs; list.nonEmpty(); list = list.tail) {
                    if (list.head instanceof JCTree.JCClassDecl) {
                        cdefs.append((JCTree.JCClassDecl) list.head);
                    }
                }
            }
            this.rootClasses = cdefs.toList();
        }
        for (JCTree.JCCompilationUnit unit4 : roots) {
            this.inputFiles.add(unit4.sourcefile);
        }
        return roots;
    }

    public void initProcessAnnotations(Iterable<? extends Processor> processors) {
        if (this.options.isSet(Option.PROC, "none")) {
            this.processAnnotations = false;
            return;
        }
        if (this.procEnvImpl == null) {
            this.procEnvImpl = JavacProcessingEnvironment.instance(this.context);
            this.procEnvImpl.setProcessors(processors);
            this.processAnnotations = this.procEnvImpl.atLeastOneProcessor();
            if (this.processAnnotations) {
                this.options.put("save-parameter-names", "save-parameter-names");
                this.reader.saveParameterNames = true;
                this.keepComments = true;
                this.genEndPos = true;
                if (!this.taskListener.isEmpty()) {
                    this.taskListener.started(new TaskEvent(TaskEvent.Kind.ANNOTATION_PROCESSING));
                }
                this.deferredDiagnosticHandler = new Log.DeferredDiagnosticHandler(this.log);
                return;
            }
            this.procEnvImpl.close();
        }
    }

    public JavaCompiler processAnnotations(List<JCTree.JCCompilationUnit> roots) {
        return processAnnotations(roots, List.nil());
    }

    public JavaCompiler processAnnotations(List<JCTree.JCCompilationUnit> roots, List<String> classnames) {
        if (shouldStop(CompileStates.CompileState.PROCESS) && unrecoverableError()) {
            this.deferredDiagnosticHandler.reportDeferredDiagnostics();
            this.log.popDiagnosticHandler(this.deferredDiagnosticHandler);
            return this;
        }
        if (!this.processAnnotations) {
            if (this.options.isSet(Option.PROC, "only")) {
                this.log.warning("proc.proc-only.requested.no.procs", new Object[0]);
                this.todo.clear();
            }
            if (!classnames.isEmpty()) {
                this.log.error("proc.no.explicit.annotation.processing.requested", classnames);
            }
            Assert.checkNull(this.deferredDiagnosticHandler);
            return this;
        }
        Assert.checkNonNull(this.deferredDiagnosticHandler);
        try {
            List<Symbol.ClassSymbol> classSymbols = List.nil();
            List<Symbol.PackageSymbol> pckSymbols = List.nil();
            if (!classnames.isEmpty()) {
                if (!explicitAnnotationProcessingRequested()) {
                    this.log.error("proc.no.explicit.annotation.processing.requested", classnames);
                    this.deferredDiagnosticHandler.reportDeferredDiagnostics();
                    this.log.popDiagnosticHandler(this.deferredDiagnosticHandler);
                    return this;
                }
                boolean errors = false;
                for (String nameStr : classnames) {
                    Symbol sym = resolveBinaryNameOrIdent(nameStr);
                    if (sym == null || ((sym.kind == 1 && !this.processPcks) || sym.kind == 137)) {
                        this.log.error("proc.cant.find.class", nameStr);
                        errors = true;
                    } else {
                        try {
                            if (sym.kind == 1) {
                                sym.complete();
                            }
                            if (sym.exists()) {
                                if (sym.kind == 1) {
                                    pckSymbols = pckSymbols.prepend((Symbol.PackageSymbol) sym);
                                } else {
                                    classSymbols = classSymbols.prepend((Symbol.ClassSymbol) sym);
                                }
                            } else {
                                Assert.check(sym.kind == 1);
                                this.log.warning("proc.package.does.not.exist", nameStr);
                                pckSymbols = pckSymbols.prepend((Symbol.PackageSymbol) sym);
                            }
                        } catch (Symbol.CompletionFailure e) {
                            this.log.error("proc.cant.find.class", nameStr);
                            errors = true;
                        }
                    }
                }
                if (errors) {
                    this.deferredDiagnosticHandler.reportDeferredDiagnostics();
                    this.log.popDiagnosticHandler(this.deferredDiagnosticHandler);
                    return this;
                }
            }
            try {
                JavaCompiler c = this.procEnvImpl.doProcessing(this.context, roots, classSymbols, pckSymbols, this.deferredDiagnosticHandler);
                if (c != this) {
                    c.annotationProcessingOccurred = true;
                    this.annotationProcessingOccurred = true;
                }
                return c;
            } finally {
                this.procEnvImpl.close();
            }
        } catch (Symbol.CompletionFailure ex) {
            this.log.error("cant.access", ex.sym, ex.getDetailValue());
            this.deferredDiagnosticHandler.reportDeferredDiagnostics();
            this.log.popDiagnosticHandler(this.deferredDiagnosticHandler);
            return this;
        }
    }

    private boolean unrecoverableError() {
        if (this.deferredDiagnosticHandler != null) {
            for (JCDiagnostic d : this.deferredDiagnosticHandler.getDiagnostics()) {
                if (d.getKind() == Diagnostic.Kind.ERROR && !d.isFlagSet(JCDiagnostic.DiagnosticFlag.RECOVERABLE)) {
                    return true;
                }
            }
            return false;
        }
        return false;
    }

    boolean explicitAnnotationProcessingRequested() {
        return this.explicitAnnotationProcessingRequested || explicitAnnotationProcessingRequested(this.options);
    }

    static boolean explicitAnnotationProcessingRequested(Options options) {
        return options.isSet(Option.PROCESSOR) || options.isSet(Option.PROCESSORPATH) || options.isSet(Option.PROC, "only") || options.isSet(Option.XPRINT);
    }

    public Queue<Env<AttrContext>> attribute(Queue<Env<AttrContext>> envs) {
        ListBuffer<Env<AttrContext>> results = new ListBuffer<>();
        while (!envs.isEmpty()) {
            results.append(attribute(envs.remove()));
        }
        return stopIfError(CompileStates.CompileState.ATTR, results);
    }

    public Env<AttrContext> attribute(Env<AttrContext> env) {
        if (this.compileStates.isDone(env, CompileStates.CompileState.ATTR)) {
            return env;
        }
        if (this.verboseCompilePolicy) {
            printNote("[attribute " + env.enclClass.sym + "]");
        }
        if (this.verbose) {
            this.log.printVerbose("checking.attribution", env.enclClass.sym);
        }
        if (!this.taskListener.isEmpty()) {
            TaskEvent e = new TaskEvent(TaskEvent.Kind.ANALYZE, env.toplevel, env.enclClass.sym);
            this.taskListener.started(e);
        }
        JavaFileObject prev = this.log.useSource(env.enclClass.sym.sourcefile != null ? env.enclClass.sym.sourcefile : env.toplevel.sourcefile);
        try {
            this.attr.attrib(env);
            if (errorCount() > 0 && !shouldStop(CompileStates.CompileState.ATTR)) {
                this.attr.postAttr(env.tree);
            }
            this.compileStates.put(env, CompileStates.CompileState.ATTR);
            if (this.rootClasses != null && this.rootClasses.contains(env.enclClass)) {
                reportPublicApi(env.enclClass.sym);
            }
            return env;
        } finally {
            this.log.useSource(prev);
        }
    }

    public void reportPublicApi(Symbol.ClassSymbol sym) {
    }

    public Queue<Env<AttrContext>> flow(Queue<Env<AttrContext>> envs) {
        ListBuffer<Env<AttrContext>> results = new ListBuffer<>();
        for (Env<AttrContext> env : envs) {
            flow(env, results);
        }
        return stopIfError(CompileStates.CompileState.FLOW, results);
    }

    public Queue<Env<AttrContext>> flow(Env<AttrContext> env) {
        ListBuffer<Env<AttrContext>> results = new ListBuffer<>();
        flow(env, results);
        return stopIfError(CompileStates.CompileState.FLOW, results);
    }

    protected void flow(Env<AttrContext> env, Queue<Env<AttrContext>> results) {
        boolean zIsEmpty;
        if (this.compileStates.isDone(env, CompileStates.CompileState.FLOW)) {
            results.add(env);
            return;
        }
        try {
            if (shouldStop(CompileStates.CompileState.FLOW)) {
                if (zIsEmpty) {
                    return;
                } else {
                    return;
                }
            }
            if (this.relax) {
                results.add(env);
                if (this.taskListener.isEmpty()) {
                    return;
                }
                TaskEvent e = new TaskEvent(TaskEvent.Kind.ANALYZE, env.toplevel, env.enclClass.sym);
                this.taskListener.finished(e);
                return;
            }
            if (this.verboseCompilePolicy) {
                printNote("[flow " + env.enclClass.sym + "]");
            }
            JavaFileObject prev = this.log.useSource(env.enclClass.sym.sourcefile != null ? env.enclClass.sym.sourcefile : env.toplevel.sourcefile);
            try {
                this.make.at(0);
                TreeMaker localMake = this.make.forToplevel(env.toplevel);
                this.flow.analyzeTree(env, localMake);
                this.compileStates.put(env, CompileStates.CompileState.FLOW);
                if (shouldStop(CompileStates.CompileState.FLOW)) {
                    if (this.taskListener.isEmpty()) {
                        return;
                    }
                    TaskEvent e2 = new TaskEvent(TaskEvent.Kind.ANALYZE, env.toplevel, env.enclClass.sym);
                    this.taskListener.finished(e2);
                    return;
                }
                results.add(env);
                if (this.taskListener.isEmpty()) {
                    return;
                }
                TaskEvent e3 = new TaskEvent(TaskEvent.Kind.ANALYZE, env.toplevel, env.enclClass.sym);
                this.taskListener.finished(e3);
            } finally {
                this.log.useSource(prev);
            }
        } finally {
            if (!this.taskListener.isEmpty()) {
                TaskEvent e4 = new TaskEvent(TaskEvent.Kind.ANALYZE, env.toplevel, env.enclClass.sym);
                this.taskListener.finished(e4);
            }
        }
    }

    public Queue<Pair<Env<AttrContext>, JCTree.JCClassDecl>> desugar(Queue<Env<AttrContext>> envs) {
        ListBuffer<Pair<Env<AttrContext>, JCTree.JCClassDecl>> results = new ListBuffer<>();
        for (Env<AttrContext> env : envs) {
            desugar(env, results);
        }
        return stopIfError(CompileStates.CompileState.FLOW, results);
    }

    /* JADX WARN: Multi-variable type inference failed */
    protected void desugar(Env<AttrContext> env, Queue<Pair<Env<AttrContext>, JCTree.JCClassDecl>> results) {
        if (shouldStop(CompileStates.CompileState.TRANSTYPES)) {
            return;
        }
        if (this.implicitSourcePolicy != ImplicitSourcePolicy.NONE || this.inputFiles.contains(env.toplevel.sourcefile)) {
            if (this.compileStates.isDone(env, CompileStates.CompileState.LOWER)) {
                results.addAll(this.desugaredEnvs.get(env));
                return;
            }
            C1ScanNested scanner = new C1ScanNested(env);
            scanner.scan(env.tree);
            for (Env<AttrContext> dep : scanner.dependencies) {
                if (!this.compileStates.isDone(dep, CompileStates.CompileState.FLOW)) {
                    this.desugaredEnvs.put(dep, desugar(flow(attribute(dep))));
                }
            }
            if (shouldStop(CompileStates.CompileState.TRANSTYPES)) {
                return;
            }
            if (this.verboseCompilePolicy) {
                printNote("[desugar " + env.enclClass.sym + "]");
            }
            JavaFileObject prev = this.log.useSource(env.enclClass.sym.sourcefile != null ? env.enclClass.sym.sourcefile : env.toplevel.sourcefile);
            try {
                JCTree untranslated = env.tree;
                this.make.at(0);
                TreeMaker localMake = this.make.forToplevel(env.toplevel);
                if (env.tree instanceof JCTree.JCCompilationUnit) {
                    if (!this.stubOutput && !this.sourceOutput && !this.printFlat) {
                        if (shouldStop(CompileStates.CompileState.LOWER)) {
                            return;
                        }
                        List<JCTree> pdef = this.lower.translateTopLevelClass(env, env.tree, localMake);
                        if (pdef.head != null) {
                            Assert.check(pdef.tail.isEmpty());
                            results.add(new Pair<>(env, (JCTree.JCClassDecl) pdef.head));
                        }
                    }
                    return;
                }
                if (this.stubOutput) {
                    JCTree.JCClassDecl cdef = (JCTree.JCClassDecl) env.tree;
                    if ((untranslated instanceof JCTree.JCClassDecl) && this.rootClasses.contains((JCTree.JCClassDecl) untranslated) && ((cdef.mods.flags & 5) != 0 || cdef.sym.packge().getQualifiedName() == this.names.java_lang)) {
                        results.add(new Pair<>(env, removeMethodBodies(cdef)));
                    }
                    return;
                }
                if (shouldStop(CompileStates.CompileState.TRANSTYPES)) {
                    return;
                }
                env.tree = this.transTypes.translateTopLevelClass(env.tree, localMake);
                this.compileStates.put(env, CompileStates.CompileState.TRANSTYPES);
                if (this.source.allowLambda() && scanner.hasLambdas) {
                    if (shouldStop(CompileStates.CompileState.UNLAMBDA)) {
                        return;
                    }
                    env.tree = LambdaToMethod.instance(this.context).translateTopLevelClass(env, env.tree, localMake);
                    this.compileStates.put(env, CompileStates.CompileState.UNLAMBDA);
                }
                if (shouldStop(CompileStates.CompileState.LOWER)) {
                    return;
                }
                if (this.sourceOutput) {
                    JCTree.JCClassDecl cdef2 = (JCTree.JCClassDecl) env.tree;
                    if ((untranslated instanceof JCTree.JCClassDecl) && this.rootClasses.contains((JCTree.JCClassDecl) untranslated)) {
                        results.add(new Pair<>(env, cdef2));
                    }
                    return;
                }
                List<JCTree> cdefs = this.lower.translateTopLevelClass(env, env.tree, localMake);
                this.compileStates.put(env, CompileStates.CompileState.LOWER);
                if (shouldStop(CompileStates.CompileState.LOWER)) {
                    return;
                }
                for (List list = cdefs; list.nonEmpty(); list = list.tail) {
                    JCTree.JCClassDecl cdef3 = (JCTree.JCClassDecl) list.head;
                    results.add(new Pair<>(env, cdef3));
                }
            } finally {
                this.log.useSource(prev);
            }
        }
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.main.JavaCompiler$1ScanNested, reason: invalid class name */
    class C1ScanNested extends TreeScanner {
        Set<Env<AttrContext>> dependencies = new LinkedHashSet();
        protected boolean hasLambdas;
        final /* synthetic */ Env val$env;

        C1ScanNested(Env env) {
            this.val$env = env;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl node) {
            Type st = JavaCompiler.this.types.supertype(node.sym.type);
            boolean envForSuperTypeFound = false;
            while (!envForSuperTypeFound && st.hasTag(TypeTag.CLASS)) {
                Symbol.ClassSymbol c = st.tsym.outermostClass();
                Env<AttrContext> stEnv = JavaCompiler.this.enter.getEnv(c);
                if (stEnv != null && this.val$env != stEnv) {
                    if (this.dependencies.add(stEnv)) {
                        boolean prevHasLambdas = this.hasLambdas;
                        try {
                            scan(stEnv.tree);
                        } finally {
                            this.hasLambdas = prevHasLambdas;
                        }
                    }
                    envForSuperTypeFound = true;
                }
                st = JavaCompiler.this.types.supertype(st);
            }
            super.visitClassDef(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            this.hasLambdas = true;
            super.visitLambda(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReference(JCTree.JCMemberReference tree) {
            this.hasLambdas = true;
            super.visitReference(tree);
        }
    }

    public void generate(Queue<Pair<Env<AttrContext>, JCTree.JCClassDecl>> queue) {
        generate(queue, null);
    }

    public void generate(Queue<Pair<Env<AttrContext>, JCTree.JCClassDecl>> queue, Queue<JavaFileObject> results) {
        JavaFileObject file;
        if (shouldStop(CompileStates.CompileState.GENERATE)) {
            return;
        }
        boolean usePrintSource = this.stubOutput || this.sourceOutput || this.printFlat;
        for (Pair<Env<AttrContext>, JCTree.JCClassDecl> x : queue) {
            Env<AttrContext> env = x.fst;
            JCTree.JCClassDecl cdef = x.snd;
            if (this.verboseCompilePolicy) {
                printNote("[generate " + (usePrintSource ? " source" : "code") + " " + cdef.sym + "]");
            }
            if (!this.taskListener.isEmpty()) {
                TaskEvent e = new TaskEvent(TaskEvent.Kind.GENERATE, env.toplevel, cdef.sym);
                this.taskListener.started(e);
            }
            JavaFileObject prev = this.log.useSource(env.enclClass.sym.sourcefile != null ? env.enclClass.sym.sourcefile : env.toplevel.sourcefile);
            if (usePrintSource) {
                try {
                    try {
                        file = printSource(env, cdef);
                    } catch (IOException ex) {
                        this.log.error(cdef.pos(), "class.cant.write", cdef.sym, ex.getMessage());
                        this.log.useSource(prev);
                        return;
                    }
                } catch (Throwable th) {
                    this.log.useSource(prev);
                    throw th;
                }
            } else {
                if (this.fileManager.hasLocation(StandardLocation.NATIVE_HEADER_OUTPUT) && this.jniWriter.needsHeader(cdef.sym)) {
                    this.jniWriter.write(cdef.sym);
                }
                file = genCode(env, cdef);
            }
            if (results != null && file != null) {
                results.add(file);
            }
            this.log.useSource(prev);
            if (!this.taskListener.isEmpty()) {
                TaskEvent e2 = new TaskEvent(TaskEvent.Kind.GENERATE, env.toplevel, cdef.sym);
                this.taskListener.finished(e2);
            }
        }
    }

    Map<JCTree.JCCompilationUnit, Queue<Env<AttrContext>>> groupByFile(Queue<Env<AttrContext>> envs) {
        Map<JCTree.JCCompilationUnit, Queue<Env<AttrContext>>> map = new LinkedHashMap<>();
        for (Env<AttrContext> env : envs) {
            Queue<Env<AttrContext>> sublist = map.get(env.toplevel);
            if (sublist == null) {
                sublist = new ListBuffer<>();
                map.put(env.toplevel, sublist);
            }
            sublist.add(env);
        }
        return map;
    }

    JCTree.JCClassDecl removeMethodBodies(JCTree.JCClassDecl cdef) {
        final boolean isInterface = (cdef.mods.flags & 512) != 0;
        return (JCTree.JCClassDecl) new TreeTranslator() { // from class: com.sun.tools.javac.main.JavaCompiler.1MethodBodyRemover
            @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitMethodDef(JCTree.JCMethodDecl tree) {
                tree.mods.flags &= -33;
                for (JCTree.JCVariableDecl vd : tree.params) {
                    vd.mods.flags &= -17;
                }
                tree.body = null;
                super.visitMethodDef(tree);
            }

            @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitVarDef(JCTree.JCVariableDecl tree) {
                if (tree.init != null && tree.init.type.constValue() == null) {
                    tree.init = null;
                }
                super.visitVarDef(tree);
            }

            /* JADX WARN: Multi-variable type inference failed */
            @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitClassDef(JCTree.JCClassDecl tree) {
                ListBuffer<JCTree> newdefs = new ListBuffer<>();
                for (List list = tree.defs; list.tail != null; list = list.tail) {
                    JCTree t = (JCTree) list.head;
                    switch (AnonymousClass2.$SwitchMap$com$sun$tools$javac$tree$JCTree$Tag[t.getTag().ordinal()]) {
                        case 1:
                            if (isInterface || (5 & ((JCTree.JCClassDecl) t).mods.flags) != 0 || ((((JCTree.JCClassDecl) t).mods.flags & 2) == 0 && ((JCTree.JCClassDecl) t).sym.packge().getQualifiedName() == JavaCompiler.this.names.java_lang)) {
                                newdefs.append(t);
                            }
                            break;
                        case 2:
                            if (isInterface || (5 & ((JCTree.JCMethodDecl) t).mods.flags) != 0 || ((JCTree.JCMethodDecl) t).sym.name == JavaCompiler.this.names.init || ((((JCTree.JCMethodDecl) t).mods.flags & 2) == 0 && ((JCTree.JCMethodDecl) t).sym.packge().getQualifiedName() == JavaCompiler.this.names.java_lang)) {
                                newdefs.append(t);
                            }
                            break;
                        case 3:
                            if (isInterface || (5 & ((JCTree.JCVariableDecl) t).mods.flags) != 0 || ((((JCTree.JCVariableDecl) t).mods.flags & 2) == 0 && ((JCTree.JCVariableDecl) t).sym.packge().getQualifiedName() == JavaCompiler.this.names.java_lang)) {
                                newdefs.append(t);
                            }
                            break;
                    }
                }
                List<JCTree> it = newdefs.toList();
                tree.defs = it;
                super.visitClassDef(tree);
            }
        }.translate(cdef);
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.main.JavaCompiler$2, reason: invalid class name */
    static /* synthetic */ class AnonymousClass2 {
        static final /* synthetic */ int[] $SwitchMap$com$sun$tools$javac$tree$JCTree$Tag = new int[JCTree.Tag.values().length];

        static {
            try {
                $SwitchMap$com$sun$tools$javac$tree$JCTree$Tag[JCTree.Tag.CLASSDEF.ordinal()] = 1;
            } catch (NoSuchFieldError e) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$tree$JCTree$Tag[JCTree.Tag.METHODDEF.ordinal()] = 2;
            } catch (NoSuchFieldError e2) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$tree$JCTree$Tag[JCTree.Tag.VARDEF.ordinal()] = 3;
            } catch (NoSuchFieldError e3) {
            }
            $SwitchMap$com$sun$tools$javac$main$JavaCompiler$CompilePolicy = new int[CompilePolicy.values().length];
            try {
                $SwitchMap$com$sun$tools$javac$main$JavaCompiler$CompilePolicy[CompilePolicy.ATTR_ONLY.ordinal()] = 1;
            } catch (NoSuchFieldError e4) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$main$JavaCompiler$CompilePolicy[CompilePolicy.CHECK_ONLY.ordinal()] = 2;
            } catch (NoSuchFieldError e5) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$main$JavaCompiler$CompilePolicy[CompilePolicy.SIMPLE.ordinal()] = 3;
            } catch (NoSuchFieldError e6) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$main$JavaCompiler$CompilePolicy[CompilePolicy.BY_FILE.ordinal()] = 4;
            } catch (NoSuchFieldError e7) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$main$JavaCompiler$CompilePolicy[CompilePolicy.BY_TODO.ordinal()] = 5;
            } catch (NoSuchFieldError e8) {
            }
        }
    }

    public void reportDeferredDiagnostics() {
        if (errorCount() == 0 && this.annotationProcessingOccurred && this.implicitSourceFilesRead && this.implicitSourcePolicy == ImplicitSourcePolicy.UNSET) {
            if (explicitAnnotationProcessingRequested()) {
                this.log.warning("proc.use.implicit", new Object[0]);
            } else {
                this.log.warning("proc.use.proc.or.implicit", new Object[0]);
            }
        }
        this.chk.reportDeferredDiagnostics();
        if (this.log.compressedOutput) {
            this.log.mandatoryNote(null, "compressed.diags", new Object[0]);
        }
    }

    public void close() {
        close(true);
    }

    public void close(boolean disposeNames) {
        this.rootClasses = null;
        this.reader = null;
        this.make = null;
        this.writer = null;
        this.enter = null;
        if (this.todo != null) {
            this.todo.clear();
        }
        this.todo = null;
        this.parserFactory = null;
        this.syms = null;
        this.source = null;
        this.attr = null;
        this.chk = null;
        this.gen = null;
        this.flow = null;
        this.transTypes = null;
        this.lower = null;
        this.annotate = null;
        this.types = null;
        this.log.flush();
        try {
            try {
                this.fileManager.flush();
                if (this.names != null && disposeNames) {
                    this.names.dispose();
                }
                this.names = null;
                for (Closeable c : this.closeables) {
                    try {
                        c.close();
                    } catch (IOException e) {
                        JCDiagnostic msg = this.diagFactory.fragment("fatal.err.cant.close", new Object[0]);
                        throw new FatalError(msg, e);
                    }
                }
                this.closeables = List.nil();
            } catch (IOException e2) {
                throw new Abort(e2);
            }
        } catch (Throwable th) {
            if (this.names != null && disposeNames) {
                this.names.dispose();
            }
            this.names = null;
            for (Closeable c2 : this.closeables) {
                try {
                    c2.close();
                } catch (IOException e3) {
                    JCDiagnostic msg2 = this.diagFactory.fragment("fatal.err.cant.close", new Object[0]);
                    throw new FatalError(msg2, e3);
                }
            }
            this.closeables = List.nil();
            throw th;
        }
    }

    protected void printNote(String lines) {
        this.log.printRawLines(Log.WriterKind.NOTICE, lines);
    }

    public void printCount(String kind, int count) {
        String key;
        if (count != 0) {
            if (count == 1) {
                key = "count." + kind;
            } else {
                key = "count." + kind + ".plural";
            }
            this.log.printLines(Log.WriterKind.ERROR, key, String.valueOf(count));
            this.log.flush(Log.WriterKind.ERROR);
        }
    }

    private static long now() {
        return System.currentTimeMillis();
    }

    private static long elapsed(long then) {
        return now() - then;
    }

    public void initRound(JavaCompiler prev) {
        this.genEndPos = prev.genEndPos;
        this.keepComments = prev.keepComments;
        this.start_msec = prev.start_msec;
        this.hasBeenUsed = true;
        this.closeables = prev.closeables;
        prev.closeables = List.nil();
        this.shouldStopPolicyIfError = prev.shouldStopPolicyIfError;
        this.shouldStopPolicyIfNoError = prev.shouldStopPolicyIfNoError;
    }
}
