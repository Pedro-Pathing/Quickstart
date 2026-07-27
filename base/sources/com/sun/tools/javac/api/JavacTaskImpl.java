package com.sun.tools.javac.api;

import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.Tree;
import com.sun.source.util.JavacTask;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.file.JavacFileManager;
import com.sun.tools.javac.main.CommandLine;
import com.sun.tools.javac.main.JavaCompiler;
import com.sun.tools.javac.main.Main;
import com.sun.tools.javac.model.JavacElements;
import com.sun.tools.javac.model.JavacTypes;
import com.sun.tools.javac.parser.Parser;
import com.sun.tools.javac.parser.ParserFactory;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Options;
import java.io.File;
import java.io.IOException;
import java.nio.CharBuffer;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import javax.annotation.processing.Processor;
import javax.lang.model.element.Element;
import javax.lang.model.element.TypeElement;
import javax.lang.model.type.TypeMirror;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class JavacTaskImpl extends BasicJavacTask {
    private String[] args;
    private String[] classNames;
    private JavaCompiler compiler;
    private Main compilerMain;
    private List<JavaFileObject> fileObjects;
    private ListBuffer<Env<AttrContext>> genList;
    private Locale locale;
    private Map<JavaFileObject, JCTree.JCCompilationUnit> notYetEntered;
    private boolean parsed;
    private Iterable<? extends Processor> processors;
    private Main.Result result;
    private final AtomicBoolean used;

    JavacTaskImpl(Main compilerMain, String[] args, String[] classNames, Context context, List<JavaFileObject> fileObjects) {
        super(null, false);
        this.used = new AtomicBoolean();
        this.result = null;
        this.parsed = false;
        this.compilerMain = compilerMain;
        this.args = args;
        this.classNames = classNames;
        this.context = context;
        this.fileObjects = fileObjects;
        setLocale(Locale.getDefault());
        compilerMain.getClass();
        args.getClass();
        fileObjects.getClass();
    }

    JavacTaskImpl(Main compilerMain, Iterable<String> args, Context context, Iterable<String> classes, Iterable<? extends JavaFileObject> fileObjects) {
        this(compilerMain, toArray(args), toArray(classes), context, toList(fileObjects));
    }

    private static String[] toArray(Iterable<String> iter) {
        ListBuffer<String> result = new ListBuffer<>();
        if (iter != null) {
            for (String s : iter) {
                result.append(s);
            }
        }
        return (String[]) result.toArray(new String[result.length()]);
    }

    private static List<JavaFileObject> toList(Iterable<? extends JavaFileObject> fileObjects) {
        if (fileObjects == null) {
            return List.nil();
        }
        ListBuffer<JavaFileObject> result = new ListBuffer<>();
        for (JavaFileObject fo : fileObjects) {
            result.append(fo);
        }
        return result.toList();
    }

    public Main.Result doCall() {
        if (!this.used.getAndSet(true)) {
            initContext();
            this.notYetEntered = new HashMap();
            this.compilerMain.setAPIMode(true);
            this.result = this.compilerMain.compile(this.args, this.classNames, this.context, this.fileObjects, this.processors);
            cleanup();
            return this.result;
        }
        throw new IllegalStateException("multiple calls to method 'call'");
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.tools.javac.api.BasicJavacTask, java.util.concurrent.Callable
    public Boolean call() {
        return Boolean.valueOf(doCall().isOK());
    }

    @Override // com.sun.tools.javac.api.BasicJavacTask, javax.tools.JavaCompiler.CompilationTask
    public void setProcessors(Iterable<? extends Processor> processors) {
        processors.getClass();
        if (this.used.get()) {
            throw new IllegalStateException();
        }
        this.processors = processors;
    }

    @Override // com.sun.tools.javac.api.BasicJavacTask, javax.tools.JavaCompiler.CompilationTask
    public void setLocale(Locale locale) {
        if (this.used.get()) {
            throw new IllegalStateException();
        }
        this.locale = locale;
    }

    private void prepareCompiler() throws IOException {
        if (this.used.getAndSet(true)) {
            if (this.compiler == null) {
                throw new IllegalStateException();
            }
            return;
        }
        initContext();
        this.compilerMain.log = Log.instance(this.context);
        this.compilerMain.setOptions(Options.instance(this.context));
        this.compilerMain.filenames = new LinkedHashSet();
        Collection<File> filenames = this.compilerMain.processArgs(CommandLine.parse(this.args), this.classNames);
        if (filenames != null && !filenames.isEmpty()) {
            throw new IllegalArgumentException("Malformed arguments " + toString(filenames, " "));
        }
        this.compiler = JavaCompiler.instance(this.context);
        this.compiler.keepComments = true;
        this.compiler.genEndPos = true;
        this.compiler.initProcessAnnotations(this.processors);
        this.notYetEntered = new HashMap();
        for (JavaFileObject file : this.fileObjects) {
            this.notYetEntered.put(file, null);
        }
        this.genList = new ListBuffer<>();
        this.args = null;
        this.classNames = null;
    }

    <T> String toString(Iterable<T> items, String sep) {
        String currSep = "";
        StringBuilder sb = new StringBuilder();
        for (T item : items) {
            sb.append(currSep);
            sb.append(item.toString());
            currSep = sep;
        }
        return sb.toString();
    }

    private void initContext() {
        this.context.put((Class<JavacTaskImpl>) JavacTask.class, this);
        this.context.put((Class<Locale>) Locale.class, this.locale);
    }

    void cleanup() {
        if (this.compiler != null) {
            this.compiler.close();
        }
        this.compiler = null;
        this.compilerMain = null;
        this.args = null;
        this.classNames = null;
        this.context = null;
        this.fileObjects = null;
        this.notYetEntered = null;
    }

    public JavaFileObject asJavaFileObject(File file) {
        JavacFileManager fm = (JavacFileManager) this.context.get(JavaFileManager.class);
        return fm.getRegularFile(file);
    }

    @Override // com.sun.tools.javac.api.BasicJavacTask, com.sun.source.util.JavacTask
    public Iterable<? extends CompilationUnitTree> parse() throws IOException {
        try {
            prepareCompiler();
            List<JCTree.JCCompilationUnit> units = this.compiler.parseFiles(this.fileObjects);
            for (JCTree.JCCompilationUnit unit : units) {
                JavaFileObject file = unit.getSourceFile();
                if (this.notYetEntered.containsKey(file)) {
                    this.notYetEntered.put(file, unit);
                }
            }
            return units;
        } finally {
            this.parsed = true;
            if (this.compiler != null && this.compiler.log != null) {
                this.compiler.log.flush();
            }
        }
    }

    public Iterable<? extends TypeElement> enter() throws IOException {
        return enter(null);
    }

    public Iterable<? extends TypeElement> enter(Iterable<? extends CompilationUnitTree> trees) throws IOException {
        if (trees == null && this.notYetEntered != null && this.notYetEntered.isEmpty()) {
            return List.nil();
        }
        prepareCompiler();
        ListBuffer<JCTree.JCCompilationUnit> roots = null;
        if (trees == null) {
            if (this.notYetEntered.size() > 0) {
                if (!this.parsed) {
                    parse();
                }
                for (JavaFileObject file : this.fileObjects) {
                    JCTree.JCCompilationUnit unit = this.notYetEntered.remove(file);
                    if (unit != null) {
                        if (roots == null) {
                            roots = new ListBuffer<>();
                        }
                        roots.append(unit);
                    }
                }
                this.notYetEntered.clear();
            }
        } else {
            for (CompilationUnitTree cu : trees) {
                if (cu instanceof JCTree.JCCompilationUnit) {
                    if (roots == null) {
                        roots = new ListBuffer<>();
                    }
                    roots.append((JCTree.JCCompilationUnit) cu);
                    this.notYetEntered.remove(cu.getSourceFile());
                } else {
                    throw new IllegalArgumentException(cu.toString());
                }
            }
        }
        if (roots == null) {
            return List.nil();
        }
        try {
            List<JCTree.JCCompilationUnit> units = this.compiler.enterTrees(roots.toList());
            if (this.notYetEntered.isEmpty()) {
                this.compiler = this.compiler.processAnnotations(units);
            }
            ListBuffer<TypeElement> elements = new ListBuffer<>();
            Iterator<JCTree.JCCompilationUnit> it = units.iterator();
            while (it.hasNext()) {
                for (JCTree node : it.next().defs) {
                    if (node.hasTag(JCTree.Tag.CLASSDEF)) {
                        JCTree.JCClassDecl cdef = (JCTree.JCClassDecl) node;
                        if (cdef.sym != null) {
                            elements.append(cdef.sym);
                        }
                    }
                }
            }
            return elements.toList();
        } finally {
            this.compiler.log.flush();
        }
    }

    @Override // com.sun.tools.javac.api.BasicJavacTask, com.sun.source.util.JavacTask
    public Iterable<? extends Element> analyze() throws IOException {
        return analyze(null);
    }

    public Iterable<? extends Element> analyze(Iterable<? extends TypeElement> classes) throws IOException {
        enter(null);
        final ListBuffer<Element> results = new ListBuffer<>();
        try {
            if (classes == null) {
                handleFlowResults(this.compiler.flow(this.compiler.attribute(this.compiler.todo)), results);
            } else {
                Filter f = new Filter() { // from class: com.sun.tools.javac.api.JavacTaskImpl.1
                    /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
                    {
                        super();
                    }

                    @Override // com.sun.tools.javac.api.JavacTaskImpl.Filter
                    public void process(Env<AttrContext> env) {
                        JavacTaskImpl.this.handleFlowResults(JavacTaskImpl.this.compiler.flow(JavacTaskImpl.this.compiler.attribute(env)), results);
                    }
                };
                f.run(this.compiler.todo, classes);
            }
            return results;
        } finally {
            this.compiler.log.flush();
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void handleFlowResults(Queue<Env<AttrContext>> queue, ListBuffer<Element> elems) {
        for (Env<AttrContext> env : queue) {
            switch (env.tree.getTag()) {
                case CLASSDEF:
                    JCTree.JCClassDecl cdef = (JCTree.JCClassDecl) env.tree;
                    if (cdef.sym != null) {
                        elems.append(cdef.sym);
                    }
                    break;
                case TOPLEVEL:
                    JCTree.JCCompilationUnit unit = (JCTree.JCCompilationUnit) env.tree;
                    if (unit.packge != null) {
                        elems.append(unit.packge);
                    }
                    break;
            }
        }
        this.genList.addAll(queue);
    }

    @Override // com.sun.tools.javac.api.BasicJavacTask, com.sun.source.util.JavacTask
    public Iterable<? extends JavaFileObject> generate() throws IOException {
        return generate(null);
    }

    public Iterable<? extends JavaFileObject> generate(Iterable<? extends TypeElement> classes) throws IOException {
        final ListBuffer<JavaFileObject> results = new ListBuffer<>();
        try {
            analyze(null);
            if (classes == null) {
                this.compiler.generate(this.compiler.desugar(this.genList), results);
                this.genList.clear();
            } else {
                Filter f = new Filter() { // from class: com.sun.tools.javac.api.JavacTaskImpl.2
                    /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
                    {
                        super();
                    }

                    @Override // com.sun.tools.javac.api.JavacTaskImpl.Filter
                    public void process(Env<AttrContext> env) {
                        JavacTaskImpl.this.compiler.generate(JavacTaskImpl.this.compiler.desugar(ListBuffer.of(env)), results);
                    }
                };
                f.run(this.genList, classes);
            }
            if (this.genList.isEmpty()) {
                this.compiler.reportDeferredDiagnostics();
                cleanup();
            }
            return results;
        } finally {
            if (this.compiler != null) {
                this.compiler.log.flush();
            }
        }
    }

    @Override // com.sun.tools.javac.api.BasicJavacTask, com.sun.source.util.JavacTask
    public TypeMirror getTypeMirror(Iterable<? extends Tree> path) {
        Tree last = null;
        for (Tree node : path) {
            last = node;
        }
        return ((JCTree) last).type;
    }

    @Override // com.sun.tools.javac.api.BasicJavacTask, com.sun.source.util.JavacTask
    public JavacElements getElements() {
        if (this.context == null) {
            throw new IllegalStateException();
        }
        return JavacElements.instance(this.context);
    }

    @Override // com.sun.tools.javac.api.BasicJavacTask, com.sun.source.util.JavacTask
    public JavacTypes getTypes() {
        if (this.context == null) {
            throw new IllegalStateException();
        }
        return JavacTypes.instance(this.context);
    }

    public Iterable<? extends Tree> pathFor(CompilationUnitTree unit, Tree node) {
        return TreeInfo.pathFor((JCTree) node, (JCTree.JCCompilationUnit) unit).reverse();
    }

    abstract class Filter {
        abstract void process(Env<AttrContext> env);

        Filter() {
        }

        void run(Queue<Env<AttrContext>> list, Iterable<? extends TypeElement> classes) {
            Set<TypeElement> set = new HashSet<>();
            for (TypeElement item : classes) {
                set.add(item);
            }
            ListBuffer<Env<AttrContext>> defer = new ListBuffer<>();
            while (list.peek() != null) {
                Env<AttrContext> env = list.remove();
                Symbol.ClassSymbol csym = env.enclClass.sym;
                if (csym != null && set.contains(csym.outermostClass())) {
                    process(env);
                } else {
                    defer = defer.append(env);
                }
            }
            list.addAll(defer);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public Type parseType(String expr, TypeElement typeElement) {
        if (expr == null || expr.equals("")) {
            throw new IllegalArgumentException();
        }
        this.compiler = JavaCompiler.instance(this.context);
        JavaFileObject prev = this.compiler.log.useSource(null);
        ParserFactory parserFactory = ParserFactory.instance(this.context);
        Attr attr = Attr.instance(this.context);
        try {
            CharBuffer buf = CharBuffer.wrap((expr + "\u0000").toCharArray(), 0, expr.length());
            Parser parser = parserFactory.newParser(buf, false, false, false);
            JCTree tree = parser.parseType();
            return attr.attribType(tree, (Symbol.TypeSymbol) typeElement);
        } finally {
            this.compiler.log.useSource(prev);
        }
    }
}
