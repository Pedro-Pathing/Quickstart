package com.sun.tools.javac.processing;

import com.sun.source.util.JavacTask;
import com.sun.source.util.TaskEvent;
import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.api.BasicJavacTask;
import com.sun.tools.javac.api.JavacTrees;
import com.sun.tools.javac.api.MultiTaskListener;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.comp.CompileStates;
import com.sun.tools.javac.file.FSInfo;
import com.sun.tools.javac.file.JavacFileManager;
import com.sun.tools.javac.jvm.ClassReader;
import com.sun.tools.javac.main.JavaCompiler;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.model.JavacElements;
import com.sun.tools.javac.model.JavacTypes;
import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.processing.ServiceProxy;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.util.Abort;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.ClientCodeException;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Convert;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.JavacMessages;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.ServiceLoader;
import java.io.Closeable;
import java.io.File;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Locale;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.ServiceConfigurationError;
import java.util.Set;
import java.util.regex.Pattern;
import javax.annotation.processing.Filer;
import javax.annotation.processing.Messager;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.Processor;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.Element;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.PackageElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.util.ElementScanner8;
import javax.lang.model.util.Elements;
import javax.tools.Diagnostic;
import javax.tools.DiagnosticListener;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.StandardLocation;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes.dex */
public class JavacProcessingEnvironment implements ProcessingEnvironment, Closeable {
    private Context context;
    JCDiagnostic.Factory diags;
    private DiscoveredProcessors discoveredProcs;
    private final JavacElements elementUtils;
    private final boolean fatalErrors;
    private final JavacFiler filer;
    private final boolean lint;
    Log log;
    private final JavacMessager messager;
    private JavacMessages messages;
    private final Options options;
    private final Set<String> platformAnnotations;
    private final boolean printProcessorInfo;
    private final boolean printRounds;
    private ClassLoader processorClassLoader;
    private SecurityException processorClassLoaderException;
    private final Map<String, String> processorOptions;
    private final boolean showResolveErrors;
    Source source;
    private Set<Symbol.PackageSymbol> specifiedPackages = Collections.emptySet();
    private MultiTaskListener taskListener;
    private final JavacTypes typeUtils;
    private final Set<String> unmatchedProcessorOptions;
    private final boolean verbose;
    private final boolean werror;
    private static final TreeScanner treeCleaner = new TreeScanner() { // from class: com.sun.tools.javac.processing.JavacProcessingEnvironment.1
        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree node) {
            super.scan(node);
            if (node != null) {
                node.type = null;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTopLevel(JCTree.JCCompilationUnit node) {
            node.packge = null;
            super.visitTopLevel(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl node) {
            node.sym = null;
            super.visitClassDef(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl node) {
            node.sym = null;
            super.visitMethodDef(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl node) {
            node.sym = null;
            super.visitVarDef(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass node) {
            node.constructor = null;
            super.visitNewClass(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssignop(JCTree.JCAssignOp node) {
            node.operator = null;
            super.visitAssignop(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitUnary(JCTree.JCUnary node) {
            node.operator = null;
            super.visitUnary(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBinary(JCTree.JCBinary node) {
            node.operator = null;
            super.visitBinary(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess node) {
            node.sym = null;
            super.visitSelect(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent node) {
            node.sym = null;
            super.visitIdent(node);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAnnotation(JCTree.JCAnnotation node) {
            node.attribute = null;
            super.visitAnnotation(node);
        }
    };
    private static final Pattern allMatches = Pattern.compile(".*");
    public static final Pattern noMatches = Pattern.compile("(\\P{all})+");

    public static JavacProcessingEnvironment instance(Context context) {
        JavacProcessingEnvironment instance = (JavacProcessingEnvironment) context.get(JavacProcessingEnvironment.class);
        if (instance == null) {
            return new JavacProcessingEnvironment(context);
        }
        return instance;
    }

    protected JavacProcessingEnvironment(Context context) {
        this.context = context;
        context.put((Class<JavacProcessingEnvironment>) JavacProcessingEnvironment.class, this);
        this.log = Log.instance(context);
        this.source = Source.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        this.options = Options.instance(context);
        this.printProcessorInfo = this.options.isSet(Option.XPRINTPROCESSORINFO);
        this.printRounds = this.options.isSet(Option.XPRINTROUNDS);
        this.verbose = this.options.isSet(Option.VERBOSE);
        this.lint = Lint.instance(context).isEnabled(Lint.LintCategory.PROCESSING);
        if (this.options.isSet(Option.PROC, "only") || this.options.isSet(Option.XPRINT)) {
            JavaCompiler compiler = JavaCompiler.instance(context);
            compiler.shouldStopPolicyIfNoError = CompileStates.CompileState.PROCESS;
        }
        this.fatalErrors = this.options.isSet("fatalEnterError");
        this.showResolveErrors = this.options.isSet("showResolveErrors");
        this.werror = this.options.isSet(Option.WERROR);
        this.platformAnnotations = initPlatformAnnotations();
        this.filer = new JavacFiler(context);
        this.messager = new JavacMessager(context, this);
        this.elementUtils = JavacElements.instance(context);
        this.typeUtils = JavacTypes.instance(context);
        this.processorOptions = initProcessorOptions(context);
        this.unmatchedProcessorOptions = initUnmatchedProcessorOptions();
        this.messages = JavacMessages.instance(context);
        this.taskListener = MultiTaskListener.instance(context);
        initProcessorClassLoader();
    }

    public void setProcessors(Iterable<? extends Processor> processors) {
        Assert.checkNull(this.discoveredProcs);
        initProcessorIterator(this.context, processors);
    }

    private Set<String> initPlatformAnnotations() {
        Set<String> platformAnnotations = new HashSet<>();
        platformAnnotations.add("java.lang.Deprecated");
        platformAnnotations.add("java.lang.Override");
        platformAnnotations.add("java.lang.SuppressWarnings");
        platformAnnotations.add("java.lang.annotation.Documented");
        platformAnnotations.add("java.lang.annotation.Inherited");
        platformAnnotations.add("java.lang.annotation.Retention");
        platformAnnotations.add("java.lang.annotation.Target");
        return Collections.unmodifiableSet(platformAnnotations);
    }

    private void initProcessorClassLoader() {
        ClassLoader classLoader;
        JavaFileManager fileManager = (JavaFileManager) this.context.get(JavaFileManager.class);
        try {
            if (fileManager.hasLocation(StandardLocation.ANNOTATION_PROCESSOR_PATH)) {
                classLoader = fileManager.getClassLoader(StandardLocation.ANNOTATION_PROCESSOR_PATH);
            } else {
                classLoader = fileManager.getClassLoader(StandardLocation.CLASS_PATH);
            }
            this.processorClassLoader = classLoader;
            if (this.processorClassLoader != null && (this.processorClassLoader instanceof Closeable)) {
                JavaCompiler compiler = JavaCompiler.instance(this.context);
                compiler.closeables = compiler.closeables.prepend((Closeable) this.processorClassLoader);
            }
        } catch (SecurityException e) {
            this.processorClassLoaderException = e;
        }
    }

    private void initProcessorIterator(Context context, Iterable<? extends Processor> processors) {
        Iterator<? extends Processor> processorIterator;
        Log log = Log.instance(context);
        if (this.options.isSet(Option.XPRINT)) {
            try {
                Processor processor = (Processor) PrintingProcessor.class.newInstance();
                processorIterator = List.of(processor).iterator();
            } catch (Throwable t) {
                AssertionError assertError = new AssertionError("Problem instantiating PrintingProcessor.");
                assertError.initCause(t);
                throw assertError;
            }
        } else if (processors != null) {
            processorIterator = processors.iterator();
        } else {
            String processorNames = this.options.get(Option.PROCESSOR);
            if (this.processorClassLoaderException == null) {
                if (processorNames != null) {
                    processorIterator = new NameProcessIterator(processorNames, this.processorClassLoader, log);
                } else {
                    processorIterator = new ServiceIterator(this.processorClassLoader, log);
                }
            } else {
                processorIterator = handleServiceLoaderUnavailability("proc.cant.create.loader", this.processorClassLoaderException);
            }
        }
        this.discoveredProcs = new DiscoveredProcessors(processorIterator);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Iterator<Processor> handleServiceLoaderUnavailability(String key, Exception e) {
        Iterable<? extends File> workingPath;
        JavaFileManager fileManager = (JavaFileManager) this.context.get(JavaFileManager.class);
        if (fileManager instanceof JavacFileManager) {
            StandardJavaFileManager standardFileManager = (JavacFileManager) fileManager;
            if (fileManager.hasLocation(StandardLocation.ANNOTATION_PROCESSOR_PATH)) {
                workingPath = standardFileManager.getLocation(StandardLocation.ANNOTATION_PROCESSOR_PATH);
            } else {
                workingPath = standardFileManager.getLocation(StandardLocation.CLASS_PATH);
            }
            if (needClassLoader(this.options.get(Option.PROCESSOR), workingPath)) {
                handleException(key, e);
            }
        } else {
            handleException(key, e);
        }
        java.util.List<Processor> pl = Collections.emptyList();
        return pl.iterator();
    }

    private void handleException(String key, Exception e) {
        if (e != null) {
            this.log.error(key, e.getLocalizedMessage());
            throw new Abort(e);
        }
        this.log.error(key, new Object[0]);
        throw new Abort();
    }

    private class ServiceIterator implements Iterator<Processor> {
        private Iterator<Processor> iterator;
        private ServiceLoader<Processor> loader;
        private Log log;

        ServiceIterator(ClassLoader classLoader, Log log) {
            this.log = log;
            try {
                try {
                    this.loader = ServiceLoader.load(Processor.class, classLoader);
                    this.iterator = this.loader.iterator();
                } catch (Exception e) {
                    this.iterator = JavacProcessingEnvironment.this.handleServiceLoaderUnavailability("proc.no.service", null);
                }
            } catch (Throwable t) {
                log.error("proc.service.problem", new Object[0]);
                throw new Abort(t);
            }
        }

        @Override // java.util.Iterator
        public boolean hasNext() {
            try {
                return this.iterator.hasNext();
            } catch (ServiceConfigurationError sce) {
                this.log.error("proc.bad.config.file", sce.getLocalizedMessage());
                throw new Abort(sce);
            } catch (Throwable t) {
                throw new Abort(t);
            }
        }

        @Override // java.util.Iterator
        public Processor next() {
            try {
                return this.iterator.next();
            } catch (ServiceConfigurationError sce) {
                this.log.error("proc.bad.config.file", sce.getLocalizedMessage());
                throw new Abort(sce);
            } catch (Throwable t) {
                throw new Abort(t);
            }
        }

        @Override // java.util.Iterator
        public void remove() {
            throw new UnsupportedOperationException();
        }

        public void close() {
            if (this.loader != null) {
                try {
                    this.loader.reload();
                } catch (Exception e) {
                }
            }
        }
    }

    private static class NameProcessIterator implements Iterator<Processor> {
        Log log;
        Iterator<String> names;
        Processor nextProc = null;
        ClassLoader processorCL;

        NameProcessIterator(String names, ClassLoader processorCL, Log log) {
            this.names = Arrays.asList(names.split(DocLint.TAGS_SEPARATOR)).iterator();
            this.processorCL = processorCL;
            this.log = log;
        }

        @Override // java.util.Iterator
        public boolean hasNext() {
            if (this.nextProc != null) {
                return true;
            }
            if (!this.names.hasNext()) {
                return false;
            }
            String processorName = this.names.next();
            try {
                try {
                    Processor processor = (Processor) this.processorCL.loadClass(processorName).newInstance();
                    this.nextProc = processor;
                    return true;
                } catch (ClientCodeException e) {
                    throw e;
                } catch (Throwable t) {
                    throw new AnnotationProcessingError(t);
                }
            } catch (ClassCastException e2) {
                this.log.error("proc.processor.wrong.type", processorName);
                return false;
            } catch (ClassNotFoundException e3) {
                this.log.error("proc.processor.not.found", processorName);
                return false;
            } catch (Exception e4) {
                this.log.error("proc.processor.cant.instantiate", processorName);
                return false;
            }
        }

        @Override // java.util.Iterator
        public Processor next() {
            if (hasNext()) {
                Processor p = this.nextProc;
                this.nextProc = null;
                return p;
            }
            throw new NoSuchElementException();
        }

        @Override // java.util.Iterator
        public void remove() {
            throw new UnsupportedOperationException();
        }
    }

    public boolean atLeastOneProcessor() {
        return this.discoveredProcs.iterator2().hasNext();
    }

    private Map<String, String> initProcessorOptions(Context context) {
        Options options = Options.instance(context);
        Set<String> keySet = options.keySet();
        Map<String, String> tempOptions = new LinkedHashMap<>();
        for (String key : keySet) {
            if (key.startsWith("-A") && key.length() > 2) {
                int sepIndex = key.indexOf(61);
                String candidateKey = null;
                String candidateValue = null;
                if (sepIndex == -1) {
                    candidateKey = key.substring(2);
                } else if (sepIndex >= 3) {
                    candidateKey = key.substring(2, sepIndex);
                    candidateValue = sepIndex < key.length() + (-1) ? key.substring(sepIndex + 1) : null;
                }
                tempOptions.put(candidateKey, candidateValue);
            }
        }
        return Collections.unmodifiableMap(tempOptions);
    }

    private Set<String> initUnmatchedProcessorOptions() {
        Set<String> unmatchedProcessorOptions = new HashSet<>();
        unmatchedProcessorOptions.addAll(this.processorOptions.keySet());
        return unmatchedProcessorOptions;
    }

    static class ProcessorState {
        public boolean contributed = false;
        public Processor processor;
        private ArrayList<Pattern> supportedAnnotationPatterns;
        private ArrayList<String> supportedOptionNames;

        ProcessorState(Processor p, Log log, Source source, ProcessingEnvironment env) {
            this.processor = p;
            try {
                this.processor.init(env);
                checkSourceVersionCompatibility(source, log);
                this.supportedAnnotationPatterns = new ArrayList<>();
                for (String importString : this.processor.getSupportedAnnotationTypes()) {
                    this.supportedAnnotationPatterns.add(JavacProcessingEnvironment.importStringToPattern(importString, this.processor, log));
                }
                this.supportedOptionNames = new ArrayList<>();
                for (String optionName : this.processor.getSupportedOptions()) {
                    if (checkOptionName(optionName, log)) {
                        this.supportedOptionNames.add(optionName);
                    }
                }
            } catch (ClientCodeException e) {
                throw e;
            } catch (Throwable t) {
                throw new AnnotationProcessingError(t);
            }
        }

        private void checkSourceVersionCompatibility(Source source, Log log) {
            SourceVersion procSourceVersion = this.processor.getSupportedSourceVersion();
            if (procSourceVersion.compareTo(Source.toSourceVersion(source)) < 0) {
                log.warning("proc.processor.incompatible.source.version", procSourceVersion, this.processor.getClass().getName(), source.name);
            }
        }

        private boolean checkOptionName(String optionName, Log log) {
            boolean valid = JavacProcessingEnvironment.isValidOptionName(optionName);
            if (!valid) {
                log.error("proc.processor.bad.option.name", optionName, this.processor.getClass().getName());
            }
            return valid;
        }

        public boolean annotationSupported(String annotationName) {
            for (Pattern p : this.supportedAnnotationPatterns) {
                if (p.matcher(annotationName).matches()) {
                    return true;
                }
            }
            return false;
        }

        public void removeSupportedOptions(Set<String> unmatchedProcessorOptions) {
            unmatchedProcessorOptions.removeAll(this.supportedOptionNames);
        }
    }

    class DiscoveredProcessors implements Iterable<ProcessorState> {
        ArrayList<ProcessorState> procStateList = new ArrayList<>();
        Iterator<? extends Processor> processorIterator;

        class ProcessorStateIterator implements Iterator<ProcessorState> {
            Iterator<ProcessorState> innerIter;
            boolean onProcInterator = false;
            DiscoveredProcessors psi;

            ProcessorStateIterator(DiscoveredProcessors psi) {
                this.psi = psi;
                this.innerIter = psi.procStateList.iterator();
            }

            /* JADX WARN: Can't rename method to resolve collision */
            @Override // java.util.Iterator
            public ProcessorState next() {
                if (!this.onProcInterator) {
                    if (this.innerIter.hasNext()) {
                        return this.innerIter.next();
                    }
                    this.onProcInterator = true;
                }
                if (this.psi.processorIterator.hasNext()) {
                    ProcessorState ps = new ProcessorState(this.psi.processorIterator.next(), JavacProcessingEnvironment.this.log, JavacProcessingEnvironment.this.source, JavacProcessingEnvironment.this);
                    this.psi.procStateList.add(ps);
                    return ps;
                }
                throw new NoSuchElementException();
            }

            @Override // java.util.Iterator
            public boolean hasNext() {
                if (this.onProcInterator) {
                    return this.psi.processorIterator.hasNext();
                }
                return this.innerIter.hasNext() || this.psi.processorIterator.hasNext();
            }

            @Override // java.util.Iterator
            public void remove() {
                throw new UnsupportedOperationException();
            }

            public void runContributingProcs(RoundEnvironment re) {
                if (!this.onProcInterator) {
                    Set<TypeElement> emptyTypeElements = Collections.emptySet();
                    while (this.innerIter.hasNext()) {
                        ProcessorState ps = this.innerIter.next();
                        if (ps.contributed) {
                            JavacProcessingEnvironment.this.callProcessor(ps.processor, emptyTypeElements, re);
                        }
                    }
                }
            }
        }

        @Override // java.lang.Iterable
        /* JADX INFO: renamed from: iterator, reason: merged with bridge method [inline-methods] */
        public Iterator<ProcessorState> iterator2() {
            return new ProcessorStateIterator(this);
        }

        DiscoveredProcessors(Iterator<? extends Processor> processorIterator) {
            this.processorIterator = processorIterator;
        }

        public void close() {
            if (this.processorIterator != null && (this.processorIterator instanceof ServiceIterator)) {
                ((ServiceIterator) this.processorIterator).close();
            }
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    /* JADX WARN: Type inference failed for: r2v4, types: [com.sun.tools.javac.processing.JavacProcessingEnvironment$DiscoveredProcessors$ProcessorStateIterator] */
    public void discoverAndRunProcs(Context context, Set<TypeElement> annotationsPresent, List<Symbol.ClassSymbol> topLevelClasses, List<Symbol.PackageSymbol> packageInfoFiles) {
        Map<String, TypeElement> unmatchedAnnotations = new HashMap<>(annotationsPresent.size());
        for (TypeElement a : annotationsPresent) {
            unmatchedAnnotations.put(a.getQualifiedName().toString(), a);
        }
        if (unmatchedAnnotations.size() == 0) {
            unmatchedAnnotations.put("", null);
        }
        ?? Iterator2 = this.discoveredProcs.iterator2();
        Set<Element> rootElements = new LinkedHashSet<>();
        rootElements.addAll(topLevelClasses);
        rootElements.addAll(packageInfoFiles);
        RoundEnvironment renv = new JavacRoundEnvironment(false, false, Collections.unmodifiableSet(rootElements), this);
        while (unmatchedAnnotations.size() > 0 && Iterator2.hasNext()) {
            ProcessorState ps = Iterator2.next();
            Set<String> matchedNames = new HashSet<>();
            Set<TypeElement> typeElements = new LinkedHashSet<>();
            for (Map.Entry<String, TypeElement> entry : unmatchedAnnotations.entrySet()) {
                String unmatchedAnnotationName = entry.getKey();
                if (ps.annotationSupported(unmatchedAnnotationName)) {
                    matchedNames.add(unmatchedAnnotationName);
                    TypeElement te = entry.getValue();
                    if (te != null) {
                        typeElements.add(te);
                    }
                }
            }
            if (matchedNames.size() > 0 || ps.contributed) {
                boolean processingResult = callProcessor(ps.processor, typeElements, renv);
                ps.contributed = true;
                ps.removeSupportedOptions(this.unmatchedProcessorOptions);
                if (this.printProcessorInfo || this.verbose) {
                    this.log.printLines("x.print.processor.info", ps.processor.getClass().getName(), matchedNames.toString(), Boolean.valueOf(processingResult));
                }
                if (processingResult) {
                    unmatchedAnnotations.keySet().removeAll(matchedNames);
                }
            }
        }
        unmatchedAnnotations.remove("");
        if (this.lint && unmatchedAnnotations.size() > 0) {
            unmatchedAnnotations.keySet().removeAll(this.platformAnnotations);
            if (unmatchedAnnotations.size() > 0) {
                this.log = Log.instance(context);
                this.log.warning("proc.annotations.without.processors", unmatchedAnnotations.keySet());
            }
        }
        Iterator2.runContributingProcs(renv);
        if (this.options.isSet("displayFilerState")) {
            this.filer.displayState();
        }
    }

    public static class ComputeAnnotationSet extends ElementScanner8<Set<TypeElement>, Set<TypeElement>> {
        final Elements elements;

        public ComputeAnnotationSet(Elements elements) {
            this.elements = elements;
        }

        @Override // javax.lang.model.util.ElementScanner6, javax.lang.model.element.ElementVisitor
        public Set<TypeElement> visitPackage(PackageElement e, Set<TypeElement> p) {
            return p;
        }

        @Override // javax.lang.model.util.ElementScanner6, javax.lang.model.element.ElementVisitor
        public Set<TypeElement> visitType(TypeElement e, Set<TypeElement> p) {
            scan(e.getTypeParameters(), p);
            return (Set) super.visitType(e, p);
        }

        @Override // javax.lang.model.util.ElementScanner6, javax.lang.model.element.ElementVisitor
        public Set<TypeElement> visitExecutable(ExecutableElement e, Set<TypeElement> p) {
            scan(e.getTypeParameters(), p);
            return (Set) super.visitExecutable(e, p);
        }

        void addAnnotations(Element e, Set<TypeElement> p) {
            for (AnnotationMirror annotationMirror : this.elements.getAllAnnotationMirrors(e)) {
                Element e2 = annotationMirror.getAnnotationType().asElement();
                p.add((TypeElement) e2);
            }
        }

        @Override // javax.lang.model.util.ElementScanner6
        public Set<TypeElement> scan(Element e, Set<TypeElement> p) {
            addAnnotations(e, p);
            return (Set) super.scan(e, p);
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public boolean callProcessor(Processor proc, Set<? extends TypeElement> tes, RoundEnvironment renv) {
        try {
            return proc.process(tes, renv);
        } catch (ClassReader.BadClassFile ex) {
            this.log.error("proc.cant.access.1", ex.sym, ex.getDetailValue());
            return false;
        } catch (Symbol.CompletionFailure ex2) {
            StringWriter out = new StringWriter();
            ex2.printStackTrace(new PrintWriter(out));
            this.log.error("proc.cant.access", ex2.sym, ex2.getDetailValue(), out.toString());
            return false;
        } catch (ClientCodeException e) {
            throw e;
        } catch (Throwable t) {
            throw new AnnotationProcessingError(t);
        }
    }

    class Round {
        Set<TypeElement> annotationsPresent;
        final JavaCompiler compiler;
        final Context context;
        final Log.DeferredDiagnosticHandler deferredDiagnosticHandler;
        Map<String, JavaFileObject> genClassFiles;
        final Log log;
        final int number;
        List<Symbol.PackageSymbol> packageInfoFiles;
        List<JCTree.JCCompilationUnit> roots;
        List<Symbol.ClassSymbol> topLevelClasses;

        private Round(Context context, int number, int priorErrors, int priorWarnings, Log.DeferredDiagnosticHandler deferredDiagnosticHandler) {
            this.context = context;
            this.number = number;
            this.compiler = JavaCompiler.instance(context);
            this.log = Log.instance(context);
            this.log.nerrors = priorErrors;
            this.log.nwarnings = priorWarnings;
            if (number == 1) {
                Assert.checkNonNull(deferredDiagnosticHandler);
                this.deferredDiagnosticHandler = deferredDiagnosticHandler;
            } else {
                this.deferredDiagnosticHandler = new Log.DeferredDiagnosticHandler(this.log);
            }
            JavacProcessingEnvironment.this.context = context;
            this.topLevelClasses = List.nil();
            this.packageInfoFiles = List.nil();
        }

        Round(JavacProcessingEnvironment this$0, Context context, List<JCTree.JCCompilationUnit> roots, List<Symbol.ClassSymbol> classSymbols, Log.DeferredDiagnosticHandler deferredDiagnosticHandler) {
            this(context, 1, 0, 0, deferredDiagnosticHandler);
            this.roots = roots;
            this.genClassFiles = new HashMap();
            this.compiler.todo.clear();
            this.topLevelClasses = this$0.getTopLevelClasses(roots).prependList(classSymbols.reverse());
            this.packageInfoFiles = this$0.getPackageInfoFiles(roots);
            findAnnotationsPresent();
        }

        private Round(JavacProcessingEnvironment this$0, Round prev, Set<JavaFileObject> newSourceFiles, Map<String, JavaFileObject> newClassFiles) {
            this(prev.nextContext(), prev.number + 1, prev.compiler.log.nerrors, prev.compiler.log.nwarnings, null);
            this.genClassFiles = prev.genClassFiles;
            List<JCTree.JCCompilationUnit> parsedFiles = this.compiler.parseFiles(newSourceFiles);
            this.roots = JavacProcessingEnvironment.cleanTrees(prev.roots).appendList(parsedFiles);
            if (unrecoverableError()) {
                return;
            }
            enterClassFiles(this.genClassFiles);
            List<Symbol.ClassSymbol> newClasses = enterClassFiles(newClassFiles);
            this.genClassFiles.putAll(newClassFiles);
            enterTrees(this.roots);
            if (!unrecoverableError()) {
                this.topLevelClasses = JavacProcessingEnvironment.join(this$0.getTopLevelClasses(parsedFiles), this$0.getTopLevelClassesFromClasses(newClasses));
                this.packageInfoFiles = JavacProcessingEnvironment.join(this$0.getPackageInfoFiles(parsedFiles), this$0.getPackageInfoFilesFromClasses(newClasses));
                findAnnotationsPresent();
            }
        }

        Round next(Set<JavaFileObject> newSourceFiles, Map<String, JavaFileObject> newClassFiles) {
            try {
                return new Round(JavacProcessingEnvironment.this, this, newSourceFiles, newClassFiles);
            } finally {
                this.compiler.close(false);
            }
        }

        JavaCompiler finalCompiler() {
            try {
                Context nextCtx = nextContext();
                JavacProcessingEnvironment.this.context = nextCtx;
                JavaCompiler c = JavaCompiler.instance(nextCtx);
                c.log.initRound(this.compiler.log);
                return c;
            } finally {
                this.compiler.close(false);
            }
        }

        int errorCount() {
            return this.compiler.errorCount();
        }

        int warningCount() {
            return this.compiler.warningCount();
        }

        boolean unrecoverableError() {
            if (JavacProcessingEnvironment.this.messager.errorRaised()) {
                return true;
            }
            for (JCDiagnostic d : this.deferredDiagnosticHandler.getDiagnostics()) {
                switch (d.getKind()) {
                    case WARNING:
                        if (JavacProcessingEnvironment.this.werror) {
                            return true;
                        }
                        break;
                        break;
                    case ERROR:
                        if (JavacProcessingEnvironment.this.fatalErrors || !d.isFlagSet(JCDiagnostic.DiagnosticFlag.RECOVERABLE)) {
                            return true;
                        }
                        break;
                        break;
                }
            }
            return false;
        }

        void findAnnotationsPresent() {
            ComputeAnnotationSet annotationComputer = new ComputeAnnotationSet(JavacProcessingEnvironment.this.elementUtils);
            this.annotationsPresent = new LinkedHashSet();
            for (Symbol.ClassSymbol classSym : this.topLevelClasses) {
                annotationComputer.scan((Element) classSym, this.annotationsPresent);
            }
            for (Symbol.PackageSymbol pkgSym : this.packageInfoFiles) {
                annotationComputer.scan((Element) pkgSym, this.annotationsPresent);
            }
        }

        private List<Symbol.ClassSymbol> enterClassFiles(Map<String, JavaFileObject> classFiles) {
            Symbol.ClassSymbol cs;
            ClassReader reader = ClassReader.instance(this.context);
            Names names = Names.instance(this.context);
            List<Symbol.ClassSymbol> list = List.nil();
            for (Map.Entry<String, JavaFileObject> entry : classFiles.entrySet()) {
                Name name = names.fromString(entry.getKey());
                JavaFileObject file = entry.getValue();
                if (file.getKind() == JavaFileObject.Kind.CLASS) {
                    if (JavacProcessingEnvironment.this.isPkgInfo(file, JavaFileObject.Kind.CLASS)) {
                        Name packageName = Convert.packagePart(name);
                        Symbol.PackageSymbol p = reader.enterPackage(packageName);
                        if (p.package_info == null) {
                            p.package_info = reader.enterClass(Convert.shortName(name), p);
                        }
                        cs = p.package_info;
                        if (cs.classfile == null) {
                            cs.classfile = file;
                        }
                    } else {
                        cs = reader.enterClass(name, file);
                    }
                    list = list.prepend(cs);
                } else {
                    throw new AssertionError(file);
                }
            }
            return list.reverse();
        }

        private void enterTrees(List<JCTree.JCCompilationUnit> roots) {
            this.compiler.enterTrees(roots);
        }

        /* JADX WARN: Type inference failed for: r2v7, types: [com.sun.tools.javac.processing.JavacProcessingEnvironment$DiscoveredProcessors$ProcessorStateIterator] */
        void run(boolean lastRound, boolean errorStatus) {
            MultiTaskListener multiTaskListener;
            printRoundInfo(lastRound);
            if (!JavacProcessingEnvironment.this.taskListener.isEmpty()) {
                JavacProcessingEnvironment.this.taskListener.started(new TaskEvent(TaskEvent.Kind.ANNOTATION_PROCESSING_ROUND));
            }
            try {
                if (lastRound) {
                    JavacProcessingEnvironment.this.filer.setLastRound(true);
                    Set<Element> emptyRootElements = Collections.emptySet();
                    RoundEnvironment renv = new JavacRoundEnvironment(true, errorStatus, emptyRootElements, JavacProcessingEnvironment.this);
                    JavacProcessingEnvironment.this.discoveredProcs.iterator2().runContributingProcs(renv);
                } else {
                    JavacProcessingEnvironment.this.discoverAndRunProcs(this.context, this.annotationsPresent, this.topLevelClasses, this.packageInfoFiles);
                }
                if (!multiTaskListener.isEmpty()) {
                    JavacProcessingEnvironment.this.taskListener.finished(new TaskEvent(TaskEvent.Kind.ANNOTATION_PROCESSING_ROUND));
                }
            } catch (Throwable t) {
                try {
                    this.deferredDiagnosticHandler.reportDeferredDiagnostics();
                    this.log.popDiagnosticHandler(this.deferredDiagnosticHandler);
                    throw t;
                } finally {
                    if (!JavacProcessingEnvironment.this.taskListener.isEmpty()) {
                        JavacProcessingEnvironment.this.taskListener.finished(new TaskEvent(TaskEvent.Kind.ANNOTATION_PROCESSING_ROUND));
                    }
                }
            }
        }

        void showDiagnostics(boolean showAll) {
            Set<Diagnostic.Kind> kinds = EnumSet.allOf(Diagnostic.Kind.class);
            if (!showAll) {
                kinds.remove(Diagnostic.Kind.ERROR);
            }
            this.deferredDiagnosticHandler.reportDeferredDiagnostics(kinds);
            this.log.popDiagnosticHandler(this.deferredDiagnosticHandler);
        }

        private void printRoundInfo(boolean lastRound) {
            if (JavacProcessingEnvironment.this.printRounds || JavacProcessingEnvironment.this.verbose) {
                List<Symbol.ClassSymbol> tlc = lastRound ? List.nil() : this.topLevelClasses;
                Set<TypeElement> ap = lastRound ? Collections.emptySet() : this.annotationsPresent;
                this.log.printLines("x.print.rounds", Integer.valueOf(this.number), "{" + tlc.toString(", ") + "}", ap, Boolean.valueOf(lastRound));
            }
        }

        private Context nextContext() {
            Context next = new Context(this.context);
            Options options = Options.instance(this.context);
            Assert.checkNonNull(options);
            next.put(Options.optionsKey, options);
            Locale locale = (Locale) this.context.get(Locale.class);
            if (locale != null) {
                next.put((Class<Locale>) Locale.class, locale);
            }
            Assert.checkNonNull(JavacProcessingEnvironment.this.messages);
            next.put(JavacMessages.messagesKey, JavacProcessingEnvironment.this.messages);
            Names names = Names.instance(this.context);
            Assert.checkNonNull(names);
            next.put(Names.namesKey, names);
            DiagnosticListener<?> dl = (DiagnosticListener) this.context.get(DiagnosticListener.class);
            if (dl != null) {
                next.put((Class<DiagnosticListener<?>>) DiagnosticListener.class, dl);
            }
            MultiTaskListener mtl = (MultiTaskListener) this.context.get(MultiTaskListener.taskListenerKey);
            if (mtl != null) {
                next.put(MultiTaskListener.taskListenerKey, mtl);
            }
            FSInfo fsInfo = (FSInfo) this.context.get(FSInfo.class);
            if (fsInfo != null) {
                next.put((Class<FSInfo>) FSInfo.class, fsInfo);
            }
            JavaFileManager jfm = (JavaFileManager) this.context.get(JavaFileManager.class);
            Assert.checkNonNull(jfm);
            next.put((Class<JavaFileManager>) JavaFileManager.class, jfm);
            if (jfm instanceof JavacFileManager) {
                ((JavacFileManager) jfm).setContext(next);
            }
            Names names2 = Names.instance(this.context);
            Assert.checkNonNull(names2);
            next.put(Names.namesKey, names2);
            Tokens tokens = Tokens.instance(this.context);
            Assert.checkNonNull(tokens);
            next.put(Tokens.tokensKey, tokens);
            Log nextLog = Log.instance(next);
            nextLog.initRound(this.log);
            JavaCompiler oldCompiler = JavaCompiler.instance(this.context);
            JavaCompiler nextCompiler = JavaCompiler.instance(next);
            nextCompiler.initRound(oldCompiler);
            JavacProcessingEnvironment.this.filer.newRound(next);
            JavacProcessingEnvironment.this.messager.newRound(next);
            JavacProcessingEnvironment.this.elementUtils.setContext(next);
            JavacProcessingEnvironment.this.typeUtils.setContext(next);
            JavacTask task = (JavacTask) this.context.get(JavacTask.class);
            if (task != null) {
                next.put((Class<JavacTask>) JavacTask.class, task);
                if (task instanceof BasicJavacTask) {
                    ((BasicJavacTask) task).updateContext(next);
                }
            }
            JavacTrees trees = (JavacTrees) this.context.get(JavacTrees.class);
            if (trees != null) {
                next.put((Class<JavacTrees>) JavacTrees.class, trees);
                trees.updateContext(next);
            }
            this.context.clear();
            return next;
        }
    }

    public JavaCompiler doProcessing(Context context, List<JCTree.JCCompilationUnit> roots, List<Symbol.ClassSymbol> classSymbols, Iterable<? extends Symbol.PackageSymbol> pckSymbols, Log.DeferredDiagnosticHandler deferredDiagnosticHandler) {
        boolean errorStatus;
        boolean errorStatus2;
        boolean z;
        this.log = Log.instance(context);
        Set<Symbol.PackageSymbol> specifiedPackages = new LinkedHashSet<>();
        for (Symbol.PackageSymbol psym : pckSymbols) {
            specifiedPackages.add(psym);
        }
        this.specifiedPackages = Collections.unmodifiableSet(specifiedPackages);
        Round round = new Round(this, context, roots, classSymbols, deferredDiagnosticHandler);
        do {
            errorStatus = false;
            round.run(false, false);
            errorStatus2 = round.unrecoverableError();
            boolean moreToDo = moreToDo();
            if (!errorStatus2 && !this.showResolveErrors) {
                z = false;
            } else {
                z = true;
            }
            round.showDiagnostics(z);
            round = round.next(new LinkedHashSet(this.filer.getGeneratedSourceFileObjects()), new LinkedHashMap(this.filer.getGeneratedClasses()));
            if (round.unrecoverableError()) {
                errorStatus2 = true;
            }
            if (!moreToDo) {
                break;
            }
        } while (!errorStatus2);
        round.run(true, errorStatus2);
        round.showDiagnostics(true);
        this.filer.warnIfUnclosedFiles();
        warnIfUnmatchedOptions();
        if (this.messager.errorRaised() || (this.werror && round.warningCount() > 0 && round.errorCount() > 0)) {
            errorStatus2 = true;
        }
        Set<JavaFileObject> newSourceFiles = new LinkedHashSet<>(this.filer.getGeneratedSourceFileObjects());
        List<JCTree.JCCompilationUnit> roots2 = cleanTrees(round.roots);
        JavaCompiler compiler = round.finalCompiler();
        if (newSourceFiles.size() > 0) {
            roots2 = roots2.appendList(compiler.parseFiles(newSourceFiles));
        }
        if (errorStatus2 || compiler.errorCount() > 0) {
            errorStatus = true;
        }
        close();
        if (!this.taskListener.isEmpty()) {
            this.taskListener.finished(new TaskEvent(TaskEvent.Kind.ANNOTATION_PROCESSING));
        }
        if (errorStatus) {
            if (compiler.errorCount() == 0) {
                compiler.log.nerrors++;
            }
            return compiler;
        }
        compiler.enterTreesIfNeeded(roots2);
        return compiler;
    }

    private void warnIfUnmatchedOptions() {
        if (!this.unmatchedProcessorOptions.isEmpty()) {
            this.log.warning("proc.unmatched.processor.options", this.unmatchedProcessorOptions.toString());
        }
    }

    @Override // java.io.Closeable, java.lang.AutoCloseable
    public void close() {
        this.filer.close();
        if (this.discoveredProcs != null) {
            this.discoveredProcs.close();
        }
        this.discoveredProcs = null;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public List<Symbol.ClassSymbol> getTopLevelClasses(List<? extends JCTree.JCCompilationUnit> units) {
        List<Symbol.ClassSymbol> classes = List.nil();
        for (JCTree.JCCompilationUnit unit : units) {
            for (JCTree node : unit.defs) {
                if (node.hasTag(JCTree.Tag.CLASSDEF)) {
                    Symbol.ClassSymbol sym = ((JCTree.JCClassDecl) node).sym;
                    Assert.checkNonNull(sym);
                    classes = classes.prepend(sym);
                }
            }
        }
        return classes.reverse();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public List<Symbol.ClassSymbol> getTopLevelClassesFromClasses(List<? extends Symbol.ClassSymbol> syms) {
        List<Symbol.ClassSymbol> classes = List.nil();
        for (Symbol.ClassSymbol sym : syms) {
            if (!isPkgInfo(sym)) {
                classes = classes.prepend(sym);
            }
        }
        return classes.reverse();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public List<Symbol.PackageSymbol> getPackageInfoFiles(List<? extends JCTree.JCCompilationUnit> units) {
        List<Symbol.PackageSymbol> packages = List.nil();
        for (JCTree.JCCompilationUnit unit : units) {
            if (isPkgInfo(unit.sourcefile, JavaFileObject.Kind.SOURCE)) {
                packages = packages.prepend(unit.packge);
            }
        }
        return packages.reverse();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public List<Symbol.PackageSymbol> getPackageInfoFilesFromClasses(List<? extends Symbol.ClassSymbol> syms) {
        List<Symbol.PackageSymbol> packages = List.nil();
        for (Symbol.ClassSymbol sym : syms) {
            if (isPkgInfo(sym)) {
                packages = packages.prepend((Symbol.PackageSymbol) sym.owner);
            }
        }
        return packages.reverse();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static <T> List<T> join(List<T> list1, List<T> list2) {
        return list1.appendList(list2);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public boolean isPkgInfo(JavaFileObject fo, JavaFileObject.Kind kind) {
        return fo.isNameCompatible("package-info", kind);
    }

    private boolean isPkgInfo(Symbol.ClassSymbol sym) {
        return isPkgInfo(sym.classfile, JavaFileObject.Kind.CLASS) && sym.packge().package_info == sym;
    }

    private boolean needClassLoader(String procNames, Iterable<? extends File> workingpath) {
        if (procNames != null) {
            return true;
        }
        URL[] urls = new URL[1];
        for (File pathElement : workingpath) {
            try {
                urls[0] = pathElement.toURI().toURL();
                if (ServiceProxy.hasService(Processor.class, urls)) {
                    return true;
                }
            } catch (ServiceProxy.ServiceConfigurationError e) {
                this.log.error("proc.bad.config.file", e.getLocalizedMessage());
                return true;
            } catch (MalformedURLException ex) {
                throw new AssertionError(ex);
            }
        }
        return false;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static <T extends JCTree> List<T> cleanTrees(List<T> nodes) {
        for (T node : nodes) {
            treeCleaner.scan(node);
        }
        return nodes;
    }

    private boolean moreToDo() {
        return this.filer.newFiles();
    }

    @Override // javax.annotation.processing.ProcessingEnvironment
    public Map<String, String> getOptions() {
        return this.processorOptions;
    }

    @Override // javax.annotation.processing.ProcessingEnvironment
    public Messager getMessager() {
        return this.messager;
    }

    @Override // javax.annotation.processing.ProcessingEnvironment
    public Filer getFiler() {
        return this.filer;
    }

    @Override // javax.annotation.processing.ProcessingEnvironment
    public JavacElements getElementUtils() {
        return this.elementUtils;
    }

    @Override // javax.annotation.processing.ProcessingEnvironment
    public JavacTypes getTypeUtils() {
        return this.typeUtils;
    }

    @Override // javax.annotation.processing.ProcessingEnvironment
    public SourceVersion getSourceVersion() {
        return Source.toSourceVersion(this.source);
    }

    @Override // javax.annotation.processing.ProcessingEnvironment
    public Locale getLocale() {
        return this.messages.getCurrentLocale();
    }

    public Set<Symbol.PackageSymbol> getSpecifiedPackages() {
        return this.specifiedPackages;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static Pattern importStringToPattern(String s, Processor p, Log log) {
        if (isValidImportString(s)) {
            return validImportStringToPattern(s);
        }
        log.warning("proc.malformed.supported.string", s, p.getClass().getName());
        return noMatches;
    }

    public static boolean isValidImportString(String s) {
        if (s.equals(Marker.ANY_MARKER)) {
            return true;
        }
        boolean valid = true;
        String t = s;
        int index = t.indexOf(42);
        if (index != -1) {
            if (index != t.length() - 1) {
                return false;
            }
            if (index - 1 >= 0) {
                valid = t.charAt(index + (-1)) == '.';
                t = t.substring(0, t.length() - 2);
            }
        }
        if (valid) {
            String[] javaIds = t.split("\\.", t.length() + 2);
            for (String javaId : javaIds) {
                valid &= SourceVersion.isIdentifier(javaId);
            }
        }
        return valid;
    }

    public static Pattern validImportStringToPattern(String s) {
        if (s.equals(Marker.ANY_MARKER)) {
            return allMatches;
        }
        String s_prime = s.replace(".", "\\.");
        if (s_prime.endsWith(Marker.ANY_MARKER)) {
            s_prime = s_prime.substring(0, s_prime.length() - 1) + ".+";
        }
        return Pattern.compile(s_prime);
    }

    public Context getContext() {
        return this.context;
    }

    public ClassLoader getProcessorClassLoader() {
        return this.processorClassLoader;
    }

    public String toString() {
        return "javac ProcessingEnvironment";
    }

    public static boolean isValidOptionName(String optionName) {
        for (String s : optionName.split("\\.", -1)) {
            if (!SourceVersion.isIdentifier(s)) {
                return false;
            }
        }
        return true;
    }
}
