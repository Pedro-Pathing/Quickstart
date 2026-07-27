package com.sun.tools.doclint;

import com.sun.source.doctree.DocCommentTree;
import com.sun.source.tree.ClassTree;
import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.MethodTree;
import com.sun.source.tree.Tree;
import com.sun.source.tree.VariableTree;
import com.sun.source.util.JavacTask;
import com.sun.source.util.Plugin;
import com.sun.source.util.TaskEvent;
import com.sun.source.util.TaskListener;
import com.sun.source.util.TreePath;
import com.sun.source.util.TreePathScanner;
import com.sun.tools.doclint.Messages;
import com.sun.tools.javac.api.JavacTaskImpl;
import com.sun.tools.javac.api.JavacTool;
import com.sun.tools.javac.file.JavacFileManager;
import com.sun.tools.javac.main.JavaCompiler;
import com.sun.tools.javac.util.Context;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import javax.lang.model.element.Name;
import javax.tools.DiagnosticListener;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.StandardLocation;

/* JADX INFO: loaded from: classes.dex */
public class DocLint implements Plugin {
    private static final String STATS = "-stats";
    public static final String TAGS_SEPARATOR = ",";
    public static final String XCUSTOM_TAGS_PREFIX = "-XcustomTags:";
    public static final String XIMPLICIT_HEADERS = "-XimplicitHeaders:";
    public static final String XMSGS_CUSTOM_PREFIX = "-Xmsgs:";
    public static final String XMSGS_OPTION = "-Xmsgs";
    Checker checker;
    Env env;
    List<File> javacBootClassPath;
    List<File> javacClassPath;
    List<File> javacFiles;
    List<String> javacOpts;
    List<File> javacSourcePath;
    boolean needHelp = false;

    public static void main(String... args) {
        DocLint dl = new DocLint();
        try {
            dl.run(args);
        } catch (BadArgs e) {
            System.err.println(e.getMessage());
            System.exit(1);
        } catch (IOException e2) {
            System.err.println(dl.localize("dc.main.ioerror", e2.getLocalizedMessage()));
            System.exit(2);
        }
    }

    public class BadArgs extends Exception {
        private static final long serialVersionUID = 0;
        final Object[] args;
        final String code;

        BadArgs(String code, Object... args) {
            super(DocLint.this.localize(code, args));
            this.code = code;
            this.args = args;
        }
    }

    public void run(String... args) throws BadArgs, IOException {
        PrintWriter out = new PrintWriter(System.out);
        try {
            run(out, args);
        } finally {
            out.flush();
        }
    }

    public void run(PrintWriter out, String... args) throws BadArgs, IOException {
        this.env = new Env();
        processArgs(args);
        if (this.needHelp) {
            showHelp(out);
        }
        if (this.javacFiles.isEmpty() && !this.needHelp) {
            out.println(localize("dc.main.no.files.given", new Object[0]));
        }
        JavacTool tool = JavacTool.create();
        JavacFileManager fm = new JavacFileManager(new Context(), false, null);
        fm.setSymbolFileEnabled(false);
        fm.setLocation(StandardLocation.PLATFORM_CLASS_PATH, this.javacBootClassPath);
        fm.setLocation(StandardLocation.CLASS_PATH, this.javacClassPath);
        fm.setLocation(StandardLocation.SOURCE_PATH, this.javacSourcePath);
        JavacTask task = tool.getTask((Writer) out, (JavaFileManager) fm, (DiagnosticListener<? super JavaFileObject>) null, this.javacOpts, (Iterable<String>) null, fm.getJavaFileObjectsFromFiles(this.javacFiles));
        Iterable<? extends CompilationUnitTree> units = task.parse();
        ((JavacTaskImpl) task).enter();
        this.env.init(task);
        this.checker = new Checker(this.env);
        DeclScanner ds = new DeclScanner() { // from class: com.sun.tools.doclint.DocLint.1
            @Override // com.sun.tools.doclint.DocLint.DeclScanner
            void visitDecl(Tree tree, Name name) {
                TreePath p = getCurrentPath();
                DocCommentTree dc = DocLint.this.env.trees.getDocCommentTree(p);
                DocLint.this.checker.scan(dc, p);
            }
        };
        ds.scan(units, (Object) null);
        reportStats(out);
        Context ctx = ((JavacTaskImpl) task).getContext();
        JavaCompiler c = JavaCompiler.instance(ctx);
        c.printCount("error", c.errorCount());
        c.printCount("warn", c.warningCount());
    }

    void processArgs(String... args) throws BadArgs {
        this.javacOpts = new ArrayList();
        this.javacFiles = new ArrayList();
        if (args.length == 0) {
            this.needHelp = true;
        }
        int i = 0;
        while (i < args.length) {
            String arg = args[i];
            if (arg.matches("-Xmax(errs|warns)") && i + 1 < args.length) {
                i++;
                if (args[i].matches("[0-9]+")) {
                    this.javacOpts.add(arg);
                    this.javacOpts.add(args[i]);
                } else {
                    throw new BadArgs("dc.bad.value.for.option", arg, args[i]);
                }
            } else if (arg.equals(STATS)) {
                this.env.messages.setStatsEnabled(true);
            } else if (arg.equals("-bootclasspath") && i + 1 < args.length) {
                i++;
                this.javacBootClassPath = splitPath(args[i]);
            } else if (arg.equals("-classpath") && i + 1 < args.length) {
                i++;
                this.javacClassPath = splitPath(args[i]);
            } else if (arg.equals("-cp") && i + 1 < args.length) {
                i++;
                this.javacClassPath = splitPath(args[i]);
            } else if (arg.equals("-sourcepath") && i + 1 < args.length) {
                i++;
                this.javacSourcePath = splitPath(args[i]);
            } else if (arg.equals(XMSGS_OPTION)) {
                this.env.messages.setOptions(null);
            } else if (arg.startsWith(XMSGS_CUSTOM_PREFIX)) {
                this.env.messages.setOptions(arg.substring(arg.indexOf(":") + 1));
            } else if (arg.startsWith(XCUSTOM_TAGS_PREFIX)) {
                this.env.setCustomTags(arg.substring(arg.indexOf(":") + 1));
            } else if (arg.equals("-h") || arg.equals("-help") || arg.equals("--help") || arg.equals("-?") || arg.equals("-usage")) {
                this.needHelp = true;
            } else {
                if (arg.startsWith("-")) {
                    throw new BadArgs("dc.bad.option", arg);
                }
                while (i < args.length) {
                    this.javacFiles.add(new File(args[i]));
                    i++;
                }
            }
            i++;
        }
    }

    void showHelp(PrintWriter out) {
        String msg = localize("dc.main.usage", new Object[0]);
        for (String line : msg.split("\n")) {
            out.println(line);
        }
    }

    List<File> splitPath(String path) {
        List<File> files = new ArrayList<>();
        for (String f : path.split(File.pathSeparator)) {
            if (f.length() > 0) {
                files.add(new File(f));
            }
        }
        return files;
    }

    @Override // com.sun.source.util.Plugin
    public String getName() {
        return "doclint";
    }

    @Override // com.sun.source.util.Plugin
    public void init(JavacTask task, String... args) {
        init(task, args, true);
    }

    public void init(JavacTask task, String[] args, boolean addTaskListener) {
        this.env = new Env();
        for (String arg : args) {
            if (arg.equals(XMSGS_OPTION)) {
                this.env.messages.setOptions(null);
            } else if (arg.startsWith(XMSGS_CUSTOM_PREFIX)) {
                this.env.messages.setOptions(arg.substring(arg.indexOf(":") + 1));
            } else if (arg.matches("-XimplicitHeaders:[1-6]")) {
                char ch = arg.charAt(arg.length() - 1);
                this.env.setImplicitHeaders(Character.digit(ch, 10));
            } else if (arg.startsWith(XCUSTOM_TAGS_PREFIX)) {
                this.env.setCustomTags(arg.substring(arg.indexOf(":") + 1));
            } else {
                throw new IllegalArgumentException(arg);
            }
        }
        this.env.init(task);
        this.checker = new Checker(this.env);
        if (addTaskListener) {
            final DeclScanner ds = new DeclScanner() { // from class: com.sun.tools.doclint.DocLint.2
                @Override // com.sun.tools.doclint.DocLint.DeclScanner
                void visitDecl(Tree tree, Name name) {
                    TreePath p = getCurrentPath();
                    DocCommentTree dc = DocLint.this.env.trees.getDocCommentTree(p);
                    DocLint.this.checker.scan(dc, p);
                }
            };
            TaskListener tl = new TaskListener() { // from class: com.sun.tools.doclint.DocLint.3
                Queue<CompilationUnitTree> todo = new LinkedList();

                @Override // com.sun.source.util.TaskListener
                public void started(TaskEvent e) {
                    switch (AnonymousClass4.$SwitchMap$com$sun$source$util$TaskEvent$Kind[e.getKind().ordinal()]) {
                        case 1:
                            break;
                        default:
                            return;
                    }
                    while (true) {
                        CompilationUnitTree tree = this.todo.poll();
                        if (tree != null) {
                            ds.scan(tree, (Object) null);
                        } else {
                            return;
                        }
                    }
                }

                @Override // com.sun.source.util.TaskListener
                public void finished(TaskEvent e) {
                    switch (AnonymousClass4.$SwitchMap$com$sun$source$util$TaskEvent$Kind[e.getKind().ordinal()]) {
                        case 2:
                            this.todo.add(e.getCompilationUnit());
                            break;
                    }
                }
            };
            task.addTaskListener(tl);
        }
    }

    /* JADX INFO: renamed from: com.sun.tools.doclint.DocLint$4, reason: invalid class name */
    static /* synthetic */ class AnonymousClass4 {
        static final /* synthetic */ int[] $SwitchMap$com$sun$source$util$TaskEvent$Kind = new int[TaskEvent.Kind.values().length];

        static {
            try {
                $SwitchMap$com$sun$source$util$TaskEvent$Kind[TaskEvent.Kind.ANALYZE.ordinal()] = 1;
            } catch (NoSuchFieldError e) {
            }
            try {
                $SwitchMap$com$sun$source$util$TaskEvent$Kind[TaskEvent.Kind.PARSE.ordinal()] = 2;
            } catch (NoSuchFieldError e2) {
            }
        }
    }

    public void scan(TreePath p) {
        DocCommentTree dc = this.env.trees.getDocCommentTree(p);
        this.checker.scan(dc, p);
    }

    public void reportStats(PrintWriter out) {
        this.env.messages.reportStats(out);
    }

    public static boolean isValidOption(String opt) {
        if (opt.equals(XMSGS_OPTION)) {
            return true;
        }
        if (opt.startsWith(XMSGS_CUSTOM_PREFIX)) {
            return Messages.Options.isValidOptions(opt.substring(XMSGS_CUSTOM_PREFIX.length()));
        }
        return false;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public String localize(String code, Object... args) {
        Messages m = this.env != null ? this.env.messages : new Messages(null);
        return m.localize(code, args);
    }

    static abstract class DeclScanner extends TreePathScanner<Void, Void> {
        abstract void visitDecl(Tree tree, Name name);

        DeclScanner() {
        }

        @Override // com.sun.source.util.TreeScanner, com.sun.source.tree.TreeVisitor
        public Void visitCompilationUnit(CompilationUnitTree tree, Void ignore) {
            if (tree.getPackageName() != null) {
                visitDecl(tree, null);
            }
            return (Void) super.visitCompilationUnit(tree, ignore);
        }

        @Override // com.sun.source.util.TreeScanner, com.sun.source.tree.TreeVisitor
        public Void visitClass(ClassTree tree, Void ignore) {
            visitDecl(tree, tree.getSimpleName());
            return (Void) super.visitClass(tree, ignore);
        }

        @Override // com.sun.source.util.TreeScanner, com.sun.source.tree.TreeVisitor
        public Void visitMethod(MethodTree tree, Void ignore) {
            visitDecl(tree, tree.getName());
            return null;
        }

        @Override // com.sun.source.util.TreeScanner, com.sun.source.tree.TreeVisitor
        public Void visitVariable(VariableTree tree, Void ignore) {
            visitDecl(tree, tree.getName());
            return (Void) super.visitVariable(tree, ignore);
        }
    }
}
