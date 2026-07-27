package com.sun.tools.javac.tree;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.sun.source.tree.MemberReferenceTree;
import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.code.BoundKind;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.Convert;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Name;
import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;
import org.firstinspires.ftc.onbotjava.RequestConditions;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes.dex */
public class Pretty extends JCTree.Visitor {
    private static final int PREFERRED_LENGTH = 20;
    private static final String trimSequence = "[...]";
    Name enclClassName;
    Writer out;
    int prec;
    private final boolean sourceOutput;
    public int width = 4;
    int lmargin = 0;
    DocCommentTable docComments = null;
    String lineSep = System.getProperty("line.separator");

    public Pretty(Writer out, boolean sourceOutput) {
        this.out = out;
        this.sourceOutput = sourceOutput;
    }

    void align() throws IOException {
        for (int i = 0; i < this.lmargin; i++) {
            this.out.write(" ");
        }
    }

    void indent() {
        this.lmargin += this.width;
    }

    void undent() {
        this.lmargin -= this.width;
    }

    void open(int contextPrec, int ownPrec) throws IOException {
        if (ownPrec < contextPrec) {
            this.out.write("(");
        }
    }

    void close(int contextPrec, int ownPrec) throws IOException {
        if (ownPrec < contextPrec) {
            this.out.write(")");
        }
    }

    public void print(Object s) throws IOException {
        this.out.write(Convert.escapeUnicode(s.toString()));
    }

    public void println() throws IOException {
        this.out.write(this.lineSep);
    }

    public static String toSimpleString(JCTree tree) {
        return toSimpleString(tree, 20);
    }

    public static String toSimpleString(JCTree tree, int maxLength) {
        StringWriter s = new StringWriter();
        try {
            new Pretty(s, false).printExpr(tree);
            String res = s.toString().trim().replaceAll("\\s+", " ").replaceAll("/\\*missing\\*/", "");
            if (res.length() < maxLength) {
                return res;
            }
            int head = ((maxLength - trimSequence.length()) * 2) / 3;
            int tail = (maxLength - trimSequence.length()) - head;
            return res.substring(0, head) + trimSequence + res.substring(res.length() - tail);
        } catch (IOException e) {
            throw new AssertionError(e);
        }
    }

    private static class UncheckedIOException extends Error {
        static final long serialVersionUID = -4032692679158424751L;

        UncheckedIOException(IOException e) {
            super(e.getMessage(), e);
        }
    }

    public void printExpr(JCTree tree, int prec) throws IOException {
        int prevPrec = this.prec;
        try {
            try {
                this.prec = prec;
                if (tree == null) {
                    print("/*missing*/");
                } else {
                    tree.accept(this);
                }
            } catch (UncheckedIOException ex) {
                IOException e = new IOException(ex.getMessage());
                e.initCause(ex);
                throw e;
            }
        } finally {
            this.prec = prevPrec;
        }
    }

    public void printExpr(JCTree tree) throws IOException {
        printExpr(tree, 0);
    }

    public void printStat(JCTree tree) throws IOException {
        printExpr(tree, -1);
    }

    /* JADX WARN: Multi-variable type inference failed */
    public <T extends JCTree> void printExprs(List<T> trees, String sep) throws IOException {
        if (trees.nonEmpty()) {
            printExpr(trees.head);
            for (List list = trees.tail; list.nonEmpty(); list = list.tail) {
                print(sep);
                printExpr((JCTree) list.head);
            }
        }
    }

    public <T extends JCTree> void printExprs(List<T> trees) throws IOException {
        printExprs(trees, ", ");
    }

    /* JADX WARN: Multi-variable type inference failed */
    public void printStats(List<? extends JCTree> trees) throws IOException {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            align();
            printStat((JCTree) list.head);
            println();
        }
    }

    public void printFlags(long flags) throws IOException {
        if ((4096 & flags) != 0) {
            print("/*synthetic*/ ");
        }
        print(TreeInfo.flagNames(flags));
        if ((Flags.ExtendedStandardFlags & flags) != 0) {
            print(" ");
        }
        if ((8192 & flags) != 0) {
            print("@");
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public void printAnnotations(List<JCTree.JCAnnotation> trees) throws IOException {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            printStat((JCTree) list.head);
            println();
            align();
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public void printTypeAnnotations(List<JCTree.JCAnnotation> trees) throws IOException {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            printExpr((JCTree) list.head);
            print(" ");
        }
    }

    public void printDocComment(JCTree tree) throws IOException {
        String dc;
        if (this.docComments != null && (dc = this.docComments.getCommentText(tree)) != null) {
            print("/**");
            println();
            int pos = 0;
            int endpos = lineEndPos(dc, 0);
            while (pos < dc.length()) {
                align();
                print(" *");
                if (pos < dc.length() && dc.charAt(pos) > ' ') {
                    print(" ");
                }
                print(dc.substring(pos, endpos));
                println();
                pos = endpos + 1;
                endpos = lineEndPos(dc, pos);
            }
            align();
            print(" */");
            println();
            align();
        }
    }

    static int lineEndPos(String s, int start) {
        int pos = s.indexOf(10, start);
        return pos < 0 ? s.length() : pos;
    }

    public void printTypeParameters(List<JCTree.JCTypeParameter> trees) throws IOException {
        if (trees.nonEmpty()) {
            print("<");
            printExprs(trees);
            print(">");
        }
    }

    public void printBlock(List<? extends JCTree> stats) throws IOException {
        print("{");
        println();
        indent();
        printStats(stats);
        undent();
        align();
        print("}");
    }

    /* JADX WARN: Multi-variable type inference failed */
    public void printEnumBody(List<JCTree> stats) throws IOException {
        print("{");
        println();
        indent();
        boolean first = true;
        for (List list = stats; list.nonEmpty(); list = list.tail) {
            if (isEnumerator((JCTree) list.head)) {
                if (!first) {
                    print(DocLint.TAGS_SEPARATOR);
                    println();
                }
                align();
                printStat((JCTree) list.head);
                first = false;
            }
        }
        print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        println();
        for (List list2 = stats; list2.nonEmpty(); list2 = list2.tail) {
            if (!isEnumerator((JCTree) list2.head)) {
                align();
                printStat((JCTree) list2.head);
                println();
            }
        }
        undent();
        align();
        print("}");
    }

    boolean isEnumerator(JCTree t) {
        return t.hasTag(JCTree.Tag.VARDEF) && (((JCTree.JCVariableDecl) t).mods.flags & 16384) != 0;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public void printUnit(JCTree.JCCompilationUnit tree, JCTree.JCClassDecl cdef) throws IOException {
        this.docComments = tree.docComments;
        printDocComment(tree);
        if (tree.pid != null) {
            print("package ");
            printExpr(tree.pid);
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
            println();
        }
        boolean firstImport = true;
        for (List list = tree.defs; list.nonEmpty() && (cdef == null || ((JCTree) list.head).hasTag(JCTree.Tag.IMPORT)); list = list.tail) {
            if (((JCTree) list.head).hasTag(JCTree.Tag.IMPORT)) {
                JCTree.JCImport imp = (JCTree.JCImport) list.head;
                Name name = TreeInfo.name(imp.qualid);
                if (name == name.table.names.asterisk || cdef == null || isUsed(TreeInfo.symbol(imp.qualid), cdef)) {
                    if (firstImport) {
                        firstImport = false;
                        println();
                    }
                    printStat(imp);
                }
            } else {
                printStat((JCTree) list.head);
            }
        }
        if (cdef != null) {
            printStat(cdef);
            println();
        }
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.tree.Pretty$1UsedVisitor, reason: invalid class name */
    class C1UsedVisitor extends TreeScanner {
        boolean result = false;
        final /* synthetic */ Symbol val$t;

        C1UsedVisitor(Symbol symbol) {
            this.val$t = symbol;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree tree) {
            if (tree == null || this.result) {
                return;
            }
            tree.accept(this);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            if (tree.sym == this.val$t) {
                this.result = true;
            }
        }
    }

    boolean isUsed(Symbol t, JCTree cdef) {
        C1UsedVisitor v = new C1UsedVisitor(t);
        v.scan(cdef);
        return v.result;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTopLevel(JCTree.JCCompilationUnit tree) {
        try {
            printUnit(tree, null);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitImport(JCTree.JCImport tree) {
        try {
            print("import ");
            if (tree.staticImport) {
                print("static ");
            }
            printExpr(tree.qualid);
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
            println();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitClassDef(JCTree.JCClassDecl tree) {
        try {
            println();
            align();
            printDocComment(tree);
            printAnnotations(tree.mods.annotations);
            printFlags(tree.mods.flags & (-513));
            Name enclClassNamePrev = this.enclClassName;
            this.enclClassName = tree.name;
            if ((tree.mods.flags & 512) != 0) {
                print("interface " + ((Object) tree.name));
                printTypeParameters(tree.typarams);
                if (tree.implementing.nonEmpty()) {
                    print(" extends ");
                    printExprs(tree.implementing);
                }
            } else {
                if ((tree.mods.flags & 16384) != 0) {
                    print("enum " + ((Object) tree.name));
                } else {
                    print("class " + ((Object) tree.name));
                }
                printTypeParameters(tree.typarams);
                if (tree.extending != null) {
                    print(" extends ");
                    printExpr(tree.extending);
                }
                if (tree.implementing.nonEmpty()) {
                    print(" implements ");
                    printExprs(tree.implementing);
                }
            }
            print(" ");
            if ((tree.mods.flags & 16384) != 0) {
                printEnumBody(tree.defs);
            } else {
                printBlock(tree.defs);
            }
            this.enclClassName = enclClassNamePrev;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitMethodDef(JCTree.JCMethodDecl tree) {
        try {
            if (tree.name == tree.name.table.names.init && this.enclClassName == null && this.sourceOutput) {
                return;
            }
            println();
            align();
            printDocComment(tree);
            printExpr(tree.mods);
            printTypeParameters(tree.typarams);
            if (tree.name == tree.name.table.names.init) {
                print(this.enclClassName != null ? this.enclClassName : tree.name);
            } else {
                printExpr(tree.restype);
                print(" " + ((Object) tree.name));
            }
            print("(");
            if (tree.recvparam != null) {
                printExpr(tree.recvparam);
                if (tree.params.size() > 0) {
                    print(", ");
                }
            }
            printExprs(tree.params);
            print(")");
            if (tree.thrown.nonEmpty()) {
                print(" throws ");
                printExprs(tree.thrown);
            }
            if (tree.defaultValue != null) {
                print(" default ");
                printExpr(tree.defaultValue);
            }
            if (tree.body != null) {
                print(" ");
                printStat(tree.body);
            } else {
                print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitVarDef(JCTree.JCVariableDecl tree) {
        try {
            if (this.docComments != null && this.docComments.hasComment(tree)) {
                println();
                align();
            }
            printDocComment(tree);
            if ((tree.mods.flags & 16384) != 0) {
                print("/*public static final*/ ");
                print(tree.name);
                if (tree.init != null) {
                    if (this.sourceOutput && tree.init.hasTag(JCTree.Tag.NEWCLASS)) {
                        print(" /*enum*/ ");
                        JCTree.JCNewClass init = (JCTree.JCNewClass) tree.init;
                        if (init.args != null && init.args.nonEmpty()) {
                            print("(");
                            print(init.args);
                            print(")");
                        }
                        if (init.def != null && init.def.defs != null) {
                            print(" ");
                            printBlock(init.def.defs);
                            return;
                        }
                        return;
                    }
                    print(" /* = ");
                    printExpr(tree.init);
                    print(" */");
                    return;
                }
                return;
            }
            printExpr(tree.mods);
            if ((tree.mods.flags & Flags.VARARGS) != 0) {
                JCTree vartype = tree.vartype;
                List<JCTree.JCAnnotation> tas = null;
                if (vartype instanceof JCTree.JCAnnotatedType) {
                    tas = ((JCTree.JCAnnotatedType) vartype).annotations;
                    vartype = ((JCTree.JCAnnotatedType) vartype).underlyingType;
                }
                printExpr(((JCTree.JCArrayTypeTree) vartype).elemtype);
                if (tas != null) {
                    print(' ');
                    printTypeAnnotations(tas);
                }
                print("... " + ((Object) tree.name));
            } else {
                printExpr(tree.vartype);
                print(" " + ((Object) tree.name));
            }
            if (tree.init != null) {
                print(" = ");
                printExpr(tree.init);
            }
            if (this.prec == -1) {
                print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSkip(JCTree.JCSkip tree) {
        try {
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBlock(JCTree.JCBlock tree) {
        try {
            printFlags(tree.flags);
            printBlock(tree.stats);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
        try {
            print("do ");
            printStat(tree.body);
            align();
            print(" while ");
            if (tree.cond.hasTag(JCTree.Tag.PARENS)) {
                printExpr(tree.cond);
            } else {
                print("(");
                printExpr(tree.cond);
                print(")");
            }
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWhileLoop(JCTree.JCWhileLoop tree) {
        try {
            print("while ");
            if (tree.cond.hasTag(JCTree.Tag.PARENS)) {
                printExpr(tree.cond);
            } else {
                print("(");
                printExpr(tree.cond);
                print(")");
            }
            print(" ");
            printStat(tree.body);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForLoop(JCTree.JCForLoop tree) {
        try {
            print("for (");
            if (tree.init.nonEmpty()) {
                if (tree.init.head.hasTag(JCTree.Tag.VARDEF)) {
                    printExpr(tree.init.head);
                    for (List list = tree.init.tail; list.nonEmpty(); list = list.tail) {
                        JCTree.JCVariableDecl vdef = (JCTree.JCVariableDecl) list.head;
                        print(", " + ((Object) vdef.name) + " = ");
                        printExpr(vdef.init);
                    }
                } else {
                    printExprs(tree.init);
                }
            }
            print("; ");
            if (tree.cond != null) {
                printExpr(tree.cond);
            }
            print("; ");
            printExprs(tree.step);
            print(") ");
            printStat(tree.body);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
        try {
            print("for (");
            printExpr(tree.var);
            print(" : ");
            printExpr(tree.expr);
            print(") ");
            printStat(tree.body);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLabelled(JCTree.JCLabeledStatement tree) {
        try {
            print(((Object) tree.label) + ": ");
            printStat(tree.body);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSwitch(JCTree.JCSwitch tree) {
        try {
            print("switch ");
            if (tree.selector.hasTag(JCTree.Tag.PARENS)) {
                printExpr(tree.selector);
            } else {
                print("(");
                printExpr(tree.selector);
                print(")");
            }
            print(" {");
            println();
            printStats(tree.cases);
            align();
            print("}");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitCase(JCTree.JCCase tree) {
        try {
            if (tree.pat == null) {
                print("default");
            } else {
                print("case ");
                printExpr(tree.pat);
            }
            print(": ");
            println();
            indent();
            printStats(tree.stats);
            undent();
            align();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSynchronized(JCTree.JCSynchronized tree) {
        try {
            print("synchronized ");
            if (tree.lock.hasTag(JCTree.Tag.PARENS)) {
                printExpr(tree.lock);
            } else {
                print("(");
                printExpr(tree.lock);
                print(")");
            }
            print(" ");
            printStat(tree.body);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTry(JCTree.JCTry tree) {
        try {
            print("try ");
            if (tree.resources.nonEmpty()) {
                print("(");
                boolean first = true;
                for (JCTree var : tree.resources) {
                    if (!first) {
                        println();
                        indent();
                    }
                    printStat(var);
                    first = false;
                }
                print(") ");
            }
            printStat(tree.body);
            for (List list = tree.catchers; list.nonEmpty(); list = list.tail) {
                printStat((JCTree) list.head);
            }
            if (tree.finalizer != null) {
                print(" finally ");
                printStat(tree.finalizer);
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitCatch(JCTree.JCCatch tree) {
        try {
            print(" catch (");
            printExpr(tree.param);
            print(") ");
            printStat(tree.body);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitConditional(JCTree.JCConditional tree) {
        try {
            open(this.prec, 3);
            printExpr(tree.cond, 4);
            print(" ? ");
            printExpr(tree.truepart);
            print(" : ");
            printExpr(tree.falsepart, 3);
            close(this.prec, 3);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIf(JCTree.JCIf tree) {
        try {
            print("if ");
            if (tree.cond.hasTag(JCTree.Tag.PARENS)) {
                printExpr(tree.cond);
            } else {
                print("(");
                printExpr(tree.cond);
                print(")");
            }
            print(" ");
            printStat(tree.thenpart);
            if (tree.elsepart != null) {
                print(" else ");
                printStat(tree.elsepart);
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitExec(JCTree.JCExpressionStatement tree) {
        try {
            printExpr(tree.expr);
            if (this.prec == -1) {
                print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBreak(JCTree.JCBreak tree) {
        try {
            print("break");
            if (tree.label != null) {
                print(" " + ((Object) tree.label));
            }
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitContinue(JCTree.JCContinue tree) {
        try {
            print("continue");
            if (tree.label != null) {
                print(" " + ((Object) tree.label));
            }
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReturn(JCTree.JCReturn tree) {
        try {
            print("return");
            if (tree.expr != null) {
                print(" ");
                printExpr(tree.expr);
            }
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitThrow(JCTree.JCThrow tree) {
        try {
            print("throw ");
            printExpr(tree.expr);
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssert(JCTree.JCAssert tree) {
        try {
            print("assert ");
            printExpr(tree.cond);
            if (tree.detail != null) {
                print(" : ");
                printExpr(tree.detail);
            }
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitApply(JCTree.JCMethodInvocation tree) {
        try {
            if (!tree.typeargs.isEmpty()) {
                if (tree.meth.hasTag(JCTree.Tag.SELECT)) {
                    JCTree.JCFieldAccess left = (JCTree.JCFieldAccess) tree.meth;
                    printExpr(left.selected);
                    print(".<");
                    printExprs(tree.typeargs);
                    print(">" + ((Object) left.name));
                } else {
                    print("<");
                    printExprs(tree.typeargs);
                    print(">");
                    printExpr(tree.meth);
                }
            } else {
                printExpr(tree.meth);
            }
            print("(");
            printExprs(tree.args);
            print(")");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewClass(JCTree.JCNewClass tree) {
        try {
            if (tree.encl != null) {
                printExpr(tree.encl);
                print(".");
            }
            print("new ");
            if (!tree.typeargs.isEmpty()) {
                print("<");
                printExprs(tree.typeargs);
                print(">");
            }
            if (tree.def != null && tree.def.mods.annotations.nonEmpty()) {
                printTypeAnnotations(tree.def.mods.annotations);
            }
            printExpr(tree.clazz);
            print("(");
            printExprs(tree.args);
            print(")");
            if (tree.def != null) {
                Name enclClassNamePrev = this.enclClassName;
                this.enclClassName = tree.def.name != null ? tree.def.name : (tree.type == null || tree.type.tsym.name == tree.type.tsym.name.table.names.empty) ? null : tree.type.tsym.name;
                if ((tree.def.mods.flags & 16384) != 0) {
                    print("/*enum*/");
                }
                printBlock(tree.def.defs);
                this.enclClassName = enclClassNamePrev;
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewArray(JCTree.JCNewArray tree) {
        try {
            if (tree.elemtype != null) {
                print("new ");
                JCTree elem = tree.elemtype;
                printBaseElementType(elem);
                if (!tree.annotations.isEmpty()) {
                    print(' ');
                    printTypeAnnotations(tree.annotations);
                }
                if (tree.elems != null) {
                    print("[]");
                }
                int i = 0;
                List<List<JCTree.JCAnnotation>> da = tree.dimAnnotations;
                for (List list = tree.dims; list.nonEmpty(); list = list.tail) {
                    if (da.size() > i && !da.get(i).isEmpty()) {
                        print(' ');
                        printTypeAnnotations(da.get(i));
                    }
                    print("[");
                    i++;
                    printExpr((JCTree) list.head);
                    print("]");
                }
                printBrackets(elem);
            }
            if (tree.elems != null) {
                print("{");
                printExprs(tree.elems);
                print("}");
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLambda(JCTree.JCLambda tree) {
        try {
            print("(");
            if (tree.paramKind == JCTree.JCLambda.ParameterKind.EXPLICIT) {
                printExprs(tree.params);
            } else {
                String sep = "";
                for (JCTree.JCVariableDecl param : tree.params) {
                    print(sep);
                    print(param.name);
                    sep = DocLint.TAGS_SEPARATOR;
                }
            }
            print(")->");
            printExpr(tree.body);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitParens(JCTree.JCParens tree) {
        try {
            print("(");
            printExpr(tree.expr);
            print(")");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssign(JCTree.JCAssign tree) {
        try {
            open(this.prec, 1);
            printExpr(tree.lhs, 2);
            print(" = ");
            printExpr(tree.rhs, 1);
            close(this.prec, 1);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    public String operatorName(JCTree.Tag tag) {
        switch (tag) {
            case POS:
                return Marker.ANY_NON_NULL_MARKER;
            case NEG:
                return "-";
            case NOT:
                return "!";
            case COMPL:
                return "~";
            case PREINC:
                return "++";
            case PREDEC:
                return "--";
            case POSTINC:
                return "++";
            case POSTDEC:
                return "--";
            case NULLCHK:
                return "<*nullchk*>";
            case OR:
                return "||";
            case AND:
                return "&&";
            case EQ:
                return "==";
            case NE:
                return "!=";
            case LT:
                return "<";
            case GT:
                return ">";
            case LE:
                return "<=";
            case GE:
                return ">=";
            case BITOR:
                return "|";
            case BITXOR:
                return "^";
            case BITAND:
                return "&";
            case SL:
                return "<<";
            case SR:
                return ">>";
            case USR:
                return ">>>";
            case PLUS:
                return Marker.ANY_NON_NULL_MARKER;
            case MINUS:
                return "-";
            case MUL:
                return Marker.ANY_MARKER;
            case DIV:
                return OnBotJavaFileSystemUtils.PATH_SEPARATOR;
            case MOD:
                return "%";
            default:
                throw new Error();
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssignop(JCTree.JCAssignOp tree) {
        try {
            open(this.prec, 2);
            printExpr(tree.lhs, 3);
            print(" " + operatorName(tree.getTag().noAssignOp()) + "= ");
            printExpr(tree.rhs, 2);
            close(this.prec, 2);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitUnary(JCTree.JCUnary tree) {
        try {
            int ownprec = TreeInfo.opPrec(tree.getTag());
            String opname = operatorName(tree.getTag());
            open(this.prec, ownprec);
            if (!tree.getTag().isPostUnaryOp()) {
                print(opname);
                printExpr(tree.arg, ownprec);
            } else {
                printExpr(tree.arg, ownprec);
                print(opname);
            }
            close(this.prec, ownprec);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBinary(JCTree.JCBinary tree) {
        try {
            int ownprec = TreeInfo.opPrec(tree.getTag());
            String opname = operatorName(tree.getTag());
            open(this.prec, ownprec);
            printExpr(tree.lhs, ownprec);
            print(" " + opname + " ");
            printExpr(tree.rhs, ownprec + 1);
            close(this.prec, ownprec);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeCast(JCTree.JCTypeCast tree) {
        try {
            open(this.prec, 14);
            print("(");
            printExpr(tree.clazz);
            print(")");
            printExpr(tree.expr, 14);
            close(this.prec, 14);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeTest(JCTree.JCInstanceOf tree) {
        try {
            open(this.prec, 10);
            printExpr(tree.expr, 10);
            print(" instanceof ");
            printExpr(tree.clazz, 11);
            close(this.prec, 10);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIndexed(JCTree.JCArrayAccess tree) {
        try {
            printExpr(tree.indexed, 15);
            print("[");
            printExpr(tree.index);
            print("]");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSelect(JCTree.JCFieldAccess tree) {
        try {
            printExpr(tree.selected, 15);
            print("." + ((Object) tree.name));
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReference(JCTree.JCMemberReference tree) {
        try {
            printExpr(tree.expr);
            print("::");
            if (tree.typeargs != null) {
                print("<");
                printExprs(tree.typeargs);
                print(">");
            }
            print(tree.getMode() == MemberReferenceTree.ReferenceMode.INVOKE ? tree.name : RequestConditions.REQUEST_KEY_NEW);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIdent(JCTree.JCIdent tree) {
        try {
            print(tree.name);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLiteral(JCTree.JCLiteral tree) {
        try {
            switch (tree.typetag) {
                case INT:
                    print(tree.value.toString());
                    return;
                case LONG:
                    print(tree.value + "L");
                    return;
                case FLOAT:
                    print(tree.value + "F");
                    return;
                case DOUBLE:
                    print(tree.value.toString());
                    return;
                case CHAR:
                    print("'" + Convert.quote(String.valueOf((char) ((Number) tree.value).intValue())) + "'");
                    return;
                case BOOLEAN:
                    print(((Number) tree.value).intValue() == 1 ? "true" : "false");
                    return;
                case BOT:
                    print("null");
                    return;
                default:
                    print("\"" + Convert.quote(tree.value.toString()) + "\"");
                    return;
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeIdent(JCTree.JCPrimitiveTypeTree tree) {
        try {
            switch (tree.typetag) {
                case INT:
                    print("int");
                    return;
                case LONG:
                    print("long");
                    return;
                case FLOAT:
                    print("float");
                    return;
                case DOUBLE:
                    print("double");
                    return;
                case CHAR:
                    print("char");
                    return;
                case BOOLEAN:
                    print("boolean");
                    return;
                case BOT:
                default:
                    print("error");
                    return;
                case BYTE:
                    print("byte");
                    return;
                case SHORT:
                    print("short");
                    return;
                case VOID:
                    print("void");
                    return;
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeArray(JCTree.JCArrayTypeTree tree) {
        try {
            printBaseElementType(tree);
            printBrackets(tree);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private void printBaseElementType(JCTree tree) throws IOException {
        printExpr(TreeInfo.innermostType(tree));
    }

    private void printBrackets(JCTree tree) throws IOException {
        JCTree elem = tree;
        while (true) {
            if (elem.hasTag(JCTree.Tag.ANNOTATED_TYPE)) {
                JCTree.JCAnnotatedType atype = (JCTree.JCAnnotatedType) elem;
                elem = atype.underlyingType;
                if (elem.hasTag(JCTree.Tag.TYPEARRAY)) {
                    print(' ');
                    printTypeAnnotations(atype.annotations);
                }
            }
            if (elem.hasTag(JCTree.Tag.TYPEARRAY)) {
                print("[]");
                elem = ((JCTree.JCArrayTypeTree) elem).elemtype;
            } else {
                return;
            }
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeApply(JCTree.JCTypeApply tree) {
        try {
            printExpr(tree.clazz);
            print("<");
            printExprs(tree.arguments);
            print(">");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeUnion(JCTree.JCTypeUnion tree) {
        try {
            printExprs(tree.alternatives, " | ");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeIntersection(JCTree.JCTypeIntersection tree) {
        try {
            printExprs(tree.bounds, " & ");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeParameter(JCTree.JCTypeParameter tree) {
        try {
            if (tree.annotations.nonEmpty()) {
                printTypeAnnotations(tree.annotations);
            }
            print(tree.name);
            if (tree.bounds.nonEmpty()) {
                print(" extends ");
                printExprs(tree.bounds, " & ");
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWildcard(JCTree.JCWildcard tree) {
        try {
            print(tree.kind);
            if (tree.kind.kind != BoundKind.UNBOUND) {
                printExpr(tree.inner);
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeBoundKind(JCTree.TypeBoundKind tree) {
        try {
            print(String.valueOf(tree.kind));
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitErroneous(JCTree.JCErroneous tree) {
        try {
            print("(ERROR)");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLetExpr(JCTree.LetExpr tree) {
        try {
            print("(let " + tree.defs + " in " + tree.expr + ")");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitModifiers(JCTree.JCModifiers mods) {
        try {
            printAnnotations(mods.annotations);
            printFlags(mods.flags);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotation(JCTree.JCAnnotation tree) {
        try {
            print("@");
            printExpr(tree.annotationType);
            print("(");
            printExprs(tree.args);
            print(")");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
        try {
            if (tree.underlyingType.hasTag(JCTree.Tag.SELECT)) {
                JCTree.JCFieldAccess access = (JCTree.JCFieldAccess) tree.underlyingType;
                printExpr(access.selected, 15);
                print(".");
                printTypeAnnotations(tree.annotations);
                print(access.name);
                return;
            }
            if (tree.underlyingType.hasTag(JCTree.Tag.TYPEARRAY)) {
                printBaseElementType(tree);
                printBrackets(tree);
            } else {
                printTypeAnnotations(tree.annotations);
                printExpr(tree.underlyingType);
            }
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTree(JCTree tree) {
        try {
            print("(UNKNOWN: " + tree + ")");
            println();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
