package com.sun.tools.javac.tree;

import com.sun.source.tree.Tree;
import com.sun.source.util.TreePath;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.tree.DCTree;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import dk.sgjesse.r8api.DescriptorUtils;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes.dex */
public class TreeInfo {
    public static final int addPrec = 12;
    public static final int andPrec = 5;
    public static final int assignPrec = 1;
    public static final int assignopPrec = 2;
    public static final int bitandPrec = 8;
    public static final int bitorPrec = 6;
    public static final int bitxorPrec = 7;
    public static final int condPrec = 3;
    public static final int eqPrec = 9;
    public static final int mulPrec = 13;
    public static final int noPrec = 0;
    public static final int notExpression = -1;
    public static final int orPrec = 4;
    public static final int ordPrec = 10;
    public static final int postfixPrec = 15;
    public static final int precCount = 16;
    public static final int prefixPrec = 14;
    public static final int shiftPrec = 11;
    protected static final Context.Key<TreeInfo> treeInfoKey = new Context.Key<>();
    private Name[] opname = new Name[JCTree.Tag.getNumberOfOperators()];

    public static TreeInfo instance(Context context) {
        TreeInfo instance = (TreeInfo) context.get(treeInfoKey);
        if (instance == null) {
            return new TreeInfo(context);
        }
        return instance;
    }

    private void setOpname(JCTree.Tag tag, String name, Names names) {
        setOpname(tag, names.fromString(name));
    }

    private void setOpname(JCTree.Tag tag, Name name) {
        this.opname[tag.operatorIndex()] = name;
    }

    private TreeInfo(Context context) {
        context.put(treeInfoKey, this);
        Names names = Names.instance(context);
        setOpname(JCTree.Tag.POS, "+++", names);
        setOpname(JCTree.Tag.NEG, "---", names);
        setOpname(JCTree.Tag.NOT, "!", names);
        setOpname(JCTree.Tag.COMPL, "~", names);
        setOpname(JCTree.Tag.PREINC, "++", names);
        setOpname(JCTree.Tag.PREDEC, "--", names);
        setOpname(JCTree.Tag.POSTINC, "++", names);
        setOpname(JCTree.Tag.POSTDEC, "--", names);
        setOpname(JCTree.Tag.NULLCHK, "<*nullchk*>", names);
        setOpname(JCTree.Tag.OR, "||", names);
        setOpname(JCTree.Tag.AND, "&&", names);
        setOpname(JCTree.Tag.EQ, "==", names);
        setOpname(JCTree.Tag.NE, "!=", names);
        setOpname(JCTree.Tag.LT, "<", names);
        setOpname(JCTree.Tag.GT, ">", names);
        setOpname(JCTree.Tag.LE, "<=", names);
        setOpname(JCTree.Tag.GE, ">=", names);
        setOpname(JCTree.Tag.BITOR, "|", names);
        setOpname(JCTree.Tag.BITXOR, "^", names);
        setOpname(JCTree.Tag.BITAND, "&", names);
        setOpname(JCTree.Tag.SL, "<<", names);
        setOpname(JCTree.Tag.SR, ">>", names);
        setOpname(JCTree.Tag.USR, ">>>", names);
        setOpname(JCTree.Tag.PLUS, Marker.ANY_NON_NULL_MARKER, names);
        setOpname(JCTree.Tag.MINUS, names.hyphen);
        setOpname(JCTree.Tag.MUL, names.asterisk);
        setOpname(JCTree.Tag.DIV, names.slash);
        setOpname(JCTree.Tag.MOD, "%", names);
    }

    public static List<JCTree.JCExpression> args(JCTree t) {
        switch (t.getTag()) {
            case APPLY:
                return ((JCTree.JCMethodInvocation) t).args;
            case NEWCLASS:
                return ((JCTree.JCNewClass) t).args;
            default:
                return null;
        }
    }

    public Name operatorName(JCTree.Tag tag) {
        return this.opname[tag.operatorIndex()];
    }

    public static boolean isConstructor(JCTree tree) {
        if (!tree.hasTag(JCTree.Tag.METHODDEF)) {
            return false;
        }
        Name name = ((JCTree.JCMethodDecl) tree).name;
        return name == name.table.names.init;
    }

    public static boolean isReceiverParam(JCTree tree) {
        return tree.hasTag(JCTree.Tag.VARDEF) && ((JCTree.JCVariableDecl) tree).nameexpr != null;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static boolean hasConstructors(List<JCTree> trees) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            if (isConstructor((JCTree) list.head)) {
                return true;
            }
        }
        return false;
    }

    public static boolean isMultiCatch(JCTree.JCCatch catchClause) {
        return catchClause.param.vartype.hasTag(JCTree.Tag.TYPEUNION);
    }

    public static boolean isSyntheticInit(JCTree stat) {
        Name selected;
        if (stat.hasTag(JCTree.Tag.EXEC)) {
            JCTree.JCExpressionStatement exec = (JCTree.JCExpressionStatement) stat;
            if (exec.expr.hasTag(JCTree.Tag.ASSIGN)) {
                JCTree.JCAssign assign = (JCTree.JCAssign) exec.expr;
                if (assign.lhs.hasTag(JCTree.Tag.SELECT)) {
                    JCTree.JCFieldAccess select = (JCTree.JCFieldAccess) assign.lhs;
                    if (select.sym != null && (select.sym.flags() & 4096) != 0 && (selected = name(select.selected)) != null && selected == selected.table.names._this) {
                        return true;
                    }
                    return false;
                }
                return false;
            }
            return false;
        }
        return false;
    }

    public static Name calledMethodName(JCTree tree) {
        if (tree.hasTag(JCTree.Tag.EXEC)) {
            JCTree.JCExpressionStatement exec = (JCTree.JCExpressionStatement) tree;
            if (exec.expr.hasTag(JCTree.Tag.APPLY)) {
                Name mname = name(((JCTree.JCMethodInvocation) exec.expr).meth);
                return mname;
            }
            return null;
        }
        return null;
    }

    public static boolean isSelfCall(JCTree tree) {
        Name name = calledMethodName(tree);
        if (name == null) {
            return false;
        }
        Names names = name.table.names;
        return name == names._this || name == names._super;
    }

    public static boolean isSuperCall(JCTree tree) {
        Name name = calledMethodName(tree);
        if (name == null) {
            return false;
        }
        Names names = name.table.names;
        return name == names._super;
    }

    public static boolean isInitialConstructor(JCTree tree) {
        JCTree.JCMethodInvocation app = firstConstructorCall(tree);
        if (app == null) {
            return false;
        }
        Name meth = name(app.meth);
        return meth == null || meth != meth.table.names._this;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static JCTree.JCMethodInvocation firstConstructorCall(JCTree tree) {
        if (!tree.hasTag(JCTree.Tag.METHODDEF)) {
            return null;
        }
        JCTree.JCMethodDecl md = (JCTree.JCMethodDecl) tree;
        Names names = md.name.table.names;
        if (md.name != names.init || md.body == null) {
            return null;
        }
        List list = md.body.stats;
        while (list.nonEmpty() && isSyntheticInit((JCTree) list.head)) {
            list = list.tail;
        }
        if (list.isEmpty() || !((JCTree.JCStatement) list.head).hasTag(JCTree.Tag.EXEC)) {
            return null;
        }
        JCTree.JCExpressionStatement exec = (JCTree.JCExpressionStatement) list.head;
        if (exec.expr.hasTag(JCTree.Tag.APPLY)) {
            return (JCTree.JCMethodInvocation) exec.expr;
        }
        return null;
    }

    public static boolean isDiamond(JCTree tree) {
        switch (tree.getTag()) {
            case NEWCLASS:
                return isDiamond(((JCTree.JCNewClass) tree).clazz);
            case TYPEAPPLY:
                return ((JCTree.JCTypeApply) tree).getTypeArguments().isEmpty();
            case ANNOTATED_TYPE:
                return isDiamond(((JCTree.JCAnnotatedType) tree).underlyingType);
            default:
                return false;
        }
    }

    public static boolean isEnumInit(JCTree tree) {
        switch (tree.getTag()) {
            case VARDEF:
                if ((((JCTree.JCVariableDecl) tree).mods.flags & 16384) != 0) {
                }
                break;
        }
        return false;
    }

    public static void setPolyKind(JCTree tree, JCTree.JCPolyExpression.PolyKind pkind) {
        switch (tree.getTag()) {
            case APPLY:
                ((JCTree.JCMethodInvocation) tree).polyKind = pkind;
                return;
            case NEWCLASS:
                ((JCTree.JCNewClass) tree).polyKind = pkind;
                return;
            case REFERENCE:
                ((JCTree.JCMemberReference) tree).refPolyKind = pkind;
                return;
            default:
                throw new AssertionError("Unexpected tree: " + tree);
        }
    }

    public static void setVarargsElement(JCTree tree, Type varargsElement) {
        switch (tree.getTag()) {
            case APPLY:
                ((JCTree.JCMethodInvocation) tree).varargsElement = varargsElement;
                return;
            case NEWCLASS:
                ((JCTree.JCNewClass) tree).varargsElement = varargsElement;
                return;
            case REFERENCE:
                ((JCTree.JCMemberReference) tree).varargsElement = varargsElement;
                return;
            default:
                throw new AssertionError("Unexpected tree: " + tree);
        }
    }

    public static boolean isExpressionStatement(JCTree.JCExpression tree) {
        switch (tree.getTag()) {
            case APPLY:
            case NEWCLASS:
            case PREINC:
            case PREDEC:
            case POSTINC:
            case POSTDEC:
            case ASSIGN:
            case BITOR_ASG:
            case BITXOR_ASG:
            case BITAND_ASG:
            case SL_ASG:
            case SR_ASG:
            case USR_ASG:
            case PLUS_ASG:
            case MINUS_ASG:
            case MUL_ASG:
            case DIV_ASG:
            case MOD_ASG:
            case ERRONEOUS:
                return true;
            case TYPEAPPLY:
            case ANNOTATED_TYPE:
            case VARDEF:
            case REFERENCE:
            default:
                return false;
        }
    }

    public static boolean isStaticSelector(JCTree base, Names names) {
        if (base == null) {
            return false;
        }
        switch (base.getTag()) {
            case TYPEAPPLY:
            case TYPEARRAY:
                break;
            case ANNOTATED_TYPE:
                break;
            case IDENT:
                JCTree.JCIdent id = (JCTree.JCIdent) base;
                if (id.name != names._this && id.name != names._super && isStaticSym(base)) {
                    break;
                }
                break;
            case SELECT:
                if (isStaticSym(base) && isStaticSelector(((JCTree.JCFieldAccess) base).selected, names)) {
                    break;
                }
                break;
        }
        return false;
    }

    private static boolean isStaticSym(JCTree tree) {
        Symbol sym = symbol(tree);
        return sym.kind == 2 || sym.kind == 1;
    }

    public static boolean isNull(JCTree tree) {
        if (!tree.hasTag(JCTree.Tag.LITERAL)) {
            return false;
        }
        JCTree.JCLiteral lit = (JCTree.JCLiteral) tree;
        return lit.typetag == TypeTag.BOT;
    }

    public static boolean isInAnnotation(Env<?> env, JCTree tree) {
        TreePath tp = TreePath.getPath(env.toplevel, tree);
        if (tp != null) {
            for (Tree t : tp) {
                if (t.getKind() == Tree.Kind.ANNOTATION) {
                    return true;
                }
            }
            return false;
        }
        return false;
    }

    public static String getCommentText(Env<?> env, JCTree tree) {
        DocCommentTable docComments = (tree.hasTag(JCTree.Tag.TOPLEVEL) ? (JCTree.JCCompilationUnit) tree : env.toplevel).docComments;
        if (docComments == null) {
            return null;
        }
        return docComments.getCommentText(tree);
    }

    public static DCTree.DCDocComment getCommentTree(Env<?> env, JCTree tree) {
        DocCommentTable docComments = (tree.hasTag(JCTree.Tag.TOPLEVEL) ? (JCTree.JCCompilationUnit) tree : env.toplevel).docComments;
        if (docComments == null) {
            return null;
        }
        return docComments.getCommentTree(tree);
    }

    public static int firstStatPos(JCTree tree) {
        if (tree.hasTag(JCTree.Tag.BLOCK) && ((JCTree.JCBlock) tree).stats.nonEmpty()) {
            return ((JCTree.JCBlock) tree).stats.head.pos;
        }
        return tree.pos;
    }

    public static int endPos(JCTree tree) {
        JCTree.JCBlock jCBlock;
        if (tree.hasTag(JCTree.Tag.BLOCK) && ((JCTree.JCBlock) tree).endpos != -1) {
            return ((JCTree.JCBlock) tree).endpos;
        }
        if (tree.hasTag(JCTree.Tag.SYNCHRONIZED)) {
            return endPos(((JCTree.JCSynchronized) tree).body);
        }
        if (tree.hasTag(JCTree.Tag.TRY)) {
            JCTree.JCTry t = (JCTree.JCTry) tree;
            if (t.finalizer != null) {
                jCBlock = t.finalizer;
            } else {
                jCBlock = t.catchers.nonEmpty() ? t.catchers.last().body : t.body;
            }
            return endPos(jCBlock);
        }
        return tree.pos;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static int getStartPos(JCTree tree) {
        if (tree == null) {
            return -1;
        }
        switch (tree.getTag()) {
            case APPLY:
                return getStartPos(((JCTree.JCMethodInvocation) tree).meth);
            case NEWCLASS:
                JCTree.JCNewClass node = (JCTree.JCNewClass) tree;
                if (node.encl != null) {
                    return getStartPos(node.encl);
                }
                break;
            case TYPEAPPLY:
                return getStartPos(((JCTree.JCTypeApply) tree).clazz);
            case ANNOTATED_TYPE:
                JCTree.JCAnnotatedType node2 = (JCTree.JCAnnotatedType) tree;
                if (node2.annotations.nonEmpty()) {
                    if (node2.underlyingType.hasTag(JCTree.Tag.TYPEARRAY) || node2.underlyingType.hasTag(JCTree.Tag.SELECT)) {
                        return getStartPos(node2.underlyingType);
                    }
                    return getStartPos(node2.annotations.head);
                }
                return getStartPos(node2.underlyingType);
            case VARDEF:
                JCTree.JCVariableDecl node3 = (JCTree.JCVariableDecl) tree;
                if (node3.mods.pos != -1) {
                    return node3.mods.pos;
                }
                if (node3.vartype == null) {
                    return node3.pos;
                }
                return getStartPos(node3.vartype);
            case POSTINC:
            case POSTDEC:
                return getStartPos(((JCTree.JCUnary) tree).arg);
            case ASSIGN:
                return getStartPos(((JCTree.JCAssign) tree).lhs);
            case BITOR_ASG:
            case BITXOR_ASG:
            case BITAND_ASG:
            case SL_ASG:
            case SR_ASG:
            case USR_ASG:
            case PLUS_ASG:
            case MINUS_ASG:
            case MUL_ASG:
            case DIV_ASG:
            case MOD_ASG:
                return getStartPos(((JCTree.JCAssignOp) tree).lhs);
            case ERRONEOUS:
                JCTree.JCErroneous node4 = (JCTree.JCErroneous) tree;
                if (node4.errs != null && node4.errs.nonEmpty()) {
                    return getStartPos((JCTree) node4.errs.head);
                }
                break;
            case SELECT:
                return getStartPos(((JCTree.JCFieldAccess) tree).selected);
            case TYPEARRAY:
                return getStartPos(((JCTree.JCArrayTypeTree) tree).elemtype);
            case OR:
            case AND:
            case BITOR:
            case BITXOR:
            case BITAND:
            case EQ:
            case NE:
            case LT:
            case GT:
            case LE:
            case GE:
            case SL:
            case SR:
            case USR:
            case PLUS:
            case MINUS:
            case MUL:
            case DIV:
            case MOD:
                return getStartPos(((JCTree.JCBinary) tree).lhs);
            case CLASSDEF:
                JCTree.JCClassDecl node5 = (JCTree.JCClassDecl) tree;
                if (node5.mods.pos != -1) {
                    return node5.mods.pos;
                }
                break;
            case CONDEXPR:
                return getStartPos(((JCTree.JCConditional) tree).cond);
            case EXEC:
                return getStartPos(((JCTree.JCExpressionStatement) tree).expr);
            case INDEXED:
                return getStartPos(((JCTree.JCArrayAccess) tree).indexed);
            case METHODDEF:
                JCTree.JCMethodDecl node6 = (JCTree.JCMethodDecl) tree;
                if (node6.mods.pos != -1) {
                    return node6.mods.pos;
                }
                if (node6.typarams.nonEmpty()) {
                    return getStartPos(node6.typarams.head);
                }
                return node6.restype == null ? node6.pos : getStartPos(node6.restype);
            case TYPETEST:
                return getStartPos(((JCTree.JCInstanceOf) tree).expr);
        }
        return tree.pos;
    }

    /* JADX WARN: Can't fix incorrect switch cases order, some code will duplicate */
    public static int getEndPos(JCTree tree, EndPosTable endPosTable) {
        if (tree == null) {
            return -1;
        }
        if (endPosTable == null) {
            return endPos(tree);
        }
        int mapPos = endPosTable.getEndPos(tree);
        if (mapPos != -1) {
            return mapPos;
        }
        switch (tree.getTag()) {
            case ANNOTATED_TYPE:
                return getEndPos(((JCTree.JCAnnotatedType) tree).underlyingType, endPosTable);
            case VARDEF:
            case REFERENCE:
            case POSTINC:
            case POSTDEC:
            case ASSIGN:
            case IDENT:
            case SELECT:
            case TYPEARRAY:
            case CLASSDEF:
            case EXEC:
            case INDEXED:
            case METHODDEF:
            default:
                return -1;
            case PREINC:
            case PREDEC:
            case POS:
            case NEG:
            case NOT:
            case COMPL:
                return getEndPos(((JCTree.JCUnary) tree).arg, endPosTable);
            case BITOR_ASG:
            case BITXOR_ASG:
            case BITAND_ASG:
            case SL_ASG:
            case SR_ASG:
            case USR_ASG:
            case PLUS_ASG:
            case MINUS_ASG:
            case MUL_ASG:
            case DIV_ASG:
            case MOD_ASG:
                return getEndPos(((JCTree.JCAssignOp) tree).rhs, endPosTable);
            case ERRONEOUS:
                JCTree.JCErroneous node = (JCTree.JCErroneous) tree;
                if (node.errs != null && node.errs.nonEmpty()) {
                    return getEndPos(node.errs.last(), endPosTable);
                }
                return -1;
            case OR:
            case AND:
            case BITOR:
            case BITXOR:
            case BITAND:
            case EQ:
            case NE:
            case LT:
            case GT:
            case LE:
            case GE:
            case SL:
            case SR:
            case USR:
            case PLUS:
            case MINUS:
            case MUL:
            case DIV:
            case MOD:
                return getEndPos(((JCTree.JCBinary) tree).rhs, endPosTable);
            case CONDEXPR:
                return getEndPos(((JCTree.JCConditional) tree).falsepart, endPosTable);
            case TYPETEST:
                return getEndPos(((JCTree.JCInstanceOf) tree).clazz, endPosTable);
            case CASE:
                return getEndPos(((JCTree.JCCase) tree).stats.last(), endPosTable);
            case CATCH:
                return getEndPos(((JCTree.JCCatch) tree).body, endPosTable);
            case FORLOOP:
                return getEndPos(((JCTree.JCForLoop) tree).body, endPosTable);
            case FOREACHLOOP:
                return getEndPos(((JCTree.JCEnhancedForLoop) tree).body, endPosTable);
            case IF:
                JCTree.JCIf node2 = (JCTree.JCIf) tree;
                if (node2.elsepart == null) {
                    return getEndPos(node2.thenpart, endPosTable);
                }
                return getEndPos(node2.elsepart, endPosTable);
            case LABELLED:
                return getEndPos(((JCTree.JCLabeledStatement) tree).body, endPosTable);
            case MODIFIERS:
                return getEndPos(((JCTree.JCModifiers) tree).annotations.last(), endPosTable);
            case SYNCHRONIZED:
                return getEndPos(((JCTree.JCSynchronized) tree).body, endPosTable);
            case TOPLEVEL:
                return getEndPos(((JCTree.JCCompilationUnit) tree).defs.last(), endPosTable);
            case TRY:
                JCTree.JCTry node3 = (JCTree.JCTry) tree;
                if (node3.finalizer != null) {
                    return getEndPos(node3.finalizer, endPosTable);
                }
                if (!node3.catchers.isEmpty()) {
                    return getEndPos(node3.catchers.last(), endPosTable);
                }
                return getEndPos(node3.body, endPosTable);
            case WILDCARD:
                return getEndPos(((JCTree.JCWildcard) tree).inner, endPosTable);
            case TYPECAST:
                return getEndPos(((JCTree.JCTypeCast) tree).expr, endPosTable);
            case WHILELOOP:
                return getEndPos(((JCTree.JCWhileLoop) tree).body, endPosTable);
        }
    }

    public static JCDiagnostic.DiagnosticPosition diagEndPos(final JCTree tree) {
        final int endPos = endPos(tree);
        return new JCDiagnostic.DiagnosticPosition() { // from class: com.sun.tools.javac.tree.TreeInfo.1
            @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
            public JCTree getTree() {
                return tree;
            }

            @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
            public int getStartPosition() {
                return TreeInfo.getStartPos(tree);
            }

            @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
            public int getPreferredPosition() {
                return endPos;
            }

            @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
            public int getEndPosition(EndPosTable endPosTable) {
                return TreeInfo.getEndPos(tree, endPosTable);
            }
        };
    }

    public static int finalizerPos(JCTree tree) {
        if (tree.hasTag(JCTree.Tag.TRY)) {
            JCTree.JCTry t = (JCTree.JCTry) tree;
            Assert.checkNonNull(t.finalizer);
            return firstStatPos(t.finalizer);
        }
        if (tree.hasTag(JCTree.Tag.SYNCHRONIZED)) {
            return endPos(((JCTree.JCSynchronized) tree).body);
        }
        throw new AssertionError();
    }

    public static int positionFor(Symbol sym, JCTree tree) {
        JCTree decl = declarationFor(sym, tree);
        return (decl != null ? decl : tree).pos;
    }

    public static JCDiagnostic.DiagnosticPosition diagnosticPositionFor(Symbol sym, JCTree tree) {
        JCTree decl = declarationFor(sym, tree);
        return (decl != null ? decl : tree).pos();
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.tree.TreeInfo$1DeclScanner, reason: invalid class name */
    class C1DeclScanner extends TreeScanner {
        JCTree result = null;
        final /* synthetic */ Symbol val$sym;

        C1DeclScanner(Symbol symbol) {
            this.val$sym = symbol;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree tree) {
            if (tree != null && this.result == null) {
                tree.accept(this);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTopLevel(JCTree.JCCompilationUnit that) {
            if (that.packge != this.val$sym) {
                super.visitTopLevel(that);
            } else {
                this.result = that;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl that) {
            if (that.sym != this.val$sym) {
                super.visitClassDef(that);
            } else {
                this.result = that;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl that) {
            if (that.sym != this.val$sym) {
                super.visitMethodDef(that);
            } else {
                this.result = that;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl that) {
            if (that.sym != this.val$sym) {
                super.visitVarDef(that);
            } else {
                this.result = that;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeParameter(JCTree.JCTypeParameter that) {
            if (that.type == null || that.type.tsym != this.val$sym) {
                super.visitTypeParameter(that);
            } else {
                this.result = that;
            }
        }
    }

    public static JCTree declarationFor(Symbol sym, JCTree tree) {
        C1DeclScanner s = new C1DeclScanner(sym);
        tree.accept(s);
        return s.result;
    }

    public static Env<AttrContext> scopeFor(JCTree node, JCTree.JCCompilationUnit unit) {
        return scopeFor(pathFor(node, unit));
    }

    public static Env<AttrContext> scopeFor(List<JCTree> path) {
        throw new UnsupportedOperationException("not implemented yet");
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.tree.TreeInfo$1Result, reason: invalid class name */
    class C1Result extends Error {
        static final long serialVersionUID = -5942088234594905625L;
        List<JCTree> path;

        C1Result(List<JCTree> path) {
            this.path = path;
        }
    }

    public static List<JCTree> pathFor(final JCTree node, JCTree.JCCompilationUnit unit) {
        try {
            new TreeScanner() { // from class: com.sun.tools.javac.tree.TreeInfo.1PathFinder
                List<JCTree> path = List.nil();

                @Override // com.sun.tools.javac.tree.TreeScanner
                public void scan(JCTree tree) {
                    if (tree != null) {
                        this.path = this.path.prepend(tree);
                        if (tree == node) {
                            throw new C1Result(this.path);
                        }
                        super.scan(tree);
                        this.path = this.path.tail;
                    }
                }
            }.scan(unit);
            return List.nil();
        } catch (C1Result result) {
            return result.path;
        }
    }

    public static JCTree referencedStatement(JCTree.JCLabeledStatement tree) {
        JCTree t = tree;
        do {
            t = ((JCTree.JCLabeledStatement) t).body;
        } while (t.hasTag(JCTree.Tag.LABELLED));
        switch (t.getTag()) {
            case FORLOOP:
            case FOREACHLOOP:
            case WHILELOOP:
            case DOLOOP:
            case SWITCH:
                return t;
            default:
                return tree;
        }
    }

    public static JCTree.JCExpression skipParens(JCTree.JCExpression tree) {
        while (tree.hasTag(JCTree.Tag.PARENS)) {
            tree = ((JCTree.JCParens) tree).expr;
        }
        return tree;
    }

    public static JCTree skipParens(JCTree tree) {
        if (tree.hasTag(JCTree.Tag.PARENS)) {
            return skipParens((JCTree.JCExpression) tree);
        }
        return tree;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static List<Type> types(List<? extends JCTree> trees) {
        ListBuffer<Type> ts = new ListBuffer<>();
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            ts.append(((JCTree) list.head).type);
        }
        return ts.toList();
    }

    public static Name name(JCTree tree) {
        switch (tree.getTag()) {
            case TYPEAPPLY:
                return name(((JCTree.JCTypeApply) tree).clazz);
            case IDENT:
                return ((JCTree.JCIdent) tree).name;
            case SELECT:
                return ((JCTree.JCFieldAccess) tree).name;
            default:
                return null;
        }
    }

    public static Name fullName(JCTree tree) {
        JCTree tree2 = skipParens(tree);
        switch (tree2.getTag()) {
            case IDENT:
                return ((JCTree.JCIdent) tree2).name;
            case SELECT:
                Name sname = fullName(((JCTree.JCFieldAccess) tree2).selected);
                if (sname == null) {
                    return null;
                }
                return sname.append(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, name(tree2));
            default:
                return null;
        }
    }

    public static Symbol symbolFor(JCTree node) {
        Symbol sym = symbolForImpl(node);
        if (sym != null) {
            return sym.baseSymbol();
        }
        return null;
    }

    private static Symbol symbolForImpl(JCTree node) {
        JCTree node2 = skipParens(node);
        switch (node2.getTag()) {
            case APPLY:
                return symbolFor(((JCTree.JCMethodInvocation) node2).meth);
            case NEWCLASS:
                return ((JCTree.JCNewClass) node2).constructor;
            case TYPEAPPLY:
                return symbolFor(((JCTree.JCTypeApply) node2).clazz);
            case VARDEF:
                return ((JCTree.JCVariableDecl) node2).sym;
            case REFERENCE:
                return ((JCTree.JCMemberReference) node2).sym;
            case IDENT:
                return ((JCTree.JCIdent) node2).sym;
            case SELECT:
                return ((JCTree.JCFieldAccess) node2).sym;
            case CLASSDEF:
                return ((JCTree.JCClassDecl) node2).sym;
            case METHODDEF:
                return ((JCTree.JCMethodDecl) node2).sym;
            case TOPLEVEL:
                return ((JCTree.JCCompilationUnit) node2).packge;
            case ANNOTATION:
            case TYPE_ANNOTATION:
            case TYPEPARAMETER:
                if (node2.type != null) {
                    return node2.type.tsym;
                }
                return null;
            default:
                return null;
        }
    }

    public static boolean isDeclaration(JCTree node) {
        switch (skipParens(node).getTag()) {
            case VARDEF:
            case CLASSDEF:
            case METHODDEF:
                return true;
            default:
                return false;
        }
    }

    public static Symbol symbol(JCTree tree) {
        JCTree tree2 = skipParens(tree);
        switch (tree2.getTag()) {
            case TYPEAPPLY:
                return symbol(((JCTree.JCTypeApply) tree2).clazz);
            case ANNOTATED_TYPE:
                return symbol(((JCTree.JCAnnotatedType) tree2).underlyingType);
            case REFERENCE:
                return ((JCTree.JCMemberReference) tree2).sym;
            case IDENT:
                return ((JCTree.JCIdent) tree2).sym;
            case SELECT:
                return ((JCTree.JCFieldAccess) tree2).sym;
            default:
                return null;
        }
    }

    public static boolean nonstaticSelect(JCTree tree) {
        JCTree tree2 = skipParens(tree);
        if (!tree2.hasTag(JCTree.Tag.SELECT)) {
            return false;
        }
        JCTree.JCFieldAccess s = (JCTree.JCFieldAccess) tree2;
        Symbol e = symbol(s.selected);
        return e == null || !(e.kind == 1 || e.kind == 2);
    }

    public static void setSymbol(JCTree tree, Symbol sym) {
        JCTree tree2 = skipParens(tree);
        switch (tree2.getTag()) {
            case IDENT:
                ((JCTree.JCIdent) tree2).sym = sym;
                break;
            case SELECT:
                ((JCTree.JCFieldAccess) tree2).sym = sym;
                break;
        }
    }

    public static long flags(JCTree tree) {
        switch (tree.getTag()) {
            case VARDEF:
                return ((JCTree.JCVariableDecl) tree).mods.flags;
            case CLASSDEF:
                return ((JCTree.JCClassDecl) tree).mods.flags;
            case METHODDEF:
                return ((JCTree.JCMethodDecl) tree).mods.flags;
            case BLOCK:
                return ((JCTree.JCBlock) tree).flags;
            default:
                return 0L;
        }
    }

    public static long firstFlag(long flags) {
        long flag = 1;
        while ((flag & flags & Flags.ExtendedStandardFlags) == 0) {
            flag <<= 1;
        }
        return flag;
    }

    public static String flagNames(long flags) {
        return Flags.toString(Flags.ExtendedStandardFlags & flags).trim();
    }

    public static int opPrec(JCTree.Tag op) {
        switch (op) {
            case PREINC:
            case PREDEC:
            case POS:
            case NEG:
            case NOT:
            case COMPL:
                return 14;
            case POSTINC:
            case POSTDEC:
            case NULLCHK:
                return 15;
            case ASSIGN:
                return 1;
            case BITOR_ASG:
            case BITXOR_ASG:
            case BITAND_ASG:
            case SL_ASG:
            case SR_ASG:
            case USR_ASG:
            case PLUS_ASG:
            case MINUS_ASG:
            case MUL_ASG:
            case DIV_ASG:
            case MOD_ASG:
                return 2;
            case ERRONEOUS:
            case IDENT:
            case SELECT:
            case TYPEARRAY:
            case CLASSDEF:
            case CONDEXPR:
            case EXEC:
            case INDEXED:
            case METHODDEF:
            case CASE:
            case CATCH:
            case FORLOOP:
            case FOREACHLOOP:
            case IF:
            case LABELLED:
            case MODIFIERS:
            case SYNCHRONIZED:
            case TOPLEVEL:
            case TRY:
            case WILDCARD:
            case TYPECAST:
            case WHILELOOP:
            case DOLOOP:
            case SWITCH:
            case ANNOTATION:
            case TYPE_ANNOTATION:
            case TYPEPARAMETER:
            case BLOCK:
            default:
                throw new AssertionError();
            case OR:
                return 4;
            case AND:
                return 5;
            case BITOR:
                return 6;
            case BITXOR:
                return 7;
            case BITAND:
                return 8;
            case EQ:
            case NE:
                return 9;
            case LT:
            case GT:
            case LE:
            case GE:
                return 10;
            case SL:
            case SR:
            case USR:
                return 11;
            case PLUS:
            case MINUS:
                return 12;
            case MUL:
            case DIV:
            case MOD:
                return 13;
            case TYPETEST:
                return 10;
        }
    }

    static Tree.Kind tagToKind(JCTree.Tag tag) {
        switch (tag) {
            case PREINC:
                return Tree.Kind.PREFIX_INCREMENT;
            case PREDEC:
                return Tree.Kind.PREFIX_DECREMENT;
            case POSTINC:
                return Tree.Kind.POSTFIX_INCREMENT;
            case POSTDEC:
                return Tree.Kind.POSTFIX_DECREMENT;
            case ASSIGN:
            case ERRONEOUS:
            case IDENT:
            case SELECT:
            case TYPEARRAY:
            case CLASSDEF:
            case CONDEXPR:
            case EXEC:
            case INDEXED:
            case METHODDEF:
            case TYPETEST:
            case CASE:
            case CATCH:
            case FORLOOP:
            case FOREACHLOOP:
            case IF:
            case LABELLED:
            case MODIFIERS:
            case SYNCHRONIZED:
            case TOPLEVEL:
            case TRY:
            case WILDCARD:
            case TYPECAST:
            case WHILELOOP:
            case DOLOOP:
            case SWITCH:
            case TYPEPARAMETER:
            case BLOCK:
            default:
                return null;
            case BITOR_ASG:
                return Tree.Kind.OR_ASSIGNMENT;
            case BITXOR_ASG:
                return Tree.Kind.XOR_ASSIGNMENT;
            case BITAND_ASG:
                return Tree.Kind.AND_ASSIGNMENT;
            case SL_ASG:
                return Tree.Kind.LEFT_SHIFT_ASSIGNMENT;
            case SR_ASG:
                return Tree.Kind.RIGHT_SHIFT_ASSIGNMENT;
            case USR_ASG:
                return Tree.Kind.UNSIGNED_RIGHT_SHIFT_ASSIGNMENT;
            case PLUS_ASG:
                return Tree.Kind.PLUS_ASSIGNMENT;
            case MINUS_ASG:
                return Tree.Kind.MINUS_ASSIGNMENT;
            case MUL_ASG:
                return Tree.Kind.MULTIPLY_ASSIGNMENT;
            case DIV_ASG:
                return Tree.Kind.DIVIDE_ASSIGNMENT;
            case MOD_ASG:
                return Tree.Kind.REMAINDER_ASSIGNMENT;
            case OR:
                return Tree.Kind.CONDITIONAL_OR;
            case AND:
                return Tree.Kind.CONDITIONAL_AND;
            case BITOR:
                return Tree.Kind.OR;
            case BITXOR:
                return Tree.Kind.XOR;
            case BITAND:
                return Tree.Kind.AND;
            case EQ:
                return Tree.Kind.EQUAL_TO;
            case NE:
                return Tree.Kind.NOT_EQUAL_TO;
            case LT:
                return Tree.Kind.LESS_THAN;
            case GT:
                return Tree.Kind.GREATER_THAN;
            case LE:
                return Tree.Kind.LESS_THAN_EQUAL;
            case GE:
                return Tree.Kind.GREATER_THAN_EQUAL;
            case SL:
                return Tree.Kind.LEFT_SHIFT;
            case SR:
                return Tree.Kind.RIGHT_SHIFT;
            case USR:
                return Tree.Kind.UNSIGNED_RIGHT_SHIFT;
            case PLUS:
                return Tree.Kind.PLUS;
            case MINUS:
                return Tree.Kind.MINUS;
            case MUL:
                return Tree.Kind.MULTIPLY;
            case DIV:
                return Tree.Kind.DIVIDE;
            case MOD:
                return Tree.Kind.REMAINDER;
            case POS:
                return Tree.Kind.UNARY_PLUS;
            case NEG:
                return Tree.Kind.UNARY_MINUS;
            case NOT:
                return Tree.Kind.LOGICAL_COMPLEMENT;
            case COMPL:
                return Tree.Kind.BITWISE_COMPLEMENT;
            case ANNOTATION:
                return Tree.Kind.ANNOTATION;
            case TYPE_ANNOTATION:
                return Tree.Kind.TYPE_ANNOTATION;
            case NULLCHK:
                return Tree.Kind.OTHER;
        }
    }

    public static JCTree.JCExpression typeIn(JCTree.JCExpression tree) {
        switch (tree.getTag()) {
            case TYPEAPPLY:
            case ERRONEOUS:
            case IDENT:
            case SELECT:
            case TYPEARRAY:
            case WILDCARD:
            case TYPEPARAMETER:
            case TYPEIDENT:
                return tree;
            case ANNOTATED_TYPE:
                return ((JCTree.JCAnnotatedType) tree).underlyingType;
            default:
                throw new AssertionError("Unexpected type tree: " + tree);
        }
    }

    public static JCTree innermostType(JCTree type) {
        JCTree lastAnnotatedType = null;
        JCTree cur = type;
        while (true) {
            switch (cur.getTag()) {
                case ANNOTATED_TYPE:
                    lastAnnotatedType = cur;
                    cur = ((JCTree.JCAnnotatedType) cur).underlyingType;
                    break;
                case TYPEARRAY:
                    lastAnnotatedType = null;
                    cur = ((JCTree.JCArrayTypeTree) cur).elemtype;
                    break;
                case WILDCARD:
                    lastAnnotatedType = null;
                    cur = ((JCTree.JCWildcard) cur).inner;
                    break;
                default:
                    if (lastAnnotatedType != null) {
                        return lastAnnotatedType;
                    }
                    return cur;
            }
        }
    }

    private static class TypeAnnotationFinder extends TreeScanner {
        public boolean foundTypeAnno;

        private TypeAnnotationFinder() {
            this.foundTypeAnno = false;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree tree) {
            if (this.foundTypeAnno || tree == null) {
                return;
            }
            super.scan(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAnnotation(JCTree.JCAnnotation tree) {
            this.foundTypeAnno = this.foundTypeAnno || tree.hasTag(JCTree.Tag.TYPE_ANNOTATION);
        }
    }

    public static boolean containsTypeAnnotation(JCTree e) {
        TypeAnnotationFinder finder = new TypeAnnotationFinder();
        finder.scan(e);
        return finder.foundTypeAnno;
    }
}
