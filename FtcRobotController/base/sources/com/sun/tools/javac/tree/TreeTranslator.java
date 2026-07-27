package com.sun.tools.javac.tree;

import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.List;

/* JADX INFO: loaded from: classes.dex */
public class TreeTranslator extends JCTree.Visitor {
    protected JCTree result;

    public <T extends JCTree> T translate(T t) {
        if (t == null) {
            return null;
        }
        t.accept(this);
        T t2 = (T) this.result;
        this.result = null;
        return t2;
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v3, types: [A, com.sun.tools.javac.tree.JCTree] */
    public <T extends JCTree> List<T> translate(List<T> trees) {
        if (trees == null) {
            return null;
        }
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            list.head = translate((JCTree) list.head);
        }
        return trees;
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v3, types: [A, com.sun.tools.javac.tree.JCTree] */
    public List<JCTree.JCVariableDecl> translateVarDefs(List<JCTree.JCVariableDecl> trees) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            list.head = translate((JCTree) list.head);
        }
        return trees;
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v3, types: [A, com.sun.tools.javac.tree.JCTree] */
    public List<JCTree.JCTypeParameter> translateTypeParams(List<JCTree.JCTypeParameter> trees) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            list.head = translate((JCTree) list.head);
        }
        return trees;
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v3, types: [A, com.sun.tools.javac.tree.JCTree] */
    public List<JCTree.JCCase> translateCases(List<JCTree.JCCase> trees) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            list.head = translate((JCTree) list.head);
        }
        return trees;
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v3, types: [A, com.sun.tools.javac.tree.JCTree] */
    public List<JCTree.JCCatch> translateCatchers(List<JCTree.JCCatch> trees) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            list.head = translate((JCTree) list.head);
        }
        return trees;
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v3, types: [A, com.sun.tools.javac.tree.JCTree] */
    public List<JCTree.JCAnnotation> translateAnnotations(List<JCTree.JCAnnotation> trees) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            list.head = translate((JCTree) list.head);
        }
        return trees;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTopLevel(JCTree.JCCompilationUnit tree) {
        tree.pid = (JCTree.JCExpression) translate(tree.pid);
        tree.defs = translate(tree.defs);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitImport(JCTree.JCImport tree) {
        tree.qualid = translate(tree.qualid);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitClassDef(JCTree.JCClassDecl tree) {
        tree.mods = (JCTree.JCModifiers) translate(tree.mods);
        tree.typarams = translateTypeParams(tree.typarams);
        tree.extending = (JCTree.JCExpression) translate(tree.extending);
        tree.implementing = translate(tree.implementing);
        tree.defs = translate(tree.defs);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitMethodDef(JCTree.JCMethodDecl tree) {
        tree.mods = (JCTree.JCModifiers) translate(tree.mods);
        tree.restype = (JCTree.JCExpression) translate(tree.restype);
        tree.typarams = translateTypeParams(tree.typarams);
        tree.recvparam = (JCTree.JCVariableDecl) translate(tree.recvparam);
        tree.params = translateVarDefs(tree.params);
        tree.thrown = translate(tree.thrown);
        tree.body = (JCTree.JCBlock) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitVarDef(JCTree.JCVariableDecl tree) {
        tree.mods = (JCTree.JCModifiers) translate(tree.mods);
        tree.nameexpr = (JCTree.JCExpression) translate(tree.nameexpr);
        tree.vartype = (JCTree.JCExpression) translate(tree.vartype);
        tree.init = (JCTree.JCExpression) translate(tree.init);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSkip(JCTree.JCSkip tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBlock(JCTree.JCBlock tree) {
        tree.stats = translate(tree.stats);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
        tree.body = (JCTree.JCStatement) translate(tree.body);
        tree.cond = (JCTree.JCExpression) translate(tree.cond);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWhileLoop(JCTree.JCWhileLoop tree) {
        tree.cond = (JCTree.JCExpression) translate(tree.cond);
        tree.body = (JCTree.JCStatement) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForLoop(JCTree.JCForLoop tree) {
        tree.init = translate(tree.init);
        tree.cond = (JCTree.JCExpression) translate(tree.cond);
        tree.step = translate(tree.step);
        tree.body = (JCTree.JCStatement) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
        tree.var = (JCTree.JCVariableDecl) translate(tree.var);
        tree.expr = (JCTree.JCExpression) translate(tree.expr);
        tree.body = (JCTree.JCStatement) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLabelled(JCTree.JCLabeledStatement tree) {
        tree.body = (JCTree.JCStatement) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSwitch(JCTree.JCSwitch tree) {
        tree.selector = (JCTree.JCExpression) translate(tree.selector);
        tree.cases = translateCases(tree.cases);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitCase(JCTree.JCCase tree) {
        tree.pat = (JCTree.JCExpression) translate(tree.pat);
        tree.stats = translate(tree.stats);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSynchronized(JCTree.JCSynchronized tree) {
        tree.lock = (JCTree.JCExpression) translate(tree.lock);
        tree.body = (JCTree.JCBlock) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTry(JCTree.JCTry tree) {
        tree.resources = translate(tree.resources);
        tree.body = (JCTree.JCBlock) translate(tree.body);
        tree.catchers = translateCatchers(tree.catchers);
        tree.finalizer = (JCTree.JCBlock) translate(tree.finalizer);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitCatch(JCTree.JCCatch tree) {
        tree.param = (JCTree.JCVariableDecl) translate(tree.param);
        tree.body = (JCTree.JCBlock) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitConditional(JCTree.JCConditional tree) {
        tree.cond = (JCTree.JCExpression) translate(tree.cond);
        tree.truepart = (JCTree.JCExpression) translate(tree.truepart);
        tree.falsepart = (JCTree.JCExpression) translate(tree.falsepart);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIf(JCTree.JCIf tree) {
        tree.cond = (JCTree.JCExpression) translate(tree.cond);
        tree.thenpart = (JCTree.JCStatement) translate(tree.thenpart);
        tree.elsepart = (JCTree.JCStatement) translate(tree.elsepart);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitExec(JCTree.JCExpressionStatement tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBreak(JCTree.JCBreak tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitContinue(JCTree.JCContinue tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReturn(JCTree.JCReturn tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitThrow(JCTree.JCThrow tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssert(JCTree.JCAssert tree) {
        tree.cond = (JCTree.JCExpression) translate(tree.cond);
        tree.detail = (JCTree.JCExpression) translate(tree.detail);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitApply(JCTree.JCMethodInvocation tree) {
        tree.meth = (JCTree.JCExpression) translate(tree.meth);
        tree.args = translate(tree.args);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewClass(JCTree.JCNewClass tree) {
        tree.encl = (JCTree.JCExpression) translate(tree.encl);
        tree.clazz = (JCTree.JCExpression) translate(tree.clazz);
        tree.args = translate(tree.args);
        tree.def = (JCTree.JCClassDecl) translate(tree.def);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLambda(JCTree.JCLambda tree) {
        tree.params = translate(tree.params);
        tree.body = translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewArray(JCTree.JCNewArray tree) {
        tree.annotations = translate(tree.annotations);
        List<List<JCTree.JCAnnotation>> dimAnnos = List.nil();
        for (List<JCTree.JCAnnotation> origDimAnnos : tree.dimAnnotations) {
            dimAnnos = dimAnnos.append(translate(origDimAnnos));
        }
        tree.dimAnnotations = dimAnnos;
        tree.elemtype = (JCTree.JCExpression) translate(tree.elemtype);
        tree.dims = translate(tree.dims);
        tree.elems = translate(tree.elems);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitParens(JCTree.JCParens tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssign(JCTree.JCAssign tree) {
        tree.lhs = (JCTree.JCExpression) translate(tree.lhs);
        tree.rhs = (JCTree.JCExpression) translate(tree.rhs);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssignop(JCTree.JCAssignOp tree) {
        tree.lhs = (JCTree.JCExpression) translate(tree.lhs);
        tree.rhs = (JCTree.JCExpression) translate(tree.rhs);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitUnary(JCTree.JCUnary tree) {
        tree.arg = (JCTree.JCExpression) translate(tree.arg);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBinary(JCTree.JCBinary tree) {
        tree.lhs = (JCTree.JCExpression) translate(tree.lhs);
        tree.rhs = (JCTree.JCExpression) translate(tree.rhs);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeCast(JCTree.JCTypeCast tree) {
        tree.clazz = translate(tree.clazz);
        tree.expr = (JCTree.JCExpression) translate(tree.expr);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeTest(JCTree.JCInstanceOf tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr);
        tree.clazz = translate(tree.clazz);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIndexed(JCTree.JCArrayAccess tree) {
        tree.indexed = (JCTree.JCExpression) translate(tree.indexed);
        tree.index = (JCTree.JCExpression) translate(tree.index);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSelect(JCTree.JCFieldAccess tree) {
        tree.selected = (JCTree.JCExpression) translate(tree.selected);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReference(JCTree.JCMemberReference tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIdent(JCTree.JCIdent tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLiteral(JCTree.JCLiteral tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeIdent(JCTree.JCPrimitiveTypeTree tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeArray(JCTree.JCArrayTypeTree tree) {
        tree.elemtype = (JCTree.JCExpression) translate(tree.elemtype);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeApply(JCTree.JCTypeApply tree) {
        tree.clazz = (JCTree.JCExpression) translate(tree.clazz);
        tree.arguments = translate(tree.arguments);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeUnion(JCTree.JCTypeUnion tree) {
        tree.alternatives = translate(tree.alternatives);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeIntersection(JCTree.JCTypeIntersection tree) {
        tree.bounds = translate(tree.bounds);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeParameter(JCTree.JCTypeParameter tree) {
        tree.annotations = translate(tree.annotations);
        tree.bounds = translate(tree.bounds);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWildcard(JCTree.JCWildcard tree) {
        tree.kind = (JCTree.TypeBoundKind) translate(tree.kind);
        tree.inner = translate(tree.inner);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeBoundKind(JCTree.TypeBoundKind tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitErroneous(JCTree.JCErroneous tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLetExpr(JCTree.LetExpr tree) {
        tree.defs = translateVarDefs(tree.defs);
        tree.expr = translate(tree.expr);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitModifiers(JCTree.JCModifiers tree) {
        tree.annotations = translateAnnotations(tree.annotations);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotation(JCTree.JCAnnotation tree) {
        tree.annotationType = translate(tree.annotationType);
        tree.args = translate(tree.args);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
        tree.annotations = translate(tree.annotations);
        tree.underlyingType = (JCTree.JCExpression) translate(tree.underlyingType);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTree(JCTree tree) {
        throw new AssertionError(tree);
    }
}
