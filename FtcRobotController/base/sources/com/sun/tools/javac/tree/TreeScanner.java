package com.sun.tools.javac.tree;

import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.List;

/* JADX INFO: loaded from: classes.dex */
public class TreeScanner extends JCTree.Visitor {
    public void scan(JCTree tree) {
        if (tree != null) {
            tree.accept(this);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public void scan(List<? extends JCTree> trees) {
        if (trees != null) {
            for (List list = trees; list.nonEmpty(); list = list.tail) {
                scan((JCTree) list.head);
            }
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTopLevel(JCTree.JCCompilationUnit tree) {
        scan(tree.packageAnnotations);
        scan(tree.pid);
        scan(tree.defs);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitImport(JCTree.JCImport tree) {
        scan(tree.qualid);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitClassDef(JCTree.JCClassDecl tree) {
        scan(tree.mods);
        scan(tree.typarams);
        scan(tree.extending);
        scan(tree.implementing);
        scan(tree.defs);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitMethodDef(JCTree.JCMethodDecl tree) {
        scan(tree.mods);
        scan(tree.restype);
        scan(tree.typarams);
        scan(tree.recvparam);
        scan(tree.params);
        scan(tree.thrown);
        scan(tree.defaultValue);
        scan(tree.body);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitVarDef(JCTree.JCVariableDecl tree) {
        scan(tree.mods);
        scan(tree.vartype);
        scan(tree.nameexpr);
        scan(tree.init);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSkip(JCTree.JCSkip tree) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBlock(JCTree.JCBlock tree) {
        scan(tree.stats);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
        scan(tree.body);
        scan(tree.cond);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWhileLoop(JCTree.JCWhileLoop tree) {
        scan(tree.cond);
        scan(tree.body);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForLoop(JCTree.JCForLoop tree) {
        scan(tree.init);
        scan(tree.cond);
        scan(tree.step);
        scan(tree.body);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
        scan(tree.var);
        scan(tree.expr);
        scan(tree.body);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLabelled(JCTree.JCLabeledStatement tree) {
        scan(tree.body);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSwitch(JCTree.JCSwitch tree) {
        scan(tree.selector);
        scan(tree.cases);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitCase(JCTree.JCCase tree) {
        scan(tree.pat);
        scan(tree.stats);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSynchronized(JCTree.JCSynchronized tree) {
        scan(tree.lock);
        scan(tree.body);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTry(JCTree.JCTry tree) {
        scan(tree.resources);
        scan(tree.body);
        scan(tree.catchers);
        scan(tree.finalizer);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitCatch(JCTree.JCCatch tree) {
        scan(tree.param);
        scan(tree.body);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitConditional(JCTree.JCConditional tree) {
        scan(tree.cond);
        scan(tree.truepart);
        scan(tree.falsepart);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIf(JCTree.JCIf tree) {
        scan(tree.cond);
        scan(tree.thenpart);
        scan(tree.elsepart);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitExec(JCTree.JCExpressionStatement tree) {
        scan(tree.expr);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBreak(JCTree.JCBreak tree) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitContinue(JCTree.JCContinue tree) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReturn(JCTree.JCReturn tree) {
        scan(tree.expr);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitThrow(JCTree.JCThrow tree) {
        scan(tree.expr);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssert(JCTree.JCAssert tree) {
        scan(tree.cond);
        scan(tree.detail);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitApply(JCTree.JCMethodInvocation tree) {
        scan(tree.typeargs);
        scan(tree.meth);
        scan(tree.args);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewClass(JCTree.JCNewClass tree) {
        scan(tree.encl);
        scan(tree.typeargs);
        scan(tree.clazz);
        scan(tree.args);
        scan(tree.def);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewArray(JCTree.JCNewArray tree) {
        scan(tree.annotations);
        scan(tree.elemtype);
        scan(tree.dims);
        for (List<JCTree.JCAnnotation> annos : tree.dimAnnotations) {
            scan(annos);
        }
        scan(tree.elems);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLambda(JCTree.JCLambda tree) {
        scan(tree.body);
        scan(tree.params);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitParens(JCTree.JCParens tree) {
        scan(tree.expr);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssign(JCTree.JCAssign tree) {
        scan(tree.lhs);
        scan(tree.rhs);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssignop(JCTree.JCAssignOp tree) {
        scan(tree.lhs);
        scan(tree.rhs);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitUnary(JCTree.JCUnary tree) {
        scan(tree.arg);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBinary(JCTree.JCBinary tree) {
        scan(tree.lhs);
        scan(tree.rhs);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeCast(JCTree.JCTypeCast tree) {
        scan(tree.clazz);
        scan(tree.expr);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeTest(JCTree.JCInstanceOf tree) {
        scan(tree.expr);
        scan(tree.clazz);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIndexed(JCTree.JCArrayAccess tree) {
        scan(tree.indexed);
        scan(tree.index);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSelect(JCTree.JCFieldAccess tree) {
        scan(tree.selected);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReference(JCTree.JCMemberReference tree) {
        scan(tree.expr);
        scan(tree.typeargs);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIdent(JCTree.JCIdent tree) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLiteral(JCTree.JCLiteral tree) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeIdent(JCTree.JCPrimitiveTypeTree tree) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeArray(JCTree.JCArrayTypeTree tree) {
        scan(tree.elemtype);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeApply(JCTree.JCTypeApply tree) {
        scan(tree.clazz);
        scan(tree.arguments);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeUnion(JCTree.JCTypeUnion tree) {
        scan(tree.alternatives);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeIntersection(JCTree.JCTypeIntersection tree) {
        scan(tree.bounds);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeParameter(JCTree.JCTypeParameter tree) {
        scan(tree.annotations);
        scan(tree.bounds);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWildcard(JCTree.JCWildcard tree) {
        scan(tree.kind);
        if (tree.inner != null) {
            scan(tree.inner);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeBoundKind(JCTree.TypeBoundKind that) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitModifiers(JCTree.JCModifiers tree) {
        scan(tree.annotations);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotation(JCTree.JCAnnotation tree) {
        scan(tree.annotationType);
        scan(tree.args);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
        scan(tree.annotations);
        scan(tree.underlyingType);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitErroneous(JCTree.JCErroneous tree) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLetExpr(JCTree.LetExpr tree) {
        scan(tree.defs);
        scan(tree.expr);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTree(JCTree tree) {
        Assert.error();
    }
}
