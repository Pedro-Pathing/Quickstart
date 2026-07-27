package com.sun.tools.javac.tree;

import com.sun.source.tree.AnnotatedTypeTree;
import com.sun.source.tree.AnnotationTree;
import com.sun.source.tree.ArrayAccessTree;
import com.sun.source.tree.ArrayTypeTree;
import com.sun.source.tree.AssertTree;
import com.sun.source.tree.AssignmentTree;
import com.sun.source.tree.BinaryTree;
import com.sun.source.tree.BlockTree;
import com.sun.source.tree.BreakTree;
import com.sun.source.tree.CaseTree;
import com.sun.source.tree.CatchTree;
import com.sun.source.tree.ClassTree;
import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.CompoundAssignmentTree;
import com.sun.source.tree.ConditionalExpressionTree;
import com.sun.source.tree.ContinueTree;
import com.sun.source.tree.DoWhileLoopTree;
import com.sun.source.tree.EmptyStatementTree;
import com.sun.source.tree.EnhancedForLoopTree;
import com.sun.source.tree.ErroneousTree;
import com.sun.source.tree.ExpressionStatementTree;
import com.sun.source.tree.ForLoopTree;
import com.sun.source.tree.IdentifierTree;
import com.sun.source.tree.IfTree;
import com.sun.source.tree.ImportTree;
import com.sun.source.tree.InstanceOfTree;
import com.sun.source.tree.IntersectionTypeTree;
import com.sun.source.tree.LabeledStatementTree;
import com.sun.source.tree.LambdaExpressionTree;
import com.sun.source.tree.LiteralTree;
import com.sun.source.tree.MemberReferenceTree;
import com.sun.source.tree.MemberSelectTree;
import com.sun.source.tree.MethodInvocationTree;
import com.sun.source.tree.MethodTree;
import com.sun.source.tree.ModifiersTree;
import com.sun.source.tree.NewArrayTree;
import com.sun.source.tree.NewClassTree;
import com.sun.source.tree.ParameterizedTypeTree;
import com.sun.source.tree.ParenthesizedTree;
import com.sun.source.tree.PrimitiveTypeTree;
import com.sun.source.tree.ReturnTree;
import com.sun.source.tree.SwitchTree;
import com.sun.source.tree.SynchronizedTree;
import com.sun.source.tree.ThrowTree;
import com.sun.source.tree.Tree;
import com.sun.source.tree.TreeVisitor;
import com.sun.source.tree.TryTree;
import com.sun.source.tree.TypeCastTree;
import com.sun.source.tree.TypeParameterTree;
import com.sun.source.tree.UnaryTree;
import com.sun.source.tree.UnionTypeTree;
import com.sun.source.tree.VariableTree;
import com.sun.source.tree.WhileLoopTree;
import com.sun.source.tree.WildcardTree;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;

/* JADX INFO: loaded from: classes.dex */
public class TreeCopier<P> implements TreeVisitor<JCTree, P> {
    private TreeMaker M;

    public TreeCopier(TreeMaker M) {
        this.M = M;
    }

    public <T extends JCTree> T copy(T t) {
        return (T) copy(t, (Object) null);
    }

    public <T extends JCTree> T copy(T tree, P p) {
        if (tree == null) {
            return null;
        }
        return (T) tree.accept(this, p);
    }

    public <T extends JCTree> List<T> copy(List<T> trees) {
        return copy(trees, (Object) null);
    }

    public <T extends JCTree> List<T> copy(List<T> trees, P p) {
        if (trees == null) {
            return null;
        }
        ListBuffer listBuffer = new ListBuffer();
        for (T tree : trees) {
            listBuffer.append(copy(tree, p));
        }
        return listBuffer.toList();
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitAnnotatedType(AnnotatedTypeTree node, P p) {
        JCTree.JCAnnotatedType t = (JCTree.JCAnnotatedType) node;
        List<T> listCopy = copy(t.annotations, p);
        JCTree.JCExpression underlyingType = (JCTree.JCExpression) copy(t.underlyingType, p);
        return this.M.at(t.pos).AnnotatedType(listCopy, underlyingType);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitAnnotation(AnnotationTree node, P p) {
        JCTree.JCAnnotation t = (JCTree.JCAnnotation) node;
        JCTree annotationType = copy(t.annotationType, p);
        List<T> listCopy = copy(t.args, p);
        if (t.getKind() == Tree.Kind.TYPE_ANNOTATION) {
            JCTree.JCAnnotation newTA = this.M.at(t.pos).TypeAnnotation(annotationType, listCopy);
            newTA.attribute = t.attribute;
            return newTA;
        }
        JCTree.JCAnnotation newT = this.M.at(t.pos).Annotation(annotationType, listCopy);
        newT.attribute = t.attribute;
        return newT;
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitAssert(AssertTree node, P p) {
        JCTree.JCAssert t = (JCTree.JCAssert) node;
        JCTree.JCExpression cond = (JCTree.JCExpression) copy(t.cond, p);
        JCTree.JCExpression detail = (JCTree.JCExpression) copy(t.detail, p);
        return this.M.at(t.pos).Assert(cond, detail);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitAssignment(AssignmentTree node, P p) {
        JCTree.JCAssign t = (JCTree.JCAssign) node;
        JCTree.JCExpression lhs = (JCTree.JCExpression) copy(t.lhs, p);
        JCTree.JCExpression rhs = (JCTree.JCExpression) copy(t.rhs, p);
        return this.M.at(t.pos).Assign(lhs, rhs);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitCompoundAssignment(CompoundAssignmentTree node, P p) {
        JCTree.JCAssignOp t = (JCTree.JCAssignOp) node;
        JCTree lhs = copy(t.lhs, p);
        JCTree rhs = copy(t.rhs, p);
        return this.M.at(t.pos).Assignop(t.getTag(), lhs, rhs);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitBinary(BinaryTree node, P p) {
        JCTree.JCBinary t = (JCTree.JCBinary) node;
        JCTree.JCExpression lhs = (JCTree.JCExpression) copy(t.lhs, p);
        JCTree.JCExpression rhs = (JCTree.JCExpression) copy(t.rhs, p);
        return this.M.at(t.pos).Binary(t.getTag(), lhs, rhs);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitBlock(BlockTree node, P p) {
        JCTree.JCBlock t = (JCTree.JCBlock) node;
        return this.M.at(t.pos).Block(t.flags, copy(t.stats, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitBreak(BreakTree node, P p) {
        JCTree.JCBreak t = (JCTree.JCBreak) node;
        return this.M.at(t.pos).Break(t.label);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitCase(CaseTree node, P p) {
        JCTree.JCCase t = (JCTree.JCCase) node;
        JCTree.JCExpression pat = (JCTree.JCExpression) copy(t.pat, p);
        return this.M.at(t.pos).Case(pat, copy(t.stats, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitCatch(CatchTree node, P p) {
        JCTree.JCCatch t = (JCTree.JCCatch) node;
        JCTree.JCVariableDecl param = (JCTree.JCVariableDecl) copy(t.param, p);
        JCTree.JCBlock body = (JCTree.JCBlock) copy(t.body, p);
        return this.M.at(t.pos).Catch(param, body);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitClass(ClassTree node, P p) {
        JCTree.JCClassDecl t = (JCTree.JCClassDecl) node;
        JCTree.JCModifiers mods = (JCTree.JCModifiers) copy(t.mods, p);
        List<T> listCopy = copy(t.typarams, p);
        JCTree.JCExpression extending = (JCTree.JCExpression) copy(t.extending, p);
        List<T> listCopy2 = copy(t.implementing, p);
        List<JCTree> defs = copy(t.defs, p);
        return this.M.at(t.pos).ClassDef(mods, t.name, listCopy, extending, listCopy2, defs);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitConditionalExpression(ConditionalExpressionTree node, P p) {
        JCTree.JCConditional t = (JCTree.JCConditional) node;
        JCTree.JCExpression cond = (JCTree.JCExpression) copy(t.cond, p);
        JCTree.JCExpression truepart = (JCTree.JCExpression) copy(t.truepart, p);
        JCTree.JCExpression falsepart = (JCTree.JCExpression) copy(t.falsepart, p);
        return this.M.at(t.pos).Conditional(cond, truepart, falsepart);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitContinue(ContinueTree node, P p) {
        JCTree.JCContinue t = (JCTree.JCContinue) node;
        return this.M.at(t.pos).Continue(t.label);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitDoWhileLoop(DoWhileLoopTree node, P p) {
        JCTree.JCDoWhileLoop t = (JCTree.JCDoWhileLoop) node;
        JCTree.JCStatement body = (JCTree.JCStatement) copy(t.body, p);
        JCTree.JCExpression cond = (JCTree.JCExpression) copy(t.cond, p);
        return this.M.at(t.pos).DoLoop(body, cond);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitErroneous(ErroneousTree node, P p) {
        JCTree.JCErroneous t = (JCTree.JCErroneous) node;
        return this.M.at(t.pos).Erroneous(copy(t.errs, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitExpressionStatement(ExpressionStatementTree node, P p) {
        JCTree.JCExpressionStatement t = (JCTree.JCExpressionStatement) node;
        JCTree.JCExpression expr = (JCTree.JCExpression) copy(t.expr, p);
        return this.M.at(t.pos).Exec(expr);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitEnhancedForLoop(EnhancedForLoopTree node, P p) {
        JCTree.JCEnhancedForLoop t = (JCTree.JCEnhancedForLoop) node;
        JCTree.JCVariableDecl var = (JCTree.JCVariableDecl) copy(t.var, p);
        JCTree.JCExpression expr = (JCTree.JCExpression) copy(t.expr, p);
        JCTree.JCStatement body = (JCTree.JCStatement) copy(t.body, p);
        return this.M.at(t.pos).ForeachLoop(var, expr, body);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitForLoop(ForLoopTree node, P p) {
        JCTree.JCForLoop t = (JCTree.JCForLoop) node;
        List<T> listCopy = copy(t.init, p);
        JCTree.JCExpression cond = (JCTree.JCExpression) copy(t.cond, p);
        List<T> listCopy2 = copy(t.step, p);
        JCTree.JCStatement body = (JCTree.JCStatement) copy(t.body, p);
        return this.M.at(t.pos).ForLoop(listCopy, cond, listCopy2, body);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitIdentifier(IdentifierTree node, P p) {
        JCTree.JCIdent t = (JCTree.JCIdent) node;
        return this.M.at(t.pos).Ident(t.name);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitIf(IfTree node, P p) {
        JCTree.JCIf t = (JCTree.JCIf) node;
        JCTree.JCExpression cond = (JCTree.JCExpression) copy(t.cond, p);
        JCTree.JCStatement thenpart = (JCTree.JCStatement) copy(t.thenpart, p);
        JCTree.JCStatement elsepart = (JCTree.JCStatement) copy(t.elsepart, p);
        return this.M.at(t.pos).If(cond, thenpart, elsepart);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitImport(ImportTree node, P p) {
        JCTree.JCImport t = (JCTree.JCImport) node;
        JCTree qualid = copy(t.qualid, p);
        return this.M.at(t.pos).Import(qualid, t.staticImport);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitArrayAccess(ArrayAccessTree node, P p) {
        JCTree.JCArrayAccess t = (JCTree.JCArrayAccess) node;
        JCTree.JCExpression indexed = (JCTree.JCExpression) copy(t.indexed, p);
        JCTree.JCExpression index = (JCTree.JCExpression) copy(t.index, p);
        return this.M.at(t.pos).Indexed(indexed, index);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitLabeledStatement(LabeledStatementTree node, P p) {
        JCTree.JCLabeledStatement t = (JCTree.JCLabeledStatement) node;
        JCTree.JCStatement body = (JCTree.JCStatement) copy(t.body, p);
        return this.M.at(t.pos).Labelled(t.label, body);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitLiteral(LiteralTree node, P p) {
        JCTree.JCLiteral t = (JCTree.JCLiteral) node;
        return this.M.at(t.pos).Literal(t.typetag, t.value);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitMethod(MethodTree node, P p) {
        JCTree.JCMethodDecl t = (JCTree.JCMethodDecl) node;
        JCTree.JCModifiers mods = (JCTree.JCModifiers) copy(t.mods, p);
        JCTree.JCExpression restype = (JCTree.JCExpression) copy(t.restype, p);
        List<T> listCopy = copy(t.typarams, p);
        List<T> listCopy2 = copy(t.params, p);
        JCTree.JCVariableDecl recvparam = (JCTree.JCVariableDecl) copy(t.recvparam, p);
        List<T> listCopy3 = copy(t.thrown, p);
        JCTree.JCBlock body = (JCTree.JCBlock) copy(t.body, p);
        JCTree.JCExpression defaultValue = (JCTree.JCExpression) copy(t.defaultValue, p);
        return this.M.at(t.pos).MethodDef(mods, t.name, restype, listCopy, recvparam, listCopy2, listCopy3, body, defaultValue);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitMethodInvocation(MethodInvocationTree node, P p) {
        JCTree.JCMethodInvocation t = (JCTree.JCMethodInvocation) node;
        List<T> listCopy = copy(t.typeargs, p);
        JCTree.JCExpression meth = (JCTree.JCExpression) copy(t.meth, p);
        return this.M.at(t.pos).Apply(listCopy, meth, copy(t.args, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitModifiers(ModifiersTree node, P p) {
        JCTree.JCModifiers t = (JCTree.JCModifiers) node;
        return this.M.at(t.pos).Modifiers(t.flags, copy(t.annotations, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitNewArray(NewArrayTree node, P p) {
        JCTree.JCNewArray t = (JCTree.JCNewArray) node;
        JCTree.JCExpression elemtype = (JCTree.JCExpression) copy(t.elemtype, p);
        return this.M.at(t.pos).NewArray(elemtype, copy(t.dims, p), copy(t.elems, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitNewClass(NewClassTree node, P p) {
        JCTree.JCNewClass t = (JCTree.JCNewClass) node;
        JCTree.JCExpression encl = (JCTree.JCExpression) copy(t.encl, p);
        List<T> listCopy = copy(t.typeargs, p);
        JCTree.JCExpression clazz = (JCTree.JCExpression) copy(t.clazz, p);
        List<T> listCopy2 = copy(t.args, p);
        JCTree.JCClassDecl def = (JCTree.JCClassDecl) copy(t.def, p);
        return this.M.at(t.pos).NewClass(encl, listCopy, clazz, listCopy2, def);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitLambdaExpression(LambdaExpressionTree node, P p) {
        JCTree.JCLambda t = (JCTree.JCLambda) node;
        List<T> listCopy = copy(t.params, p);
        JCTree body = copy(t.body, p);
        return this.M.at(t.pos).Lambda(listCopy, body);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitParenthesized(ParenthesizedTree node, P p) {
        JCTree.JCParens t = (JCTree.JCParens) node;
        JCTree.JCExpression expr = (JCTree.JCExpression) copy(t.expr, p);
        return this.M.at(t.pos).Parens(expr);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitReturn(ReturnTree node, P p) {
        JCTree.JCReturn t = (JCTree.JCReturn) node;
        JCTree.JCExpression expr = (JCTree.JCExpression) copy(t.expr, p);
        return this.M.at(t.pos).Return(expr);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitMemberSelect(MemberSelectTree node, P p) {
        JCTree.JCFieldAccess t = (JCTree.JCFieldAccess) node;
        JCTree.JCExpression selected = (JCTree.JCExpression) copy(t.selected, p);
        return this.M.at(t.pos).Select(selected, t.name);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitMemberReference(MemberReferenceTree node, P p) {
        JCTree.JCMemberReference t = (JCTree.JCMemberReference) node;
        JCTree.JCExpression expr = (JCTree.JCExpression) copy(t.expr, p);
        return this.M.at(t.pos).Reference(t.mode, t.name, expr, copy(t.typeargs, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitEmptyStatement(EmptyStatementTree node, P p) {
        JCTree.JCSkip t = (JCTree.JCSkip) node;
        return this.M.at(t.pos).Skip();
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitSwitch(SwitchTree node, P p) {
        JCTree.JCSwitch t = (JCTree.JCSwitch) node;
        JCTree.JCExpression selector = (JCTree.JCExpression) copy(t.selector, p);
        return this.M.at(t.pos).Switch(selector, copy(t.cases, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitSynchronized(SynchronizedTree node, P p) {
        JCTree.JCSynchronized t = (JCTree.JCSynchronized) node;
        JCTree.JCExpression lock = (JCTree.JCExpression) copy(t.lock, p);
        JCTree.JCBlock body = (JCTree.JCBlock) copy(t.body, p);
        return this.M.at(t.pos).Synchronized(lock, body);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitThrow(ThrowTree node, P p) {
        JCTree.JCThrow t = (JCTree.JCThrow) node;
        JCTree.JCExpression expr = (JCTree.JCExpression) copy(t.expr, p);
        return this.M.at(t.pos).Throw(expr);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitCompilationUnit(CompilationUnitTree node, P p) {
        JCTree.JCCompilationUnit t = (JCTree.JCCompilationUnit) node;
        List<T> listCopy = copy(t.packageAnnotations, p);
        JCTree.JCExpression pid = (JCTree.JCExpression) copy(t.pid, p);
        List<JCTree> defs = copy(t.defs, p);
        return this.M.at(t.pos).TopLevel(listCopy, pid, defs);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitTry(TryTree node, P p) {
        JCTree.JCTry t = (JCTree.JCTry) node;
        List<JCTree> resources = copy(t.resources, p);
        JCTree.JCBlock body = (JCTree.JCBlock) copy(t.body, p);
        List<T> listCopy = copy(t.catchers, p);
        JCTree.JCBlock finalizer = (JCTree.JCBlock) copy(t.finalizer, p);
        return this.M.at(t.pos).Try(resources, body, listCopy, finalizer);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitParameterizedType(ParameterizedTypeTree node, P p) {
        JCTree.JCTypeApply t = (JCTree.JCTypeApply) node;
        JCTree.JCExpression clazz = (JCTree.JCExpression) copy(t.clazz, p);
        return this.M.at(t.pos).TypeApply(clazz, copy(t.arguments, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitUnionType(UnionTypeTree node, P p) {
        JCTree.JCTypeUnion t = (JCTree.JCTypeUnion) node;
        return this.M.at(t.pos).TypeUnion(copy(t.alternatives, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitIntersectionType(IntersectionTypeTree node, P p) {
        JCTree.JCTypeIntersection t = (JCTree.JCTypeIntersection) node;
        return this.M.at(t.pos).TypeIntersection(copy(t.bounds, p));
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitArrayType(ArrayTypeTree node, P p) {
        JCTree.JCArrayTypeTree t = (JCTree.JCArrayTypeTree) node;
        JCTree.JCExpression elemtype = (JCTree.JCExpression) copy(t.elemtype, p);
        return this.M.at(t.pos).TypeArray(elemtype);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitTypeCast(TypeCastTree node, P p) {
        JCTree.JCTypeCast t = (JCTree.JCTypeCast) node;
        JCTree clazz = copy(t.clazz, p);
        JCTree.JCExpression expr = (JCTree.JCExpression) copy(t.expr, p);
        return this.M.at(t.pos).TypeCast(clazz, expr);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitPrimitiveType(PrimitiveTypeTree node, P p) {
        JCTree.JCPrimitiveTypeTree t = (JCTree.JCPrimitiveTypeTree) node;
        return this.M.at(t.pos).TypeIdent(t.typetag);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitTypeParameter(TypeParameterTree node, P p) {
        JCTree.JCTypeParameter t = (JCTree.JCTypeParameter) node;
        List<T> listCopy = copy(t.annotations, p);
        return this.M.at(t.pos).TypeParameter(t.name, copy(t.bounds, p), listCopy);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitInstanceOf(InstanceOfTree node, P p) {
        JCTree.JCInstanceOf t = (JCTree.JCInstanceOf) node;
        JCTree.JCExpression expr = (JCTree.JCExpression) copy(t.expr, p);
        JCTree clazz = copy(t.clazz, p);
        return this.M.at(t.pos).TypeTest(expr, clazz);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitUnary(UnaryTree node, P p) {
        JCTree.JCUnary t = (JCTree.JCUnary) node;
        JCTree.JCExpression arg = (JCTree.JCExpression) copy(t.arg, p);
        return this.M.at(t.pos).Unary(t.getTag(), arg);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitVariable(VariableTree node, P p) {
        JCTree.JCVariableDecl t = (JCTree.JCVariableDecl) node;
        JCTree.JCModifiers mods = (JCTree.JCModifiers) copy(t.mods, p);
        JCTree.JCExpression vartype = (JCTree.JCExpression) copy(t.vartype, p);
        if (t.nameexpr == null) {
            JCTree.JCExpression init = (JCTree.JCExpression) copy(t.init, p);
            return this.M.at(t.pos).VarDef(mods, t.name, vartype, init);
        }
        JCTree.JCExpression init2 = t.nameexpr;
        JCTree.JCExpression nameexpr = (JCTree.JCExpression) copy(init2, p);
        return this.M.at(t.pos).ReceiverVarDef(mods, nameexpr, vartype);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitWhileLoop(WhileLoopTree node, P p) {
        JCTree.JCWhileLoop t = (JCTree.JCWhileLoop) node;
        JCTree.JCStatement body = (JCTree.JCStatement) copy(t.body, p);
        JCTree.JCExpression cond = (JCTree.JCExpression) copy(t.cond, p);
        return this.M.at(t.pos).WhileLoop(cond, body);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitWildcard(WildcardTree node, P p) {
        JCTree.JCWildcard t = (JCTree.JCWildcard) node;
        JCTree.TypeBoundKind kind = this.M.at(t.kind.pos).TypeBoundKind(t.kind.kind);
        JCTree inner = copy(t.inner, p);
        return this.M.at(t.pos).Wildcard(kind, inner);
    }

    /* JADX WARN: Can't rename method to resolve collision */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.tree.TreeVisitor
    public JCTree visitOther(Tree node, P p) {
        JCTree tree = (JCTree) node;
        switch (tree.getTag()) {
            case LETEXPR:
                JCTree.LetExpr t = (JCTree.LetExpr) node;
                List<T> listCopy = copy(t.defs, p);
                JCTree expr = copy(t.expr, p);
                return this.M.at(t.pos).LetExpr((List<JCTree.JCVariableDecl>) listCopy, expr);
            default:
                throw new AssertionError("unknown tree tag: " + tree.getTag());
        }
    }
}
