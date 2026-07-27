package com.sun.source.util;

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
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public class SimpleTreeVisitor<R, P> implements TreeVisitor<R, P> {
    protected final R DEFAULT_VALUE;

    protected SimpleTreeVisitor() {
        this.DEFAULT_VALUE = null;
    }

    protected SimpleTreeVisitor(R defaultValue) {
        this.DEFAULT_VALUE = defaultValue;
    }

    protected R defaultAction(Tree node, P p) {
        return this.DEFAULT_VALUE;
    }

    public final R visit(Tree tree, P p) {
        if (tree == null) {
            return null;
        }
        return (R) tree.accept(this, p);
    }

    public final R visit(Iterable<? extends Tree> nodes, P p) {
        R r = null;
        if (nodes != null) {
            for (Tree node : nodes) {
                r = visit(node, p);
            }
        }
        return r;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitCompilationUnit(CompilationUnitTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitImport(ImportTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitClass(ClassTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitMethod(MethodTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitVariable(VariableTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitEmptyStatement(EmptyStatementTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitBlock(BlockTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitDoWhileLoop(DoWhileLoopTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitWhileLoop(WhileLoopTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitForLoop(ForLoopTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitEnhancedForLoop(EnhancedForLoopTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitLabeledStatement(LabeledStatementTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitSwitch(SwitchTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitCase(CaseTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitSynchronized(SynchronizedTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitTry(TryTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitCatch(CatchTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitConditionalExpression(ConditionalExpressionTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitIf(IfTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitExpressionStatement(ExpressionStatementTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitBreak(BreakTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitContinue(ContinueTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitReturn(ReturnTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitThrow(ThrowTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitAssert(AssertTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitMethodInvocation(MethodInvocationTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitNewClass(NewClassTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitNewArray(NewArrayTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitLambdaExpression(LambdaExpressionTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitParenthesized(ParenthesizedTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitAssignment(AssignmentTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitCompoundAssignment(CompoundAssignmentTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitUnary(UnaryTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitBinary(BinaryTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitTypeCast(TypeCastTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitInstanceOf(InstanceOfTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitArrayAccess(ArrayAccessTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitMemberSelect(MemberSelectTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitMemberReference(MemberReferenceTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitIdentifier(IdentifierTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitLiteral(LiteralTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitPrimitiveType(PrimitiveTypeTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitArrayType(ArrayTypeTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitParameterizedType(ParameterizedTypeTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitUnionType(UnionTypeTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitIntersectionType(IntersectionTypeTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitTypeParameter(TypeParameterTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitWildcard(WildcardTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitModifiers(ModifiersTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitAnnotation(AnnotationTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitAnnotatedType(AnnotatedTypeTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitErroneous(ErroneousTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitOther(Tree node, P p) {
        return defaultAction(node, p);
    }
}
