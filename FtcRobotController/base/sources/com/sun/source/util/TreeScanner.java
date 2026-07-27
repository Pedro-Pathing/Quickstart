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
public class TreeScanner<R, P> implements TreeVisitor<R, P> {
    public R scan(Tree tree, P p) {
        if (tree == null) {
            return null;
        }
        return (R) tree.accept(this, p);
    }

    private R scanAndReduce(Tree node, P p, R r) {
        return reduce(scan(node, p), r);
    }

    public R scan(Iterable<? extends Tree> iterable, P p) {
        R rScan = null;
        if (iterable != null) {
            boolean z = true;
            for (Tree tree : iterable) {
                rScan = z ? scan(tree, p) : scanAndReduce(tree, p, rScan);
                z = false;
            }
        }
        return rScan;
    }

    private R scanAndReduce(Iterable<? extends Tree> nodes, P p, R r) {
        return reduce(scan(nodes, p), r);
    }

    public R reduce(R r1, R r2) {
        return r1;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitCompilationUnit(CompilationUnitTree node, P p) {
        R r = scan(node.getPackageAnnotations(), p);
        return scanAndReduce(node.getTypeDecls(), p, scanAndReduce(node.getImports(), p, scanAndReduce(node.getPackageName(), p, r)));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitImport(ImportTree node, P p) {
        return scan(node.getQualifiedIdentifier(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitClass(ClassTree node, P p) {
        R r = scan(node.getModifiers(), p);
        return scanAndReduce(node.getMembers(), p, scanAndReduce(node.getImplementsClause(), p, scanAndReduce(node.getExtendsClause(), p, scanAndReduce(node.getTypeParameters(), p, r))));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitMethod(MethodTree node, P p) {
        R r = scan(node.getModifiers(), p);
        return scanAndReduce(node.getDefaultValue(), p, scanAndReduce(node.getBody(), p, scanAndReduce(node.getThrows(), p, scanAndReduce(node.getReceiverParameter(), p, scanAndReduce(node.getParameters(), p, scanAndReduce(node.getTypeParameters(), p, scanAndReduce(node.getReturnType(), p, r)))))));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitVariable(VariableTree node, P p) {
        R r = scan(node.getModifiers(), p);
        return scanAndReduce(node.getInitializer(), p, scanAndReduce(node.getNameExpression(), p, scanAndReduce(node.getType(), p, r)));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitEmptyStatement(EmptyStatementTree node, P p) {
        return null;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitBlock(BlockTree node, P p) {
        return scan(node.getStatements(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitDoWhileLoop(DoWhileLoopTree node, P p) {
        R r = scan(node.getStatement(), p);
        return scanAndReduce(node.getCondition(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitWhileLoop(WhileLoopTree node, P p) {
        R r = scan(node.getCondition(), p);
        return scanAndReduce(node.getStatement(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitForLoop(ForLoopTree node, P p) {
        R r = scan(node.getInitializer(), p);
        return scanAndReduce(node.getStatement(), p, scanAndReduce(node.getUpdate(), p, scanAndReduce(node.getCondition(), p, r)));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitEnhancedForLoop(EnhancedForLoopTree node, P p) {
        R r = scan(node.getVariable(), p);
        return scanAndReduce(node.getStatement(), p, scanAndReduce(node.getExpression(), p, r));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitLabeledStatement(LabeledStatementTree node, P p) {
        return scan(node.getStatement(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitSwitch(SwitchTree node, P p) {
        R r = scan(node.getExpression(), p);
        return scanAndReduce(node.getCases(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitCase(CaseTree node, P p) {
        R r = scan(node.getExpression(), p);
        return scanAndReduce(node.getStatements(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitSynchronized(SynchronizedTree node, P p) {
        R r = scan(node.getExpression(), p);
        return scanAndReduce(node.getBlock(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitTry(TryTree node, P p) {
        R r = scan(node.getResources(), p);
        return scanAndReduce(node.getFinallyBlock(), p, scanAndReduce(node.getCatches(), p, scanAndReduce(node.getBlock(), p, r)));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitCatch(CatchTree node, P p) {
        R r = scan(node.getParameter(), p);
        return scanAndReduce(node.getBlock(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitConditionalExpression(ConditionalExpressionTree node, P p) {
        R r = scan(node.getCondition(), p);
        return scanAndReduce(node.getFalseExpression(), p, scanAndReduce(node.getTrueExpression(), p, r));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitIf(IfTree node, P p) {
        R r = scan(node.getCondition(), p);
        return scanAndReduce(node.getElseStatement(), p, scanAndReduce(node.getThenStatement(), p, r));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitExpressionStatement(ExpressionStatementTree node, P p) {
        return scan(node.getExpression(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitBreak(BreakTree node, P p) {
        return null;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitContinue(ContinueTree node, P p) {
        return null;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitReturn(ReturnTree node, P p) {
        return scan(node.getExpression(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitThrow(ThrowTree node, P p) {
        return scan(node.getExpression(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitAssert(AssertTree node, P p) {
        R r = scan(node.getCondition(), p);
        return scanAndReduce(node.getDetail(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitMethodInvocation(MethodInvocationTree node, P p) {
        R r = scan(node.getTypeArguments(), p);
        return scanAndReduce(node.getArguments(), p, scanAndReduce(node.getMethodSelect(), p, r));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitNewClass(NewClassTree node, P p) {
        R r = scan(node.getEnclosingExpression(), p);
        return scanAndReduce(node.getClassBody(), p, scanAndReduce(node.getArguments(), p, scanAndReduce(node.getTypeArguments(), p, scanAndReduce(node.getIdentifier(), p, r))));
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitNewArray(NewArrayTree node, P p) {
        R r = scan(node.getType(), p);
        R r2 = scanAndReduce(node.getAnnotations(), p, scanAndReduce(node.getInitializers(), p, scanAndReduce(node.getDimensions(), p, r)));
        for (Iterable<? extends Tree> dimAnno : node.getDimAnnotations()) {
            r2 = scanAndReduce(dimAnno, p, r2);
        }
        return r2;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitLambdaExpression(LambdaExpressionTree node, P p) {
        R r = scan(node.getParameters(), p);
        return scanAndReduce(node.getBody(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitParenthesized(ParenthesizedTree node, P p) {
        return scan(node.getExpression(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitAssignment(AssignmentTree node, P p) {
        R r = scan(node.getVariable(), p);
        return scanAndReduce(node.getExpression(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitCompoundAssignment(CompoundAssignmentTree node, P p) {
        R r = scan(node.getVariable(), p);
        return scanAndReduce(node.getExpression(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitUnary(UnaryTree node, P p) {
        return scan(node.getExpression(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitBinary(BinaryTree node, P p) {
        R r = scan(node.getLeftOperand(), p);
        return scanAndReduce(node.getRightOperand(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitTypeCast(TypeCastTree node, P p) {
        R r = scan(node.getType(), p);
        return scanAndReduce(node.getExpression(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitInstanceOf(InstanceOfTree node, P p) {
        R r = scan(node.getExpression(), p);
        return scanAndReduce(node.getType(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitArrayAccess(ArrayAccessTree node, P p) {
        R r = scan(node.getExpression(), p);
        return scanAndReduce(node.getIndex(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitMemberSelect(MemberSelectTree node, P p) {
        return scan(node.getExpression(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitMemberReference(MemberReferenceTree node, P p) {
        R r = scan(node.getQualifierExpression(), p);
        return scanAndReduce(node.getTypeArguments(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitIdentifier(IdentifierTree node, P p) {
        return null;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitLiteral(LiteralTree node, P p) {
        return null;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitPrimitiveType(PrimitiveTypeTree node, P p) {
        return null;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitArrayType(ArrayTypeTree node, P p) {
        return scan(node.getType(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitParameterizedType(ParameterizedTypeTree node, P p) {
        R r = scan(node.getType(), p);
        return scanAndReduce(node.getTypeArguments(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitUnionType(UnionTypeTree node, P p) {
        return scan(node.getTypeAlternatives(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitIntersectionType(IntersectionTypeTree node, P p) {
        return scan(node.getBounds(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitTypeParameter(TypeParameterTree node, P p) {
        R r = scan(node.getAnnotations(), p);
        return scanAndReduce(node.getBounds(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitWildcard(WildcardTree node, P p) {
        return scan(node.getBound(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitModifiers(ModifiersTree node, P p) {
        return scan(node.getAnnotations(), p);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitAnnotation(AnnotationTree node, P p) {
        R r = scan(node.getAnnotationType(), p);
        return scanAndReduce(node.getArguments(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitAnnotatedType(AnnotatedTypeTree node, P p) {
        R r = scan(node.getAnnotations(), p);
        return scanAndReduce(node.getUnderlyingType(), p, r);
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitOther(Tree node, P p) {
        return null;
    }

    @Override // com.sun.source.tree.TreeVisitor
    public R visitErroneous(ErroneousTree node, P p) {
        return null;
    }
}
