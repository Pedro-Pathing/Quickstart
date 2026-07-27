package com.sun.source.tree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface TreeVisitor<R, P> {
    R visitAnnotatedType(AnnotatedTypeTree annotatedTypeTree, P p);

    R visitAnnotation(AnnotationTree annotationTree, P p);

    R visitArrayAccess(ArrayAccessTree arrayAccessTree, P p);

    R visitArrayType(ArrayTypeTree arrayTypeTree, P p);

    R visitAssert(AssertTree assertTree, P p);

    R visitAssignment(AssignmentTree assignmentTree, P p);

    R visitBinary(BinaryTree binaryTree, P p);

    R visitBlock(BlockTree blockTree, P p);

    R visitBreak(BreakTree breakTree, P p);

    R visitCase(CaseTree caseTree, P p);

    R visitCatch(CatchTree catchTree, P p);

    R visitClass(ClassTree classTree, P p);

    R visitCompilationUnit(CompilationUnitTree compilationUnitTree, P p);

    R visitCompoundAssignment(CompoundAssignmentTree compoundAssignmentTree, P p);

    R visitConditionalExpression(ConditionalExpressionTree conditionalExpressionTree, P p);

    R visitContinue(ContinueTree continueTree, P p);

    R visitDoWhileLoop(DoWhileLoopTree doWhileLoopTree, P p);

    R visitEmptyStatement(EmptyStatementTree emptyStatementTree, P p);

    R visitEnhancedForLoop(EnhancedForLoopTree enhancedForLoopTree, P p);

    R visitErroneous(ErroneousTree erroneousTree, P p);

    R visitExpressionStatement(ExpressionStatementTree expressionStatementTree, P p);

    R visitForLoop(ForLoopTree forLoopTree, P p);

    R visitIdentifier(IdentifierTree identifierTree, P p);

    R visitIf(IfTree ifTree, P p);

    R visitImport(ImportTree importTree, P p);

    R visitInstanceOf(InstanceOfTree instanceOfTree, P p);

    R visitIntersectionType(IntersectionTypeTree intersectionTypeTree, P p);

    R visitLabeledStatement(LabeledStatementTree labeledStatementTree, P p);

    R visitLambdaExpression(LambdaExpressionTree lambdaExpressionTree, P p);

    R visitLiteral(LiteralTree literalTree, P p);

    R visitMemberReference(MemberReferenceTree memberReferenceTree, P p);

    R visitMemberSelect(MemberSelectTree memberSelectTree, P p);

    R visitMethod(MethodTree methodTree, P p);

    R visitMethodInvocation(MethodInvocationTree methodInvocationTree, P p);

    R visitModifiers(ModifiersTree modifiersTree, P p);

    R visitNewArray(NewArrayTree newArrayTree, P p);

    R visitNewClass(NewClassTree newClassTree, P p);

    R visitOther(Tree tree, P p);

    R visitParameterizedType(ParameterizedTypeTree parameterizedTypeTree, P p);

    R visitParenthesized(ParenthesizedTree parenthesizedTree, P p);

    R visitPrimitiveType(PrimitiveTypeTree primitiveTypeTree, P p);

    R visitReturn(ReturnTree returnTree, P p);

    R visitSwitch(SwitchTree switchTree, P p);

    R visitSynchronized(SynchronizedTree synchronizedTree, P p);

    R visitThrow(ThrowTree throwTree, P p);

    R visitTry(TryTree tryTree, P p);

    R visitTypeCast(TypeCastTree typeCastTree, P p);

    R visitTypeParameter(TypeParameterTree typeParameterTree, P p);

    R visitUnary(UnaryTree unaryTree, P p);

    R visitUnionType(UnionTypeTree unionTypeTree, P p);

    R visitVariable(VariableTree variableTree, P p);

    R visitWhileLoop(WhileLoopTree whileLoopTree, P p);

    R visitWildcard(WildcardTree wildcardTree, P p);
}
