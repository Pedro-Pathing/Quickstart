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
import com.sun.source.tree.ExpressionTree;
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
import com.sun.source.tree.StatementTree;
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
import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.BoundKind;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Position;
import java.io.IOException;
import java.io.StringWriter;
import java.util.Set;
import javax.lang.model.element.Modifier;
import javax.lang.model.type.TypeKind;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public abstract class JCTree implements Tree, Cloneable, JCDiagnostic.DiagnosticPosition {
    public int pos;
    public Type type;

    public interface Factory {
        JCAnnotation Annotation(JCTree jCTree, List<JCExpression> list);

        JCMethodInvocation Apply(List<JCExpression> list, JCExpression jCExpression, List<JCExpression> list2);

        JCAssert Assert(JCExpression jCExpression, JCExpression jCExpression2);

        JCAssign Assign(JCExpression jCExpression, JCExpression jCExpression2);

        JCAssignOp Assignop(Tag tag, JCTree jCTree, JCTree jCTree2);

        JCBinary Binary(Tag tag, JCExpression jCExpression, JCExpression jCExpression2);

        JCBlock Block(long j, List<JCStatement> list);

        JCBreak Break(Name name);

        JCCase Case(JCExpression jCExpression, List<JCStatement> list);

        JCCatch Catch(JCVariableDecl jCVariableDecl, JCBlock jCBlock);

        JCClassDecl ClassDef(JCModifiers jCModifiers, Name name, List<JCTypeParameter> list, JCExpression jCExpression, List<JCExpression> list2, List<JCTree> list3);

        JCConditional Conditional(JCExpression jCExpression, JCExpression jCExpression2, JCExpression jCExpression3);

        JCContinue Continue(Name name);

        JCDoWhileLoop DoLoop(JCStatement jCStatement, JCExpression jCExpression);

        JCErroneous Erroneous(List<? extends JCTree> list);

        JCExpressionStatement Exec(JCExpression jCExpression);

        JCForLoop ForLoop(List<JCStatement> list, JCExpression jCExpression, List<JCExpressionStatement> list2, JCStatement jCStatement);

        JCEnhancedForLoop ForeachLoop(JCVariableDecl jCVariableDecl, JCExpression jCExpression, JCStatement jCStatement);

        JCIdent Ident(Name name);

        JCIf If(JCExpression jCExpression, JCStatement jCStatement, JCStatement jCStatement2);

        JCImport Import(JCTree jCTree, boolean z);

        JCArrayAccess Indexed(JCExpression jCExpression, JCExpression jCExpression2);

        JCLabeledStatement Labelled(Name name, JCStatement jCStatement);

        LetExpr LetExpr(List<JCVariableDecl> list, JCTree jCTree);

        JCLiteral Literal(TypeTag typeTag, Object obj);

        JCMethodDecl MethodDef(JCModifiers jCModifiers, Name name, JCExpression jCExpression, List<JCTypeParameter> list, JCVariableDecl jCVariableDecl, List<JCVariableDecl> list2, List<JCExpression> list3, JCBlock jCBlock, JCExpression jCExpression2);

        JCModifiers Modifiers(long j, List<JCAnnotation> list);

        JCNewArray NewArray(JCExpression jCExpression, List<JCExpression> list, List<JCExpression> list2);

        JCNewClass NewClass(JCExpression jCExpression, List<JCExpression> list, JCExpression jCExpression2, List<JCExpression> list2, JCClassDecl jCClassDecl);

        JCParens Parens(JCExpression jCExpression);

        JCReturn Return(JCExpression jCExpression);

        JCFieldAccess Select(JCExpression jCExpression, Name name);

        JCSkip Skip();

        JCSwitch Switch(JCExpression jCExpression, List<JCCase> list);

        JCSynchronized Synchronized(JCExpression jCExpression, JCBlock jCBlock);

        JCThrow Throw(JCExpression jCExpression);

        JCCompilationUnit TopLevel(List<JCAnnotation> list, JCExpression jCExpression, List<JCTree> list2);

        JCTry Try(JCBlock jCBlock, List<JCCatch> list, JCBlock jCBlock2);

        JCTry Try(List<JCTree> list, JCBlock jCBlock, List<JCCatch> list2, JCBlock jCBlock2);

        JCTypeApply TypeApply(JCExpression jCExpression, List<JCExpression> list);

        JCArrayTypeTree TypeArray(JCExpression jCExpression);

        TypeBoundKind TypeBoundKind(BoundKind boundKind);

        JCTypeCast TypeCast(JCTree jCTree, JCExpression jCExpression);

        JCPrimitiveTypeTree TypeIdent(TypeTag typeTag);

        JCTypeParameter TypeParameter(Name name, List<JCExpression> list);

        JCInstanceOf TypeTest(JCExpression jCExpression, JCTree jCTree);

        JCUnary Unary(Tag tag, JCExpression jCExpression);

        JCVariableDecl VarDef(JCModifiers jCModifiers, Name name, JCExpression jCExpression, JCExpression jCExpression2);

        JCWhileLoop WhileLoop(JCExpression jCExpression, JCStatement jCStatement);

        JCWildcard Wildcard(TypeBoundKind typeBoundKind, JCTree jCTree);
    }

    @Override // com.sun.source.tree.Tree
    public abstract <R, D> R accept(TreeVisitor<R, D> treeVisitor, D d);

    public abstract void accept(Visitor visitor);

    public abstract Tag getTag();

    public enum Tag {
        NO_TAG,
        TOPLEVEL,
        IMPORT,
        CLASSDEF,
        METHODDEF,
        VARDEF,
        SKIP,
        BLOCK,
        DOLOOP,
        WHILELOOP,
        FORLOOP,
        FOREACHLOOP,
        LABELLED,
        SWITCH,
        CASE,
        SYNCHRONIZED,
        TRY,
        CATCH,
        CONDEXPR,
        IF,
        EXEC,
        BREAK,
        CONTINUE,
        RETURN,
        THROW,
        ASSERT,
        APPLY,
        NEWCLASS,
        NEWARRAY,
        LAMBDA,
        PARENS,
        ASSIGN,
        TYPECAST,
        TYPETEST,
        INDEXED,
        SELECT,
        REFERENCE,
        IDENT,
        LITERAL,
        TYPEIDENT,
        TYPEARRAY,
        TYPEAPPLY,
        TYPEUNION,
        TYPEINTERSECTION,
        TYPEPARAMETER,
        WILDCARD,
        TYPEBOUNDKIND,
        ANNOTATION,
        TYPE_ANNOTATION,
        MODIFIERS,
        ANNOTATED_TYPE,
        ERRONEOUS,
        POS,
        NEG,
        NOT,
        COMPL,
        PREINC,
        PREDEC,
        POSTINC,
        POSTDEC,
        NULLCHK,
        OR,
        AND,
        BITOR,
        BITXOR,
        BITAND,
        EQ,
        NE,
        LT,
        GT,
        LE,
        GE,
        SL,
        SR,
        USR,
        PLUS,
        MINUS,
        MUL,
        DIV,
        MOD,
        BITOR_ASG(BITOR),
        BITXOR_ASG(BITXOR),
        BITAND_ASG(BITAND),
        SL_ASG(SL),
        SR_ASG(SR),
        USR_ASG(USR),
        PLUS_ASG(PLUS),
        MINUS_ASG(MINUS),
        MUL_ASG(MUL),
        DIV_ASG(DIV),
        MOD_ASG(MOD),
        LETEXPR;

        private final Tag noAssignTag;
        private static final int numberOfOperators = (MOD.ordinal() - POS.ordinal()) + 1;

        Tag(Tag noAssignTag) {
            this.noAssignTag = noAssignTag;
        }

        Tag() {
            this(null);
        }

        public static int getNumberOfOperators() {
            return numberOfOperators;
        }

        public Tag noAssignOp() {
            if (this.noAssignTag != null) {
                return this.noAssignTag;
            }
            throw new AssertionError("noAssignOp() method is not available for non assignment tags");
        }

        public boolean isPostUnaryOp() {
            return this == POSTINC || this == POSTDEC;
        }

        public boolean isIncOrDecUnaryOp() {
            return this == PREINC || this == PREDEC || this == POSTINC || this == POSTDEC;
        }

        public boolean isAssignop() {
            return this.noAssignTag != null;
        }

        public int operatorIndex() {
            return ordinal() - POS.ordinal();
        }
    }

    public boolean hasTag(Tag tag) {
        return tag == getTag();
    }

    public String toString() {
        StringWriter s = new StringWriter();
        try {
            new Pretty(s, false).printExpr(this);
            return s.toString();
        } catch (IOException e) {
            throw new AssertionError(e);
        }
    }

    public JCTree setPos(int pos) {
        this.pos = pos;
        return this;
    }

    public JCTree setType(Type type) {
        this.type = type;
        return this;
    }

    public Object clone() {
        try {
            return super.clone();
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException(e);
        }
    }

    public JCDiagnostic.DiagnosticPosition pos() {
        return this;
    }

    @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
    public JCTree getTree() {
        return this;
    }

    @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
    public int getStartPosition() {
        return TreeInfo.getStartPos(this);
    }

    @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
    public int getPreferredPosition() {
        return this.pos;
    }

    @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
    public int getEndPosition(EndPosTable endPosTable) {
        return TreeInfo.getEndPos(this, endPosTable);
    }

    public static class JCCompilationUnit extends JCTree implements CompilationUnitTree {
        public List<JCTree> defs;
        public Scope.ImportScope namedImportScope;
        public List<JCAnnotation> packageAnnotations;
        public Symbol.PackageSymbol packge;
        public JCExpression pid;
        public JavaFileObject sourcefile;
        public Scope.StarImportScope starImportScope;
        public Position.LineMap lineMap = null;
        public DocCommentTable docComments = null;
        public EndPosTable endPositions = null;

        protected JCCompilationUnit(List<JCAnnotation> packageAnnotations, JCExpression pid, List<JCTree> defs, JavaFileObject sourcefile, Symbol.PackageSymbol packge, Scope.ImportScope namedImportScope, Scope.StarImportScope starImportScope) {
            this.packageAnnotations = packageAnnotations;
            this.pid = pid;
            this.defs = defs;
            this.sourcefile = sourcefile;
            this.packge = packge;
            this.namedImportScope = namedImportScope;
            this.starImportScope = starImportScope;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTopLevel(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.COMPILATION_UNIT;
        }

        @Override // com.sun.source.tree.CompilationUnitTree
        public List<JCAnnotation> getPackageAnnotations() {
            return this.packageAnnotations;
        }

        @Override // com.sun.source.tree.CompilationUnitTree
        public List<JCImport> getImports() {
            ListBuffer<JCImport> imports = new ListBuffer<>();
            for (JCTree tree : this.defs) {
                if (tree.hasTag(Tag.IMPORT)) {
                    imports.append((JCImport) tree);
                } else if (!tree.hasTag(Tag.SKIP)) {
                    break;
                }
            }
            return imports.toList();
        }

        @Override // com.sun.source.tree.CompilationUnitTree
        public JCExpression getPackageName() {
            return this.pid;
        }

        @Override // com.sun.source.tree.CompilationUnitTree
        public JavaFileObject getSourceFile() {
            return this.sourcefile;
        }

        @Override // com.sun.source.tree.CompilationUnitTree
        public Position.LineMap getLineMap() {
            return this.lineMap;
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.source.tree.CompilationUnitTree
        public List<JCTree> getTypeDecls() {
            List list = this.defs;
            while (!list.isEmpty() && ((JCTree) list.head).hasTag(Tag.IMPORT)) {
                list = list.tail;
            }
            return list;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitCompilationUnit(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TOPLEVEL;
        }
    }

    public static class JCImport extends JCTree implements ImportTree {
        public JCTree qualid;
        public boolean staticImport;

        protected JCImport(JCTree qualid, boolean importStatic) {
            this.qualid = qualid;
            this.staticImport = importStatic;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitImport(this);
        }

        @Override // com.sun.source.tree.ImportTree
        public boolean isStatic() {
            return this.staticImport;
        }

        @Override // com.sun.source.tree.ImportTree
        public JCTree getQualifiedIdentifier() {
            return this.qualid;
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.IMPORT;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitImport(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.IMPORT;
        }
    }

    public static abstract class JCStatement extends JCTree implements StatementTree {
        @Override // com.sun.tools.javac.tree.JCTree
        public JCStatement setType(Type type) {
            super.setType(type);
            return this;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public JCStatement setPos(int pos) {
            super.setPos(pos);
            return this;
        }
    }

    public static abstract class JCExpression extends JCTree implements ExpressionTree {
        @Override // com.sun.tools.javac.tree.JCTree
        public JCExpression setType(Type type) {
            super.setType(type);
            return this;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public JCExpression setPos(int pos) {
            super.setPos(pos);
            return this;
        }

        public boolean isPoly() {
            return false;
        }

        public boolean isStandalone() {
            return true;
        }
    }

    public static abstract class JCPolyExpression extends JCExpression {
        public PolyKind polyKind;

        public enum PolyKind {
            STANDALONE,
            POLY
        }

        @Override // com.sun.tools.javac.tree.JCTree.JCExpression
        public boolean isPoly() {
            return this.polyKind == PolyKind.POLY;
        }

        @Override // com.sun.tools.javac.tree.JCTree.JCExpression
        public boolean isStandalone() {
            return this.polyKind == PolyKind.STANDALONE;
        }
    }

    public static abstract class JCFunctionalExpression extends JCPolyExpression {
        public List<Type> targets;

        public JCFunctionalExpression() {
            this.polyKind = JCPolyExpression.PolyKind.POLY;
        }

        public Type getDescriptorType(Types types) {
            return this.targets.nonEmpty() ? types.findDescriptorType(this.targets.head) : types.createErrorType(null);
        }
    }

    public static class JCClassDecl extends JCStatement implements ClassTree {
        public List<JCTree> defs;
        public JCExpression extending;
        public List<JCExpression> implementing;
        public JCModifiers mods;
        public Name name;
        public Symbol.ClassSymbol sym;
        public List<JCTypeParameter> typarams;

        protected JCClassDecl(JCModifiers mods, Name name, List<JCTypeParameter> typarams, JCExpression extending, List<JCExpression> implementing, List<JCTree> defs, Symbol.ClassSymbol sym) {
            this.mods = mods;
            this.name = name;
            this.typarams = typarams;
            this.extending = extending;
            this.implementing = implementing;
            this.defs = defs;
            this.sym = sym;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitClassDef(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            if ((this.mods.flags & 8192) != 0) {
                return Tree.Kind.ANNOTATION_TYPE;
            }
            if ((this.mods.flags & 512) != 0) {
                return Tree.Kind.INTERFACE;
            }
            if ((this.mods.flags & 16384) != 0) {
                return Tree.Kind.ENUM;
            }
            return Tree.Kind.CLASS;
        }

        @Override // com.sun.source.tree.ClassTree
        public JCModifiers getModifiers() {
            return this.mods;
        }

        @Override // com.sun.source.tree.ClassTree
        public Name getSimpleName() {
            return this.name;
        }

        @Override // com.sun.source.tree.ClassTree
        public List<JCTypeParameter> getTypeParameters() {
            return this.typarams;
        }

        @Override // com.sun.source.tree.ClassTree
        public JCExpression getExtendsClause() {
            return this.extending;
        }

        @Override // com.sun.source.tree.ClassTree
        public List<JCExpression> getImplementsClause() {
            return this.implementing;
        }

        @Override // com.sun.source.tree.ClassTree
        public List<JCTree> getMembers() {
            return this.defs;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitClass(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.CLASSDEF;
        }
    }

    public static class JCMethodDecl extends JCTree implements MethodTree {
        public JCBlock body;
        public JCExpression defaultValue;
        public JCModifiers mods;
        public Name name;
        public List<JCVariableDecl> params;
        public JCVariableDecl recvparam;
        public JCExpression restype;
        public Symbol.MethodSymbol sym;
        public List<JCExpression> thrown;
        public List<JCTypeParameter> typarams;

        protected JCMethodDecl(JCModifiers mods, Name name, JCExpression restype, List<JCTypeParameter> typarams, JCVariableDecl recvparam, List<JCVariableDecl> params, List<JCExpression> thrown, JCBlock body, JCExpression defaultValue, Symbol.MethodSymbol sym) {
            this.mods = mods;
            this.name = name;
            this.restype = restype;
            this.typarams = typarams;
            this.params = params;
            this.recvparam = recvparam;
            this.thrown = thrown;
            this.body = body;
            this.defaultValue = defaultValue;
            this.sym = sym;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitMethodDef(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.METHOD;
        }

        @Override // com.sun.source.tree.MethodTree
        public JCModifiers getModifiers() {
            return this.mods;
        }

        @Override // com.sun.source.tree.MethodTree
        public Name getName() {
            return this.name;
        }

        @Override // com.sun.source.tree.MethodTree
        public JCTree getReturnType() {
            return this.restype;
        }

        @Override // com.sun.source.tree.MethodTree
        public List<JCTypeParameter> getTypeParameters() {
            return this.typarams;
        }

        @Override // com.sun.source.tree.MethodTree
        public List<JCVariableDecl> getParameters() {
            return this.params;
        }

        @Override // com.sun.source.tree.MethodTree
        public JCVariableDecl getReceiverParameter() {
            return this.recvparam;
        }

        @Override // com.sun.source.tree.MethodTree
        public List<JCExpression> getThrows() {
            return this.thrown;
        }

        @Override // com.sun.source.tree.MethodTree
        public JCBlock getBody() {
            return this.body;
        }

        @Override // com.sun.source.tree.MethodTree
        public JCTree getDefaultValue() {
            return this.defaultValue;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitMethod(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.METHODDEF;
        }
    }

    public static class JCVariableDecl extends JCStatement implements VariableTree {
        public JCExpression init;
        public JCModifiers mods;
        public Name name;
        public JCExpression nameexpr;
        public Symbol.VarSymbol sym;
        public JCExpression vartype;

        protected JCVariableDecl(JCModifiers mods, Name name, JCExpression vartype, JCExpression init, Symbol.VarSymbol sym) {
            this.mods = mods;
            this.name = name;
            this.vartype = vartype;
            this.init = init;
            this.sym = sym;
        }

        protected JCVariableDecl(JCModifiers mods, JCExpression nameexpr, JCExpression vartype) {
            this(mods, null, vartype, null, null);
            this.nameexpr = nameexpr;
            if (nameexpr.hasTag(Tag.IDENT)) {
                this.name = ((JCIdent) nameexpr).name;
            } else {
                this.name = ((JCFieldAccess) nameexpr).name;
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitVarDef(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.VARIABLE;
        }

        @Override // com.sun.source.tree.VariableTree
        public JCModifiers getModifiers() {
            return this.mods;
        }

        @Override // com.sun.source.tree.VariableTree
        public Name getName() {
            return this.name;
        }

        @Override // com.sun.source.tree.VariableTree
        public JCExpression getNameExpression() {
            return this.nameexpr;
        }

        @Override // com.sun.source.tree.VariableTree
        public JCTree getType() {
            return this.vartype;
        }

        @Override // com.sun.source.tree.VariableTree
        public JCExpression getInitializer() {
            return this.init;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitVariable(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.VARDEF;
        }
    }

    public static class JCSkip extends JCStatement implements EmptyStatementTree {
        protected JCSkip() {
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitSkip(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.EMPTY_STATEMENT;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitEmptyStatement(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.SKIP;
        }
    }

    public static class JCBlock extends JCStatement implements BlockTree {
        public int endpos = -1;
        public long flags;
        public List<JCStatement> stats;

        protected JCBlock(long flags, List<JCStatement> stats) {
            this.stats = stats;
            this.flags = flags;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitBlock(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.BLOCK;
        }

        @Override // com.sun.source.tree.BlockTree
        public List<JCStatement> getStatements() {
            return this.stats;
        }

        @Override // com.sun.source.tree.BlockTree
        public boolean isStatic() {
            return (this.flags & 8) != 0;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitBlock(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.BLOCK;
        }
    }

    public static class JCDoWhileLoop extends JCStatement implements DoWhileLoopTree {
        public JCStatement body;
        public JCExpression cond;

        protected JCDoWhileLoop(JCStatement body, JCExpression cond) {
            this.body = body;
            this.cond = cond;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitDoLoop(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.DO_WHILE_LOOP;
        }

        @Override // com.sun.source.tree.DoWhileLoopTree
        public JCExpression getCondition() {
            return this.cond;
        }

        @Override // com.sun.source.tree.DoWhileLoopTree
        public JCStatement getStatement() {
            return this.body;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitDoWhileLoop(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.DOLOOP;
        }
    }

    public static class JCWhileLoop extends JCStatement implements WhileLoopTree {
        public JCStatement body;
        public JCExpression cond;

        protected JCWhileLoop(JCExpression cond, JCStatement body) {
            this.cond = cond;
            this.body = body;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitWhileLoop(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.WHILE_LOOP;
        }

        @Override // com.sun.source.tree.WhileLoopTree
        public JCExpression getCondition() {
            return this.cond;
        }

        @Override // com.sun.source.tree.WhileLoopTree
        public JCStatement getStatement() {
            return this.body;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitWhileLoop(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.WHILELOOP;
        }
    }

    public static class JCForLoop extends JCStatement implements ForLoopTree {
        public JCStatement body;
        public JCExpression cond;
        public List<JCStatement> init;
        public List<JCExpressionStatement> step;

        protected JCForLoop(List<JCStatement> init, JCExpression cond, List<JCExpressionStatement> update, JCStatement body) {
            this.init = init;
            this.cond = cond;
            this.step = update;
            this.body = body;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitForLoop(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.FOR_LOOP;
        }

        @Override // com.sun.source.tree.ForLoopTree
        public JCExpression getCondition() {
            return this.cond;
        }

        @Override // com.sun.source.tree.ForLoopTree
        public JCStatement getStatement() {
            return this.body;
        }

        @Override // com.sun.source.tree.ForLoopTree
        public List<JCStatement> getInitializer() {
            return this.init;
        }

        @Override // com.sun.source.tree.ForLoopTree
        public List<JCExpressionStatement> getUpdate() {
            return this.step;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitForLoop(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.FORLOOP;
        }
    }

    public static class JCEnhancedForLoop extends JCStatement implements EnhancedForLoopTree {
        public JCStatement body;
        public JCExpression expr;
        public JCVariableDecl var;

        protected JCEnhancedForLoop(JCVariableDecl var, JCExpression expr, JCStatement body) {
            this.var = var;
            this.expr = expr;
            this.body = body;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitForeachLoop(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.ENHANCED_FOR_LOOP;
        }

        @Override // com.sun.source.tree.EnhancedForLoopTree
        public JCVariableDecl getVariable() {
            return this.var;
        }

        @Override // com.sun.source.tree.EnhancedForLoopTree
        public JCExpression getExpression() {
            return this.expr;
        }

        @Override // com.sun.source.tree.EnhancedForLoopTree
        public JCStatement getStatement() {
            return this.body;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitEnhancedForLoop(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.FOREACHLOOP;
        }
    }

    public static class JCLabeledStatement extends JCStatement implements LabeledStatementTree {
        public JCStatement body;
        public Name label;

        protected JCLabeledStatement(Name label, JCStatement body) {
            this.label = label;
            this.body = body;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitLabelled(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.LABELED_STATEMENT;
        }

        @Override // com.sun.source.tree.LabeledStatementTree
        public Name getLabel() {
            return this.label;
        }

        @Override // com.sun.source.tree.LabeledStatementTree
        public JCStatement getStatement() {
            return this.body;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitLabeledStatement(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.LABELLED;
        }
    }

    public static class JCSwitch extends JCStatement implements SwitchTree {
        public List<JCCase> cases;
        public JCExpression selector;

        protected JCSwitch(JCExpression selector, List<JCCase> cases) {
            this.selector = selector;
            this.cases = cases;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitSwitch(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.SWITCH;
        }

        @Override // com.sun.source.tree.SwitchTree
        public JCExpression getExpression() {
            return this.selector;
        }

        @Override // com.sun.source.tree.SwitchTree
        public List<JCCase> getCases() {
            return this.cases;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitSwitch(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.SWITCH;
        }
    }

    public static class JCCase extends JCStatement implements CaseTree {
        public JCExpression pat;
        public List<JCStatement> stats;

        protected JCCase(JCExpression pat, List<JCStatement> stats) {
            this.pat = pat;
            this.stats = stats;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitCase(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.CASE;
        }

        @Override // com.sun.source.tree.CaseTree
        public JCExpression getExpression() {
            return this.pat;
        }

        @Override // com.sun.source.tree.CaseTree
        public List<JCStatement> getStatements() {
            return this.stats;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitCase(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.CASE;
        }
    }

    public static class JCSynchronized extends JCStatement implements SynchronizedTree {
        public JCBlock body;
        public JCExpression lock;

        protected JCSynchronized(JCExpression lock, JCBlock body) {
            this.lock = lock;
            this.body = body;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitSynchronized(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.SYNCHRONIZED;
        }

        @Override // com.sun.source.tree.SynchronizedTree
        public JCExpression getExpression() {
            return this.lock;
        }

        @Override // com.sun.source.tree.SynchronizedTree
        public JCBlock getBlock() {
            return this.body;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitSynchronized(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.SYNCHRONIZED;
        }
    }

    public static class JCTry extends JCStatement implements TryTree {
        public JCBlock body;
        public List<JCCatch> catchers;
        public JCBlock finalizer;
        public boolean finallyCanCompleteNormally;
        public List<JCTree> resources;

        protected JCTry(List<JCTree> resources, JCBlock body, List<JCCatch> catchers, JCBlock finalizer) {
            this.body = body;
            this.catchers = catchers;
            this.finalizer = finalizer;
            this.resources = resources;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTry(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.TRY;
        }

        @Override // com.sun.source.tree.TryTree
        public JCBlock getBlock() {
            return this.body;
        }

        @Override // com.sun.source.tree.TryTree
        public List<JCCatch> getCatches() {
            return this.catchers;
        }

        @Override // com.sun.source.tree.TryTree
        public JCBlock getFinallyBlock() {
            return this.finalizer;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitTry(this, d);
        }

        @Override // com.sun.source.tree.TryTree
        public List<JCTree> getResources() {
            return this.resources;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TRY;
        }
    }

    public static class JCCatch extends JCTree implements CatchTree {
        public JCBlock body;
        public JCVariableDecl param;

        protected JCCatch(JCVariableDecl param, JCBlock body) {
            this.param = param;
            this.body = body;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitCatch(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.CATCH;
        }

        @Override // com.sun.source.tree.CatchTree
        public JCVariableDecl getParameter() {
            return this.param;
        }

        @Override // com.sun.source.tree.CatchTree
        public JCBlock getBlock() {
            return this.body;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitCatch(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.CATCH;
        }
    }

    public static class JCConditional extends JCPolyExpression implements ConditionalExpressionTree {
        public JCExpression cond;
        public JCExpression falsepart;
        public JCExpression truepart;

        protected JCConditional(JCExpression cond, JCExpression truepart, JCExpression falsepart) {
            this.cond = cond;
            this.truepart = truepart;
            this.falsepart = falsepart;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitConditional(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.CONDITIONAL_EXPRESSION;
        }

        @Override // com.sun.source.tree.ConditionalExpressionTree
        public JCExpression getCondition() {
            return this.cond;
        }

        @Override // com.sun.source.tree.ConditionalExpressionTree
        public JCExpression getTrueExpression() {
            return this.truepart;
        }

        @Override // com.sun.source.tree.ConditionalExpressionTree
        public JCExpression getFalseExpression() {
            return this.falsepart;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitConditionalExpression(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.CONDEXPR;
        }
    }

    public static class JCIf extends JCStatement implements IfTree {
        public JCExpression cond;
        public JCStatement elsepart;
        public JCStatement thenpart;

        protected JCIf(JCExpression cond, JCStatement thenpart, JCStatement elsepart) {
            this.cond = cond;
            this.thenpart = thenpart;
            this.elsepart = elsepart;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitIf(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.IF;
        }

        @Override // com.sun.source.tree.IfTree
        public JCExpression getCondition() {
            return this.cond;
        }

        @Override // com.sun.source.tree.IfTree
        public JCStatement getThenStatement() {
            return this.thenpart;
        }

        @Override // com.sun.source.tree.IfTree
        public JCStatement getElseStatement() {
            return this.elsepart;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitIf(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.IF;
        }
    }

    public static class JCExpressionStatement extends JCStatement implements ExpressionStatementTree {
        public JCExpression expr;

        protected JCExpressionStatement(JCExpression expr) {
            this.expr = expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitExec(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.EXPRESSION_STATEMENT;
        }

        @Override // com.sun.source.tree.ExpressionStatementTree
        public JCExpression getExpression() {
            return this.expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitExpressionStatement(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.EXEC;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public String toString() {
            StringWriter s = new StringWriter();
            try {
                new Pretty(s, false).printStat(this);
                return s.toString();
            } catch (IOException e) {
                throw new AssertionError(e);
            }
        }
    }

    public static class JCBreak extends JCStatement implements BreakTree {
        public Name label;
        public JCTree target;

        protected JCBreak(Name label, JCTree target) {
            this.label = label;
            this.target = target;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitBreak(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.BREAK;
        }

        @Override // com.sun.source.tree.BreakTree
        public Name getLabel() {
            return this.label;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitBreak(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.BREAK;
        }
    }

    public static class JCContinue extends JCStatement implements ContinueTree {
        public Name label;
        public JCTree target;

        protected JCContinue(Name label, JCTree target) {
            this.label = label;
            this.target = target;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitContinue(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.CONTINUE;
        }

        @Override // com.sun.source.tree.ContinueTree
        public Name getLabel() {
            return this.label;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitContinue(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.CONTINUE;
        }
    }

    public static class JCReturn extends JCStatement implements ReturnTree {
        public JCExpression expr;

        protected JCReturn(JCExpression expr) {
            this.expr = expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitReturn(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.RETURN;
        }

        @Override // com.sun.source.tree.ReturnTree
        public JCExpression getExpression() {
            return this.expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitReturn(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.RETURN;
        }
    }

    public static class JCThrow extends JCStatement implements ThrowTree {
        public JCExpression expr;

        protected JCThrow(JCExpression expr) {
            this.expr = expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitThrow(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.THROW;
        }

        @Override // com.sun.source.tree.ThrowTree
        public JCExpression getExpression() {
            return this.expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitThrow(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.THROW;
        }
    }

    public static class JCAssert extends JCStatement implements AssertTree {
        public JCExpression cond;
        public JCExpression detail;

        protected JCAssert(JCExpression cond, JCExpression detail) {
            this.cond = cond;
            this.detail = detail;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitAssert(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.ASSERT;
        }

        @Override // com.sun.source.tree.AssertTree
        public JCExpression getCondition() {
            return this.cond;
        }

        @Override // com.sun.source.tree.AssertTree
        public JCExpression getDetail() {
            return this.detail;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitAssert(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.ASSERT;
        }
    }

    public static class JCMethodInvocation extends JCPolyExpression implements MethodInvocationTree {
        public List<JCExpression> args;
        public JCExpression meth;
        public List<JCExpression> typeargs;
        public Type varargsElement;

        protected JCMethodInvocation(List<JCExpression> typeargs, JCExpression meth, List<JCExpression> args) {
            this.typeargs = typeargs == null ? List.nil() : typeargs;
            this.meth = meth;
            this.args = args;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitApply(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.METHOD_INVOCATION;
        }

        @Override // com.sun.source.tree.MethodInvocationTree
        public List<JCExpression> getTypeArguments() {
            return this.typeargs;
        }

        @Override // com.sun.source.tree.MethodInvocationTree
        public JCExpression getMethodSelect() {
            return this.meth;
        }

        @Override // com.sun.source.tree.MethodInvocationTree
        public List<JCExpression> getArguments() {
            return this.args;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitMethodInvocation(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree.JCExpression, com.sun.tools.javac.tree.JCTree
        public JCMethodInvocation setType(Type type) {
            super.setType(type);
            return this;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.APPLY;
        }
    }

    public static class JCNewClass extends JCPolyExpression implements NewClassTree {
        public List<JCExpression> args;
        public JCExpression clazz;
        public Symbol constructor;
        public Type constructorType;
        public JCClassDecl def;
        public JCExpression encl;
        public List<JCExpression> typeargs;
        public Type varargsElement;

        protected JCNewClass(JCExpression encl, List<JCExpression> typeargs, JCExpression clazz, List<JCExpression> args, JCClassDecl def) {
            this.encl = encl;
            this.typeargs = typeargs == null ? List.nil() : typeargs;
            this.clazz = clazz;
            this.args = args;
            this.def = def;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitNewClass(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.NEW_CLASS;
        }

        @Override // com.sun.source.tree.NewClassTree
        public JCExpression getEnclosingExpression() {
            return this.encl;
        }

        @Override // com.sun.source.tree.NewClassTree
        public List<JCExpression> getTypeArguments() {
            return this.typeargs;
        }

        @Override // com.sun.source.tree.NewClassTree
        public JCExpression getIdentifier() {
            return this.clazz;
        }

        @Override // com.sun.source.tree.NewClassTree
        public List<JCExpression> getArguments() {
            return this.args;
        }

        @Override // com.sun.source.tree.NewClassTree
        public JCClassDecl getClassBody() {
            return this.def;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitNewClass(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.NEWCLASS;
        }
    }

    public static class JCNewArray extends JCExpression implements NewArrayTree {
        public List<JCAnnotation> annotations = List.nil();
        public List<List<JCAnnotation>> dimAnnotations = List.nil();
        public List<JCExpression> dims;
        public List<JCExpression> elems;
        public JCExpression elemtype;

        protected JCNewArray(JCExpression elemtype, List<JCExpression> dims, List<JCExpression> elems) {
            this.elemtype = elemtype;
            this.dims = dims;
            this.elems = elems;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitNewArray(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.NEW_ARRAY;
        }

        @Override // com.sun.source.tree.NewArrayTree
        public JCExpression getType() {
            return this.elemtype;
        }

        @Override // com.sun.source.tree.NewArrayTree
        public List<JCExpression> getDimensions() {
            return this.dims;
        }

        @Override // com.sun.source.tree.NewArrayTree
        public List<JCExpression> getInitializers() {
            return this.elems;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitNewArray(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.NEWARRAY;
        }

        @Override // com.sun.source.tree.NewArrayTree
        public List<JCAnnotation> getAnnotations() {
            return this.annotations;
        }

        @Override // com.sun.source.tree.NewArrayTree
        public List<List<JCAnnotation>> getDimAnnotations() {
            return this.dimAnnotations;
        }
    }

    public static class JCLambda extends JCFunctionalExpression implements LambdaExpressionTree {
        public JCTree body;
        public boolean canCompleteNormally = true;
        public ParameterKind paramKind;
        public List<JCVariableDecl> params;

        public enum ParameterKind {
            IMPLICIT,
            EXPLICIT
        }

        public JCLambda(List<JCVariableDecl> params, JCTree body) {
            this.params = params;
            this.body = body;
            if (params.isEmpty() || params.head.vartype != null) {
                this.paramKind = ParameterKind.EXPLICIT;
            } else {
                this.paramKind = ParameterKind.IMPLICIT;
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.LAMBDA;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitLambda(this);
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitLambdaExpression(this, d);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.LAMBDA_EXPRESSION;
        }

        @Override // com.sun.source.tree.LambdaExpressionTree
        public JCTree getBody() {
            return this.body;
        }

        @Override // com.sun.source.tree.LambdaExpressionTree
        public java.util.List<? extends VariableTree> getParameters() {
            return this.params;
        }

        @Override // com.sun.tools.javac.tree.JCTree.JCExpression, com.sun.tools.javac.tree.JCTree
        public JCLambda setType(Type type) {
            super.setType(type);
            return this;
        }

        @Override // com.sun.source.tree.LambdaExpressionTree
        public LambdaExpressionTree.BodyKind getBodyKind() {
            return this.body.hasTag(Tag.BLOCK) ? LambdaExpressionTree.BodyKind.STATEMENT : LambdaExpressionTree.BodyKind.EXPRESSION;
        }
    }

    public static class JCParens extends JCExpression implements ParenthesizedTree {
        public JCExpression expr;

        protected JCParens(JCExpression expr) {
            this.expr = expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitParens(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.PARENTHESIZED;
        }

        @Override // com.sun.source.tree.ParenthesizedTree
        public JCExpression getExpression() {
            return this.expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitParenthesized(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.PARENS;
        }
    }

    public static class JCAssign extends JCExpression implements AssignmentTree {
        public JCExpression lhs;
        public JCExpression rhs;

        protected JCAssign(JCExpression lhs, JCExpression rhs) {
            this.lhs = lhs;
            this.rhs = rhs;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitAssign(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.ASSIGNMENT;
        }

        @Override // com.sun.source.tree.AssignmentTree
        public JCExpression getVariable() {
            return this.lhs;
        }

        @Override // com.sun.source.tree.AssignmentTree
        public JCExpression getExpression() {
            return this.rhs;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitAssignment(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.ASSIGN;
        }
    }

    public static class JCAssignOp extends JCExpression implements CompoundAssignmentTree {
        public JCExpression lhs;
        private Tag opcode;
        public Symbol operator;
        public JCExpression rhs;

        protected JCAssignOp(Tag opcode, JCTree lhs, JCTree rhs, Symbol operator) {
            this.opcode = opcode;
            this.lhs = (JCExpression) lhs;
            this.rhs = (JCExpression) rhs;
            this.operator = operator;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitAssignop(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return TreeInfo.tagToKind(getTag());
        }

        @Override // com.sun.source.tree.CompoundAssignmentTree
        public JCExpression getVariable() {
            return this.lhs;
        }

        @Override // com.sun.source.tree.CompoundAssignmentTree
        public JCExpression getExpression() {
            return this.rhs;
        }

        public Symbol getOperator() {
            return this.operator;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitCompoundAssignment(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return this.opcode;
        }
    }

    public static class JCUnary extends JCExpression implements UnaryTree {
        public JCExpression arg;
        private Tag opcode;
        public Symbol operator;

        protected JCUnary(Tag opcode, JCExpression arg) {
            this.opcode = opcode;
            this.arg = arg;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitUnary(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return TreeInfo.tagToKind(getTag());
        }

        @Override // com.sun.source.tree.UnaryTree
        public JCExpression getExpression() {
            return this.arg;
        }

        public Symbol getOperator() {
            return this.operator;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitUnary(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return this.opcode;
        }

        public void setTag(Tag tag) {
            this.opcode = tag;
        }
    }

    public static class JCBinary extends JCExpression implements BinaryTree {
        public JCExpression lhs;
        private Tag opcode;
        public Symbol operator;
        public JCExpression rhs;

        protected JCBinary(Tag opcode, JCExpression lhs, JCExpression rhs, Symbol operator) {
            this.opcode = opcode;
            this.lhs = lhs;
            this.rhs = rhs;
            this.operator = operator;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitBinary(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return TreeInfo.tagToKind(getTag());
        }

        @Override // com.sun.source.tree.BinaryTree
        public JCExpression getLeftOperand() {
            return this.lhs;
        }

        @Override // com.sun.source.tree.BinaryTree
        public JCExpression getRightOperand() {
            return this.rhs;
        }

        public Symbol getOperator() {
            return this.operator;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitBinary(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return this.opcode;
        }
    }

    public static class JCTypeCast extends JCExpression implements TypeCastTree {
        public JCTree clazz;
        public JCExpression expr;

        protected JCTypeCast(JCTree clazz, JCExpression expr) {
            this.clazz = clazz;
            this.expr = expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTypeCast(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.TYPE_CAST;
        }

        @Override // com.sun.source.tree.TypeCastTree
        public JCTree getType() {
            return this.clazz;
        }

        @Override // com.sun.source.tree.TypeCastTree
        public JCExpression getExpression() {
            return this.expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitTypeCast(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TYPECAST;
        }
    }

    public static class JCInstanceOf extends JCExpression implements InstanceOfTree {
        public JCTree clazz;
        public JCExpression expr;

        protected JCInstanceOf(JCExpression expr, JCTree clazz) {
            this.expr = expr;
            this.clazz = clazz;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTypeTest(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.INSTANCE_OF;
        }

        @Override // com.sun.source.tree.InstanceOfTree
        public JCTree getType() {
            return this.clazz;
        }

        @Override // com.sun.source.tree.InstanceOfTree
        public JCExpression getExpression() {
            return this.expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitInstanceOf(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TYPETEST;
        }
    }

    public static class JCArrayAccess extends JCExpression implements ArrayAccessTree {
        public JCExpression index;
        public JCExpression indexed;

        protected JCArrayAccess(JCExpression indexed, JCExpression index) {
            this.indexed = indexed;
            this.index = index;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitIndexed(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.ARRAY_ACCESS;
        }

        @Override // com.sun.source.tree.ArrayAccessTree
        public JCExpression getExpression() {
            return this.indexed;
        }

        @Override // com.sun.source.tree.ArrayAccessTree
        public JCExpression getIndex() {
            return this.index;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitArrayAccess(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.INDEXED;
        }
    }

    public static class JCFieldAccess extends JCExpression implements MemberSelectTree {
        public Name name;
        public JCExpression selected;
        public Symbol sym;

        protected JCFieldAccess(JCExpression selected, Name name, Symbol sym) {
            this.selected = selected;
            this.name = name;
            this.sym = sym;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitSelect(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.MEMBER_SELECT;
        }

        @Override // com.sun.source.tree.MemberSelectTree
        public JCExpression getExpression() {
            return this.selected;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitMemberSelect(this, d);
        }

        @Override // com.sun.source.tree.MemberSelectTree
        public Name getIdentifier() {
            return this.name;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.SELECT;
        }
    }

    public static class JCMemberReference extends JCFunctionalExpression implements MemberReferenceTree {
        public JCExpression expr;
        public ReferenceKind kind;
        public MemberReferenceTree.ReferenceMode mode;
        public Name name;
        public OverloadKind overloadKind;
        public boolean ownerAccessible;
        public JCPolyExpression.PolyKind refPolyKind;
        public Symbol sym;
        public List<JCExpression> typeargs;
        public Type varargsElement;

        public enum OverloadKind {
            OVERLOADED,
            UNOVERLOADED
        }

        public enum ReferenceKind {
            SUPER(MemberReferenceTree.ReferenceMode.INVOKE, false),
            UNBOUND(MemberReferenceTree.ReferenceMode.INVOKE, true),
            STATIC(MemberReferenceTree.ReferenceMode.INVOKE, false),
            BOUND(MemberReferenceTree.ReferenceMode.INVOKE, false),
            IMPLICIT_INNER(MemberReferenceTree.ReferenceMode.NEW, false),
            TOPLEVEL(MemberReferenceTree.ReferenceMode.NEW, false),
            ARRAY_CTOR(MemberReferenceTree.ReferenceMode.NEW, false);

            final MemberReferenceTree.ReferenceMode mode;
            final boolean unbound;

            ReferenceKind(MemberReferenceTree.ReferenceMode mode, boolean unbound) {
                this.mode = mode;
                this.unbound = unbound;
            }

            public boolean isUnbound() {
                return this.unbound;
            }
        }

        protected JCMemberReference(MemberReferenceTree.ReferenceMode mode, Name name, JCExpression expr, List<JCExpression> typeargs) {
            this.mode = mode;
            this.name = name;
            this.expr = expr;
            this.typeargs = typeargs;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitReference(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.MEMBER_REFERENCE;
        }

        @Override // com.sun.source.tree.MemberReferenceTree
        public MemberReferenceTree.ReferenceMode getMode() {
            return this.mode;
        }

        @Override // com.sun.source.tree.MemberReferenceTree
        public JCExpression getQualifierExpression() {
            return this.expr;
        }

        @Override // com.sun.source.tree.MemberReferenceTree
        public Name getName() {
            return this.name;
        }

        @Override // com.sun.source.tree.MemberReferenceTree
        public List<JCExpression> getTypeArguments() {
            return this.typeargs;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitMemberReference(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.REFERENCE;
        }

        public boolean hasKind(ReferenceKind kind) {
            return this.kind == kind;
        }
    }

    public static class JCIdent extends JCExpression implements IdentifierTree {
        public Name name;
        public Symbol sym;

        protected JCIdent(Name name, Symbol sym) {
            this.name = name;
            this.sym = sym;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitIdent(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.IDENTIFIER;
        }

        @Override // com.sun.source.tree.IdentifierTree
        public Name getName() {
            return this.name;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitIdentifier(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.IDENT;
        }
    }

    public static class JCLiteral extends JCExpression implements LiteralTree {
        public TypeTag typetag;
        public Object value;

        protected JCLiteral(TypeTag typetag, Object value) {
            this.typetag = typetag;
            this.value = value;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitLiteral(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return this.typetag.getKindLiteral();
        }

        @Override // com.sun.source.tree.LiteralTree
        public Object getValue() {
            switch (this.typetag) {
                case BOOLEAN:
                    int bi = ((Integer) this.value).intValue();
                    return Boolean.valueOf(bi != 0);
                case CHAR:
                    int ci = ((Integer) this.value).intValue();
                    char c = (char) ci;
                    if (c != ci) {
                        throw new AssertionError("bad value for char literal");
                    }
                    return Character.valueOf(c);
                default:
                    return this.value;
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitLiteral(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree.JCExpression, com.sun.tools.javac.tree.JCTree
        public JCLiteral setType(Type type) {
            super.setType(type);
            return this;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.LITERAL;
        }
    }

    public static class JCPrimitiveTypeTree extends JCExpression implements PrimitiveTypeTree {
        public TypeTag typetag;

        protected JCPrimitiveTypeTree(TypeTag typetag) {
            this.typetag = typetag;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTypeIdent(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.PRIMITIVE_TYPE;
        }

        @Override // com.sun.source.tree.PrimitiveTypeTree
        public TypeKind getPrimitiveTypeKind() {
            return this.typetag.getPrimitiveTypeKind();
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitPrimitiveType(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TYPEIDENT;
        }
    }

    public static class JCArrayTypeTree extends JCExpression implements ArrayTypeTree {
        public JCExpression elemtype;

        protected JCArrayTypeTree(JCExpression elemtype) {
            this.elemtype = elemtype;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTypeArray(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.ARRAY_TYPE;
        }

        @Override // com.sun.source.tree.ArrayTypeTree
        public JCTree getType() {
            return this.elemtype;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitArrayType(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TYPEARRAY;
        }
    }

    public static class JCTypeApply extends JCExpression implements ParameterizedTypeTree {
        public List<JCExpression> arguments;
        public JCExpression clazz;

        protected JCTypeApply(JCExpression clazz, List<JCExpression> arguments) {
            this.clazz = clazz;
            this.arguments = arguments;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTypeApply(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.PARAMETERIZED_TYPE;
        }

        @Override // com.sun.source.tree.ParameterizedTypeTree
        public JCTree getType() {
            return this.clazz;
        }

        @Override // com.sun.source.tree.ParameterizedTypeTree
        public List<JCExpression> getTypeArguments() {
            return this.arguments;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitParameterizedType(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TYPEAPPLY;
        }
    }

    public static class JCTypeUnion extends JCExpression implements UnionTypeTree {
        public List<JCExpression> alternatives;

        protected JCTypeUnion(List<JCExpression> components) {
            this.alternatives = components;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTypeUnion(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.UNION_TYPE;
        }

        @Override // com.sun.source.tree.UnionTypeTree
        public List<JCExpression> getTypeAlternatives() {
            return this.alternatives;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitUnionType(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TYPEUNION;
        }
    }

    public static class JCTypeIntersection extends JCExpression implements IntersectionTypeTree {
        public List<JCExpression> bounds;

        protected JCTypeIntersection(List<JCExpression> bounds) {
            this.bounds = bounds;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTypeIntersection(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.INTERSECTION_TYPE;
        }

        @Override // com.sun.source.tree.IntersectionTypeTree
        public List<JCExpression> getBounds() {
            return this.bounds;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitIntersectionType(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TYPEINTERSECTION;
        }
    }

    public static class JCTypeParameter extends JCTree implements TypeParameterTree {
        public List<JCAnnotation> annotations;
        public List<JCExpression> bounds;
        public Name name;

        protected JCTypeParameter(Name name, List<JCExpression> bounds, List<JCAnnotation> annotations) {
            this.name = name;
            this.bounds = bounds;
            this.annotations = annotations;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTypeParameter(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.TYPE_PARAMETER;
        }

        @Override // com.sun.source.tree.TypeParameterTree
        public Name getName() {
            return this.name;
        }

        @Override // com.sun.source.tree.TypeParameterTree
        public List<JCExpression> getBounds() {
            return this.bounds;
        }

        @Override // com.sun.source.tree.TypeParameterTree
        public List<JCAnnotation> getAnnotations() {
            return this.annotations;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitTypeParameter(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TYPEPARAMETER;
        }
    }

    public static class JCWildcard extends JCExpression implements WildcardTree {
        public JCTree inner;
        public TypeBoundKind kind;

        protected JCWildcard(TypeBoundKind kind, JCTree inner) {
            kind.getClass();
            this.kind = kind;
            this.inner = inner;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitWildcard(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            switch (this.kind.kind) {
                case UNBOUND:
                    return Tree.Kind.UNBOUNDED_WILDCARD;
                case EXTENDS:
                    return Tree.Kind.EXTENDS_WILDCARD;
                case SUPER:
                    return Tree.Kind.SUPER_WILDCARD;
                default:
                    throw new AssertionError("Unknown wildcard bound " + this.kind);
            }
        }

        @Override // com.sun.source.tree.WildcardTree
        public JCTree getBound() {
            return this.inner;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitWildcard(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.WILDCARD;
        }
    }

    public static class TypeBoundKind extends JCTree {
        public BoundKind kind;

        protected TypeBoundKind(BoundKind kind) {
            this.kind = kind;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitTypeBoundKind(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            throw new AssertionError("TypeBoundKind is not part of a public API");
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            throw new AssertionError("TypeBoundKind is not part of a public API");
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.TYPEBOUNDKIND;
        }
    }

    public static class JCAnnotation extends JCExpression implements AnnotationTree {
        public JCTree annotationType;
        public List<JCExpression> args;
        public Attribute.Compound attribute;
        private Tag tag;

        protected JCAnnotation(Tag tag, JCTree annotationType, List<JCExpression> args) {
            this.tag = tag;
            this.annotationType = annotationType;
            this.args = args;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitAnnotation(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return TreeInfo.tagToKind(getTag());
        }

        @Override // com.sun.source.tree.AnnotationTree
        public JCTree getAnnotationType() {
            return this.annotationType;
        }

        @Override // com.sun.source.tree.AnnotationTree
        public List<JCExpression> getArguments() {
            return this.args;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitAnnotation(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return this.tag;
        }
    }

    public static class JCModifiers extends JCTree implements ModifiersTree {
        public List<JCAnnotation> annotations;
        public long flags;

        protected JCModifiers(long flags, List<JCAnnotation> annotations) {
            this.flags = flags;
            this.annotations = annotations;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitModifiers(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.MODIFIERS;
        }

        @Override // com.sun.source.tree.ModifiersTree
        public Set<Modifier> getFlags() {
            return Flags.asModifierSet(this.flags);
        }

        @Override // com.sun.source.tree.ModifiersTree
        public List<JCAnnotation> getAnnotations() {
            return this.annotations;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitModifiers(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.MODIFIERS;
        }
    }

    public static class JCAnnotatedType extends JCExpression implements AnnotatedTypeTree {
        public List<JCAnnotation> annotations;
        public JCExpression underlyingType;

        protected JCAnnotatedType(List<JCAnnotation> annotations, JCExpression underlyingType) {
            Assert.check(annotations != null && annotations.nonEmpty());
            this.annotations = annotations;
            this.underlyingType = underlyingType;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitAnnotatedType(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.ANNOTATED_TYPE;
        }

        @Override // com.sun.source.tree.AnnotatedTypeTree
        public List<JCAnnotation> getAnnotations() {
            return this.annotations;
        }

        @Override // com.sun.source.tree.AnnotatedTypeTree
        public JCExpression getUnderlyingType() {
            return this.underlyingType;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitAnnotatedType(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.ANNOTATED_TYPE;
        }
    }

    public static class JCErroneous extends JCExpression implements ErroneousTree {
        public List<? extends JCTree> errs;

        protected JCErroneous(List<? extends JCTree> errs) {
            this.errs = errs;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitErroneous(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            return Tree.Kind.ERRONEOUS;
        }

        @Override // com.sun.source.tree.ErroneousTree
        public List<? extends JCTree> getErrorTrees() {
            return this.errs;
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            return v.visitErroneous(this, d);
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.ERRONEOUS;
        }
    }

    public static class LetExpr extends JCExpression {
        public List<JCVariableDecl> defs;
        public JCTree expr;

        protected LetExpr(List<JCVariableDecl> defs, JCTree expr) {
            this.defs = defs;
            this.expr = expr;
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public void accept(Visitor v) {
            v.visitLetExpr(this);
        }

        @Override // com.sun.source.tree.Tree
        public Tree.Kind getKind() {
            throw new AssertionError("LetExpr is not part of a public API");
        }

        @Override // com.sun.tools.javac.tree.JCTree, com.sun.source.tree.Tree
        public <R, D> R accept(TreeVisitor<R, D> v, D d) {
            throw new AssertionError("LetExpr is not part of a public API");
        }

        @Override // com.sun.tools.javac.tree.JCTree
        public Tag getTag() {
            return Tag.LETEXPR;
        }
    }

    public static abstract class Visitor {
        public void visitTopLevel(JCCompilationUnit that) {
            visitTree(that);
        }

        public void visitImport(JCImport that) {
            visitTree(that);
        }

        public void visitClassDef(JCClassDecl that) {
            visitTree(that);
        }

        public void visitMethodDef(JCMethodDecl that) {
            visitTree(that);
        }

        public void visitVarDef(JCVariableDecl that) {
            visitTree(that);
        }

        public void visitSkip(JCSkip that) {
            visitTree(that);
        }

        public void visitBlock(JCBlock that) {
            visitTree(that);
        }

        public void visitDoLoop(JCDoWhileLoop that) {
            visitTree(that);
        }

        public void visitWhileLoop(JCWhileLoop that) {
            visitTree(that);
        }

        public void visitForLoop(JCForLoop that) {
            visitTree(that);
        }

        public void visitForeachLoop(JCEnhancedForLoop that) {
            visitTree(that);
        }

        public void visitLabelled(JCLabeledStatement that) {
            visitTree(that);
        }

        public void visitSwitch(JCSwitch that) {
            visitTree(that);
        }

        public void visitCase(JCCase that) {
            visitTree(that);
        }

        public void visitSynchronized(JCSynchronized that) {
            visitTree(that);
        }

        public void visitTry(JCTry that) {
            visitTree(that);
        }

        public void visitCatch(JCCatch that) {
            visitTree(that);
        }

        public void visitConditional(JCConditional that) {
            visitTree(that);
        }

        public void visitIf(JCIf that) {
            visitTree(that);
        }

        public void visitExec(JCExpressionStatement that) {
            visitTree(that);
        }

        public void visitBreak(JCBreak that) {
            visitTree(that);
        }

        public void visitContinue(JCContinue that) {
            visitTree(that);
        }

        public void visitReturn(JCReturn that) {
            visitTree(that);
        }

        public void visitThrow(JCThrow that) {
            visitTree(that);
        }

        public void visitAssert(JCAssert that) {
            visitTree(that);
        }

        public void visitApply(JCMethodInvocation that) {
            visitTree(that);
        }

        public void visitNewClass(JCNewClass that) {
            visitTree(that);
        }

        public void visitNewArray(JCNewArray that) {
            visitTree(that);
        }

        public void visitLambda(JCLambda that) {
            visitTree(that);
        }

        public void visitParens(JCParens that) {
            visitTree(that);
        }

        public void visitAssign(JCAssign that) {
            visitTree(that);
        }

        public void visitAssignop(JCAssignOp that) {
            visitTree(that);
        }

        public void visitUnary(JCUnary that) {
            visitTree(that);
        }

        public void visitBinary(JCBinary that) {
            visitTree(that);
        }

        public void visitTypeCast(JCTypeCast that) {
            visitTree(that);
        }

        public void visitTypeTest(JCInstanceOf that) {
            visitTree(that);
        }

        public void visitIndexed(JCArrayAccess that) {
            visitTree(that);
        }

        public void visitSelect(JCFieldAccess that) {
            visitTree(that);
        }

        public void visitReference(JCMemberReference that) {
            visitTree(that);
        }

        public void visitIdent(JCIdent that) {
            visitTree(that);
        }

        public void visitLiteral(JCLiteral that) {
            visitTree(that);
        }

        public void visitTypeIdent(JCPrimitiveTypeTree that) {
            visitTree(that);
        }

        public void visitTypeArray(JCArrayTypeTree that) {
            visitTree(that);
        }

        public void visitTypeApply(JCTypeApply that) {
            visitTree(that);
        }

        public void visitTypeUnion(JCTypeUnion that) {
            visitTree(that);
        }

        public void visitTypeIntersection(JCTypeIntersection that) {
            visitTree(that);
        }

        public void visitTypeParameter(JCTypeParameter that) {
            visitTree(that);
        }

        public void visitWildcard(JCWildcard that) {
            visitTree(that);
        }

        public void visitTypeBoundKind(TypeBoundKind that) {
            visitTree(that);
        }

        public void visitAnnotation(JCAnnotation that) {
            visitTree(that);
        }

        public void visitModifiers(JCModifiers that) {
            visitTree(that);
        }

        public void visitAnnotatedType(JCAnnotatedType that) {
            visitTree(that);
        }

        public void visitErroneous(JCErroneous that) {
            visitTree(that);
        }

        public void visitLetExpr(LetExpr that) {
            visitTree(that);
        }

        public void visitTree(JCTree that) {
            Assert.error();
        }
    }
}
