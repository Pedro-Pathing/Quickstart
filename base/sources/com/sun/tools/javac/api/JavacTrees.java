package com.sun.tools.javac.api;

import com.sun.source.doctree.DocCommentTree;
import com.sun.source.doctree.DocTree;
import com.sun.source.tree.CatchTree;
import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.Tree;
import com.sun.source.util.DocSourcePositions;
import com.sun.source.util.DocTreePath;
import com.sun.source.util.DocTreeScanner;
import com.sun.source.util.DocTrees;
import com.sun.source.util.JavacTask;
import com.sun.source.util.TreePath;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Enter;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.comp.MemberEnter;
import com.sun.tools.javac.comp.Resolve;
import com.sun.tools.javac.model.JavacElements;
import com.sun.tools.javac.processing.JavacProcessingEnvironment;
import com.sun.tools.javac.tree.DCTree;
import com.sun.tools.javac.tree.EndPosTable;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeCopier;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Pair;
import java.io.IOException;
import java.util.HashSet;
import java.util.Set;
import javax.annotation.processing.ProcessingEnvironment;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.AnnotationValue;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.type.DeclaredType;
import javax.lang.model.type.ErrorType;
import javax.lang.model.type.TypeKind;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class JavacTrees extends DocTrees {
    private Attr attr;
    private JavacElements elements;
    private Enter enter;
    Types.TypeRelation fuzzyMatcher = new Types.TypeRelation() { // from class: com.sun.tools.javac.api.JavacTrees.3
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Type s) {
            if (t == s) {
                return true;
            }
            if (s.isPartial()) {
                return visit(s, t);
            }
            switch (AnonymousClass4.$SwitchMap$com$sun$tools$javac$code$TypeTag[t.getTag().ordinal()]) {
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
                case 11:
                    return Boolean.valueOf(t.hasTag(s.getTag()));
                default:
                    throw new AssertionError("fuzzyMatcher " + t.getTag());
            }
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitArrayType(Type.ArrayType t, Type s) {
            if (t == s) {
                return true;
            }
            if (s.isPartial()) {
                return visit(s, t);
            }
            return Boolean.valueOf(s.hasTag(TypeTag.ARRAY) && visit(t.elemtype, JavacTrees.this.types.elemtype(s)).booleanValue());
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitClassType(Type.ClassType t, Type s) {
            if (t == s) {
                return true;
            }
            if (s.isPartial()) {
                return visit(s, t);
            }
            return Boolean.valueOf(t.tsym == s.tsym);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitErrorType(Type.ErrorType t, Type s) {
            return Boolean.valueOf(s.hasTag(TypeTag.CLASS) && t.tsym.name == ((Type.ClassType) s).tsym.name);
        }
    };
    private JavacTaskImpl javacTaskImpl;
    private Log log;
    private MemberEnter memberEnter;
    private Names names;
    private Resolve resolve;
    private TreeMaker treeMaker;
    private Types types;

    public static JavacTrees instance(JavaCompiler.CompilationTask task) {
        if (!(task instanceof BasicJavacTask)) {
            throw new IllegalArgumentException();
        }
        return instance(((BasicJavacTask) task).getContext());
    }

    public static JavacTrees instance(ProcessingEnvironment env) {
        if (!(env instanceof JavacProcessingEnvironment)) {
            throw new IllegalArgumentException();
        }
        return instance(((JavacProcessingEnvironment) env).getContext());
    }

    public static JavacTrees instance(Context context) {
        JavacTrees instance = (JavacTrees) context.get(JavacTrees.class);
        if (instance == null) {
            return new JavacTrees(context);
        }
        return instance;
    }

    protected JavacTrees(Context context) {
        context.put((Class<JavacTrees>) JavacTrees.class, this);
        init(context);
    }

    public void updateContext(Context context) {
        init(context);
    }

    private void init(Context context) {
        this.attr = Attr.instance(context);
        this.enter = Enter.instance(context);
        this.elements = JavacElements.instance(context);
        this.log = Log.instance(context);
        this.resolve = Resolve.instance(context);
        this.treeMaker = TreeMaker.instance(context);
        this.memberEnter = MemberEnter.instance(context);
        this.names = Names.instance(context);
        this.types = Types.instance(context);
        JavacTask t = (JavacTask) context.get(JavacTask.class);
        if (t instanceof JavacTaskImpl) {
            this.javacTaskImpl = (JavacTaskImpl) t;
        }
    }

    @Override // com.sun.source.util.DocTrees, com.sun.source.util.Trees
    public DocSourcePositions getSourcePositions() {
        return new DocSourcePositions() { // from class: com.sun.tools.javac.api.JavacTrees.1
            @Override // com.sun.source.util.SourcePositions
            public long getStartPosition(CompilationUnitTree file, Tree tree) {
                return TreeInfo.getStartPos((JCTree) tree);
            }

            @Override // com.sun.source.util.SourcePositions
            public long getEndPosition(CompilationUnitTree file, Tree tree) {
                EndPosTable endPosTable = ((JCTree.JCCompilationUnit) file).endPositions;
                return TreeInfo.getEndPos((JCTree) tree, endPosTable);
            }

            @Override // com.sun.source.util.DocSourcePositions
            public long getStartPosition(CompilationUnitTree file, DocCommentTree comment, DocTree tree) {
                return ((DCTree) tree).getSourcePosition((DCTree.DCDocComment) comment);
            }

            @Override // com.sun.source.util.DocSourcePositions
            public long getEndPosition(CompilationUnitTree file, DocCommentTree comment, DocTree tree) {
                int endPos;
                DCTree.DCDocComment dcComment = (DCTree.DCDocComment) comment;
                if ((tree instanceof DCTree.DCEndPosTree) && (endPos = ((DCTree.DCEndPosTree) tree).getEndPos(dcComment)) != -1) {
                    return endPos;
                }
                int correction = 0;
                switch (AnonymousClass4.$SwitchMap$com$sun$source$doctree$DocTree$Kind[tree.getKind().ordinal()]) {
                    case 1:
                        DCTree.DCText text = (DCTree.DCText) tree;
                        return dcComment.comment.getSourcePos(text.pos + text.text.length());
                    case 2:
                        DCTree.DCErroneous err = (DCTree.DCErroneous) tree;
                        return dcComment.comment.getSourcePos(err.pos + err.body.length());
                    case 3:
                        DCTree.DCIdentifier ident = (DCTree.DCIdentifier) tree;
                        return dcComment.comment.getSourcePos(ident.pos + (ident.name != JavacTrees.this.names.error ? ident.name.length() : 0));
                    case 4:
                        DCTree.DCParam param = (DCTree.DCParam) tree;
                        if (param.isTypeParameter && param.getDescription().isEmpty()) {
                            correction = 1;
                        }
                        break;
                    case 5:
                    case 6:
                    case 7:
                    case 8:
                    case 9:
                    case 10:
                    case 11:
                    case 12:
                    case 13:
                    case 14:
                    case 15:
                        break;
                    default:
                        DocTree last = JavacTrees.this.getLastChild(tree);
                        if (last != null) {
                            return getEndPosition(file, comment, last);
                        }
                        return -1L;
                }
                DocTree last2 = JavacTrees.this.getLastChild(tree);
                if (last2 != null) {
                    return getEndPosition(file, comment, last2) + ((long) correction);
                }
                DCTree.DCBlockTag block = (DCTree.DCBlockTag) tree;
                return dcComment.comment.getSourcePos(block.pos + block.getTagName().length() + 1);
            }
        };
    }

    /* JADX INFO: Access modifiers changed from: private */
    public DocTree getLastChild(DocTree tree) {
        final DocTree[] last = {null};
        tree.accept(new DocTreeScanner<Void, Void>() { // from class: com.sun.tools.javac.api.JavacTrees.2
            @Override // com.sun.source.util.DocTreeScanner
            public Void scan(DocTree node, Void p) {
                if (node != null) {
                    last[0] = node;
                    return null;
                }
                return null;
            }
        }, null);
        return last[0];
    }

    @Override // com.sun.source.util.Trees
    public JCTree.JCClassDecl getTree(TypeElement element) {
        return (JCTree.JCClassDecl) getTree((Element) element);
    }

    @Override // com.sun.source.util.Trees
    public JCTree.JCMethodDecl getTree(ExecutableElement method) {
        return (JCTree.JCMethodDecl) getTree((Element) method);
    }

    @Override // com.sun.source.util.Trees
    public JCTree getTree(Element element) {
        JCTree.JCClassDecl classNode;
        Symbol symbol = (Symbol) element;
        Symbol.TypeSymbol enclosing = symbol.enclClass();
        Env<AttrContext> env = this.enter.getEnv(enclosing);
        if (env != null && (classNode = env.enclClass) != null) {
            if (TreeInfo.symbolFor(classNode) == element) {
                return classNode;
            }
            for (JCTree node : classNode.getMembers()) {
                if (TreeInfo.symbolFor(node) == element) {
                    return node;
                }
            }
        }
        return null;
    }

    @Override // com.sun.source.util.Trees
    public JCTree getTree(Element e, AnnotationMirror a) {
        return getTree(e, a, (AnnotationValue) null);
    }

    @Override // com.sun.source.util.Trees
    public JCTree getTree(Element e, AnnotationMirror a, AnnotationValue v) {
        Pair<JCTree, JCTree.JCCompilationUnit> treeTopLevel = this.elements.getTreeAndTopLevel(e, a, v);
        if (treeTopLevel == null) {
            return null;
        }
        return treeTopLevel.fst;
    }

    @Override // com.sun.source.util.Trees
    public TreePath getPath(CompilationUnitTree unit, Tree node) {
        return TreePath.getPath(unit, node);
    }

    @Override // com.sun.source.util.Trees
    public TreePath getPath(Element e) {
        return getPath(e, null, null);
    }

    @Override // com.sun.source.util.Trees
    public TreePath getPath(Element e, AnnotationMirror a) {
        return getPath(e, a, null);
    }

    @Override // com.sun.source.util.Trees
    public TreePath getPath(Element e, AnnotationMirror a, AnnotationValue v) {
        Pair<JCTree, JCTree.JCCompilationUnit> treeTopLevel = this.elements.getTreeAndTopLevel(e, a, v);
        if (treeTopLevel == null) {
            return null;
        }
        return TreePath.getPath(treeTopLevel.snd, treeTopLevel.fst);
    }

    @Override // com.sun.source.util.Trees
    public Symbol getElement(TreePath path) {
        JCTree tree = (JCTree) path.getLeaf();
        Symbol sym = TreeInfo.symbolFor(tree);
        if (sym == null && TreeInfo.isDeclaration(tree)) {
            for (TreePath p = path; p != null; p = p.getParentPath()) {
                JCTree t = (JCTree) p.getLeaf();
                if (t.hasTag(JCTree.Tag.CLASSDEF)) {
                    JCTree.JCClassDecl ct = (JCTree.JCClassDecl) t;
                    if (ct.sym != null) {
                        if ((ct.sym.flags_field & 268435456) != 0) {
                            this.attr.attribClass(ct.pos(), ct.sym);
                            return TreeInfo.symbolFor(tree);
                        }
                        return sym;
                    }
                }
            }
            return sym;
        }
        return sym;
    }

    @Override // com.sun.source.util.DocTrees
    public Element getElement(DocTreePath path) {
        DocTree forTree = path.getLeaf();
        if (forTree instanceof DCTree.DCReference) {
            return attributeDocReference(path.getTreePath(), (DCTree.DCReference) forTree);
        }
        if ((forTree instanceof DCTree.DCIdentifier) && (path.getParentPath().getLeaf() instanceof DCTree.DCParam)) {
            return attributeParamIdentifier(path.getTreePath(), (DCTree.DCParam) path.getParentPath().getLeaf());
        }
        return null;
    }

    /* JADX WARN: Code restructure failed: missing block: B:50:0x00e3, code lost:
    
        if (r12.types.isSubtypeUnchecked(r8.enclClass().asType(), r7.enclClass().asType()) != false) goto L52;
     */
    /* JADX WARN: Multi-variable type inference failed */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private com.sun.tools.javac.code.Symbol attributeDocReference(com.sun.source.util.TreePath r13, com.sun.tools.javac.tree.DCTree.DCReference r14) {
        /*
            Method dump skipped, instruction units count: 258
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.api.JavacTrees.attributeDocReference(com.sun.source.util.TreePath, com.sun.tools.javac.tree.DCTree$DCReference):com.sun.tools.javac.code.Symbol");
    }

    /* JADX WARN: Multi-variable type inference failed */
    private Symbol attributeParamIdentifier(TreePath path, DCTree.DCParam ptag) {
        List<? extends Symbol> parameters;
        Symbol javadocSymbol = getElement(path);
        if (javadocSymbol == null) {
            return null;
        }
        ElementKind kind = javadocSymbol.getKind();
        List<? extends Symbol> params = List.nil();
        if (kind == ElementKind.METHOD || kind == ElementKind.CONSTRUCTOR) {
            Symbol.MethodSymbol ee = (Symbol.MethodSymbol) javadocSymbol;
            if (ptag.isTypeParameter()) {
                parameters = ee.getTypeParameters();
            } else {
                parameters = ee.getParameters();
            }
            params = parameters;
        } else if (kind.isClass() || kind.isInterface()) {
            params = ((Symbol.ClassSymbol) javadocSymbol).getTypeParameters();
        }
        for (Symbol param : params) {
            if (param.getSimpleName() == ptag.getName().getName()) {
                return param;
            }
        }
        return null;
    }

    private Symbol.VarSymbol findField(Symbol.ClassSymbol tsym, Name fieldName) {
        return searchField(tsym, fieldName, new HashSet());
    }

    /* JADX WARN: Multi-variable type inference failed */
    private Symbol.VarSymbol searchField(Symbol.ClassSymbol tsym, Name fieldName, Set<Symbol.ClassSymbol> searched) {
        Symbol.VarSymbol vsym;
        Symbol.VarSymbol vsym2;
        Symbol.VarSymbol vsym3;
        if (searched.contains(tsym)) {
            return null;
        }
        searched.add(tsym);
        for (Scope.Entry e = tsym.members().lookup(fieldName); e.scope != null; e = e.next()) {
            if (e.sym.kind == 4) {
                return (Symbol.VarSymbol) e.sym;
            }
        }
        Symbol.ClassSymbol encl = tsym.owner.enclClass();
        if (encl != null && (vsym3 = searchField(encl, fieldName, searched)) != null) {
            return vsym3;
        }
        Type superclass = tsym.getSuperclass();
        if (superclass.tsym != null && (vsym2 = searchField((Symbol.ClassSymbol) superclass.tsym, fieldName, searched)) != null) {
            return vsym2;
        }
        List<Type> intfs = tsym.getInterfaces();
        for (List list = intfs; list.nonEmpty(); list = list.tail) {
            Type intf = (Type) list.head;
            if (!intf.isErroneous() && (vsym = searchField((Symbol.ClassSymbol) intf.tsym, fieldName, searched)) != null) {
                return vsym;
            }
        }
        return null;
    }

    Symbol.MethodSymbol findConstructor(Symbol.ClassSymbol tsym, List<Type> paramTypes) {
        for (Scope.Entry e = tsym.members().lookup(this.names.init); e.scope != null; e = e.next()) {
            if (e.sym.kind == 16 && hasParameterTypes((Symbol.MethodSymbol) e.sym, paramTypes)) {
                return (Symbol.MethodSymbol) e.sym;
            }
        }
        return null;
    }

    private Symbol.MethodSymbol findMethod(Symbol.ClassSymbol tsym, Name methodName, List<Type> paramTypes) {
        return searchMethod(tsym, methodName, paramTypes, new HashSet());
    }

    /* JADX WARN: Multi-variable type inference failed */
    private Symbol.MethodSymbol searchMethod(Symbol.ClassSymbol tsym, Name methodName, List<Type> paramTypes, Set<Symbol.ClassSymbol> searched) {
        Symbol.MethodSymbol msym;
        Symbol.MethodSymbol msym2;
        Symbol.MethodSymbol msym3;
        if (methodName == this.names.init || searched.contains(tsym)) {
            return null;
        }
        searched.add(tsym);
        Scope.Entry e = tsym.members().lookup(methodName);
        if (paramTypes == null) {
            Symbol.MethodSymbol lastFound = null;
            while (e.scope != null) {
                if (e.sym.kind == 16 && e.sym.name == methodName) {
                    lastFound = (Symbol.MethodSymbol) e.sym;
                }
                e = e.next();
            }
            if (lastFound != null) {
                return lastFound;
            }
        } else {
            while (e.scope != null) {
                if (e.sym == null || e.sym.kind != 16 || !hasParameterTypes((Symbol.MethodSymbol) e.sym, paramTypes)) {
                    e = e.next();
                } else {
                    return (Symbol.MethodSymbol) e.sym;
                }
            }
        }
        Type superclass = tsym.getSuperclass();
        if (superclass.tsym != null && (msym3 = searchMethod((Symbol.ClassSymbol) superclass.tsym, methodName, paramTypes, searched)) != null) {
            return msym3;
        }
        List<Type> intfs = tsym.getInterfaces();
        for (List list = intfs; list.nonEmpty(); list = list.tail) {
            Type intf = (Type) list.head;
            if (!intf.isErroneous() && (msym2 = searchMethod((Symbol.ClassSymbol) intf.tsym, methodName, paramTypes, searched)) != null) {
                return msym2;
            }
        }
        Symbol.ClassSymbol encl = tsym.owner.enclClass();
        if (encl == null || (msym = searchMethod(encl, methodName, paramTypes, searched)) == null) {
            return null;
        }
        return msym;
    }

    private boolean hasParameterTypes(Symbol.MethodSymbol method, List<Type> paramTypes) {
        if (paramTypes == null) {
            return true;
        }
        if (method.params().size() != paramTypes.size()) {
            return false;
        }
        List<Type> methodParamTypes = this.types.erasureRecursive(method.asType()).mo176getParameterTypes();
        if (Type.isErroneous(paramTypes)) {
            return fuzzyMatch(paramTypes, methodParamTypes);
        }
        return this.types.isSameTypes(paramTypes, methodParamTypes);
    }

    /* JADX WARN: Multi-variable type inference failed */
    boolean fuzzyMatch(List<Type> paramTypes, List<Type> methodParamTypes) {
        List list = paramTypes;
        List list2 = methodParamTypes;
        while (list.nonEmpty()) {
            if (!fuzzyMatch((Type) list.head, (Type) list2.head)) {
                return false;
            }
            list = list.tail;
            list2 = list2.tail;
        }
        return true;
    }

    boolean fuzzyMatch(Type paramType, Type methodParamType) {
        Boolean b = this.fuzzyMatcher.visit(paramType, methodParamType);
        return b == Boolean.TRUE;
    }

    @Override // com.sun.source.util.Trees
    public TypeMirror getTypeMirror(TreePath path) {
        Tree t = path.getLeaf();
        return ((JCTree) t).type;
    }

    @Override // com.sun.source.util.Trees
    public JavacScope getScope(TreePath path) {
        return new JavacScope(getAttrContext(path));
    }

    @Override // com.sun.source.util.Trees
    public String getDocComment(TreePath path) {
        CompilationUnitTree t = path.getCompilationUnit();
        Tree leaf = path.getLeaf();
        if ((t instanceof JCTree.JCCompilationUnit) && (leaf instanceof JCTree)) {
            JCTree.JCCompilationUnit cu = (JCTree.JCCompilationUnit) t;
            if (cu.docComments != null) {
                return cu.docComments.getCommentText((JCTree) leaf);
            }
            return null;
        }
        return null;
    }

    @Override // com.sun.source.util.DocTrees
    public DocCommentTree getDocCommentTree(TreePath path) {
        CompilationUnitTree t = path.getCompilationUnit();
        Tree leaf = path.getLeaf();
        if ((t instanceof JCTree.JCCompilationUnit) && (leaf instanceof JCTree)) {
            JCTree.JCCompilationUnit cu = (JCTree.JCCompilationUnit) t;
            if (cu.docComments != null) {
                return cu.docComments.getCommentTree((JCTree) leaf);
            }
            return null;
        }
        return null;
    }

    @Override // com.sun.source.util.Trees
    public boolean isAccessible(com.sun.source.tree.Scope scope, TypeElement type) {
        if ((scope instanceof JavacScope) && (type instanceof Symbol.ClassSymbol)) {
            Env<AttrContext> env = ((JavacScope) scope).env;
            return this.resolve.isAccessible(env, (Symbol.TypeSymbol) type, true);
        }
        return false;
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.source.util.Trees
    public boolean isAccessible(com.sun.source.tree.Scope scope, Element member, DeclaredType declaredType) {
        if ((scope instanceof JavacScope) && (member instanceof Symbol) && (declaredType instanceof Type)) {
            Env<AttrContext> env = ((JavacScope) scope).env;
            return this.resolve.isAccessible(env, (Type) declaredType, (Symbol) member, true);
        }
        return false;
    }

    private Env<AttrContext> getAttrContext(TreePath path) {
        if (!(path.getLeaf() instanceof JCTree)) {
            throw new IllegalArgumentException();
        }
        if (this.javacTaskImpl != null) {
            try {
                this.javacTaskImpl.enter(null);
            } catch (IOException e) {
                throw new Error("unexpected error while entering symbols: " + e);
            }
        }
        JCTree.JCCompilationUnit unit = (JCTree.JCCompilationUnit) path.getCompilationUnit();
        Copier copier = createCopier(this.treeMaker.forToplevel(unit));
        Env<AttrContext> env = null;
        JCTree.JCMethodDecl method = null;
        JCTree.JCVariableDecl field = null;
        List listNil = List.nil();
        for (TreePath p = path; p != null; p = p.getParentPath()) {
            listNil = listNil.prepend(p.getLeaf());
        }
        while (listNil.nonEmpty()) {
            Tree tree = (Tree) listNil.head;
            switch (tree.getKind()) {
                case COMPILATION_UNIT:
                    env = this.enter.getTopLevelEnv((JCTree.JCCompilationUnit) tree);
                    break;
                case ANNOTATION_TYPE:
                case CLASS:
                case ENUM:
                case INTERFACE:
                    env = this.enter.getClassEnv(((JCTree.JCClassDecl) tree).sym);
                    break;
                case METHOD:
                    method = (JCTree.JCMethodDecl) tree;
                    env = this.memberEnter.getMethodEnv(method, env);
                    break;
                case VARIABLE:
                    field = (JCTree.JCVariableDecl) tree;
                    break;
                case BLOCK:
                    if (method != null) {
                        try {
                            Assert.check(method.body == tree);
                            method.body = (JCTree.JCBlock) copier.copy((JCTree.JCBlock) tree, (JCTree) path.getLeaf());
                            return attribStatToTree(method.body, env, copier.leafCopy);
                        } finally {
                            method.body = (JCTree.JCBlock) tree;
                        }
                    }
                    JCTree.JCBlock body = (JCTree.JCBlock) copier.copy((JCTree.JCBlock) tree, (JCTree) path.getLeaf());
                    return attribStatToTree(body, env, copier.leafCopy);
                default:
                    if (field != null && field.getInitializer() == tree) {
                        Env<AttrContext> env2 = this.memberEnter.getInitEnv(field, env);
                        JCTree.JCExpression expr = (JCTree.JCExpression) copier.copy((JCTree.JCExpression) tree, (JCTree) path.getLeaf());
                        return attribExprToTree(expr, env2, copier.leafCopy);
                    }
                    break;
                    break;
            }
            listNil = listNil.tail;
        }
        return field != null ? this.memberEnter.getInitEnv(field, env) : env;
    }

    private Env<AttrContext> attribStatToTree(JCTree stat, Env<AttrContext> env, JCTree tree) {
        JavaFileObject prev = this.log.useSource(env.toplevel.sourcefile);
        try {
            return this.attr.attribStatToTree(stat, env, tree);
        } finally {
            this.log.useSource(prev);
        }
    }

    private Env<AttrContext> attribExprToTree(JCTree.JCExpression expr, Env<AttrContext> env, JCTree tree) {
        JavaFileObject prev = this.log.useSource(env.toplevel.sourcefile);
        try {
            return this.attr.attribExprToTree(expr, env, tree);
        } finally {
            this.log.useSource(prev);
        }
    }

    protected static class Copier extends TreeCopier<JCTree> {
        JCTree leafCopy;

        protected Copier(TreeMaker M) {
            super(M);
            this.leafCopy = null;
        }

        @Override // com.sun.tools.javac.tree.TreeCopier
        public <T extends JCTree> T copy(T t, JCTree jCTree) {
            T t2 = (T) super.copy((JCTree) t, jCTree);
            if (t == jCTree) {
                this.leafCopy = t2;
            }
            return t2;
        }
    }

    protected Copier createCopier(TreeMaker maker) {
        return new Copier(maker);
    }

    @Override // com.sun.source.util.Trees
    public TypeMirror getOriginalType(ErrorType errorType) {
        if (errorType instanceof Type.ErrorType) {
            return ((Type.ErrorType) errorType).getOriginalType();
        }
        return Type.noType;
    }

    @Override // com.sun.source.util.Trees
    public void printMessage(Diagnostic.Kind kind, CharSequence msg, Tree t, CompilationUnitTree root) {
        printMessage(kind, msg, ((JCTree) t).pos(), root);
    }

    @Override // com.sun.source.util.DocTrees
    public void printMessage(Diagnostic.Kind kind, CharSequence msg, DocTree t, DocCommentTree c, CompilationUnitTree root) {
        printMessage(kind, msg, ((DCTree) t).pos((DCTree.DCDocComment) c), root);
    }

    /* JADX WARN: Removed duplicated region for block: B:20:0x0066 A[DONT_GENERATE] */
    /* JADX WARN: Removed duplicated region for block: B:29:? A[RETURN, SYNTHETIC] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private void printMessage(javax.tools.Diagnostic.Kind r7, java.lang.CharSequence r8, com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition r9, com.sun.source.tree.CompilationUnitTree r10) {
        /*
            r6 = this;
            r0 = 0
            r1 = 0
            javax.tools.JavaFileObject r1 = r10.getSourceFile()
            if (r1 != 0) goto La
            r9 = 0
            goto L10
        La:
            com.sun.tools.javac.util.Log r2 = r6.log
            javax.tools.JavaFileObject r0 = r2.useSource(r1)
        L10:
            int[] r2 = com.sun.tools.javac.api.JavacTrees.AnonymousClass4.$SwitchMap$javax$tools$Diagnostic$Kind     // Catch: java.lang.Throwable -> L6c
            int r3 = r7.ordinal()     // Catch: java.lang.Throwable -> L6c
            r2 = r2[r3]     // Catch: java.lang.Throwable -> L6c
            java.lang.String r3 = "proc.messager"
            switch(r2) {
                case 1: goto L3c;
                case 2: goto L2e;
                case 3: goto L20;
                default: goto L1d;
            }
        L1d:
            com.sun.tools.javac.util.Log r2 = r6.log     // Catch: java.lang.Throwable -> L6c
            goto L59
        L20:
            com.sun.tools.javac.util.Log r2 = r6.log     // Catch: java.lang.Throwable -> L6c
            java.lang.String r4 = r8.toString()     // Catch: java.lang.Throwable -> L6c
            java.lang.Object[] r4 = new java.lang.Object[]{r4}     // Catch: java.lang.Throwable -> L6c
            r2.mandatoryWarning(r9, r3, r4)     // Catch: java.lang.Throwable -> L6c
            goto L64
        L2e:
            com.sun.tools.javac.util.Log r2 = r6.log     // Catch: java.lang.Throwable -> L6c
            java.lang.String r4 = r8.toString()     // Catch: java.lang.Throwable -> L6c
            java.lang.Object[] r4 = new java.lang.Object[]{r4}     // Catch: java.lang.Throwable -> L6c
            r2.warning(r9, r3, r4)     // Catch: java.lang.Throwable -> L6c
            goto L64
        L3c:
            com.sun.tools.javac.util.Log r2 = r6.log     // Catch: java.lang.Throwable -> L6c
            boolean r2 = r2.multipleErrors     // Catch: java.lang.Throwable -> L6c
            com.sun.tools.javac.util.Log r4 = r6.log     // Catch: java.lang.Throwable -> L53
            java.lang.String r5 = r8.toString()     // Catch: java.lang.Throwable -> L53
            java.lang.Object[] r5 = new java.lang.Object[]{r5}     // Catch: java.lang.Throwable -> L53
            r4.error(r9, r3, r5)     // Catch: java.lang.Throwable -> L53
            com.sun.tools.javac.util.Log r3 = r6.log     // Catch: java.lang.Throwable -> L6c
            r3.multipleErrors = r2     // Catch: java.lang.Throwable -> L6c
            goto L64
        L53:
            r3 = move-exception
            com.sun.tools.javac.util.Log r4 = r6.log     // Catch: java.lang.Throwable -> L6c
            r4.multipleErrors = r2     // Catch: java.lang.Throwable -> L6c
            throw r3     // Catch: java.lang.Throwable -> L6c
        L59:
            java.lang.String r4 = r8.toString()     // Catch: java.lang.Throwable -> L6c
            java.lang.Object[] r4 = new java.lang.Object[]{r4}     // Catch: java.lang.Throwable -> L6c
            r2.note(r9, r3, r4)     // Catch: java.lang.Throwable -> L6c
        L64:
            if (r0 == 0) goto L6b
            com.sun.tools.javac.util.Log r2 = r6.log
            r2.useSource(r0)
        L6b:
            return
        L6c:
            r2 = move-exception
            if (r0 == 0) goto L74
            com.sun.tools.javac.util.Log r3 = r6.log
            r3.useSource(r0)
        L74:
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.api.JavacTrees.printMessage(javax.tools.Diagnostic$Kind, java.lang.CharSequence, com.sun.tools.javac.util.JCDiagnostic$DiagnosticPosition, com.sun.source.tree.CompilationUnitTree):void");
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.api.JavacTrees$4, reason: invalid class name */
    static /* synthetic */ class AnonymousClass4 {
        static final /* synthetic */ int[] $SwitchMap$com$sun$source$doctree$DocTree$Kind;
        static final /* synthetic */ int[] $SwitchMap$com$sun$tools$javac$code$TypeTag;

        static {
            try {
                $SwitchMap$javax$tools$Diagnostic$Kind[Diagnostic.Kind.ERROR.ordinal()] = 1;
            } catch (NoSuchFieldError e) {
            }
            try {
                $SwitchMap$javax$tools$Diagnostic$Kind[Diagnostic.Kind.WARNING.ordinal()] = 2;
            } catch (NoSuchFieldError e2) {
            }
            try {
                $SwitchMap$javax$tools$Diagnostic$Kind[Diagnostic.Kind.MANDATORY_WARNING.ordinal()] = 3;
            } catch (NoSuchFieldError e3) {
            }
            $SwitchMap$com$sun$source$tree$Tree$Kind = new int[Tree.Kind.values().length];
            try {
                $SwitchMap$com$sun$source$tree$Tree$Kind[Tree.Kind.COMPILATION_UNIT.ordinal()] = 1;
            } catch (NoSuchFieldError e4) {
            }
            try {
                $SwitchMap$com$sun$source$tree$Tree$Kind[Tree.Kind.ANNOTATION_TYPE.ordinal()] = 2;
            } catch (NoSuchFieldError e5) {
            }
            try {
                $SwitchMap$com$sun$source$tree$Tree$Kind[Tree.Kind.CLASS.ordinal()] = 3;
            } catch (NoSuchFieldError e6) {
            }
            try {
                $SwitchMap$com$sun$source$tree$Tree$Kind[Tree.Kind.ENUM.ordinal()] = 4;
            } catch (NoSuchFieldError e7) {
            }
            try {
                $SwitchMap$com$sun$source$tree$Tree$Kind[Tree.Kind.INTERFACE.ordinal()] = 5;
            } catch (NoSuchFieldError e8) {
            }
            try {
                $SwitchMap$com$sun$source$tree$Tree$Kind[Tree.Kind.METHOD.ordinal()] = 6;
            } catch (NoSuchFieldError e9) {
            }
            try {
                $SwitchMap$com$sun$source$tree$Tree$Kind[Tree.Kind.VARIABLE.ordinal()] = 7;
            } catch (NoSuchFieldError e10) {
            }
            try {
                $SwitchMap$com$sun$source$tree$Tree$Kind[Tree.Kind.BLOCK.ordinal()] = 8;
            } catch (NoSuchFieldError e11) {
            }
            $SwitchMap$com$sun$tools$javac$code$TypeTag = new int[TypeTag.values().length];
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.BYTE.ordinal()] = 1;
            } catch (NoSuchFieldError e12) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.CHAR.ordinal()] = 2;
            } catch (NoSuchFieldError e13) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.SHORT.ordinal()] = 3;
            } catch (NoSuchFieldError e14) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.INT.ordinal()] = 4;
            } catch (NoSuchFieldError e15) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.LONG.ordinal()] = 5;
            } catch (NoSuchFieldError e16) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.FLOAT.ordinal()] = 6;
            } catch (NoSuchFieldError e17) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.DOUBLE.ordinal()] = 7;
            } catch (NoSuchFieldError e18) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.BOOLEAN.ordinal()] = 8;
            } catch (NoSuchFieldError e19) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.VOID.ordinal()] = 9;
            } catch (NoSuchFieldError e20) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.BOT.ordinal()] = 10;
            } catch (NoSuchFieldError e21) {
            }
            try {
                $SwitchMap$com$sun$tools$javac$code$TypeTag[TypeTag.NONE.ordinal()] = 11;
            } catch (NoSuchFieldError e22) {
            }
            $SwitchMap$com$sun$source$doctree$DocTree$Kind = new int[DocTree.Kind.values().length];
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.TEXT.ordinal()] = 1;
            } catch (NoSuchFieldError e23) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.ERRONEOUS.ordinal()] = 2;
            } catch (NoSuchFieldError e24) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.IDENTIFIER.ordinal()] = 3;
            } catch (NoSuchFieldError e25) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.PARAM.ordinal()] = 4;
            } catch (NoSuchFieldError e26) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.AUTHOR.ordinal()] = 5;
            } catch (NoSuchFieldError e27) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.DEPRECATED.ordinal()] = 6;
            } catch (NoSuchFieldError e28) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.RETURN.ordinal()] = 7;
            } catch (NoSuchFieldError e29) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.SEE.ordinal()] = 8;
            } catch (NoSuchFieldError e30) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.SERIAL.ordinal()] = 9;
            } catch (NoSuchFieldError e31) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.SERIAL_DATA.ordinal()] = 10;
            } catch (NoSuchFieldError e32) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.SERIAL_FIELD.ordinal()] = 11;
            } catch (NoSuchFieldError e33) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.SINCE.ordinal()] = 12;
            } catch (NoSuchFieldError e34) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.THROWS.ordinal()] = 13;
            } catch (NoSuchFieldError e35) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.UNKNOWN_BLOCK_TAG.ordinal()] = 14;
            } catch (NoSuchFieldError e36) {
            }
            try {
                $SwitchMap$com$sun$source$doctree$DocTree$Kind[DocTree.Kind.VERSION.ordinal()] = 15;
            } catch (NoSuchFieldError e37) {
            }
        }
    }

    @Override // com.sun.source.util.Trees
    public TypeMirror getLub(CatchTree tree) {
        JCTree.JCCatch ct = (JCTree.JCCatch) tree;
        JCTree.JCVariableDecl v = ct.param;
        if (v.type != null && v.type.getKind() == TypeKind.UNION) {
            Type.UnionClassType ut = (Type.UnionClassType) v.type;
            return ut.getLub();
        }
        return v.type;
    }
}
