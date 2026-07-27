package com.sun.tools.javac.model;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Enter;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.main.JavaCompiler;
import com.sun.tools.javac.processing.PrintingProcessor;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.util.Constants;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Pair;
import java.io.Writer;
import java.util.Map;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.AnnotationValue;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.PackageElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.type.DeclaredType;
import javax.lang.model.util.ElementFilter;
import javax.lang.model.util.Elements;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class JavacElements implements Elements {
    private Enter enter;
    private JavaCompiler javaCompiler;
    private Names names;
    private Symtab syms;
    private Types types;

    public static JavacElements instance(Context context) {
        JavacElements instance = (JavacElements) context.get(JavacElements.class);
        if (instance == null) {
            return new JavacElements(context);
        }
        return instance;
    }

    protected JavacElements(Context context) {
        setContext(context);
    }

    public void setContext(Context context) {
        context.put((Class<JavacElements>) JavacElements.class, this);
        this.javaCompiler = JavaCompiler.instance(context);
        this.syms = Symtab.instance(context);
        this.names = Names.instance(context);
        this.types = Types.instance(context);
        this.enter = Enter.instance(context);
    }

    @Override // javax.lang.model.util.Elements
    public Symbol.PackageSymbol getPackageElement(CharSequence name) {
        String strName = name.toString();
        if (strName.equals("")) {
            return this.syms.unnamedPackage;
        }
        if (SourceVersion.isName(strName)) {
            return (Symbol.PackageSymbol) nameToSymbol(strName, Symbol.PackageSymbol.class);
        }
        return null;
    }

    @Override // javax.lang.model.util.Elements
    public Symbol.ClassSymbol getTypeElement(CharSequence name) {
        String strName = name.toString();
        if (SourceVersion.isName(strName)) {
            return (Symbol.ClassSymbol) nameToSymbol(strName, Symbol.ClassSymbol.class);
        }
        return null;
    }

    private <S extends Symbol> S nameToSymbol(String nameStr, Class<S> clazz) {
        Symbol.PackageSymbol sym;
        Name name = this.names.fromString(nameStr);
        if (clazz == Symbol.ClassSymbol.class) {
            sym = this.syms.classes.get(name);
        } else {
            sym = this.syms.packages.get(name);
        }
        if (sym == null) {
            try {
                sym = this.javaCompiler.resolveIdent(nameStr);
            } catch (Symbol.CompletionFailure e) {
                return null;
            }
        }
        sym.complete();
        if (sym.kind == 63 || !sym.exists() || !clazz.isInstance(sym) || !name.equals(sym.getQualifiedName())) {
            return null;
        }
        return clazz.cast(sym);
    }

    public JavacSourcePosition getSourcePosition(Element e) {
        Pair<JCTree, JCTree.JCCompilationUnit> treeTop = getTreeAndTopLevel(e);
        if (treeTop == null) {
            return null;
        }
        JCTree tree = treeTop.fst;
        JCTree.JCCompilationUnit toplevel = treeTop.snd;
        JavaFileObject sourcefile = toplevel.sourcefile;
        if (sourcefile == null) {
            return null;
        }
        return new JavacSourcePosition(sourcefile, tree.pos, toplevel.lineMap);
    }

    public JavacSourcePosition getSourcePosition(Element e, AnnotationMirror a) {
        JCTree annoTree;
        Pair<JCTree, JCTree.JCCompilationUnit> treeTop = getTreeAndTopLevel(e);
        if (treeTop == null) {
            return null;
        }
        JCTree tree = treeTop.fst;
        JCTree.JCCompilationUnit toplevel = treeTop.snd;
        JavaFileObject sourcefile = toplevel.sourcefile;
        if (sourcefile == null || (annoTree = matchAnnoToTree(a, e, tree)) == null) {
            return null;
        }
        return new JavacSourcePosition(sourcefile, annoTree.pos, toplevel.lineMap);
    }

    public JavacSourcePosition getSourcePosition(Element e, AnnotationMirror a, AnnotationValue v) {
        return getSourcePosition(e, a);
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.model.JavacElements$1Vis, reason: invalid class name */
    class C1Vis extends JCTree.Visitor {
        List<JCTree.JCAnnotation> result = null;

        C1Vis() {
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTopLevel(JCTree.JCCompilationUnit tree) {
            this.result = tree.packageAnnotations;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            this.result = tree.mods.annotations;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl tree) {
            this.result = tree.mods.annotations;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            this.result = tree.mods.annotations;
        }
    }

    private JCTree matchAnnoToTree(AnnotationMirror findme, Element e, JCTree tree) {
        Symbol sym = (Symbol) cast(Symbol.class, e);
        C1Vis vis = new C1Vis();
        tree.accept(vis);
        if (vis.result == null) {
            return null;
        }
        List<Attribute.Compound> annos = sym.getRawAttributes();
        return matchAnnoToTree((Attribute.Compound) cast(Attribute.Compound.class, findme), annos, vis.result);
    }

    private JCTree matchAnnoToTree(Attribute.Compound findme, List<Attribute.Compound> annos, List<JCTree.JCAnnotation> trees) {
        for (Attribute.Compound anno : annos) {
            for (JCTree.JCAnnotation tree : trees) {
                JCTree match = matchAnnoToTree(findme, anno, tree);
                if (match != null) {
                    return match;
                }
            }
        }
        return null;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public JCTree matchAnnoToTree(Attribute.Compound findme, Attribute attr, JCTree tree) {
        if (attr == findme) {
            if (tree.type.tsym == findme.type.tsym) {
                return tree;
            }
            return null;
        }
        C2Vis vis = new C2Vis(tree, findme);
        attr.accept(vis);
        return vis.result;
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.model.JavacElements$2Vis, reason: invalid class name */
    class C2Vis implements Attribute.Visitor {
        JCTree result = null;
        final /* synthetic */ Attribute.Compound val$findme;
        final /* synthetic */ JCTree val$tree;

        C2Vis(JCTree jCTree, Attribute.Compound compound) {
            this.val$tree = jCTree;
            this.val$findme = compound;
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitConstant(Attribute.Constant value) {
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitClass(Attribute.Class clazz) {
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitCompound(Attribute.Compound anno) {
            JCTree match;
            for (Pair<Symbol.MethodSymbol, Attribute> pair : anno.values) {
                JCTree.JCExpression expr = JavacElements.this.scanForAssign(pair.fst, this.val$tree);
                if (expr != null && (match = JavacElements.this.matchAnnoToTree(this.val$findme, pair.snd, expr)) != null) {
                    this.result = match;
                    return;
                }
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitArray(Attribute.Array array) {
            if (this.val$tree.hasTag(JCTree.Tag.NEWARRAY) && JavacElements.this.types.elemtype(array.type).tsym == this.val$findme.type.tsym) {
                List list = ((JCTree.JCNewArray) this.val$tree).elems;
                for (Attribute value : array.values) {
                    if (value == this.val$findme) {
                        this.result = (JCTree) list.head;
                        return;
                    }
                    list = list.tail;
                }
            }
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitEnum(Attribute.Enum e) {
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitError(Attribute.Error e) {
        }
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.model.JavacElements$1TS, reason: invalid class name */
    class C1TS extends TreeScanner {
        JCTree.JCExpression result = null;
        final /* synthetic */ Symbol.MethodSymbol val$sym;
        final /* synthetic */ JCTree val$tree;

        C1TS(JCTree jCTree, Symbol.MethodSymbol methodSymbol) {
            this.val$tree = jCTree;
            this.val$sym = methodSymbol;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree t) {
            if (t != null && this.result == null) {
                t.accept(this);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAnnotation(JCTree.JCAnnotation t) {
            if (t == this.val$tree) {
                scan(t.args);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssign(JCTree.JCAssign t) {
            if (t.lhs.hasTag(JCTree.Tag.IDENT)) {
                JCTree.JCIdent ident = (JCTree.JCIdent) t.lhs;
                if (ident.sym == this.val$sym) {
                    this.result = t.rhs;
                }
            }
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public JCTree.JCExpression scanForAssign(Symbol.MethodSymbol sym, JCTree tree) {
        C1TS scanner = new C1TS(tree, sym);
        tree.accept(scanner);
        return scanner.result;
    }

    public JCTree getTree(Element e) {
        Pair<JCTree, ?> treeTop = getTreeAndTopLevel(e);
        if (treeTop != null) {
            return treeTop.fst;
        }
        return null;
    }

    @Override // javax.lang.model.util.Elements
    public String getDocComment(Element e) {
        Pair<JCTree, JCTree.JCCompilationUnit> treeTop = getTreeAndTopLevel(e);
        if (treeTop == null) {
            return null;
        }
        JCTree tree = treeTop.fst;
        JCTree.JCCompilationUnit toplevel = treeTop.snd;
        if (toplevel.docComments == null) {
            return null;
        }
        return toplevel.docComments.getCommentText(tree);
    }

    @Override // javax.lang.model.util.Elements
    public PackageElement getPackageOf(Element e) {
        return ((Symbol) cast(Symbol.class, e)).packge();
    }

    @Override // javax.lang.model.util.Elements
    public boolean isDeprecated(Element e) {
        Symbol sym = (Symbol) cast(Symbol.class, e);
        return (sym.flags() & 131072) != 0;
    }

    @Override // javax.lang.model.util.Elements
    public Name getBinaryName(TypeElement type) {
        return ((Symbol.TypeSymbol) cast(Symbol.TypeSymbol.class, type)).flatName();
    }

    @Override // javax.lang.model.util.Elements
    public Map<Symbol.MethodSymbol, Attribute> getElementValuesWithDefaults(AnnotationMirror a) {
        Attribute.Compound anno = (Attribute.Compound) cast(Attribute.Compound.class, a);
        DeclaredType annotype = a.getAnnotationType();
        Map<Symbol.MethodSymbol, Attribute> valmap = anno.getElementValues();
        for (ExecutableElement ex : ElementFilter.methodsIn(annotype.asElement().getEnclosedElements())) {
            Symbol.MethodSymbol meth = (Symbol.MethodSymbol) ex;
            Attribute defaultValue = meth.getDefaultValue();
            if (defaultValue != null && !valmap.containsKey(meth)) {
                valmap.put(meth, defaultValue);
            }
        }
        return valmap;
    }

    @Override // javax.lang.model.util.Elements
    public FilteredMemberList getAllMembers(TypeElement element) {
        Symbol sym = (Symbol) cast(Symbol.class, element);
        Scope scope = sym.members().dupUnshared();
        List<Type> closure = this.types.closure(sym.asType());
        for (Type t : closure) {
            addMembers(scope, t);
        }
        return new FilteredMemberList(scope);
    }

    private void addMembers(Scope scope, Type type) {
        for (Scope.Entry e = type.asElement().members().elems; e != null; e = e.sibling) {
            Scope.Entry overrider = scope.lookup(e.sym.getSimpleName());
            while (true) {
                if (overrider.scope != null) {
                    if (overrider.sym.kind != e.sym.kind || (overrider.sym.flags() & 4096) != 0 || overrider.sym.getKind() != ElementKind.METHOD || !overrides((ExecutableElement) overrider.sym, (ExecutableElement) e.sym, (TypeElement) type.asElement())) {
                        overrider = overrider.next();
                    }
                } else {
                    boolean initializer = true;
                    boolean derived = e.sym.getEnclosingElement() != scope.owner;
                    ElementKind kind = e.sym.getKind();
                    if (kind != ElementKind.CONSTRUCTOR && kind != ElementKind.INSTANCE_INIT && kind != ElementKind.STATIC_INIT) {
                        initializer = false;
                    }
                    if (!derived || (!initializer && e.sym.isInheritedIn(scope.owner, this.types))) {
                        scope.enter(e.sym);
                    }
                }
            }
        }
    }

    @Override // javax.lang.model.util.Elements
    public List<Attribute.Compound> getAllAnnotationMirrors(Element e) {
        Symbol sym = (Symbol) cast(Symbol.class, e);
        List<Attribute.Compound> annos = sym.getAnnotationMirrors();
        while (sym.getKind() == ElementKind.CLASS) {
            Type sup = ((Symbol.ClassSymbol) sym).getSuperclass();
            if (!sup.hasTag(TypeTag.CLASS) || sup.isErroneous() || sup.tsym == this.syms.objectType.tsym) {
                break;
            }
            sym = sup.tsym;
            List<Attribute.Compound> oldAnnos = annos;
            List<Attribute.Compound> newAnnos = sym.getAnnotationMirrors();
            for (Attribute.Compound anno : newAnnos) {
                if (isInherited(anno.type) && !containsAnnoOfType(oldAnnos, anno.type)) {
                    annos = annos.prepend(anno);
                }
            }
        }
        return annos;
    }

    private boolean isInherited(Type annotype) {
        return annotype.tsym.attribute(this.syms.inheritedType.tsym) != null;
    }

    private static boolean containsAnnoOfType(List<Attribute.Compound> annos, Type type) {
        for (Attribute.Compound anno : annos) {
            if (anno.type.tsym == type.tsym) {
                return true;
            }
        }
        return false;
    }

    @Override // javax.lang.model.util.Elements
    public boolean hides(Element hiderEl, Element hideeEl) {
        Symbol hider = (Symbol) cast(Symbol.class, hiderEl);
        Symbol hidee = (Symbol) cast(Symbol.class, hideeEl);
        if (hider == hidee || hider.kind != hidee.kind || hider.name != hidee.name) {
            return false;
        }
        if (hider.kind == 16 && (!hider.isStatic() || !this.types.isSubSignature(hider.type, hidee.type))) {
            return false;
        }
        Symbol.ClassSymbol hiderClass = hider.owner.enclClass();
        Symbol.ClassSymbol hideeClass = hidee.owner.enclClass();
        if (hiderClass == null || hideeClass == null || !hiderClass.isSubClass(hideeClass, this.types)) {
            return false;
        }
        return hidee.isInheritedIn(hiderClass, this.types);
    }

    @Override // javax.lang.model.util.Elements
    public boolean overrides(ExecutableElement riderEl, ExecutableElement rideeEl, TypeElement typeEl) {
        Symbol.MethodSymbol rider = (Symbol.MethodSymbol) cast(Symbol.MethodSymbol.class, riderEl);
        Symbol.MethodSymbol ridee = (Symbol.MethodSymbol) cast(Symbol.MethodSymbol.class, rideeEl);
        Symbol.ClassSymbol origin = (Symbol.ClassSymbol) cast(Symbol.ClassSymbol.class, typeEl);
        return rider.name == ridee.name && rider != ridee && !rider.isStatic() && ridee.isMemberOf(origin, this.types) && rider.overrides(ridee, origin, this.types, false);
    }

    @Override // javax.lang.model.util.Elements
    public String getConstantExpression(Object value) {
        return Constants.format(value);
    }

    @Override // javax.lang.model.util.Elements
    public void printElements(Writer w, Element... elements) {
        for (Element element : elements) {
            new PrintingProcessor.PrintingElementVisitor(w, this).visit(element).flush();
        }
    }

    @Override // javax.lang.model.util.Elements
    public Name getName(CharSequence cs) {
        return this.names.fromString(cs.toString());
    }

    @Override // javax.lang.model.util.Elements
    public boolean isFunctionalInterface(TypeElement element) {
        if (element.getKind() != ElementKind.INTERFACE) {
            return false;
        }
        Symbol.TypeSymbol tsym = (Symbol.TypeSymbol) cast(Symbol.TypeSymbol.class, element);
        return this.types.isFunctionalInterface(tsym);
    }

    private Pair<JCTree, JCTree.JCCompilationUnit> getTreeAndTopLevel(Element e) {
        JCTree tree;
        Symbol sym = (Symbol) cast(Symbol.class, e);
        Env<AttrContext> enterEnv = getEnterEnv(sym);
        if (enterEnv == null || (tree = TreeInfo.declarationFor(sym, enterEnv.tree)) == null || enterEnv.toplevel == null) {
            return null;
        }
        return new Pair<>(tree, enterEnv.toplevel);
    }

    public Pair<JCTree, JCTree.JCCompilationUnit> getTreeAndTopLevel(Element e, AnnotationMirror a, AnnotationValue v) {
        Pair<JCTree, JCTree.JCCompilationUnit> elemTreeTop;
        JCTree annoTree;
        if (e == null || (elemTreeTop = getTreeAndTopLevel(e)) == null) {
            return null;
        }
        if (a == null || (annoTree = matchAnnoToTree(a, e, elemTreeTop.fst)) == null) {
            return elemTreeTop;
        }
        return new Pair<>(annoTree, elemTreeTop.snd);
    }

    private Env<AttrContext> getEnterEnv(Symbol sym) {
        Symbol.TypeSymbol ts = sym.kind != 1 ? sym.enclClass() : (Symbol.PackageSymbol) sym;
        if (ts != null) {
            return this.enter.getEnv(ts);
        }
        return null;
    }

    private static <T> T cast(Class<T> clazz, Object o) {
        if (!clazz.isInstance(o)) {
            throw new IllegalArgumentException(o.toString());
        }
        return clazz.cast(o);
    }
}
