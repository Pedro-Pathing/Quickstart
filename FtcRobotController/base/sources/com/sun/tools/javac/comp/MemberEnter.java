package com.sun.tools.javac.comp;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.DeferredLintHandler;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Kinds;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeAnnotations;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.Annotate;
import com.sun.tools.javac.comp.Annotate.AnnotateRepeatedContext;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.jvm.ClassReader;
import com.sun.tools.javac.jvm.Target;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.FatalError;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class MemberEnter extends JCTree.Visitor implements Symbol.Completer {
    static final boolean checkClash = true;
    protected static final Context.Key<MemberEnter> memberEnterKey = new Context.Key<>();
    boolean allowRepeatedAnnos;
    boolean allowTypeAnnos;
    private final Annotate annotate;
    private final Attr attr;
    private final Check chk;
    private final DeferredLintHandler deferredLintHandler;
    private final JCDiagnostic.Factory diags;
    private final Enter enter;
    protected Env<AttrContext> env;
    private final Lint lint;
    private final Log log;
    private final TreeMaker make;
    private final Names names;
    private final ClassReader reader;
    private final Source source;
    private final Symtab syms;
    private final Target target;
    private final Todo todo;
    private final TypeAnnotations typeAnnotations;
    private final TypeEnvs typeEnvs;
    private final Types types;
    ListBuffer<Env<AttrContext>> halfcompleted = new ListBuffer<>();
    boolean isFirst = true;
    boolean completionEnabled = true;

    public static MemberEnter instance(Context context) {
        MemberEnter instance = (MemberEnter) context.get(memberEnterKey);
        if (instance == null) {
            return new MemberEnter(context);
        }
        return instance;
    }

    protected MemberEnter(Context context) {
        context.put(memberEnterKey, this);
        this.names = Names.instance(context);
        this.enter = Enter.instance(context);
        this.log = Log.instance(context);
        this.chk = Check.instance(context);
        this.attr = Attr.instance(context);
        this.syms = Symtab.instance(context);
        this.make = TreeMaker.instance(context);
        this.reader = ClassReader.instance(context);
        this.todo = Todo.instance(context);
        this.annotate = Annotate.instance(context);
        this.typeAnnotations = TypeAnnotations.instance(context);
        this.types = Types.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        this.source = Source.instance(context);
        this.target = Target.instance(context);
        this.deferredLintHandler = DeferredLintHandler.instance(context);
        this.lint = Lint.instance(context);
        this.typeEnvs = TypeEnvs.instance(context);
        this.allowTypeAnnos = this.source.allowTypeAnnotations();
        this.allowRepeatedAnnos = this.source.allowRepeatedAnnotations();
    }

    private void importAll(int pos, Symbol.TypeSymbol tsym, Env<AttrContext> env) {
        if (tsym.kind == 1 && tsym.members().elems == null && !tsym.exists()) {
            if (((Symbol.PackageSymbol) tsym).fullname.equals(this.names.java_lang)) {
                JCDiagnostic msg = this.diags.fragment("fatal.err.no.java.lang", new Object[0]);
                throw new FatalError(msg);
            }
            this.log.error(JCDiagnostic.DiagnosticFlag.RESOLVE_ERROR, pos, "doesnt.exist", tsym);
        }
        env.toplevel.starImportScope.importAll(tsym.members());
    }

    /* JADX WARN: Type inference failed for: r0v3, types: [com.sun.tools.javac.comp.MemberEnter$1] */
    private void importStaticAll(int pos, final Symbol.TypeSymbol tsym, Env<AttrContext> env) {
        final JavaFileObject sourcefile = env.toplevel.sourcefile;
        final Scope toScope = env.toplevel.starImportScope;
        final Symbol.PackageSymbol packge = env.toplevel.packge;
        new Object() { // from class: com.sun.tools.javac.comp.MemberEnter.1
            Set<Symbol> processed = new HashSet();

            void importFrom(Symbol.TypeSymbol tsym2) {
                if (tsym2 != null && this.processed.add(tsym2)) {
                    importFrom(MemberEnter.this.types.supertype(tsym2.type).tsym);
                    for (Type t : MemberEnter.this.types.interfaces(tsym2.type)) {
                        importFrom(t.tsym);
                    }
                    Scope fromScope = tsym2.members();
                    for (Scope.Entry e = fromScope.elems; e != null; e = e.sibling) {
                        Symbol sym = e.sym;
                        if (sym.kind == 2 && (sym.flags() & 8) != 0 && MemberEnter.this.staticImportAccessible(sym, packge) && sym.isMemberOf(tsym, MemberEnter.this.types) && !toScope.includes(sym)) {
                            toScope.enter(sym, fromScope, tsym.members(), true);
                        }
                    }
                }
            }
        }.importFrom(tsym);
        this.annotate.earlier(new Annotate.Worker() { // from class: com.sun.tools.javac.comp.MemberEnter.2
            Set<Symbol> processed = new HashSet();

            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public String toString() {
                return "import static " + tsym + ".* in " + sourcefile;
            }

            void importFrom(Symbol.TypeSymbol tsym2) {
                if (tsym2 != null && this.processed.add(tsym2)) {
                    importFrom(MemberEnter.this.types.supertype(tsym2.type).tsym);
                    for (Type t : MemberEnter.this.types.interfaces(tsym2.type)) {
                        importFrom(t.tsym);
                    }
                    Scope fromScope = tsym2.members();
                    for (Scope.Entry e = fromScope.elems; e != null; e = e.sibling) {
                        Symbol sym = e.sym;
                        if (sym.isStatic() && sym.kind != 2 && MemberEnter.this.staticImportAccessible(sym, packge) && !toScope.includes(sym) && sym.isMemberOf(tsym, MemberEnter.this.types)) {
                            toScope.enter(sym, fromScope, tsym.members(), true);
                        }
                    }
                }
            }

            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public void run() {
                importFrom(tsym);
            }
        });
    }

    boolean staticImportAccessible(Symbol sym, Symbol.PackageSymbol packge) {
        int flags = (int) (sym.flags() & 7);
        switch (flags) {
            case 0:
            case 4:
                return sym.packge() == packge;
            case 1:
            case 3:
            default:
                return true;
            case 2:
                return false;
        }
    }

    /* JADX WARN: Type inference failed for: r7v0, types: [com.sun.tools.javac.comp.MemberEnter$3] */
    private void importNamedStatic(final JCDiagnostic.DiagnosticPosition pos, final Symbol.TypeSymbol tsym, final Name name, final Env<AttrContext> env) {
        if (tsym.kind != 2) {
            this.log.error(JCDiagnostic.DiagnosticFlag.RECOVERABLE, pos, "static.imp.only.classes.and.interfaces", new Object[0]);
            return;
        }
        final Scope toScope = env.toplevel.namedImportScope;
        final Symbol.PackageSymbol packge = env.toplevel.packge;
        new Object() { // from class: com.sun.tools.javac.comp.MemberEnter.3
            Set<Symbol> processed = new HashSet();

            void importFrom(Symbol.TypeSymbol tsym2) {
                if (tsym2 != null && this.processed.add(tsym2)) {
                    importFrom(MemberEnter.this.types.supertype(tsym2.type).tsym);
                    for (Type t : MemberEnter.this.types.interfaces(tsym2.type)) {
                        importFrom(t.tsym);
                    }
                    for (Scope.Entry e = tsym2.members().lookup(name); e.scope != null; e = e.next()) {
                        Symbol sym = e.sym;
                        if (sym.isStatic() && sym.kind == 2 && MemberEnter.this.staticImportAccessible(sym, packge) && sym.isMemberOf(tsym, MemberEnter.this.types) && MemberEnter.this.chk.checkUniqueStaticImport(pos, sym, toScope)) {
                            toScope.enter(sym, sym.owner.members(), tsym.members(), true);
                        }
                    }
                }
            }
        }.importFrom(tsym);
        this.annotate.earlier(new Annotate.Worker() { // from class: com.sun.tools.javac.comp.MemberEnter.4
            Set<Symbol> processed = new HashSet();
            boolean found = false;

            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public String toString() {
                return "import static " + tsym + "." + ((Object) name);
            }

            void importFrom(Symbol.TypeSymbol tsym2) {
                if (tsym2 != null && this.processed.add(tsym2)) {
                    importFrom(MemberEnter.this.types.supertype(tsym2.type).tsym);
                    for (Type t : MemberEnter.this.types.interfaces(tsym2.type)) {
                        importFrom(t.tsym);
                    }
                    for (Scope.Entry e = tsym2.members().lookup(name); e.scope != null; e = e.next()) {
                        Symbol sym = e.sym;
                        if (sym.isStatic() && MemberEnter.this.staticImportAccessible(sym, packge) && sym.isMemberOf(tsym, MemberEnter.this.types)) {
                            this.found = true;
                            if (sym.kind != 2) {
                                toScope.enter(sym, sym.owner.members(), tsym.members(), true);
                            }
                        }
                    }
                }
            }

            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public void run() {
                JavaFileObject prev = MemberEnter.this.log.useSource(env.toplevel.sourcefile);
                try {
                    importFrom(tsym);
                    if (!this.found) {
                        MemberEnter.this.log.error(pos, "cant.resolve.location", Kinds.KindName.STATIC, name, List.nil(), List.nil(), Kinds.typeKindName(tsym.type), tsym.type);
                    }
                } finally {
                    MemberEnter.this.log.useSource(prev);
                }
            }
        });
    }

    private void importNamed(JCDiagnostic.DiagnosticPosition pos, Symbol tsym, Env<AttrContext> env) {
        if (tsym.kind == 2 && this.chk.checkUniqueImport(pos, tsym, env.toplevel.namedImportScope)) {
            env.toplevel.namedImportScope.enter(tsym, tsym.owner.members());
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    Type signature(Symbol.MethodSymbol msym, List<JCTree.JCTypeParameter> typarams, List<JCTree.JCVariableDecl> params, JCTree res, JCTree.JCVariableDecl recvparam, List<JCTree.JCExpression> thrown, Env<AttrContext> env) {
        Type recvtype;
        List<Type> tvars = this.enter.classEnter(typarams, env);
        this.attr.attribTypeVariables(typarams, env);
        ListBuffer<Type> argbuf = new ListBuffer<>();
        for (List list = params; list.nonEmpty(); list = list.tail) {
            memberEnter((JCTree) list.head, env);
            argbuf.append(((JCTree.JCVariableDecl) list.head).vartype.type);
        }
        Type restype = res == null ? this.syms.voidType : this.attr.attribType(res, env);
        if (recvparam != null) {
            memberEnter(recvparam, env);
            recvtype = recvparam.vartype.type;
        } else {
            recvtype = null;
        }
        ListBuffer<Type> thrownbuf = new ListBuffer<>();
        for (List list2 = thrown; list2.nonEmpty(); list2 = list2.tail) {
            Type exc = this.attr.attribType((JCTree) list2.head, env);
            if (!exc.hasTag(TypeTag.TYPEVAR)) {
                exc = this.chk.checkClassType(((JCTree.JCExpression) list2.head).pos(), exc);
            } else if (exc.tsym.owner == msym) {
                exc.tsym.flags_field |= Flags.THROWS;
            }
            thrownbuf.append(exc);
        }
        Type.MethodType mtype = new Type.MethodType(argbuf.toList(), restype, thrownbuf.toList(), this.syms.methodClass);
        mtype.recvtype = recvtype;
        return tvars.isEmpty() ? mtype : new Type.ForAll(tvars, mtype);
    }

    protected void memberEnter(JCTree tree, Env<AttrContext> env) {
        Env<AttrContext> prevEnv = this.env;
        try {
            try {
                this.env = env;
                tree.accept(this);
            } catch (Symbol.CompletionFailure ex) {
                this.chk.completionError(tree.pos(), ex);
            }
        } finally {
            this.env = prevEnv;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void memberEnter(List<? extends JCTree> trees, Env<AttrContext> env) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            memberEnter((JCTree) list.head, env);
        }
    }

    void finishClass(JCTree.JCClassDecl tree, Env<AttrContext> env) {
        if ((tree.mods.flags & 16384) != 0 && (this.types.supertype(tree.sym.type).tsym.flags() & 16384) == 0) {
            addEnumMembers(tree, env);
        }
        memberEnter(tree.defs, env);
    }

    private void addEnumMembers(JCTree.JCClassDecl tree, Env<AttrContext> env) {
        JCTree.JCExpression valuesType = this.make.Type(new Type.ArrayType(tree.sym.type, this.syms.arrayClass));
        JCTree.JCMethodDecl values = this.make.MethodDef(this.make.Modifiers(9L), this.names.values, valuesType, List.nil(), List.nil(), List.nil(), null, null);
        memberEnter(values, env);
        JCTree.JCMethodDecl valueOf = this.make.MethodDef(this.make.Modifiers(9L), this.names.valueOf, this.make.Type(tree.sym.type), List.nil(), List.of(this.make.VarDef(this.make.Modifiers(8589967360L), this.names.fromString("name"), this.make.Type(this.syms.stringType), null)), List.nil(), null, null);
        memberEnter(valueOf, env);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTopLevel(JCTree.JCCompilationUnit tree) {
        if (tree.starImportScope.elems != null) {
            return;
        }
        if (tree.pid != null) {
            for (Symbol p = tree.packge; p.owner != this.syms.rootPackage; p = p.owner) {
                p.owner.complete();
                if (this.syms.classes.get(p.getQualifiedName()) != null) {
                    this.log.error(tree.pos, "pkg.clashes.with.class.of.same.name", p);
                }
            }
        }
        annotateLater(tree.packageAnnotations, this.env, tree.packge, null);
        JCDiagnostic.DiagnosticPosition prevLintPos = this.deferredLintHandler.immediate();
        Lint prevLint = this.chk.setLint(this.lint);
        try {
            importAll(tree.pos, this.reader.enterPackage(this.names.java_lang), this.env);
            memberEnter(tree.defs, this.env);
        } finally {
            this.chk.setLint(prevLint);
            this.deferredLintHandler.setPos(prevLintPos);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitImport(JCTree.JCImport tree) {
        JCTree.JCFieldAccess imp = (JCTree.JCFieldAccess) tree.qualid;
        Name name = TreeInfo.name(imp);
        Env<AttrContext> localEnv = this.env.dup(tree);
        Symbol.TypeSymbol p = this.attr.attribImportQualifier(tree, localEnv).tsym;
        if (name == this.names.asterisk) {
            this.chk.checkCanonical(imp.selected);
            if (tree.staticImport) {
                importStaticAll(tree.pos, p, this.env);
                return;
            } else {
                importAll(tree.pos, p, this.env);
                return;
            }
        }
        if (tree.staticImport) {
            importNamedStatic(tree.pos(), p, name, localEnv);
            this.chk.checkCanonical(imp.selected);
        } else {
            Symbol c = attribImportType(imp, localEnv).tsym;
            this.chk.checkCanonical(imp);
            importNamed(tree.pos(), c, this.env);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitMethodDef(JCTree.JCMethodDecl tree) throws Throwable {
        JCDiagnostic.DiagnosticPosition prevLintPos;
        Scope enclScope = this.enter.enterScope(this.env);
        Symbol.MethodSymbol m = new Symbol.MethodSymbol(0L, tree.name, null, enclScope.owner);
        m.flags_field = this.chk.checkFlags(tree.pos(), tree.mods.flags, m, tree);
        tree.sym = m;
        if ((tree.mods.flags & Flags.DEFAULT) != 0) {
            m.enclClass().flags_field |= Flags.DEFAULT;
        }
        Env<AttrContext> localEnv = methodEnv(tree, this.env);
        JCDiagnostic.DiagnosticPosition prevLintPos2 = this.deferredLintHandler.setPos(tree.pos());
        try {
            prevLintPos = prevLintPos2;
            try {
                m.type = signature(m, tree.typarams, tree.params, tree.restype, tree.recvparam, tree.thrown, localEnv);
                this.deferredLintHandler.setPos(prevLintPos);
                if (this.types.isSignaturePolymorphic(m)) {
                    m.flags_field |= Flags.SIGNATURE_POLYMORPHIC;
                }
                ListBuffer listBuffer = new ListBuffer();
                JCTree.JCVariableDecl lastParam = null;
                for (List list = tree.params; list.nonEmpty(); list = list.tail) {
                    JCTree.JCVariableDecl param = (JCTree.JCVariableDecl) list.head;
                    lastParam = param;
                    listBuffer.append(Assert.checkNonNull(param.sym));
                }
                m.params = listBuffer.toList();
                if (lastParam != null && (lastParam.mods.flags & Flags.VARARGS) != 0) {
                    m.flags_field |= Flags.VARARGS;
                }
                localEnv.info.scope.leave();
                if (this.chk.checkUnique(tree.pos(), m, enclScope)) {
                    enclScope.enter(m);
                }
                annotateLater(tree.mods.annotations, localEnv, m, tree.pos());
                typeAnnotate(tree, localEnv, m, tree.pos());
                if (tree.defaultValue != null) {
                    annotateDefaultValueLater(tree.defaultValue, localEnv, m);
                }
            } catch (Throwable th) {
                th = th;
                this.deferredLintHandler.setPos(prevLintPos);
                throw th;
            }
        } catch (Throwable th2) {
            th = th2;
            prevLintPos = prevLintPos2;
        }
    }

    Env<AttrContext> methodEnv(JCTree.JCMethodDecl tree, Env<AttrContext> env) {
        Env<AttrContext> localEnv = env.dup(tree, env.info.dup(env.info.scope.dupUnshared()));
        localEnv.enclMethod = tree;
        localEnv.info.scope.owner = tree.sym;
        if (tree.sym.type != null) {
            AttrContext attrContext = localEnv.info;
            Attr attr = this.attr;
            attr.getClass();
            attrContext.returnResult = new Attr.ResultInfo(attr, 12, tree.sym.type.mo178getReturnType());
        }
        if ((tree.mods.flags & 8) != 0) {
            localEnv.info.staticLevel++;
        }
        return localEnv;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitVarDef(JCTree.JCVariableDecl tree) {
        Env<AttrContext> localEnv;
        Env<AttrContext> localEnv2 = this.env;
        if ((tree.mods.flags & 8) == 0 && (this.env.info.scope.owner.flags() & 512) == 0) {
            localEnv = localEnv2;
        } else {
            Env<AttrContext> localEnv3 = this.env.dup(tree, this.env.info.dup());
            localEnv3.info.staticLevel++;
            localEnv = localEnv3;
        }
        JCDiagnostic.DiagnosticPosition prevLintPos = this.deferredLintHandler.setPos(tree.pos());
        try {
            if (TreeInfo.isEnumInit(tree)) {
                this.attr.attribIdentAsEnumType(localEnv, (JCTree.JCIdent) tree.vartype);
            } else {
                this.attr.attribType(tree.vartype, localEnv);
                if (TreeInfo.isReceiverParam(tree)) {
                    checkReceiver(tree, localEnv);
                }
            }
            this.deferredLintHandler.setPos(prevLintPos);
            if ((tree.mods.flags & Flags.VARARGS) != 0) {
                Type.ArrayType atype = (Type.ArrayType) tree.vartype.type.unannotatedType();
                tree.vartype.type = atype.makeVarargs();
            }
            Scope enclScope = this.enter.enterScope(this.env);
            Symbol.VarSymbol v = new Symbol.VarSymbol(0L, tree.name, tree.vartype.type, enclScope.owner);
            v.flags_field = this.chk.checkFlags(tree.pos(), tree.mods.flags, v, tree);
            tree.sym = v;
            if (tree.init != null) {
                v.flags_field |= 262144;
                if ((v.flags_field & 16) != 0 && needsLazyConstValue(tree.init)) {
                    Env<AttrContext> initEnv = getInitEnv(tree, this.env);
                    initEnv.info.enclVar = v;
                    v.setLazyConstValue(initEnv(tree, initEnv), this.attr, tree);
                }
            }
            if (this.chk.checkUnique(tree.pos(), v, enclScope)) {
                this.chk.checkTransparentVar(tree.pos(), v, enclScope);
                enclScope.enter(v);
            }
            annotateLater(tree.mods.annotations, localEnv, v, tree.pos());
            typeAnnotate(tree.vartype, this.env, v, tree.pos());
            v.pos = tree.pos;
        } catch (Throwable th) {
            this.deferredLintHandler.setPos(prevLintPos);
            throw th;
        }
    }

    void checkType(JCTree tree, Type type, String diag) {
        if (!tree.type.isErroneous() && !this.types.isSameType(tree.type, type)) {
            this.log.error(tree, diag, type, tree.type);
        }
    }

    void checkReceiver(JCTree.JCVariableDecl tree, Env<AttrContext> localEnv) {
        this.attr.attribExpr(tree.nameexpr, localEnv);
        Symbol.MethodSymbol m = localEnv.enclMethod.sym;
        if (m.isConstructor()) {
            Type outertype = m.owner.owner.type;
            if (outertype.hasTag(TypeTag.METHOD)) {
                outertype = m.owner.owner.owner.type;
            }
            if (outertype.hasTag(TypeTag.CLASS)) {
                checkType(tree.vartype, outertype, "incorrect.constructor.receiver.type");
                checkType(tree.nameexpr, outertype, "incorrect.constructor.receiver.name");
                return;
            } else {
                this.log.error(tree, "receiver.parameter.not.applicable.constructor.toplevel.class", new Object[0]);
                return;
            }
        }
        checkType(tree.vartype, m.owner.type, "incorrect.receiver.type");
        checkType(tree.nameexpr, m.owner.type, "incorrect.receiver.name");
    }

    public boolean needsLazyConstValue(JCTree tree) {
        InitTreeVisitor initTreeVisitor = new InitTreeVisitor();
        tree.accept(initTreeVisitor);
        return initTreeVisitor.result;
    }

    static class InitTreeVisitor extends JCTree.Visitor {
        private boolean result = true;

        InitTreeVisitor() {
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTree(JCTree tree) {
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass that) {
            this.result = false;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewArray(JCTree.JCNewArray that) {
            this.result = false;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda that) {
            this.result = false;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReference(JCTree.JCMemberReference that) {
            this.result = false;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation that) {
            this.result = false;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            tree.selected.accept(this);
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitConditional(JCTree.JCConditional tree) {
            tree.cond.accept(this);
            tree.truepart.accept(this);
            tree.falsepart.accept(this);
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitParens(JCTree.JCParens tree) {
            tree.expr.accept(this);
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeCast(JCTree.JCTypeCast tree) {
            tree.expr.accept(this);
        }
    }

    Env<AttrContext> initEnv(JCTree.JCVariableDecl tree, Env<AttrContext> env) {
        Env<AttrContext> localEnv = env.dupto(new AttrContextEnv(tree, env.info.dup()));
        if (tree.sym.owner.kind == 2) {
            localEnv.info.scope = env.info.scope.dupUnshared();
            localEnv.info.scope.owner = tree.sym;
        }
        if ((tree.mods.flags & 8) != 0 || ((env.enclClass.sym.flags() & 512) != 0 && env.enclMethod == null)) {
            localEnv.info.staticLevel++;
        }
        return localEnv;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTree(JCTree tree) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitErroneous(JCTree.JCErroneous tree) {
        if (tree.errs != null) {
            memberEnter(tree.errs, this.env);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public Env<AttrContext> getMethodEnv(JCTree.JCMethodDecl tree, Env<AttrContext> env) {
        Env<AttrContext> mEnv = methodEnv(tree, env);
        mEnv.info.lint = mEnv.info.lint.augment(tree.sym);
        for (List list = tree.typarams; list.nonEmpty(); list = list.tail) {
            mEnv.info.scope.enterIfAbsent(((JCTree.JCTypeParameter) list.head).type.tsym);
        }
        for (List list2 = tree.params; list2.nonEmpty(); list2 = list2.tail) {
            mEnv.info.scope.enterIfAbsent(((JCTree.JCVariableDecl) list2.head).sym);
        }
        return mEnv;
    }

    public Env<AttrContext> getInitEnv(JCTree.JCVariableDecl tree, Env<AttrContext> env) {
        Env<AttrContext> iEnv = initEnv(tree, env);
        return iEnv;
    }

    Type attribImportType(JCTree tree, Env<AttrContext> env) {
        Assert.check(this.completionEnabled);
        try {
            this.completionEnabled = false;
            return this.attr.attribType(tree, env);
        } finally {
            this.completionEnabled = true;
        }
    }

    void annotateLater(final List<JCTree.JCAnnotation> annotations, final Env<AttrContext> localEnv, final Symbol s, final JCDiagnostic.DiagnosticPosition deferPos) {
        if (annotations.isEmpty()) {
            return;
        }
        if (s.kind != 1) {
            s.resetAnnotations();
        }
        this.annotate.normal(new Annotate.Worker() { // from class: com.sun.tools.javac.comp.MemberEnter.5
            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public String toString() {
                return "annotate " + annotations + " onto " + s + " in " + s.owner;
            }

            /* JADX WARN: Multi-variable type inference failed */
            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public void run() {
                boolean z = true;
                if (s.kind != 1 && !s.annotationsPendingCompletion()) {
                    z = false;
                }
                Assert.check(z);
                JavaFileObject prev = MemberEnter.this.log.useSource(localEnv.toplevel.sourcefile);
                JCDiagnostic.DiagnosticPosition prevLintPos = deferPos != null ? MemberEnter.this.deferredLintHandler.setPos(deferPos) : MemberEnter.this.deferredLintHandler.immediate();
                Lint prevLint = deferPos != null ? null : MemberEnter.this.chk.setLint(MemberEnter.this.lint);
                try {
                    if (s.hasAnnotations() && annotations.nonEmpty()) {
                        MemberEnter.this.log.error(((JCTree.JCAnnotation) annotations.head).pos, "already.annotated", Kinds.kindName(s), s);
                    }
                    MemberEnter.this.actualEnterAnnotations(annotations, localEnv, s);
                } finally {
                    if (prevLint != null) {
                        MemberEnter.this.chk.setLint(prevLint);
                    }
                    MemberEnter.this.deferredLintHandler.setPos(prevLintPos);
                    MemberEnter.this.log.useSource(prev);
                }
            }
        });
        this.annotate.validate(new Annotate.Worker() { // from class: com.sun.tools.javac.comp.MemberEnter.6
            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public void run() {
                JavaFileObject prev = MemberEnter.this.log.useSource(localEnv.toplevel.sourcefile);
                try {
                    MemberEnter.this.chk.validateAnnotations(annotations, s);
                } finally {
                    MemberEnter.this.log.useSource(prev);
                }
            }
        });
    }

    /* JADX WARN: Multi-variable type inference failed */
    private boolean hasDeprecatedAnnotation(List<JCTree.JCAnnotation> annotations) {
        for (List list = annotations; !list.isEmpty(); list = list.tail) {
            JCTree.JCAnnotation a = (JCTree.JCAnnotation) list.head;
            if (a.annotationType.type == this.syms.deprecatedType && a.args.isEmpty()) {
                return true;
            }
        }
        return false;
    }

    /* JADX INFO: Access modifiers changed from: private */
    /* JADX WARN: Multi-variable type inference failed */
    public void actualEnterAnnotations(List<JCTree.JCAnnotation> annotations, Env<AttrContext> env, Symbol s) {
        Map<Symbol.TypeSymbol, ListBuffer<Attribute.Compound>> annotated = new LinkedHashMap<>();
        Map<Attribute.Compound, JCDiagnostic.DiagnosticPosition> pos = new HashMap<>();
        for (List list = annotations; !list.isEmpty(); list = list.tail) {
            JCTree.JCAnnotation a = (JCTree.JCAnnotation) list.head;
            Attribute.Compound c = this.annotate.enterAnnotation(a, this.syms.annotationType, env);
            if (c != null) {
                if (annotated.containsKey(a.type.tsym)) {
                    if (!this.allowRepeatedAnnos) {
                        this.log.error(a.pos(), "repeatable.annotations.not.supported.in.source", new Object[0]);
                        this.allowRepeatedAnnos = true;
                    }
                    ListBuffer<Attribute.Compound> l = annotated.get(a.type.tsym);
                    annotated.put(a.type.tsym, l.append(c));
                    pos.put(c, a.pos());
                } else {
                    annotated.put(a.type.tsym, ListBuffer.of(c));
                    pos.put(c, a.pos());
                }
                if (!c.type.isErroneous() && s.owner.kind != 16 && this.types.isSameType(c.type, this.syms.deprecatedType)) {
                    s.flags_field |= 131072;
                }
            }
        }
        Annotate annotate = this.annotate;
        annotate.getClass();
        s.setDeclarationAttributesWithCompletion(annotate.new AnnotateRepeatedContext<>(env, annotated, pos, this.log, false));
    }

    void annotateDefaultValueLater(final JCTree.JCExpression defaultValue, final Env<AttrContext> localEnv, final Symbol.MethodSymbol m) {
        this.annotate.normal(new Annotate.Worker() { // from class: com.sun.tools.javac.comp.MemberEnter.7
            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public String toString() {
                return "annotate " + m.owner + "." + m + " default " + defaultValue;
            }

            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public void run() {
                JavaFileObject prev = MemberEnter.this.log.useSource(localEnv.toplevel.sourcefile);
                try {
                    MemberEnter.this.enterDefaultValue(defaultValue, localEnv, m);
                } finally {
                    MemberEnter.this.log.useSource(prev);
                }
            }
        });
        this.annotate.validate(new Annotate.Worker() { // from class: com.sun.tools.javac.comp.MemberEnter.8
            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public void run() {
                JavaFileObject prev = MemberEnter.this.log.useSource(localEnv.toplevel.sourcefile);
                try {
                    MemberEnter.this.chk.validateAnnotationTree(defaultValue);
                } finally {
                    MemberEnter.this.log.useSource(prev);
                }
            }
        });
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void enterDefaultValue(JCTree.JCExpression defaultValue, Env<AttrContext> localEnv, Symbol.MethodSymbol m) {
        m.defaultValue = this.annotate.enterAttributeValue(m.type.mo178getReturnType(), defaultValue, localEnv);
    }

    /* JADX WARN: Removed duplicated region for block: B:249:0x0590 A[EXC_TOP_SPLITTER, LOOP:0: B:249:0x0590->B:281:0x0590, LOOP_START, SYNTHETIC] */
    @Override // com.sun.tools.javac.code.Symbol.Completer
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public void complete(com.sun.tools.javac.code.Symbol r35) throws java.lang.Throwable {
        /*
            Method dump skipped, instruction units count: 1507
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.MemberEnter.complete(com.sun.tools.javac.code.Symbol):void");
    }

    /* JADX INFO: Access modifiers changed from: private */
    /* JADX WARN: Multi-variable type inference failed */
    public void actualEnterTypeAnnotations(List<JCTree.JCAnnotation> annotations, Env<AttrContext> env, Symbol s) {
        Map<Symbol.TypeSymbol, ListBuffer<Attribute.TypeCompound>> annotated = new LinkedHashMap<>();
        Map<Attribute.TypeCompound, JCDiagnostic.DiagnosticPosition> pos = new HashMap<>();
        for (List list = annotations; !list.isEmpty(); list = list.tail) {
            JCTree.JCAnnotation a = (JCTree.JCAnnotation) list.head;
            Attribute.TypeCompound tc = this.annotate.enterTypeAnnotation(a, this.syms.annotationType, env);
            if (tc != null) {
                if (annotated.containsKey(a.type.tsym)) {
                    if (this.source.allowRepeatedAnnotations()) {
                        ListBuffer<Attribute.TypeCompound> l = annotated.get(a.type.tsym);
                        annotated.put(a.type.tsym, l.append(tc));
                        pos.put(tc, a.pos());
                    } else {
                        this.log.error(a.pos(), "repeatable.annotations.not.supported.in.source", new Object[0]);
                    }
                } else {
                    annotated.put(a.type.tsym, ListBuffer.of(tc));
                    pos.put(tc, a.pos());
                }
            }
        }
        if (s != null) {
            Annotate annotate = this.annotate;
            annotate.getClass();
            s.appendTypeAttributesWithCompletion(annotate.new AnnotateRepeatedContext<>(env, annotated, pos, this.log, true));
        }
    }

    public void typeAnnotate(JCTree tree, Env<AttrContext> env, Symbol sym, JCDiagnostic.DiagnosticPosition deferPos) {
        if (this.allowTypeAnnos) {
            tree.accept(new TypeAnnotate(env, sym, deferPos));
        }
    }

    private class TypeAnnotate extends TreeScanner {
        private JCDiagnostic.DiagnosticPosition deferPos;
        private Env<AttrContext> env;
        private Symbol sym;

        public TypeAnnotate(Env<AttrContext> env, Symbol sym, JCDiagnostic.DiagnosticPosition deferPos) {
            this.env = env;
            this.sym = sym;
            this.deferPos = deferPos;
        }

        void annotateTypeLater(final List<JCTree.JCAnnotation> annotations) {
            if (annotations.isEmpty()) {
                return;
            }
            final JCDiagnostic.DiagnosticPosition deferPos = this.deferPos;
            MemberEnter.this.annotate.normal(new Annotate.Worker() { // from class: com.sun.tools.javac.comp.MemberEnter.TypeAnnotate.1
                @Override // com.sun.tools.javac.comp.Annotate.Worker
                public String toString() {
                    return "type annotate " + annotations + " onto " + TypeAnnotate.this.sym + " in " + TypeAnnotate.this.sym.owner;
                }

                @Override // com.sun.tools.javac.comp.Annotate.Worker
                public void run() {
                    JavaFileObject prev = MemberEnter.this.log.useSource(TypeAnnotate.this.env.toplevel.sourcefile);
                    JCDiagnostic.DiagnosticPosition prevLintPos = null;
                    if (deferPos != null) {
                        prevLintPos = MemberEnter.this.deferredLintHandler.setPos(deferPos);
                    }
                    try {
                        MemberEnter.this.actualEnterTypeAnnotations(annotations, TypeAnnotate.this.env, TypeAnnotate.this.sym);
                    } finally {
                        if (prevLintPos != null) {
                            MemberEnter.this.deferredLintHandler.setPos(prevLintPos);
                        }
                        MemberEnter.this.log.useSource(prev);
                    }
                }
            });
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
            annotateTypeLater(tree.annotations);
            super.visitAnnotatedType(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeParameter(JCTree.JCTypeParameter tree) {
            annotateTypeLater(tree.annotations);
            super.visitTypeParameter(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewArray(JCTree.JCNewArray tree) {
            annotateTypeLater(tree.annotations);
            for (List<JCTree.JCAnnotation> dimAnnos : tree.dimAnnotations) {
                annotateTypeLater(dimAnnos);
            }
            super.visitNewArray(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl tree) {
            scan(tree.mods);
            scan(tree.restype);
            scan(tree.typarams);
            scan(tree.recvparam);
            scan(tree.params);
            scan(tree.thrown);
            scan(tree.defaultValue);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            JCDiagnostic.DiagnosticPosition prevPos = this.deferPos;
            this.deferPos = tree.pos();
            try {
                if (this.sym != null && this.sym.kind == 4) {
                    scan(tree.mods);
                    scan(tree.vartype);
                }
                scan(tree.init);
            } finally {
                this.deferPos = prevPos;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            if (tree.def == null) {
                super.visitNewClass(tree);
            }
        }
    }

    /* JADX WARN: Type inference incomplete: some casts might be missing */
    private Env<AttrContext> baseEnv(JCTree.JCClassDecl jCClassDecl, Env<AttrContext> env) {
        Scope scope = new Scope(jCClassDecl.sym);
        for (Scope.Entry entry = env.outer.info.scope.elems; entry != null; entry = entry.sibling) {
            if (entry.sym.isLocal()) {
                scope.enter(entry.sym);
            }
        }
        if (jCClassDecl.typarams != null) {
            for (List list = jCClassDecl.typarams; list.nonEmpty(); list = list.tail) {
                scope.enter(((JCTree.JCTypeParameter) list.head).type.tsym);
            }
        }
        Env<A> env2 = env.outer;
        Env<AttrContext> envDup = env2.dup(jCClassDecl, (A) ((AttrContext) env2.info).dup(scope));
        envDup.baseClause = true;
        envDup.outer = env2;
        envDup.info.isSelfCall = false;
        return envDup;
    }

    private void finish(Env<AttrContext> env) {
        JavaFileObject prev = this.log.useSource(env.toplevel.sourcefile);
        try {
            JCTree.JCClassDecl tree = (JCTree.JCClassDecl) env.tree;
            finishClass(tree, env);
        } finally {
            this.log.useSource(prev);
        }
    }

    private JCTree.JCExpression enumBase(int pos, Symbol.ClassSymbol c) {
        JCTree.JCExpression result = this.make.at(pos).TypeApply(this.make.QualIdent(this.syms.enumSym), List.of(this.make.Type(c.type)));
        return result;
    }

    Type modelMissingTypes(Type t, final JCTree.JCExpression tree, final boolean interfaceExpected) {
        if (!t.hasTag(TypeTag.ERROR)) {
            return t;
        }
        return new Type.ErrorType(t.getOriginalType(), t.tsym) { // from class: com.sun.tools.javac.comp.MemberEnter.9
            private Type modelType;

            @Override // com.sun.tools.javac.code.Type
            public Type getModelType() {
                if (this.modelType == null) {
                    this.modelType = MemberEnter.this.new Synthesizer(getOriginalType(), interfaceExpected).visit(tree);
                }
                return this.modelType;
            }
        };
    }

    private class Synthesizer extends JCTree.Visitor {
        boolean interfaceExpected;
        Type originalType;
        Type result;
        List<Symbol.ClassSymbol> synthesizedSymbols = List.nil();

        Synthesizer(Type originalType, boolean interfaceExpected) {
            this.originalType = originalType;
            this.interfaceExpected = interfaceExpected;
        }

        Type visit(JCTree tree) {
            tree.accept(this);
            return this.result;
        }

        List<Type> visit(List<? extends JCTree> trees) {
            ListBuffer<Type> lb = new ListBuffer<>();
            for (JCTree t : trees) {
                lb.append(visit(t));
            }
            return lb.toList();
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTree(JCTree tree) {
            this.result = MemberEnter.this.syms.errType;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            if (tree.type.hasTag(TypeTag.ERROR)) {
                this.result = synthesizeClass(tree.name, MemberEnter.this.syms.unnamedPackage).type;
            } else {
                this.result = tree.type;
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            if (!tree.type.hasTag(TypeTag.ERROR)) {
                this.result = tree.type;
                return;
            }
            boolean prev = this.interfaceExpected;
            try {
                this.interfaceExpected = false;
                Type selectedType = visit(tree.selected);
                this.interfaceExpected = prev;
                Symbol.ClassSymbol c = synthesizeClass(tree.name, selectedType.tsym);
                this.result = c.type;
            } catch (Throwable th) {
                this.interfaceExpected = prev;
                throw th;
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeApply(JCTree.JCTypeApply tree) {
            if (!tree.type.hasTag(TypeTag.ERROR)) {
                this.result = tree.type;
                return;
            }
            Type.ClassType clazzType = (Type.ClassType) visit(tree.clazz);
            if (this.synthesizedSymbols.contains(clazzType.tsym)) {
                synthesizeTyparams((Symbol.ClassSymbol) clazzType.tsym, tree.arguments.size());
            }
            final List<Type> actuals = visit(tree.arguments);
            this.result = new Type.ErrorType(tree.type, clazzType.tsym) { // from class: com.sun.tools.javac.comp.MemberEnter.Synthesizer.1
                @Override // com.sun.tools.javac.code.Type.ErrorType, com.sun.tools.javac.code.Type.ClassType, javax.lang.model.type.DeclaredType
                public List<Type> getTypeArguments() {
                    return actuals;
                }
            };
        }

        Symbol.ClassSymbol synthesizeClass(Name name, Symbol owner) {
            int flags = this.interfaceExpected ? 512 : 0;
            Symbol.ClassSymbol c = new Symbol.ClassSymbol(flags, name, owner);
            c.members_field = new Scope.ErrorScope(c);
            c.type = new Type.ErrorType(this.originalType, c) { // from class: com.sun.tools.javac.comp.MemberEnter.Synthesizer.2
                @Override // com.sun.tools.javac.code.Type.ErrorType, com.sun.tools.javac.code.Type.ClassType, javax.lang.model.type.DeclaredType
                public List<Type> getTypeArguments() {
                    return this.typarams_field;
                }
            };
            this.synthesizedSymbols = this.synthesizedSymbols.prepend(c);
            return c;
        }

        void synthesizeTyparams(Symbol.ClassSymbol sym, int n) {
            Type.ClassType ct = (Type.ClassType) sym.type;
            Assert.check(ct.typarams_field.isEmpty());
            if (n == 1) {
                Type.TypeVar v = new Type.TypeVar(MemberEnter.this.names.fromString("T"), sym, MemberEnter.this.syms.botType);
                ct.typarams_field = ct.typarams_field.prepend(v);
                return;
            }
            for (int i = n; i > 0; i--) {
                Type.TypeVar v2 = new Type.TypeVar(MemberEnter.this.names.fromString("T" + i), sym, MemberEnter.this.syms.botType);
                ct.typarams_field = ct.typarams_field.prepend(v2);
            }
        }
    }

    JCTree DefaultConstructor(TreeMaker make, Symbol.ClassSymbol c, Symbol.MethodSymbol baseInit, List<Type> typarams, List<Type> argtypes, List<Type> thrown, long flags, boolean based) {
        long flags2;
        long flags3;
        if ((c.flags() & 16384) != 0 && this.types.supertype(c.type).tsym == this.syms.enumSym) {
            flags2 = (flags & (-8)) | 2 | Flags.GENERATEDCONSTR;
        } else {
            flags2 = flags | (c.flags() & 7) | Flags.GENERATEDCONSTR;
        }
        if (!c.name.isEmpty()) {
            flags3 = flags2;
        } else {
            flags3 = flags2 | 536870912;
        }
        Type mType = new Type.MethodType(argtypes, null, thrown, c);
        Type initType = typarams.nonEmpty() ? new Type.ForAll(typarams, mType) : mType;
        Symbol.MethodSymbol init = new Symbol.MethodSymbol(flags3, this.names.init, initType, c);
        init.params = createDefaultConstructorParams(make, baseInit, init, argtypes, based);
        List<JCTree.JCVariableDecl> params = make.Params(argtypes, init);
        List<JCTree.JCStatement> stats = List.nil();
        if (c.type != this.syms.objectType) {
            stats = stats.prepend(SuperCall(make, typarams, params, based));
        }
        JCTree result = make.MethodDef(init, make.Block(0L, stats));
        return result;
    }

    /* JADX WARN: Multi-variable type inference failed */
    private List<Symbol.VarSymbol> createDefaultConstructorParams(TreeMaker make, Symbol.MethodSymbol baseInit, Symbol.MethodSymbol init, List<Type> argtypes, boolean based) {
        List<Symbol.VarSymbol> initParams = null;
        List list = argtypes;
        if (based) {
            List<Symbol.VarSymbol> initParams2 = List.nil();
            Symbol.VarSymbol param = new Symbol.VarSymbol(8589934592L, make.paramName(0), argtypes.head, init);
            initParams = initParams2.append(param);
            list = list.tail;
        }
        if (baseInit != null && baseInit.params != null && baseInit.params.nonEmpty() && list.nonEmpty()) {
            initParams = initParams == null ? List.nil() : initParams;
            List list2 = baseInit.params;
            while (list2.nonEmpty() && list.nonEmpty()) {
                Symbol.VarSymbol param2 = new Symbol.VarSymbol(((Symbol.VarSymbol) list2.head).flags() | 8589934592L, ((Symbol.VarSymbol) list2.head).name, (Type) list.head, init);
                initParams = initParams.append(param2);
                list2 = list2.tail;
                list = list.tail;
            }
        }
        return initParams;
    }

    JCTree.JCExpressionStatement SuperCall(TreeMaker treeMaker, List<Type> list, List<JCTree.JCVariableDecl> list2, boolean z) {
        JCTree.JCExpression jCExpressionIdent;
        List<JCTree.JCVariableDecl> list3;
        if (z) {
            jCExpressionIdent = treeMaker.Select(treeMaker.Ident(list2.head), this.names._super);
            list3 = list2.tail;
        } else {
            jCExpressionIdent = treeMaker.Ident(this.names._super);
            list3 = list2;
        }
        return treeMaker.Exec(treeMaker.Apply(list.nonEmpty() ? treeMaker.Types(list) : null, jCExpressionIdent, treeMaker.Idents(list3)));
    }
}
