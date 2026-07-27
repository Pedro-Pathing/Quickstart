package com.sun.tools.javac.comp;

import com.sun.source.tree.IdentifierTree;
import com.sun.source.tree.LambdaExpressionTree;
import com.sun.source.tree.MemberReferenceTree;
import com.sun.source.tree.MemberSelectTree;
import com.sun.source.tree.Tree;
import com.sun.source.tree.TreeVisitor;
import com.sun.source.util.SimpleTreeVisitor;
import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.BoundKind;
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
import com.sun.tools.javac.comp.Check;
import com.sun.tools.javac.comp.DeferredAttr;
import com.sun.tools.javac.comp.DeferredAttr.DeferredType;
import com.sun.tools.javac.comp.DeferredAttr.DeferredTypeMap;
import com.sun.tools.javac.comp.DeferredAttr.RecoveryDeferredTypeMap;
import com.sun.tools.javac.comp.Infer;
import com.sun.tools.javac.comp.Resolve;
import com.sun.tools.javac.comp.Resolve.AccessError;
import com.sun.tools.javac.comp.Resolve.MethodReferenceCheck;
import com.sun.tools.javac.comp.Resolve.ResolveDeferredRecoveryMap;
import com.sun.tools.javac.comp.Resolve.StaticError;
import com.sun.tools.javac.jvm.Target;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.tree.TreeTranslator;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.Pair;
import com.sun.tools.javac.util.Warner;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import javax.lang.model.element.ElementKind;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class Attr extends JCTree.Visitor {
    static final boolean allowDiamondFinder = true;
    boolean allowAnonOuterThis;
    boolean allowBoxing;
    boolean allowCovariantReturns;
    boolean allowDefaultMethods;
    boolean allowEnums;
    boolean allowGenerics;
    boolean allowLambda;
    boolean allowPoly;
    boolean allowStaticInterfaceMethods;
    boolean allowStringsInSwitch;
    boolean allowTypeAnnos;
    boolean allowVarargs;
    final Annotate annotate;
    final ConstFold cfolder;
    final Check chk;
    final DeferredAttr deferredAttr;
    final DeferredLintHandler deferredLintHandler;
    final JCDiagnostic.Factory diags;
    final Enter enter;
    Env<AttrContext> env;
    boolean findDiamonds;
    final Flow flow;
    boolean identifyLambdaCandidate;
    final Infer infer;
    final Log log;
    final TreeMaker make;
    final MemberEnter memberEnter;
    final Names names;
    JCTree noCheckTree;
    final ResultInfo recoveryInfo;
    boolean relax;
    Type result;
    ResultInfo resultInfo;
    final Resolve rs;
    String sourceName;
    final ResultInfo statInfo;
    final Symtab syms;
    final Target target;
    final TypeAnnotations typeAnnotations;
    final TypeEnvs typeEnvs;
    final Types types;
    final ResultInfo unknownAnyPolyInfo;
    final ResultInfo unknownExprInfo;
    final ResultInfo unknownTypeExprInfo;
    final ResultInfo unknownTypeInfo;
    boolean useBeforeDeclarationWarning;
    final ResultInfo varInfo;
    protected static final Context.Key<Attr> attrKey = new Context.Key<>();
    static final TypeTag[] primitiveTags = {TypeTag.BYTE, TypeTag.CHAR, TypeTag.SHORT, TypeTag.INT, TypeTag.LONG, TypeTag.FLOAT, TypeTag.DOUBLE, TypeTag.BOOLEAN};
    public static final Filter<Symbol> anyNonAbstractOrDefaultMethod = new Filter<Symbol>() { // from class: com.sun.tools.javac.comp.Attr.14
        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol s) {
            return s.kind == 16 && (s.flags() & 8796093023232L) != 1024;
        }
    };
    private TreeVisitor<Symbol, Env<AttrContext>> identAttributer = new IdentAttributer();
    private JCTree breakTree = null;
    TreeTranslator removeClassParams = new TreeTranslator() { // from class: com.sun.tools.javac.comp.Attr.4
        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeApply(JCTree.JCTypeApply tree) {
            this.result = translate(tree.clazz);
        }
    };
    Types.MapVisitor<JCDiagnostic.DiagnosticPosition> targetChecker = new Types.MapVisitor<JCDiagnostic.DiagnosticPosition>() { // from class: com.sun.tools.javac.comp.Attr.8
        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitClassType(Type.ClassType t, JCDiagnostic.DiagnosticPosition pos) {
            return t.isIntersection() ? visitIntersectionClassType((Type.IntersectionClassType) t, pos) : t;
        }

        public Type visitIntersectionClassType(Type.IntersectionClassType ict, JCDiagnostic.DiagnosticPosition pos) {
            Symbol desc = Attr.this.types.findDescriptorSymbol(makeNotionalInterface(ict));
            Type target = null;
            for (Type bound : ict.getExplicitComponents()) {
                Symbol.TypeSymbol boundSym = bound.tsym;
                if (Attr.this.types.isFunctionalInterface(boundSym) && Attr.this.types.findDescriptorSymbol(boundSym) == desc) {
                    target = bound;
                } else if (!boundSym.isInterface() || (boundSym.flags() & 8192) != 0) {
                    reportIntersectionError(pos, "not.an.intf.component", boundSym);
                }
            }
            return target != null ? target : ict.getExplicitComponents().head;
        }

        private Symbol.TypeSymbol makeNotionalInterface(Type.IntersectionClassType ict) {
            ListBuffer<Type> targs = new ListBuffer<>();
            ListBuffer<Type> supertypes = new ListBuffer<>();
            for (Type i : ict.interfaces_field) {
                if (i.isParameterized()) {
                    targs.appendList(i.tsym.type.allparams());
                }
                supertypes.append(i.tsym.type);
            }
            Type.IntersectionClassType notionalIntf = Attr.this.types.makeIntersectionType(supertypes.toList());
            notionalIntf.allparams_field = targs.toList();
            notionalIntf.tsym.flags_field |= 512;
            return notionalIntf.tsym;
        }

        private void reportIntersectionError(JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
            Attr.this.resultInfo.checkContext.report(pos, Attr.this.diags.fragment("bad.intersection.target.for.functional.expr", Attr.this.diags.fragment(key, args)));
        }
    };
    private Map<Symbol.ClassSymbol, Symbol.MethodSymbol> clinits = new HashMap();
    Warner noteWarner = new Warner();

    public static Attr instance(Context context) {
        Attr instance = (Attr) context.get(attrKey);
        if (instance == null) {
            return new Attr(context);
        }
        return instance;
    }

    protected Attr(Context context) {
        context.put(attrKey, this);
        this.names = Names.instance(context);
        this.log = Log.instance(context);
        this.syms = Symtab.instance(context);
        this.rs = Resolve.instance(context);
        this.chk = Check.instance(context);
        this.flow = Flow.instance(context);
        this.memberEnter = MemberEnter.instance(context);
        this.make = TreeMaker.instance(context);
        this.enter = Enter.instance(context);
        this.infer = Infer.instance(context);
        this.deferredAttr = DeferredAttr.instance(context);
        this.cfolder = ConstFold.instance(context);
        this.target = Target.instance(context);
        this.types = Types.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        this.annotate = Annotate.instance(context);
        this.typeAnnotations = TypeAnnotations.instance(context);
        this.deferredLintHandler = DeferredLintHandler.instance(context);
        this.typeEnvs = TypeEnvs.instance(context);
        Options options = Options.instance(context);
        Source source = Source.instance(context);
        this.allowGenerics = source.allowGenerics();
        this.allowVarargs = source.allowVarargs();
        this.allowEnums = source.allowEnums();
        this.allowBoxing = source.allowBoxing();
        this.allowCovariantReturns = source.allowCovariantReturns();
        this.allowAnonOuterThis = source.allowAnonOuterThis();
        this.allowStringsInSwitch = source.allowStringsInSwitch();
        this.allowPoly = source.allowPoly();
        this.allowTypeAnnos = source.allowTypeAnnotations();
        this.allowLambda = source.allowLambda();
        this.allowDefaultMethods = source.allowDefaultMethods();
        this.allowStaticInterfaceMethods = source.allowStaticInterfaceMethods();
        this.sourceName = source.name;
        this.relax = options.isSet("-retrofit") || options.isSet("-relax");
        this.findDiamonds = options.get("findDiamond") != null && source.allowDiamond();
        this.useBeforeDeclarationWarning = options.isSet("useBeforeDeclarationWarning");
        this.identifyLambdaCandidate = options.getBoolean("identifyLambdaCandidate", false);
        this.statInfo = new ResultInfo(this, 0, Type.noType);
        this.varInfo = new ResultInfo(this, 4, Type.noType);
        this.unknownExprInfo = new ResultInfo(this, 12, Type.noType);
        this.unknownAnyPolyInfo = new ResultInfo(this, 12, Infer.anyPoly);
        this.unknownTypeInfo = new ResultInfo(this, 2, Type.noType);
        this.unknownTypeExprInfo = new ResultInfo(this, 14, Type.noType);
        this.recoveryInfo = new RecoveryInfo(this.deferredAttr.emptyDeferredAttrContext);
        this.noCheckTree = this.make.at(-1).Skip();
    }

    Type check(final JCTree tree, final Type found, final int ownkind, final ResultInfo resultInfo) {
        Type owntype;
        Infer.InferenceContext inferenceContext = resultInfo.checkContext.inferenceContext();
        boolean shouldCheck = (found.hasTag(TypeTag.ERROR) || resultInfo.pt.hasTag(TypeTag.METHOD) || resultInfo.pt.hasTag(TypeTag.FORALL)) ? false : true;
        if (shouldCheck && ((~resultInfo.pkind) & ownkind) != 0) {
            this.log.error(tree.pos(), "unexpected.type", Kinds.kindNames(resultInfo.pkind), Kinds.kindName(ownkind));
            owntype = this.types.createErrorType(found);
        } else if (this.allowPoly && inferenceContext.free(found)) {
            owntype = shouldCheck ? resultInfo.pt : found;
            inferenceContext.addFreeTypeListener(List.of(found, resultInfo.pt), new Infer.FreeTypeListener() { // from class: com.sun.tools.javac.comp.Attr.1
                @Override // com.sun.tools.javac.comp.Infer.FreeTypeListener
                public void typesInferred(Infer.InferenceContext inferenceContext2) {
                    ResultInfo pendingResult = resultInfo.dup(inferenceContext2.asInstType(resultInfo.pt));
                    Attr.this.check(tree, inferenceContext2.asInstType(found), ownkind, pendingResult);
                }
            });
        } else {
            owntype = shouldCheck ? resultInfo.check(tree, found) : found;
        }
        if (tree != this.noCheckTree) {
            tree.type = owntype;
        }
        return owntype;
    }

    boolean isAssignableAsBlankFinal(Symbol.VarSymbol v, Env<AttrContext> env) {
        Symbol owner = env.info.scope.owner;
        if (v.owner == owner) {
            return true;
        }
        if ((owner.name == this.names.init || owner.kind == 4 || (owner.flags() & 1048576) != 0) && v.owner == owner.owner) {
            if (((v.flags() & 8) != 0) == Resolve.isStatic(env)) {
                return true;
            }
        }
        return false;
    }

    void checkAssignable(JCDiagnostic.DiagnosticPosition pos, Symbol.VarSymbol v, JCTree base, Env<AttrContext> env) {
        if ((v.flags() & 16) != 0) {
            if ((v.flags() & 262144) != 0 || ((base != null && (!base.hasTag(JCTree.Tag.IDENT) || TreeInfo.name(base) != this.names._this)) || !isAssignableAsBlankFinal(v, env))) {
                if (v.isResourceVariable()) {
                    this.log.error(pos, "try.resource.may.not.be.assigned", v);
                } else {
                    this.log.error(pos, "cant.assign.val.to.final.var", v);
                }
            }
        }
    }

    boolean isStaticReference(JCTree tree) {
        if (tree.hasTag(JCTree.Tag.SELECT)) {
            Symbol lsym = TreeInfo.symbol(((JCTree.JCFieldAccess) tree).selected);
            if (lsym == null || lsym.kind != 2) {
                return false;
            }
            return true;
        }
        return true;
    }

    static boolean isType(Symbol sym) {
        return sym != null && sym.kind == 2;
    }

    Symbol thisSym(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env) {
        return this.rs.resolveSelf(pos, env, env.enclClass.sym, this.names._this);
    }

    public Symbol attribIdent(JCTree tree, JCTree.JCCompilationUnit topLevel) {
        Env<AttrContext> localEnv = this.enter.topLevelEnv(topLevel);
        localEnv.enclClass = this.make.ClassDef(this.make.Modifiers(0L), this.syms.errSymbol.name, null, null, null, null);
        localEnv.enclClass.sym = this.syms.errSymbol;
        return (Symbol) tree.accept(this.identAttributer, localEnv);
    }

    private class IdentAttributer extends SimpleTreeVisitor<Symbol, Env<AttrContext>> {
        private IdentAttributer() {
        }

        @Override // com.sun.source.util.SimpleTreeVisitor, com.sun.source.tree.TreeVisitor
        public Symbol visitMemberSelect(MemberSelectTree node, Env<AttrContext> env) {
            Symbol site = visit(node.getExpression(), env);
            if (site.kind == 63 || site.kind == 137) {
                return site;
            }
            Name name = (Name) node.getIdentifier();
            if (site.kind == 1) {
                env.toplevel.packge = (Symbol.PackageSymbol) site;
                return Attr.this.rs.findIdentInPackage(env, (Symbol.TypeSymbol) site, name, 3);
            }
            env.enclClass.sym = (Symbol.ClassSymbol) site;
            return Attr.this.rs.findMemberType(env, site.asType(), name, (Symbol.TypeSymbol) site);
        }

        @Override // com.sun.source.util.SimpleTreeVisitor, com.sun.source.tree.TreeVisitor
        public Symbol visitIdentifier(IdentifierTree node, Env<AttrContext> env) {
            return Attr.this.rs.findIdent(env, (Name) node.getName(), 3);
        }
    }

    public Type coerce(Type etype, Type ttype) {
        return this.cfolder.coerce(etype, ttype);
    }

    public Type attribType(JCTree node, Symbol.TypeSymbol sym) {
        Env<AttrContext> env = this.typeEnvs.get(sym);
        Env<AttrContext> localEnv = env.dup(node, env.info.dup());
        return attribTree(node, localEnv, this.unknownTypeInfo);
    }

    public Type attribImportQualifier(JCTree.JCImport tree, Env<AttrContext> env) {
        JCTree.JCFieldAccess s = (JCTree.JCFieldAccess) tree.qualid;
        return attribTree(s.selected, env, new ResultInfo(this, tree.staticImport ? 2 : 3, Type.noType));
    }

    public Env<AttrContext> attribExprToTree(JCTree expr, Env<AttrContext> env, JCTree tree) {
        this.breakTree = tree;
        JavaFileObject prev = this.log.useSource(env.toplevel.sourcefile);
        try {
            attribExpr(expr, env);
            return env;
        } catch (BreakAttr b) {
            return b.env;
        } catch (AssertionError ae) {
            if (!(ae.getCause() instanceof BreakAttr)) {
                throw ae;
            }
            BreakAttr breakAttr = (BreakAttr) ae.getCause();
            return breakAttr.env;
        } finally {
            this.breakTree = null;
            this.log.useSource(prev);
        }
    }

    public Env<AttrContext> attribStatToTree(JCTree stmt, Env<AttrContext> env, JCTree tree) {
        this.breakTree = tree;
        JavaFileObject prev = this.log.useSource(env.toplevel.sourcefile);
        try {
            attribStat(stmt, env);
            return env;
        } catch (BreakAttr b) {
            return b.env;
        } catch (AssertionError ae) {
            if (!(ae.getCause() instanceof BreakAttr)) {
                throw ae;
            }
            BreakAttr breakAttr = (BreakAttr) ae.getCause();
            return breakAttr.env;
        } finally {
            this.breakTree = null;
            this.log.useSource(prev);
        }
    }

    private static class BreakAttr extends RuntimeException {
        static final long serialVersionUID = -6924771130405446405L;
        private Env<AttrContext> env;

        private BreakAttr(Env<AttrContext> env) {
            this.env = env;
        }
    }

    class ResultInfo {
        final Check.CheckContext checkContext;
        final int pkind;
        final Type pt;

        ResultInfo(Attr this$0, int pkind, Type pt) {
            this(pkind, pt, this$0.chk.basicHandler);
        }

        protected ResultInfo(int pkind, Type pt, Check.CheckContext checkContext) {
            this.pkind = pkind;
            this.pt = pt;
            this.checkContext = checkContext;
        }

        protected Type check(JCDiagnostic.DiagnosticPosition pos, Type found) {
            return Attr.this.chk.checkType(pos, found, this.pt, this.checkContext);
        }

        protected ResultInfo dup(Type newPt) {
            return Attr.this.new ResultInfo(this.pkind, newPt, this.checkContext);
        }

        protected ResultInfo dup(Check.CheckContext newContext) {
            return Attr.this.new ResultInfo(this.pkind, this.pt, newContext);
        }

        protected ResultInfo dup(Type newPt, Check.CheckContext newContext) {
            return Attr.this.new ResultInfo(this.pkind, newPt, newContext);
        }

        public String toString() {
            if (this.pt != null) {
                return this.pt.toString();
            }
            return "";
        }
    }

    class RecoveryInfo extends ResultInfo {
        public RecoveryInfo(final DeferredAttr.DeferredAttrContext deferredAttrContext) {
            super(12, Type.recoveryType, new Check.NestedCheckContext(Attr.this.chk.basicHandler) { // from class: com.sun.tools.javac.comp.Attr.RecoveryInfo.1
                @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                public DeferredAttr.DeferredAttrContext deferredAttrContext() {
                    return deferredAttrContext;
                }

                @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                public boolean compatible(Type found, Type req, Warner warn) {
                    return true;
                }

                @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                public void report(JCDiagnostic.DiagnosticPosition pos, JCDiagnostic details) {
                    attr.chk.basicHandler.report(pos, details);
                }
            });
        }
    }

    Type pt() {
        return this.resultInfo.pt;
    }

    int pkind() {
        return this.resultInfo.pkind;
    }

    Type attribTree(JCTree tree, Env<AttrContext> env, ResultInfo resultInfo) {
        Env<AttrContext> prevEnv = this.env;
        ResultInfo prevResult = this.resultInfo;
        try {
            this.env = env;
            this.resultInfo = resultInfo;
            tree.accept(this);
            if (tree == this.breakTree && resultInfo.checkContext.deferredAttrContext().mode == DeferredAttr.AttrMode.CHECK) {
                throw new BreakAttr(copyEnv(env));
            }
            return this.result;
        } catch (Symbol.CompletionFailure ex) {
            tree.type = this.syms.errType;
            return this.chk.completionError(tree.pos(), ex);
        } finally {
            this.env = prevEnv;
            this.resultInfo = prevResult;
        }
    }

    Env<AttrContext> copyEnv(Env<AttrContext> env) {
        Env<AttrContext> newEnv = env.dup(env.tree, env.info.dup(copyScope(env.info.scope)));
        if (newEnv.outer != null) {
            newEnv.outer = copyEnv(newEnv.outer);
        }
        return newEnv;
    }

    Scope copyScope(Scope sc) {
        Scope newScope = new Scope(sc.owner);
        List<Symbol> elemsList = List.nil();
        while (sc != null) {
            for (Scope.Entry e = sc.elems; e != null; e = e.sibling) {
                elemsList = elemsList.prepend(e.sym);
            }
            sc = sc.next;
        }
        for (Symbol s : elemsList) {
            newScope.enter(s);
        }
        return newScope;
    }

    public Type attribExpr(JCTree tree, Env<AttrContext> env, Type pt) {
        return attribTree(tree, env, new ResultInfo(this, 12, !pt.hasTag(TypeTag.ERROR) ? pt : Type.noType));
    }

    public Type attribExpr(JCTree tree, Env<AttrContext> env) {
        return attribTree(tree, env, this.unknownExprInfo);
    }

    public Type attribType(JCTree tree, Env<AttrContext> env) {
        Type result = attribType(tree, env, Type.noType);
        return result;
    }

    Type attribType(JCTree tree, Env<AttrContext> env, Type pt) {
        Type result = attribTree(tree, env, new ResultInfo(this, 2, pt));
        return result;
    }

    public Type attribStat(JCTree tree, Env<AttrContext> env) {
        return attribTree(tree, env, this.statInfo);
    }

    /* JADX WARN: Multi-variable type inference failed */
    List<Type> attribExprs(List<JCTree.JCExpression> trees, Env<AttrContext> env, Type pt) {
        ListBuffer<Type> ts = new ListBuffer<>();
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            ts.append(attribExpr((JCTree) list.head, env, pt));
        }
        return ts.toList();
    }

    /* JADX WARN: Multi-variable type inference failed */
    <T extends JCTree> void attribStats(List<T> trees, Env<AttrContext> env) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            attribStat((JCTree) list.head, env);
        }
    }

    int attribArgs(int initialKind, List<JCTree.JCExpression> trees, Env<AttrContext> env, ListBuffer<Type> argtypes) {
        Type argtype;
        int kind = initialKind;
        for (JCTree.JCExpression arg : trees) {
            if (this.allowPoly && this.deferredAttr.isDeferred(env, arg)) {
                DeferredAttr deferredAttr = this.deferredAttr;
                deferredAttr.getClass();
                argtype = deferredAttr.new DeferredType(arg, env);
                kind |= 32;
            } else {
                argtype = this.chk.checkNonVoid(arg, attribTree(arg, env, this.unknownAnyPolyInfo));
            }
            argtypes.append(argtype);
        }
        return kind;
    }

    /* JADX WARN: Multi-variable type inference failed */
    List<Type> attribAnyTypes(List<JCTree.JCExpression> trees, Env<AttrContext> env) {
        ListBuffer<Type> argtypes = new ListBuffer<>();
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            argtypes.append(attribType((JCTree) list.head, env));
        }
        return argtypes.toList();
    }

    List<Type> attribTypes(List<JCTree.JCExpression> trees, Env<AttrContext> env) {
        List<Type> types = attribAnyTypes(trees, env);
        return this.chk.checkRefTypes(trees, types);
    }

    void attribTypeVariables(List<JCTree.JCTypeParameter> typarams, Env<AttrContext> env) {
        for (JCTree.JCTypeParameter tvar : typarams) {
            Type.TypeVar a = (Type.TypeVar) tvar.type;
            a.tsym.flags_field |= 268435456;
            a.bound = Type.noType;
            if (!tvar.bounds.isEmpty()) {
                List<Type> bounds = List.of(attribType(tvar.bounds.head, env));
                for (JCTree.JCExpression bound : tvar.bounds.tail) {
                    bounds = bounds.prepend(attribType(bound, env));
                }
                this.types.setBounds(a, bounds.reverse());
            } else {
                this.types.setBounds(a, List.of(this.syms.objectType));
            }
            a.tsym.flags_field &= -268435457;
        }
        for (JCTree.JCTypeParameter tvar2 : typarams) {
            this.chk.checkNonCyclic(tvar2.pos(), (Type.TypeVar) tvar2.type);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void attribAnnotationTypes(List<JCTree.JCAnnotation> annotations, Env<AttrContext> env) {
        for (List list = annotations; list.nonEmpty(); list = list.tail) {
            JCTree.JCAnnotation a = (JCTree.JCAnnotation) list.head;
            attribType(a.annotationType, env);
        }
    }

    public Object attribLazyConstantValue(Env<AttrContext> env, JCTree.JCVariableDecl variable, Type type) {
        JCDiagnostic.DiagnosticPosition prevLintPos = this.deferredLintHandler.setPos(variable.pos());
        try {
            this.memberEnter.typeAnnotate(variable.init, env, null, variable.pos());
            this.annotate.flush();
            Type itype = attribExpr(variable.init, env, type);
            if (itype.constValue() != null) {
                return coerce(itype, type).constValue();
            }
            return null;
        } finally {
            this.deferredLintHandler.setPos(prevLintPos);
        }
    }

    Type attribBase(JCTree tree, Env<AttrContext> env, boolean classExpected, boolean interfaceExpected, boolean checkExtensible) {
        Type t = tree.type != null ? tree.type : attribType(tree, env);
        return checkBase(t, tree, env, classExpected, interfaceExpected, checkExtensible);
    }

    Type checkBase(Type t, JCTree tree, Env<AttrContext> env, boolean classExpected, boolean interfaceExpected, boolean checkExtensible) {
        if (t.tsym.isAnonymous()) {
            this.log.error(tree.pos(), "cant.inherit.from.anon", new Object[0]);
            return this.types.createErrorType(t);
        }
        if (t.isErroneous()) {
            return t;
        }
        if (t.hasTag(TypeTag.TYPEVAR) && !classExpected && !interfaceExpected) {
            if (t.getUpperBound() == null) {
                this.log.error(tree.pos(), "illegal.forward.ref", new Object[0]);
                return this.types.createErrorType(t);
            }
        } else {
            t = this.chk.checkClassType(tree.pos(), t, (!this.allowGenerics) | checkExtensible);
        }
        if (interfaceExpected && (t.tsym.flags() & 512) == 0) {
            this.log.error(tree.pos(), "intf.expected.here", new Object[0]);
            return this.types.createErrorType(t);
        }
        if (checkExtensible && classExpected && (512 & t.tsym.flags()) != 0) {
            this.log.error(tree.pos(), "no.intf.expected.here", new Object[0]);
            return this.types.createErrorType(t);
        }
        if (checkExtensible && (t.tsym.flags() & 16) != 0) {
            this.log.error(tree.pos(), "cant.inherit.from.final", t.tsym);
        }
        this.chk.checkNonCyclic(tree.pos(), t);
        return t;
    }

    Type attribIdentAsEnumType(Env<AttrContext> env, JCTree.JCIdent id) {
        Assert.check((env.enclClass.sym.flags() & 16384) != 0);
        id.type = env.info.scope.owner.type;
        id.sym = env.info.scope.owner;
        return id.type;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitClassDef(JCTree.JCClassDecl tree) {
        if ((this.env.info.scope.owner.kind & 20) != 0) {
            this.enter.classEnter(tree, this.env);
        } else if (this.env.tree.hasTag(JCTree.Tag.NEWCLASS) && TreeInfo.isInAnnotation(this.env, tree)) {
            this.enter.classEnter(tree, this.env);
        }
        Symbol.ClassSymbol c = tree.sym;
        if (c == null) {
            this.result = null;
            return;
        }
        c.complete();
        if (this.env.info.isSelfCall && this.env.tree.hasTag(JCTree.Tag.NEWCLASS) && ((JCTree.JCNewClass) this.env.tree).encl == null) {
            c.flags_field |= 4194304;
        }
        attribClass(tree.pos(), c);
        Type type = c.type;
        tree.type = type;
        this.result = type;
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitMethodDef(JCTree.JCMethodDecl tree) {
        Symbol.MethodSymbol m = tree.sym;
        boolean isDefaultMethod = (m.flags() & Flags.DEFAULT) != 0;
        Lint lint = this.env.info.lint.augment(m);
        Lint prevLint = this.chk.setLint(lint);
        Symbol.MethodSymbol prevMethod = this.chk.setMethod(m);
        try {
            this.deferredLintHandler.flush(tree.pos());
            this.chk.checkDeprecatedAnnotation(tree.pos(), m);
            Env<AttrContext> localEnv = this.memberEnter.methodEnv(tree, this.env);
            localEnv.info.lint = lint;
            attribStats(tree.typarams, localEnv);
            if (m.isStatic()) {
                this.chk.checkHideClashes(tree.pos(), this.env.enclClass.type, m);
            } else {
                this.chk.checkOverrideClashes(tree.pos(), this.env.enclClass.type, m);
            }
            this.chk.checkOverride(tree, m);
            if (isDefaultMethod && this.types.overridesObjectMethod(m.enclClass(), m)) {
                this.log.error(tree, "default.overrides.object.member", m.name, Kinds.kindName(m.location()), m.location());
            }
            for (List list = tree.typarams; list.nonEmpty(); list = list.tail) {
                localEnv.info.scope.enterIfAbsent(((JCTree.JCTypeParameter) list.head).type.tsym);
            }
            Symbol.ClassSymbol owner = this.env.enclClass.sym;
            if ((owner.flags() & 8192) != 0 && tree.params.nonEmpty()) {
                this.log.error(tree.params.head.pos(), "intf.annotation.members.cant.have.params", new Object[0]);
            }
            for (List list2 = tree.params; list2.nonEmpty(); list2 = list2.tail) {
                attribStat((JCTree) list2.head, localEnv);
            }
            this.chk.checkVarargsMethodDecl(localEnv, tree);
            this.chk.validate(tree.typarams, localEnv);
            if (tree.restype != null && !tree.restype.type.hasTag(TypeTag.VOID)) {
                this.chk.validate(tree.restype, localEnv);
            }
            if (tree.recvparam != null) {
                Env<AttrContext> newEnv = this.memberEnter.methodEnv(tree, this.env);
                attribType(tree.recvparam, newEnv);
                this.chk.validate(tree.recvparam, newEnv);
            }
            if ((owner.flags() & 8192) != 0) {
                if (tree.thrown.nonEmpty()) {
                    this.log.error(tree.thrown.head.pos(), "throws.not.allowed.in.intf.annotation", new Object[0]);
                }
                if (tree.typarams.nonEmpty()) {
                    this.log.error(tree.typarams.head.pos(), "intf.annotation.members.cant.have.type.params", new Object[0]);
                }
                this.chk.validateAnnotationType(tree.restype);
                this.chk.validateAnnotationMethod(tree.pos(), m);
            }
            for (List list3 = tree.thrown; list3.nonEmpty(); list3 = list3.tail) {
                this.chk.checkType(((JCTree.JCExpression) list3.head).pos(), ((JCTree.JCExpression) list3.head).type, this.syms.throwableType);
            }
            if (tree.body == null) {
                if (isDefaultMethod || ((tree.sym.flags() & 1280) == 0 && !this.relax)) {
                    this.log.error(tree.pos(), "missing.meth.body.or.decl.abstract", new Object[0]);
                }
                if (tree.defaultValue != null && (owner.flags() & 8192) == 0) {
                    this.log.error(tree.pos(), "default.allowed.in.intf.annotation.member", new Object[0]);
                }
            } else if ((tree.sym.flags() & 1024) == 0 || isDefaultMethod) {
                if ((tree.mods.flags & 256) == 0) {
                    if (tree.name == this.names.init && owner.type != this.syms.objectType) {
                        JCTree.JCBlock body = tree.body;
                        if (body.stats.isEmpty() || !TreeInfo.isSelfCall(body.stats.head)) {
                            body.stats = body.stats.prepend(this.memberEnter.SuperCall(this.make.at(body.pos), List.nil(), List.nil(), false));
                        } else if ((this.env.enclClass.sym.flags() & 16384) != 0 && (tree.mods.flags & Flags.GENERATEDCONSTR) == 0 && TreeInfo.isSuperCall(body.stats.head)) {
                            this.log.error(tree.body.stats.head.pos(), "call.to.super.not.allowed.in.enum.ctor", this.env.enclClass.sym);
                        }
                    }
                    this.memberEnter.typeAnnotate(tree.body, localEnv, m, null);
                    this.annotate.flush();
                    attribStat(tree.body, localEnv);
                } else {
                    this.log.error(tree.pos(), "native.meth.cant.have.body", new Object[0]);
                }
            } else if ((owner.flags() & 512) != 0) {
                this.log.error(tree.body.pos(), "intf.meth.cant.have.body", new Object[0]);
            } else {
                this.log.error(tree.pos(), "abstract.meth.cant.have.body", new Object[0]);
            }
            localEnv.info.scope.leave();
            Type type = m.type;
            tree.type = type;
            this.result = type;
        } finally {
            this.chk.setLint(prevLint);
            this.chk.setMethod(prevMethod);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitVarDef(JCTree.JCVariableDecl tree) {
        if (this.env.info.scope.owner.kind == 16) {
            if (tree.sym != null) {
                this.env.info.scope.enter(tree.sym);
            } else {
                try {
                    this.annotate.enterStart();
                    this.memberEnter.memberEnter(tree, this.env);
                } finally {
                    this.annotate.enterDone();
                }
            }
        } else if (tree.init != null) {
            this.memberEnter.typeAnnotate(tree.init, this.env, tree.sym, tree.pos());
            this.annotate.flush();
        }
        Symbol.VarSymbol v = tree.sym;
        Lint lint = this.env.info.lint.augment(v);
        Lint prevLint = this.chk.setLint(lint);
        boolean isImplicitLambdaParameter = this.env.tree.hasTag(JCTree.Tag.LAMBDA) && ((JCTree.JCLambda) this.env.tree).paramKind == JCTree.JCLambda.ParameterKind.IMPLICIT && (tree.sym.flags() & 8589934592L) != 0;
        this.chk.validate(tree.vartype, this.env, isImplicitLambdaParameter ? false : true);
        try {
            v.getConstValue();
            this.deferredLintHandler.flush(tree.pos());
            this.chk.checkDeprecatedAnnotation(tree.pos(), v);
            if (tree.init != null && ((v.flags_field & 16) == 0 || !this.memberEnter.needsLazyConstValue(tree.init))) {
                Env<AttrContext> initEnv = this.memberEnter.initEnv(tree, this.env);
                initEnv.info.lint = lint;
                initEnv.info.enclVar = v;
                attribExpr(tree.init, initEnv, v.type);
            }
            Type type = v.type;
            tree.type = type;
            this.result = type;
        } finally {
            this.chk.setLint(prevLint);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSkip(JCTree.JCSkip tree) {
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBlock(JCTree.JCBlock tree) {
        if (this.env.info.scope.owner.kind == 2) {
            Env<AttrContext> localEnv = this.env.dup(tree, this.env.info.dup(this.env.info.scope.dupUnshared()));
            localEnv.info.scope.owner = new Symbol.MethodSymbol(tree.flags | 1048576 | (this.env.info.scope.owner.flags() & 2048), this.names.empty, null, this.env.info.scope.owner);
            if ((tree.flags & 8) != 0) {
                localEnv.info.staticLevel++;
            }
            this.memberEnter.typeAnnotate(tree, localEnv, localEnv.info.scope.owner, null);
            this.annotate.flush();
            Symbol.ClassSymbol cs = (Symbol.ClassSymbol) this.env.info.scope.owner;
            List<Attribute.TypeCompound> tas = localEnv.info.scope.owner.getRawTypeAttributes();
            if ((tree.flags & 8) != 0) {
                cs.appendClassInitTypeAttributes(tas);
            } else {
                cs.appendInitTypeAttributes(tas);
            }
            attribStats(tree.stats, localEnv);
        } else {
            Env<AttrContext> localEnv2 = this.env.dup(tree, this.env.info.dup(this.env.info.scope.dup()));
            try {
                attribStats(tree.stats, localEnv2);
            } finally {
                localEnv2.info.scope.leave();
            }
        }
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
        attribStat(tree.body, this.env.dup(tree));
        attribExpr(tree.cond, this.env, this.syms.booleanType);
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWhileLoop(JCTree.JCWhileLoop tree) {
        attribExpr(tree.cond, this.env, this.syms.booleanType);
        attribStat(tree.body, this.env.dup(tree));
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForLoop(JCTree.JCForLoop tree) {
        Env<AttrContext> loopEnv = this.env.dup(this.env.tree, this.env.info.dup(this.env.info.scope.dup()));
        try {
            attribStats(tree.init, loopEnv);
            if (tree.cond != null) {
                attribExpr(tree.cond, loopEnv, this.syms.booleanType);
            }
            loopEnv.tree = tree;
            attribStats(tree.step, loopEnv);
            attribStat(tree.body, loopEnv);
            this.result = null;
        } finally {
            loopEnv.info.scope.leave();
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
        Env<AttrContext> loopEnv = this.env.dup(this.env.tree, this.env.info.dup(this.env.info.scope.dup()));
        try {
            Type exprType = this.types.cvarUpperBound(attribExpr(tree.expr, loopEnv));
            attribStat(tree.var, loopEnv);
            this.chk.checkNonVoid(tree.pos(), exprType);
            Type elemtype = this.types.elemtype(exprType);
            if (elemtype == null) {
                Type base = this.types.asSuper(exprType, this.syms.iterableType.tsym);
                if (base == null) {
                    this.log.error(tree.expr.pos(), "foreach.not.applicable.to.type", exprType, this.diags.fragment("type.req.array.or.iterable", new Object[0]));
                    elemtype = this.types.createErrorType(exprType);
                } else {
                    List<Type> iterableParams = base.allparams();
                    elemtype = iterableParams.isEmpty() ? this.syms.objectType : this.types.wildUpperBound(iterableParams.head);
                }
            }
            this.chk.checkType(tree.expr.pos(), elemtype, tree.var.sym.type);
            loopEnv.tree = tree;
            attribStat(tree.body, loopEnv);
            this.result = null;
        } finally {
            loopEnv.info.scope.leave();
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLabelled(JCTree.JCLabeledStatement tree) {
        Env env = this.env;
        while (true) {
            if (env != null && !env.tree.hasTag(JCTree.Tag.CLASSDEF)) {
                if (env.tree.hasTag(JCTree.Tag.LABELLED) && ((JCTree.JCLabeledStatement) env.tree).label == tree.label) {
                    this.log.error(tree.pos(), "label.already.in.use", tree.label);
                    break;
                }
                env = env.next;
            } else {
                break;
            }
        }
        attribStat(tree.body, this.env.dup(tree));
        this.result = null;
    }

    /* JADX WARN: Incorrect condition in loop: B:21:0x008c */
    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public void visitSwitch(com.sun.tools.javac.tree.JCTree.JCSwitch r17) {
        /*
            Method dump skipped, instruction units count: 391
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Attr.visitSwitch(com.sun.tools.javac.tree.JCTree$JCSwitch):void");
    }

    /* JADX WARN: Multi-variable type inference failed */
    private static void addVars(List<JCTree.JCStatement> list, Scope scope) {
        for (List list2 = list; list2.nonEmpty(); list2 = list2.tail) {
            JCTree jCTree = (JCTree) list2.head;
            if (jCTree.hasTag(JCTree.Tag.VARDEF)) {
                scope.enter(((JCTree.JCVariableDecl) jCTree).sym);
            }
        }
    }

    private Symbol enumConstant(JCTree tree, Type enumType) {
        if (!tree.hasTag(JCTree.Tag.IDENT)) {
            this.log.error(tree.pos(), "enum.label.must.be.unqualified.enum", new Object[0]);
            return this.syms.errSymbol;
        }
        JCTree.JCIdent ident = (JCTree.JCIdent) tree;
        Name name = ident.name;
        for (Scope.Entry e = enumType.tsym.members().lookup(name); e.scope != null; e = e.next()) {
            if (e.sym.kind == 4) {
                Symbol s = e.sym;
                ident.sym = s;
                ((Symbol.VarSymbol) s).getConstValue();
                ident.type = s.type;
                if ((s.flags_field & 16384) == 0) {
                    return null;
                }
                return s;
            }
        }
        return null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSynchronized(JCTree.JCSynchronized tree) {
        this.chk.checkRefType(tree.pos(), attribExpr(tree.lock, this.env));
        attribStat(tree.body, this.env);
        this.result = null;
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTry(JCTree.JCTry tree) {
        Env<AttrContext> catchEnv = this.env.dup(tree, this.env.info.dup(this.env.info.scope.dup()));
        try {
            boolean isTryWithResource = tree.resources.nonEmpty();
            Env<AttrContext> tryEnv = isTryWithResource ? this.env.dup(tree, catchEnv.info.dup(catchEnv.info.scope.dup())) : catchEnv;
            try {
                for (JCTree resource : tree.resources) {
                    Check.CheckContext twrContext = new Check.NestedCheckContext(this.resultInfo.checkContext) { // from class: com.sun.tools.javac.comp.Attr.2
                        @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                        public void report(JCDiagnostic.DiagnosticPosition pos, JCDiagnostic details) {
                            Attr.this.chk.basicHandler.report(pos, Attr.this.diags.fragment("try.not.applicable.to.type", details));
                        }
                    };
                    ResultInfo twrResult = new ResultInfo(12, this.syms.autoCloseableType, twrContext);
                    if (resource.hasTag(JCTree.Tag.VARDEF)) {
                        attribStat(resource, tryEnv);
                        twrResult.check(resource, resource.type);
                        checkAutoCloseable(resource.pos(), catchEnv, resource.type);
                        Symbol.VarSymbol var = ((JCTree.JCVariableDecl) resource).sym;
                        var.setData(ElementKind.RESOURCE_VARIABLE);
                    } else {
                        attribTree(resource, tryEnv, twrResult);
                    }
                }
                attribStat(tree.body, tryEnv);
                for (List list = tree.catchers; list.nonEmpty(); list = list.tail) {
                    JCTree.JCCatch c = (JCTree.JCCatch) list.head;
                    catchEnv = catchEnv.dup(c, catchEnv.info.dup(catchEnv.info.scope.dup()));
                    try {
                        Type ctype = attribStat(c.param, catchEnv);
                        if (TreeInfo.isMultiCatch(c)) {
                            c.param.sym.flags_field |= 549755813904L;
                        }
                        if (c.param.sym.kind == 4) {
                            c.param.sym.setData(ElementKind.EXCEPTION_PARAMETER);
                        }
                        this.chk.checkType(c.param.vartype.pos(), this.chk.checkClassType(c.param.vartype.pos(), ctype), this.syms.throwableType);
                        attribStat(c.body, catchEnv);
                        catchEnv.info.scope.leave();
                    } finally {
                        catchEnv.info.scope.leave();
                    }
                }
                if (tree.finalizer != null) {
                    attribStat(tree.finalizer, catchEnv);
                }
                this.result = null;
            } finally {
                if (isTryWithResource) {
                    tryEnv.info.scope.leave();
                }
            }
        } catch (Throwable th) {
            throw th;
        }
    }

    void checkAutoCloseable(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type resource) {
        if (!resource.isErroneous() && this.types.asSuper(resource, this.syms.autoCloseableType.tsym) != null && !this.types.isSameType(resource, this.syms.autoCloseableType)) {
            Symbol.TypeSymbol typeSymbol = this.syms.noSymbol;
            Log.DiagnosticHandler discardHandler = new Log.DiscardDiagnosticHandler(this.log);
            try {
                Symbol close = this.rs.resolveQualifiedMethod(pos, env, resource, this.names.close, List.nil(), List.nil());
                this.log.popDiagnosticHandler(discardHandler);
                if (close.kind == 16 && close.overrides(this.syms.autoCloseableClose, resource.tsym, this.types, true) && this.chk.isHandled(this.syms.interruptedExceptionType, this.types.memberType(resource, close).mo179getThrownTypes()) && env.info.lint.isEnabled(Lint.LintCategory.TRY)) {
                    this.log.warning(Lint.LintCategory.TRY, pos, "try.resource.throws.interrupted.exc", resource);
                }
            } catch (Throwable th) {
                this.log.popDiagnosticHandler(discardHandler);
                throw th;
            }
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitConditional(JCTree.JCConditional tree) {
        Type condtype = attribExpr(tree.cond, this.env, this.syms.booleanType);
        tree.polyKind = (!this.allowPoly || (pt().hasTag(TypeTag.NONE) && pt() != Type.recoveryType) || isBooleanOrNumeric(this.env, tree)) ? JCTree.JCPolyExpression.PolyKind.STANDALONE : JCTree.JCPolyExpression.PolyKind.POLY;
        if (tree.polyKind == JCTree.JCPolyExpression.PolyKind.POLY && this.resultInfo.pt.hasTag(TypeTag.VOID)) {
            this.resultInfo.checkContext.report(tree, this.diags.fragment("conditional.target.cant.be.void", new Object[0]));
            Type typeCreateErrorType = this.types.createErrorType(this.resultInfo.pt);
            tree.type = typeCreateErrorType;
            this.result = typeCreateErrorType;
            return;
        }
        ResultInfo condInfo = tree.polyKind == JCTree.JCPolyExpression.PolyKind.STANDALONE ? this.unknownExprInfo : this.resultInfo.dup(new Check.NestedCheckContext(this.resultInfo.checkContext) { // from class: com.sun.tools.javac.comp.Attr.3
            @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
            public void report(JCDiagnostic.DiagnosticPosition pos, JCDiagnostic details) {
                this.enclosingContext.report(pos, Attr.this.diags.fragment("incompatible.type.in.conditional", details));
            }
        });
        Type truetype = attribTree(tree.truepart, this.env, condInfo);
        Type falsetype = attribTree(tree.falsepart, this.env, condInfo);
        Type owntype = tree.polyKind == JCTree.JCPolyExpression.PolyKind.STANDALONE ? condType(tree, truetype, falsetype) : pt();
        if (condtype.constValue() != null && truetype.constValue() != null && falsetype.constValue() != null && !owntype.hasTag(TypeTag.NONE)) {
            owntype = this.cfolder.coerce(condtype.isTrue() ? truetype : falsetype, owntype);
        }
        this.result = check(tree, owntype, 12, this.resultInfo);
    }

    private boolean isBooleanOrNumeric(Env<AttrContext> env, JCTree.JCExpression tree) {
        switch (tree.getTag()) {
            case LITERAL:
                return ((JCTree.JCLiteral) tree).typetag.isSubRangeOf(TypeTag.DOUBLE) || ((JCTree.JCLiteral) tree).typetag == TypeTag.BOOLEAN || ((JCTree.JCLiteral) tree).typetag == TypeTag.BOT;
            case LAMBDA:
            case REFERENCE:
                return false;
            case PARENS:
                return isBooleanOrNumeric(env, ((JCTree.JCParens) tree).expr);
            case CONDEXPR:
                JCTree.JCConditional condTree = (JCTree.JCConditional) tree;
                return isBooleanOrNumeric(env, condTree.truepart) && isBooleanOrNumeric(env, condTree.falsepart);
            case APPLY:
                JCTree.JCMethodInvocation speculativeMethodTree = (JCTree.JCMethodInvocation) this.deferredAttr.attribSpeculative(tree, env, this.unknownExprInfo);
                Type owntype = TreeInfo.symbol(speculativeMethodTree.meth).type.mo178getReturnType();
                return this.types.unboxedTypeOrType(owntype).isPrimitive();
            case NEWCLASS:
                JCTree.JCExpression className = (JCTree.JCExpression) this.removeClassParams.translate(((JCTree.JCNewClass) tree).clazz);
                JCTree.JCExpression speculativeNewClassTree = (JCTree.JCExpression) this.deferredAttr.attribSpeculative(className, env, this.unknownTypeInfo);
                return this.types.unboxedTypeOrType(speculativeNewClassTree.type).isPrimitive();
            default:
                Type speculativeType = this.deferredAttr.attribSpeculative(tree, env, this.unknownExprInfo).type;
                return this.types.unboxedTypeOrType(speculativeType).isPrimitive();
        }
    }

    private Type condType(JCDiagnostic.DiagnosticPosition pos, Type thentype, Type elsetype) {
        Type thenUnboxed;
        Type elseUnboxed;
        if (this.types.isSameType(thentype, elsetype)) {
            return thentype.baseType();
        }
        if (!this.allowBoxing || thentype.isPrimitive()) {
            thenUnboxed = thentype;
        } else {
            thenUnboxed = this.types.unboxedType(thentype);
        }
        if (!this.allowBoxing || elsetype.isPrimitive()) {
            elseUnboxed = elsetype;
        } else {
            elseUnboxed = this.types.unboxedType(elsetype);
        }
        if (thenUnboxed.isPrimitive() && elseUnboxed.isPrimitive()) {
            if (thenUnboxed.getTag().isStrictSubRangeOf(TypeTag.INT) && elseUnboxed.hasTag(TypeTag.INT) && this.types.isAssignable(elseUnboxed, thenUnboxed)) {
                return thenUnboxed.baseType();
            }
            if (elseUnboxed.getTag().isStrictSubRangeOf(TypeTag.INT) && thenUnboxed.hasTag(TypeTag.INT) && this.types.isAssignable(thenUnboxed, elseUnboxed)) {
                return elseUnboxed.baseType();
            }
            for (TypeTag tag : primitiveTags) {
                Type candidate = this.syms.typeOfTag[tag.ordinal()];
                if (this.types.isSubtype(thenUnboxed, candidate) && this.types.isSubtype(elseUnboxed, candidate)) {
                    return candidate;
                }
            }
        }
        if (this.allowBoxing) {
            if (thentype.isPrimitive()) {
                thentype = this.types.boxedClass(thentype).type;
            }
            if (elsetype.isPrimitive()) {
                elsetype = this.types.boxedClass(elsetype).type;
            }
        }
        if (this.types.isSubtype(thentype, elsetype)) {
            return elsetype.baseType();
        }
        if (this.types.isSubtype(elsetype, thentype)) {
            return thentype.baseType();
        }
        if (this.allowBoxing && !thentype.hasTag(TypeTag.VOID) && !elsetype.hasTag(TypeTag.VOID)) {
            return this.types.lub(thentype.baseType(), elsetype.baseType());
        }
        this.log.error(pos, "neither.conditional.subtype", thentype, elsetype);
        return thentype.baseType();
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIf(JCTree.JCIf tree) {
        attribExpr(tree.cond, this.env, this.syms.booleanType);
        attribStat(tree.thenpart, this.env);
        if (tree.elsepart != null) {
            attribStat(tree.elsepart, this.env);
        }
        this.chk.checkEmptyIf(tree);
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitExec(JCTree.JCExpressionStatement tree) {
        Env<AttrContext> localEnv = this.env.dup(tree);
        attribExpr(tree.expr, localEnv);
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBreak(JCTree.JCBreak tree) {
        tree.target = findJumpTarget(tree.pos(), tree.getTag(), tree.label, this.env);
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitContinue(JCTree.JCContinue tree) {
        tree.target = findJumpTarget(tree.pos(), tree.getTag(), tree.label, this.env);
        this.result = null;
    }

    /* JADX WARN: Removed duplicated region for block: B:34:0x006e  */
    /* JADX WARN: Removed duplicated region for block: B:35:0x007a  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private com.sun.tools.javac.tree.JCTree findJumpTarget(com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition r6, com.sun.tools.javac.tree.JCTree.Tag r7, com.sun.tools.javac.util.Name r8, com.sun.tools.javac.comp.Env<com.sun.tools.javac.comp.AttrContext> r9) {
        /*
            r5 = this;
            r0 = r9
        L1:
            if (r0 == 0) goto L6c
            int[] r1 = com.sun.tools.javac.comp.Attr.AnonymousClass15.$SwitchMap$com$sun$tools$javac$tree$JCTree$Tag
            com.sun.tools.javac.tree.JCTree r2 = r0.tree
            com.sun.tools.javac.tree.JCTree$Tag r2 = r2.getTag()
            int r2 = r2.ordinal()
            r1 = r1[r2]
            switch(r1) {
                case 2: goto L68;
                case 3: goto L14;
                case 4: goto L14;
                case 5: goto L14;
                case 6: goto L14;
                case 7: goto L14;
                case 8: goto L23;
                case 9: goto L1e;
                case 10: goto L1e;
                case 11: goto L1e;
                case 12: goto L1e;
                case 13: goto L15;
                case 14: goto L68;
                case 15: goto L68;
                default: goto L14;
            }
        L14:
            goto L69
        L15:
            if (r8 != 0) goto L69
            com.sun.tools.javac.tree.JCTree$Tag r1 = com.sun.tools.javac.tree.JCTree.Tag.BREAK
            if (r7 != r1) goto L69
            com.sun.tools.javac.tree.JCTree r1 = r0.tree
            return r1
        L1e:
            if (r8 != 0) goto L69
            com.sun.tools.javac.tree.JCTree r1 = r0.tree
            return r1
        L23:
            com.sun.tools.javac.tree.JCTree r1 = r0.tree
            com.sun.tools.javac.tree.JCTree$JCLabeledStatement r1 = (com.sun.tools.javac.tree.JCTree.JCLabeledStatement) r1
            com.sun.tools.javac.util.Name r2 = r1.label
            if (r8 != r2) goto L69
            com.sun.tools.javac.tree.JCTree$Tag r2 = com.sun.tools.javac.tree.JCTree.Tag.CONTINUE
            if (r7 != r2) goto L67
            com.sun.tools.javac.tree.JCTree$JCStatement r2 = r1.body
            com.sun.tools.javac.tree.JCTree$Tag r3 = com.sun.tools.javac.tree.JCTree.Tag.DOLOOP
            boolean r2 = r2.hasTag(r3)
            if (r2 != 0) goto L62
            com.sun.tools.javac.tree.JCTree$JCStatement r2 = r1.body
            com.sun.tools.javac.tree.JCTree$Tag r3 = com.sun.tools.javac.tree.JCTree.Tag.WHILELOOP
            boolean r2 = r2.hasTag(r3)
            if (r2 != 0) goto L62
            com.sun.tools.javac.tree.JCTree$JCStatement r2 = r1.body
            com.sun.tools.javac.tree.JCTree$Tag r3 = com.sun.tools.javac.tree.JCTree.Tag.FORLOOP
            boolean r2 = r2.hasTag(r3)
            if (r2 != 0) goto L62
            com.sun.tools.javac.tree.JCTree$JCStatement r2 = r1.body
            com.sun.tools.javac.tree.JCTree$Tag r3 = com.sun.tools.javac.tree.JCTree.Tag.FOREACHLOOP
            boolean r2 = r2.hasTag(r3)
            if (r2 != 0) goto L62
            com.sun.tools.javac.util.Log r2 = r5.log
            java.lang.String r3 = "not.loop.label"
            java.lang.Object[] r4 = new java.lang.Object[]{r8}
            r2.error(r6, r3, r4)
        L62:
            com.sun.tools.javac.tree.JCTree r2 = com.sun.tools.javac.tree.TreeInfo.referencedStatement(r1)
            return r2
        L67:
            return r1
        L68:
            goto L6c
        L69:
            com.sun.tools.javac.comp.Env<A> r0 = r0.next
            goto L1
        L6c:
            if (r8 == 0) goto L7a
            com.sun.tools.javac.util.Log r1 = r5.log
            java.lang.String r2 = "undef.label"
            java.lang.Object[] r3 = new java.lang.Object[]{r8}
            r1.error(r6, r2, r3)
            goto L92
        L7a:
            com.sun.tools.javac.tree.JCTree$Tag r1 = com.sun.tools.javac.tree.JCTree.Tag.CONTINUE
            r2 = 0
            if (r7 != r1) goto L89
            com.sun.tools.javac.util.Log r1 = r5.log
            java.lang.String r3 = "cont.outside.loop"
            java.lang.Object[] r2 = new java.lang.Object[r2]
            r1.error(r6, r3, r2)
            goto L92
        L89:
            com.sun.tools.javac.util.Log r1 = r5.log
            java.lang.String r3 = "break.outside.switch.loop"
            java.lang.Object[] r2 = new java.lang.Object[r2]
            r1.error(r6, r3, r2)
        L92:
            r1 = 0
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Attr.findJumpTarget(com.sun.tools.javac.util.JCDiagnostic$DiagnosticPosition, com.sun.tools.javac.tree.JCTree$Tag, com.sun.tools.javac.util.Name, com.sun.tools.javac.comp.Env):com.sun.tools.javac.tree.JCTree");
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReturn(JCTree.JCReturn tree) {
        if (this.env.info.returnResult == null) {
            this.log.error(tree.pos(), "ret.outside.meth", new Object[0]);
        } else if (tree.expr != null) {
            if (this.env.info.returnResult.pt.hasTag(TypeTag.VOID)) {
                this.env.info.returnResult.checkContext.report(tree.expr.pos(), this.diags.fragment("unexpected.ret.val", new Object[0]));
            }
            attribTree(tree.expr, this.env, this.env.info.returnResult);
        } else if (!this.env.info.returnResult.pt.hasTag(TypeTag.VOID) && !this.env.info.returnResult.pt.hasTag(TypeTag.NONE)) {
            this.env.info.returnResult.checkContext.report(tree.pos(), this.diags.fragment("missing.ret.val", new Object[0]));
        }
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitThrow(JCTree.JCThrow tree) {
        Type owntype = attribExpr(tree.expr, this.env, this.allowPoly ? Type.noType : this.syms.throwableType);
        if (this.allowPoly) {
            this.chk.checkType(tree, owntype, this.syms.throwableType);
        }
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssert(JCTree.JCAssert tree) {
        attribExpr(tree.cond, this.env, this.syms.booleanType);
        if (tree.detail != null) {
            this.chk.checkNonVoid(tree.detail.pos(), attribExpr(tree.detail, this.env));
        }
        this.result = null;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitApply(JCTree.JCMethodInvocation tree) {
        Type site;
        List<Type> typeargtypes;
        Env<AttrContext> localEnv = this.env.dup(tree, this.env.info.dup());
        Name methName = TreeInfo.name(tree.meth);
        boolean isConstructorCall = methName == this.names._this || methName == this.names._super;
        ListBuffer<Type> argtypesBuf = new ListBuffer<>();
        if (isConstructorCall) {
            if (checkFirstConstructorStat(tree, this.env)) {
                localEnv.info.isSelfCall = true;
                int kind = attribArgs(16, tree.args, localEnv, argtypesBuf);
                List<Type> argtypes = argtypesBuf.toList();
                List<Type> typeargtypes2 = attribTypes(tree.typeargs, localEnv);
                Type site2 = this.env.enclClass.sym.type;
                if (methName != this.names._super) {
                    site = site2;
                } else if (site2 == this.syms.objectType) {
                    this.log.error(tree.meth.pos(), "no.superclass", site2);
                    site = this.types.createErrorType(this.syms.objectType);
                } else {
                    site = this.types.supertype(site2);
                }
                if (!site.hasTag(TypeTag.CLASS)) {
                    typeargtypes = typeargtypes2;
                } else {
                    Type encl = site.getEnclosingType();
                    while (encl != null && encl.hasTag(TypeTag.TYPEVAR)) {
                        encl = encl.getUpperBound();
                    }
                    if (encl.hasTag(TypeTag.CLASS)) {
                        if (tree.meth.hasTag(JCTree.Tag.SELECT)) {
                            JCTree qualifier = ((JCTree.JCFieldAccess) tree.meth).selected;
                            this.chk.checkRefType(qualifier.pos(), attribExpr(qualifier, localEnv, encl));
                        } else if (methName == this.names._super) {
                            this.rs.resolveImplicitThis(tree.meth.pos(), localEnv, site, true);
                        }
                    } else if (tree.meth.hasTag(JCTree.Tag.SELECT)) {
                        this.log.error(tree.meth.pos(), "illegal.qual.not.icls", site.tsym);
                    }
                    if (site.tsym == this.syms.enumSym && this.allowEnums) {
                        argtypes = argtypes.prepend(this.syms.intType).prepend(this.syms.stringType);
                    }
                    boolean selectSuperPrev = localEnv.info.selectSuper;
                    localEnv.info.selectSuper = true;
                    localEnv.info.pendingResolutionPhase = null;
                    Symbol sym = this.rs.resolveConstructor(tree.meth.pos(), localEnv, site, argtypes, typeargtypes2);
                    localEnv.info.selectSuper = selectSuperPrev;
                    TreeInfo.setSymbol(tree.meth, sym);
                    Type mpt = newMethodTemplate(this.resultInfo.pt, argtypes, typeargtypes2);
                    typeargtypes = typeargtypes2;
                    checkId(tree.meth, site, sym, localEnv, new ResultInfo(this, kind, mpt));
                }
            }
            Type.JCVoidType jCVoidType = this.syms.voidType;
            tree.type = jCVoidType;
            this.result = jCVoidType;
        } else {
            int kind2 = attribArgs(12, tree.args, localEnv, argtypesBuf);
            List<Type> argtypes2 = argtypesBuf.toList();
            List<Type> typeargtypes3 = attribAnyTypes(tree.typeargs, localEnv);
            Type mpt2 = newMethodTemplate(this.resultInfo.pt, argtypes2, typeargtypes3);
            localEnv.info.pendingResolutionPhase = null;
            Type mtype = attribTree(tree.meth, localEnv, new ResultInfo(kind2, mpt2, this.resultInfo.checkContext));
            Type restype = mtype.mo178getReturnType();
            if (restype.hasTag(TypeTag.WILDCARD)) {
                throw new AssertionError(mtype);
            }
            Type restype2 = adjustMethodReturnType(tree.meth.hasTag(JCTree.Tag.SELECT) ? ((JCTree.JCFieldAccess) tree.meth).selected.type : this.env.enclClass.sym.type, methName, argtypes2, restype);
            this.chk.checkRefTypes(tree.typeargs, typeargtypes3);
            this.result = check(tree, capture(restype2), 12, this.resultInfo);
        }
        this.chk.validate(tree.typeargs, localEnv);
    }

    Type adjustMethodReturnType(Type qualifierType, Name methodName, List<Type> argtypes, Type restype) {
        if (this.allowCovariantReturns && methodName == this.names.clone && this.types.isArray(qualifierType)) {
            return qualifierType;
        }
        if (this.allowGenerics && methodName == this.names.getClass && argtypes.isEmpty()) {
            return new Type.ClassType(restype.getEnclosingType(), List.of(new Type.WildcardType(this.types.erasure(qualifierType), BoundKind.EXTENDS, this.syms.boundClass)), restype.tsym);
        }
        return restype;
    }

    boolean checkFirstConstructorStat(JCTree.JCMethodInvocation tree, Env<AttrContext> env) {
        JCTree.JCMethodDecl enclMethod = env.enclMethod;
        if (enclMethod != null && enclMethod.name == this.names.init) {
            JCTree.JCBlock body = enclMethod.body;
            if (body.stats.head.hasTag(JCTree.Tag.EXEC) && ((JCTree.JCExpressionStatement) body.stats.head).expr == tree) {
                return true;
            }
        }
        this.log.error(tree.pos(), "call.must.be.first.stmt.in.ctor", TreeInfo.name(tree.meth));
        return false;
    }

    Type newMethodTemplate(Type restype, List<Type> argtypes, List<Type> typeargtypes) {
        Type.MethodType mt = new Type.MethodType(argtypes, restype, List.nil(), this.syms.methodClass);
        return typeargtypes == null ? mt : new Type.ForAll(typeargtypes, mt);
    }

    /* JADX WARN: Removed duplicated region for block: B:117:0x03ce  */
    /* JADX WARN: Removed duplicated region for block: B:140:0x048b  */
    /* JADX WARN: Removed duplicated region for block: B:146:0x04a7  */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public void visitNewClass(final com.sun.tools.javac.tree.JCTree.JCNewClass r31) {
        /*
            Method dump skipped, instruction units count: 1263
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Attr.visitNewClass(com.sun.tools.javac.tree.JCTree$JCNewClass):void");
    }

    /* JADX WARN: Removed duplicated region for block: B:28:0x007f  */
    /* JADX WARN: Removed duplicated region for block: B:29:0x0082  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    void findDiamond(com.sun.tools.javac.comp.Env<com.sun.tools.javac.comp.AttrContext> r10, com.sun.tools.javac.tree.JCTree.JCNewClass r11, com.sun.tools.javac.code.Type r12) {
        /*
            r9 = this;
            com.sun.tools.javac.tree.JCTree$JCExpression r0 = r11.clazz
            com.sun.tools.javac.tree.JCTree$JCTypeApply r0 = (com.sun.tools.javac.tree.JCTree.JCTypeApply) r0
            com.sun.tools.javac.util.List<com.sun.tools.javac.tree.JCTree$JCExpression> r1 = r0.arguments
            com.sun.tools.javac.util.List r2 = com.sun.tools.javac.util.List.nil()     // Catch: java.lang.Throwable -> L97
            r0.arguments = r2     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.comp.Attr$ResultInfo r2 = new com.sun.tools.javac.comp.Attr$ResultInfo     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.comp.Attr$ResultInfo r3 = r9.resultInfo     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.comp.Check$CheckContext r3 = r3.checkContext     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.comp.Infer$InferenceContext r3 = r3.inferenceContext()     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.comp.Attr$ResultInfo r4 = r9.resultInfo     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.code.Type r4 = r4.pt     // Catch: java.lang.Throwable -> L97
            boolean r3 = r3.free(r4)     // Catch: java.lang.Throwable -> L97
            if (r3 == 0) goto L23
            com.sun.tools.javac.code.Type$JCNoType r3 = com.sun.tools.javac.code.Type.noType     // Catch: java.lang.Throwable -> L97
            goto L27
        L23:
            com.sun.tools.javac.code.Type r3 = r9.pt()     // Catch: java.lang.Throwable -> L97
        L27:
            r4 = 12
            r2.<init>(r9, r4, r3)     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.comp.DeferredAttr r3 = r9.deferredAttr     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.tree.JCTree r3 = r3.attribSpeculative(r11, r10, r2)     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.code.Type r3 = r3.type     // Catch: java.lang.Throwable -> L97
            boolean r4 = r9.allowPoly     // Catch: java.lang.Throwable -> L97
            if (r4 == 0) goto L3d
            com.sun.tools.javac.code.Symtab r4 = r9.syms     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.code.Type r4 = r4.objectType     // Catch: java.lang.Throwable -> L97
            goto L3e
        L3d:
            r4 = r12
        L3e:
            boolean r5 = r3.isErroneous()     // Catch: java.lang.Throwable -> L97
            if (r5 != 0) goto L93
            boolean r5 = r9.allowPoly     // Catch: java.lang.Throwable -> L97
            if (r5 == 0) goto L59
            com.sun.tools.javac.code.Type r5 = r9.pt()     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.code.Type r6 = com.sun.tools.javac.comp.Infer.anyPoly     // Catch: java.lang.Throwable -> L97
            if (r5 != r6) goto L59
            com.sun.tools.javac.code.Types r5 = r9.types     // Catch: java.lang.Throwable -> L97
            boolean r5 = r5.isSameType(r3, r12)     // Catch: java.lang.Throwable -> L97
            if (r5 == 0) goto L93
            goto L77
        L59:
            com.sun.tools.javac.code.Types r5 = r9.types     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.code.Type r6 = r9.pt()     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.code.TypeTag r7 = com.sun.tools.javac.code.TypeTag.NONE     // Catch: java.lang.Throwable -> L97
            boolean r6 = r6.hasTag(r7)     // Catch: java.lang.Throwable -> L97
            if (r6 == 0) goto L69
            r6 = r4
            goto L6d
        L69:
            com.sun.tools.javac.code.Type r6 = r9.pt()     // Catch: java.lang.Throwable -> L97
        L6d:
            com.sun.tools.javac.code.Types r7 = r9.types     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.util.Warner r7 = r7.noWarnings     // Catch: java.lang.Throwable -> L97
            boolean r5 = r5.isAssignable(r3, r6, r7)     // Catch: java.lang.Throwable -> L97
            if (r5 == 0) goto L93
        L77:
            com.sun.tools.javac.code.Types r5 = r9.types     // Catch: java.lang.Throwable -> L97
            boolean r5 = r5.isSameType(r12, r3)     // Catch: java.lang.Throwable -> L97
            if (r5 == 0) goto L82
            java.lang.String r5 = "diamond.redundant.args"
            goto L84
        L82:
            java.lang.String r5 = "diamond.redundant.args.1"
        L84:
            com.sun.tools.javac.util.Log r6 = r9.log     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.tree.JCTree$JCExpression r7 = r11.clazz     // Catch: java.lang.Throwable -> L97
            com.sun.tools.javac.util.JCDiagnostic$DiagnosticPosition r7 = r7.pos()     // Catch: java.lang.Throwable -> L97
            java.lang.Object[] r8 = new java.lang.Object[]{r12, r3}     // Catch: java.lang.Throwable -> L97
            r6.warning(r7, r5, r8)     // Catch: java.lang.Throwable -> L97
        L93:
            r0.arguments = r1
            return
        L97:
            r2 = move-exception
            r0.arguments = r1
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Attr.findDiamond(com.sun.tools.javac.comp.Env, com.sun.tools.javac.tree.JCTree$JCNewClass, com.sun.tools.javac.code.Type):void");
    }

    private void checkLambdaCandidate(JCTree.JCNewClass tree, Symbol.ClassSymbol csym, Type clazztype) {
        if (this.allowLambda && this.identifyLambdaCandidate && clazztype.hasTag(TypeTag.CLASS) && !pt().hasTag(TypeTag.NONE) && this.types.isFunctionalInterface(clazztype.tsym)) {
            Symbol descriptor = this.types.findDescriptorSymbol(clazztype.tsym);
            int count = 0;
            boolean found = false;
            for (Symbol sym : csym.members().getElements()) {
                if ((sym.flags() & 4096) == 0 && !sym.isConstructor()) {
                    count++;
                    if (sym.kind == 16 && sym.name.equals(descriptor.name)) {
                        Type mtype = this.types.memberType(clazztype, sym);
                        if (this.types.overrideEquivalent(mtype, this.types.memberType(clazztype, descriptor))) {
                            found = true;
                        }
                    }
                }
            }
            if (found && count == 1) {
                this.log.note(tree.def, "potential.lambda.found", new Object[0]);
            }
        }
    }

    public JCTree.JCExpression makeNullCheck(JCTree.JCExpression arg) {
        Name name = TreeInfo.name(arg);
        if (name == this.names._this || name == this.names._super) {
            return arg;
        }
        JCTree.Tag optag = JCTree.Tag.NULLCHK;
        JCTree.JCUnary tree = this.make.at(arg.pos).Unary(optag, arg);
        tree.operator = this.syms.nullcheck;
        tree.type = arg.type;
        return tree;
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewArray(JCTree.JCNewArray tree) {
        Type elemtype;
        Type owntype = this.types.createErrorType(tree.type);
        Env<AttrContext> localEnv = this.env.dup(tree);
        if (tree.elemtype != null) {
            elemtype = attribType(tree.elemtype, localEnv);
            this.chk.validate(tree.elemtype, localEnv);
            owntype = elemtype;
            for (List list = tree.dims; list.nonEmpty(); list = list.tail) {
                attribExpr((JCTree) list.head, localEnv, this.syms.intType);
                owntype = new Type.ArrayType(owntype, this.syms.arrayClass);
            }
        } else {
            Type elemtype2 = pt();
            if (elemtype2.hasTag(TypeTag.ARRAY)) {
                elemtype = this.types.elemtype(pt());
            } else {
                Type elemtype3 = pt();
                if (!elemtype3.hasTag(TypeTag.ERROR)) {
                    this.log.error(tree.pos(), "illegal.initializer.for.type", pt());
                }
                elemtype = this.types.createErrorType(pt());
            }
        }
        if (tree.elems != null) {
            attribExprs(tree.elems, localEnv, elemtype);
            owntype = new Type.ArrayType(elemtype, this.syms.arrayClass);
        }
        if (!this.types.isReifiable(elemtype)) {
            this.log.error(tree.pos(), "generic.array.creation", new Object[0]);
        }
        this.result = check(tree, owntype, 12, this.resultInfo);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLambda(JCTree.JCLambda that) {
        List<Type> explicitParamTypes;
        Type currentTarget;
        Type lambdaType;
        if (pt().isErroneous() || (pt().hasTag(TypeTag.NONE) && pt() != Type.recoveryType)) {
            if (pt().hasTag(TypeTag.NONE)) {
                this.log.error(that.pos(), "unexpected.lambda", new Object[0]);
            }
            Type typeCreateErrorType = this.types.createErrorType(pt());
            that.type = typeCreateErrorType;
            this.result = typeCreateErrorType;
            return;
        }
        Env<AttrContext> localEnv = lambdaEnv(that, this.env);
        boolean needsRecovery = this.resultInfo.checkContext.deferredAttrContext().mode == DeferredAttr.AttrMode.CHECK;
        try {
            try {
                Type currentTarget2 = pt();
                if (needsRecovery && isSerializable(currentTarget2)) {
                    localEnv.info.isSerializable = true;
                }
                if (that.paramKind == JCTree.JCLambda.ParameterKind.EXPLICIT) {
                    attribStats(that.params, localEnv);
                    List<Type> explicitParamTypes2 = TreeInfo.types(that.params);
                    explicitParamTypes = explicitParamTypes2;
                } else {
                    explicitParamTypes = null;
                }
                if (pt() != Type.recoveryType) {
                    Type currentTarget3 = this.targetChecker.visit(currentTarget2, that);
                    if (explicitParamTypes != null) {
                        currentTarget3 = this.infer.instantiateFunctionalInterface(that, currentTarget3, explicitParamTypes, this.resultInfo.checkContext);
                    }
                    Type currentTarget4 = this.types.removeWildcards(currentTarget3);
                    currentTarget = currentTarget4;
                    lambdaType = this.types.findDescriptorType(currentTarget4);
                } else {
                    Type lambdaType2 = Type.recoveryType;
                    currentTarget = lambdaType2;
                    lambdaType = fallbackDescriptorType(that);
                }
                setFunctionalInfo(localEnv, that, pt(), lambdaType, currentTarget, this.resultInfo.checkContext);
                if (lambdaType.hasTag(TypeTag.FORALL)) {
                    this.resultInfo.checkContext.report(that, this.diags.fragment("invalid.generic.lambda.target", lambdaType, Kinds.kindName(currentTarget.tsym), currentTarget.tsym));
                    Type typeCreateErrorType2 = this.types.createErrorType(pt());
                    that.type = typeCreateErrorType2;
                    this.result = typeCreateErrorType2;
                    localEnv.info.scope.leave();
                    if (needsRecovery) {
                        attribTree(that, this.env, this.recoveryInfo);
                        return;
                    }
                    return;
                }
                if (that.paramKind == JCTree.JCLambda.ParameterKind.IMPLICIT) {
                    List listMo176getParameterTypes = lambdaType.mo176getParameterTypes();
                    boolean arityMismatch = false;
                    for (List list = that.params; list.nonEmpty(); list = list.tail) {
                        if (listMo176getParameterTypes.isEmpty()) {
                            arityMismatch = true;
                        }
                        Type argType = arityMismatch ? this.syms.errType : (Type) listMo176getParameterTypes.head;
                        ((JCTree.JCVariableDecl) list.head).vartype = this.make.at((JCDiagnostic.DiagnosticPosition) list.head).Type(argType);
                        ((JCTree.JCVariableDecl) list.head).sym = null;
                        listMo176getParameterTypes = listMo176getParameterTypes.isEmpty() ? listMo176getParameterTypes : listMo176getParameterTypes.tail;
                    }
                    attribStats(that.params, localEnv);
                    if (arityMismatch) {
                        this.resultInfo.checkContext.report(that, this.diags.fragment("incompatible.arg.types.in.lambda", new Object[0]));
                        Type typeCreateErrorType3 = this.types.createErrorType(currentTarget);
                        that.type = typeCreateErrorType3;
                        this.result = typeCreateErrorType3;
                        localEnv.info.scope.leave();
                        if (needsRecovery) {
                            attribTree(that, this.env, this.recoveryInfo);
                            return;
                        }
                        return;
                    }
                }
                FunctionalReturnContext funcContext = that.getBodyKind() == LambdaExpressionTree.BodyKind.EXPRESSION ? new ExpressionLambdaReturnContext((JCTree.JCExpression) that.getBody(), this.resultInfo.checkContext) : new FunctionalReturnContext(this.resultInfo.checkContext);
                ResultInfo bodyResultInfo = lambdaType.mo178getReturnType() == Type.recoveryType ? this.recoveryInfo : new ResultInfo(12, lambdaType.mo178getReturnType(), funcContext);
                localEnv.info.returnResult = bodyResultInfo;
                if (that.getBodyKind() == LambdaExpressionTree.BodyKind.EXPRESSION) {
                    attribTree(that.getBody(), localEnv, bodyResultInfo);
                } else {
                    JCTree.JCBlock body = (JCTree.JCBlock) that.body;
                    attribStats(body.stats, localEnv);
                }
                this.result = check(that, currentTarget, 12, this.resultInfo);
                boolean isSpeculativeRound = this.resultInfo.checkContext.deferredAttrContext().mode == DeferredAttr.AttrMode.SPECULATIVE;
                preFlow(that);
                this.flow.analyzeLambda(this.env, that, this.make, isSpeculativeRound);
                that.type = currentTarget;
                checkLambdaCompatible(that, lambdaType, this.resultInfo.checkContext);
                if (!isSpeculativeRound) {
                    if (this.resultInfo.checkContext.inferenceContext().free(lambdaType.mo179getThrownTypes())) {
                        List<Type> inferredThrownTypes = this.flow.analyzeLambdaThrownTypes(this.env, that, this.make);
                        List<Type> thrownTypes = this.resultInfo.checkContext.inferenceContext().asUndetVars(lambdaType.mo179getThrownTypes());
                        this.chk.unhandled(inferredThrownTypes, thrownTypes);
                    }
                    checkAccessibleTypes(that, localEnv, this.resultInfo.checkContext.inferenceContext(), lambdaType, currentTarget);
                }
                this.result = check(that, currentTarget, 12, this.resultInfo);
                localEnv.info.scope.leave();
                if (0 != 0) {
                    attribTree(that, this.env, this.recoveryInfo);
                }
            } catch (Types.FunctionDescriptorLookupError ex) {
                JCDiagnostic cause = ex.getDiagnostic();
                this.resultInfo.checkContext.report(that, cause);
                Type typeCreateErrorType4 = this.types.createErrorType(pt());
                that.type = typeCreateErrorType4;
                this.result = typeCreateErrorType4;
                localEnv.info.scope.leave();
                if (needsRecovery) {
                    attribTree(that, this.env, this.recoveryInfo);
                }
            }
        } catch (Throwable th) {
            localEnv.info.scope.leave();
            if (needsRecovery) {
                attribTree(that, this.env, this.recoveryInfo);
            }
            throw th;
        }
    }

    void preFlow(JCTree.JCLambda tree) {
        new PostAttrAnalyzer() { // from class: com.sun.tools.javac.comp.Attr.7
            @Override // com.sun.tools.javac.comp.Attr.PostAttrAnalyzer, com.sun.tools.javac.tree.TreeScanner
            public void scan(JCTree tree2) {
                if (tree2 != null) {
                    if (tree2.type != null && tree2.type == Type.stuckType) {
                        return;
                    }
                    super.scan(tree2);
                }
            }
        }.scan(tree);
    }

    private Type fallbackDescriptorType(JCTree.JCExpression tree) {
        List<Type> listAppend;
        switch (tree.getTag()) {
            case LAMBDA:
                JCTree.JCLambda lambda = (JCTree.JCLambda) tree;
                List<Type> argtypes = List.nil();
                for (JCTree.JCVariableDecl param : lambda.params) {
                    if (param.vartype != null) {
                        listAppend = argtypes.append(param.vartype.type);
                    } else {
                        listAppend = argtypes.append(this.syms.errType);
                    }
                    argtypes = listAppend;
                }
                return new Type.MethodType(argtypes, Type.recoveryType, List.of(this.syms.throwableType), this.syms.methodClass);
            case REFERENCE:
                return new Type.MethodType(List.nil(), Type.recoveryType, List.of(this.syms.throwableType), this.syms.methodClass);
            default:
                Assert.error("Cannot get here!");
                return null;
        }
    }

    private void checkAccessibleTypes(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Infer.InferenceContext inferenceContext, Type... ts) {
        checkAccessibleTypes(pos, env, inferenceContext, List.from(ts));
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void checkAccessibleTypes(final JCDiagnostic.DiagnosticPosition pos, final Env<AttrContext> env, Infer.InferenceContext inferenceContext, final List<Type> ts) {
        if (inferenceContext.free(ts)) {
            inferenceContext.addFreeTypeListener(ts, new Infer.FreeTypeListener() { // from class: com.sun.tools.javac.comp.Attr.9
                @Override // com.sun.tools.javac.comp.Infer.FreeTypeListener
                public void typesInferred(Infer.InferenceContext inferenceContext2) {
                    Attr.this.checkAccessibleTypes(pos, (Env<AttrContext>) env, inferenceContext2, inferenceContext2.asInstTypes(ts));
                }
            });
            return;
        }
        for (Type t : ts) {
            this.rs.checkAccessibleType(env, t);
        }
    }

    class FunctionalReturnContext extends Check.NestedCheckContext {
        FunctionalReturnContext(Check.CheckContext enclosingContext) {
            super(enclosingContext);
        }

        @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
        public boolean compatible(Type found, Type req, Warner warn) {
            return Attr.this.chk.basicHandler.compatible(found, inferenceContext().asUndetVar(req), warn);
        }

        @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
        public void report(JCDiagnostic.DiagnosticPosition pos, JCDiagnostic details) {
            this.enclosingContext.report(pos, Attr.this.diags.fragment("incompatible.ret.type.in.lambda", details));
        }
    }

    class ExpressionLambdaReturnContext extends FunctionalReturnContext {
        JCTree.JCExpression expr;

        ExpressionLambdaReturnContext(JCTree.JCExpression expr, Check.CheckContext enclosingContext) {
            super(enclosingContext);
            this.expr = expr;
        }

        @Override // com.sun.tools.javac.comp.Attr.FunctionalReturnContext, com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
        public boolean compatible(Type found, Type req, Warner warn) {
            return (TreeInfo.isExpressionStatement(this.expr) && req.hasTag(TypeTag.VOID)) || super.compatible(found, req, warn);
        }
    }

    private void checkLambdaCompatible(JCTree.JCLambda tree, Type descriptor, Check.CheckContext checkContext) {
        Type returnType = checkContext.inferenceContext().asUndetVar(descriptor.mo178getReturnType());
        if (tree.getBodyKind() == LambdaExpressionTree.BodyKind.STATEMENT && tree.canCompleteNormally && !returnType.hasTag(TypeTag.VOID) && returnType != Type.recoveryType) {
            checkContext.report(tree, this.diags.fragment("incompatible.ret.type.in.lambda", this.diags.fragment("missing.ret.val", returnType)));
        }
        List<Type> argTypes = checkContext.inferenceContext().asUndetVars(descriptor.mo176getParameterTypes());
        if (!this.types.isSameTypes(argTypes, TreeInfo.types(tree.params))) {
            checkContext.report(tree, this.diags.fragment("incompatible.arg.types.in.lambda", new Object[0]));
        }
    }

    public Symbol.MethodSymbol removeClinit(Symbol.ClassSymbol sym) {
        return this.clinits.remove(sym);
    }

    public Env<AttrContext> lambdaEnv(JCTree.JCLambda that, Env<AttrContext> env) {
        Symbol owner = env.info.scope.owner;
        if (owner.kind == 4 && owner.owner.kind == 2) {
            Env<AttrContext> lambdaEnv = env.dup(that, env.info.dup(env.info.scope.dupUnshared()));
            Symbol.ClassSymbol enclClass = owner.enclClass();
            if ((owner.flags() & 8) == 0) {
                Iterator<Symbol> it = enclClass.members_field.getElementsByName(this.names.init).iterator();
                if (it.hasNext()) {
                    Symbol s = it.next();
                    lambdaEnv.info.scope.owner = s;
                    return lambdaEnv;
                }
                return lambdaEnv;
            }
            Symbol.MethodSymbol clinit = this.clinits.get(enclClass);
            if (clinit == null) {
                Type clinitType = new Type.MethodType(List.nil(), this.syms.voidType, List.nil(), this.syms.methodClass);
                clinit = new Symbol.MethodSymbol(4106L, this.names.clinit, clinitType, enclClass);
                clinit.params = List.nil();
                this.clinits.put(enclClass, clinit);
            }
            lambdaEnv.info.scope.owner = clinit;
            return lambdaEnv;
        }
        return env.dup(that, env.info.dup(env.info.scope.dup()));
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r0v34, types: [com.sun.tools.javac.comp.Resolve] */
    /* JADX WARN: Type inference failed for: r10v10 */
    /* JADX WARN: Type inference failed for: r10v14 */
    /* JADX WARN: Type inference failed for: r10v17 */
    /* JADX WARN: Type inference failed for: r10v2 */
    /* JADX WARN: Type inference failed for: r10v21 */
    /* JADX WARN: Type inference failed for: r10v3, types: [com.sun.tools.javac.tree.JCTree$JCMemberReference, com.sun.tools.javac.util.JCDiagnostic$DiagnosticPosition] */
    /* JADX WARN: Type inference failed for: r10v4 */
    /* JADX WARN: Type inference failed for: r10v5 */
    /* JADX WARN: Type inference failed for: r10v6 */
    /* JADX WARN: Type inference failed for: r17v0 */
    /* JADX WARN: Type inference failed for: r23v1, types: [com.sun.tools.javac.util.List] */
    /* JADX WARN: Type inference failed for: r26v0, types: [com.sun.tools.javac.comp.Attr] */
    /* JADX WARN: Type inference failed for: r2v2, types: [com.sun.tools.javac.comp.Check$CheckContext] */
    /* JADX WARN: Type inference failed for: r9v9, types: [com.sun.tools.javac.comp.Resolve$ResolveError] */
    /* JADX WARN: Type inference fix 'apply assigned field type' failed
    java.lang.UnsupportedOperationException: ArgType.getObject(), call class: class jadx.core.dex.instructions.args.ArgType$UnknownArg
    	at jadx.core.dex.instructions.args.ArgType.getObject(ArgType.java:593)
    	at jadx.core.dex.attributes.nodes.ClassTypeVarsAttr.getTypeVarsMapFor(ClassTypeVarsAttr.java:35)
    	at jadx.core.dex.nodes.utils.TypeUtils.replaceClassGenerics(TypeUtils.java:177)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.insertExplicitUseCast(FixTypesVisitor.java:397)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.tryFieldTypeWithNewCasts(FixTypesVisitor.java:359)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.applyFieldType(FixTypesVisitor.java:309)
    	at jadx.core.dex.visitors.typeinference.FixTypesVisitor.visit(FixTypesVisitor.java:94)
     */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReference(JCTree.JCMemberReference that) throws Throwable {
        JCTree.JCMemberReference jCMemberReference;
        ?? r10;
        Type exprType;
        Type currentTarget;
        Type desc;
        Resolve.MethodCheck referenceCheck;
        List<Type> saved_undet;
        Type currentTarget2;
        ?? r23;
        Type currentTarget3;
        Pair<Symbol, Resolve.ReferenceLookupHelper> pair;
        Type exprType2;
        boolean targetError;
        JCDiagnostic.DiagnosticType diagnosticType;
        if (pt().isErroneous()) {
            jCMemberReference = that;
        } else {
            if (!pt().hasTag(TypeTag.NONE) || pt() == Type.recoveryType) {
                Env<AttrContext> localEnv = this.env.dup(that);
                try {
                    Type exprType3 = attribTree(that.expr, this.env, memberReferenceQualifierResult(that));
                    if (that.getMode() == MemberReferenceTree.ReferenceMode.NEW) {
                        try {
                            Type exprType4 = this.chk.checkConstructorRefType(that.expr, exprType3);
                            if (exprType4.isErroneous() || !exprType4.isRaw() || that.typeargs == null) {
                                exprType = exprType4;
                            } else {
                                this.log.error(that.expr.pos(), "invalid.mref", Kinds.kindName(that.getMode()), this.diags.fragment("mref.infer.and.explicit.params", new Object[0]));
                                exprType = this.types.createErrorType(exprType4);
                            }
                        } catch (Types.FunctionDescriptorLookupError e) {
                            ex = e;
                            r10 = that;
                        }
                    } else {
                        exprType = exprType3;
                    }
                    if (exprType.isErroneous()) {
                        that.type = exprType;
                        this.result = exprType;
                        return;
                    }
                    if (TreeInfo.isStaticSelector(that.expr, this.names)) {
                        this.chk.validate(that.expr, this.env, false);
                    }
                    List<Type> typeargtypes = List.nil();
                    if (that.typeargs != null) {
                        List<Type> typeargtypes2 = attribTypes(that.typeargs, localEnv);
                        r10 = typeargtypes2;
                    } else {
                        r10 = typeargtypes;
                    }
                    Type currentTarget4 = pt();
                    boolean isTargetSerializable = this.resultInfo.checkContext.deferredAttrContext().mode == DeferredAttr.AttrMode.CHECK && isSerializable(currentTarget4);
                    if (currentTarget4 != Type.recoveryType) {
                        Type currentTarget5 = this.types.removeWildcards(this.targetChecker.visit(currentTarget4, that));
                        currentTarget = currentTarget5;
                        desc = this.types.findDescriptorType(currentTarget5);
                    } else {
                        Type desc2 = Type.recoveryType;
                        currentTarget = desc2;
                        desc = fallbackDescriptorType(that);
                    }
                    setFunctionalInfo(localEnv, that, pt(), desc, currentTarget, this.resultInfo.checkContext);
                    List<Type> argtypes = desc.mo176getParameterTypes();
                    Resolve.MethodCheck referenceCheck2 = this.rs.resolveMethodCheck;
                    if (this.resultInfo.checkContext.inferenceContext().free(argtypes)) {
                        Resolve resolve = this.rs;
                        resolve.getClass();
                        Resolve.MethodCheck referenceCheck3 = resolve.new MethodReferenceCheck(this.resultInfo.checkContext.inferenceContext());
                        referenceCheck = referenceCheck3;
                    } else {
                        referenceCheck = referenceCheck2;
                    }
                    List<Type> saved_undet2 = this.resultInfo.checkContext.inferenceContext().save();
                    try {
                        try {
                            currentTarget2 = currentTarget;
                            r23 = r10;
                            currentTarget3 = exprType;
                        } catch (Throwable th) {
                            th = th;
                            saved_undet = saved_undet2;
                        }
                        try {
                            Pair<Symbol, Resolve.ReferenceLookupHelper> refResult = this.rs.resolveMemberReference(localEnv, that, that.expr.type, that.name, argtypes, r23, referenceCheck, this.resultInfo.checkContext.inferenceContext(), this.resultInfo.checkContext.deferredAttrContext().mode);
                            try {
                                this.resultInfo.checkContext.inferenceContext().rollback(saved_undet2);
                                Symbol refSym = refResult.fst;
                                Resolve.ReferenceLookupHelper lookupHelper = refResult.snd;
                                if (refSym.kind == 16) {
                                    that.sym = refSym.baseSymbol();
                                    that.kind = lookupHelper.referenceKind(that.sym);
                                    that.ownerAccessible = this.rs.isAccessible(localEnv, that.sym.enclClass());
                                    if (desc.mo178getReturnType() == Type.recoveryType) {
                                        that.type = currentTarget2;
                                        this.result = currentTarget2;
                                        return;
                                    }
                                    if (this.resultInfo.checkContext.deferredAttrContext().mode == DeferredAttr.AttrMode.CHECK) {
                                        if (that.getMode() == MemberReferenceTree.ReferenceMode.INVOKE && TreeInfo.isStaticSelector(that.expr, this.names) && that.kind.isUnbound() && !desc.mo176getParameterTypes().head.isParameterized()) {
                                            this.chk.checkRaw(that.expr, localEnv);
                                        }
                                        if (that.sym.isStatic() && TreeInfo.isStaticSelector(that.expr, this.names) && currentTarget3.getTypeArguments().nonEmpty()) {
                                            this.log.error(that.expr.pos(), "invalid.mref", Kinds.kindName(that.getMode()), this.diags.fragment("static.mref.with.targs", new Object[0]));
                                            Type typeCreateErrorType = this.types.createErrorType(currentTarget2);
                                            that.type = typeCreateErrorType;
                                            this.result = typeCreateErrorType;
                                            return;
                                        }
                                        if (that.sym.isStatic() && !TreeInfo.isStaticSelector(that.expr, this.names) && !that.kind.isUnbound()) {
                                            this.log.error(that.expr.pos(), "invalid.mref", Kinds.kindName(that.getMode()), this.diags.fragment("static.bound.mref", new Object[0]));
                                            Type typeCreateErrorType2 = this.types.createErrorType(currentTarget2);
                                            that.type = typeCreateErrorType2;
                                            this.result = typeCreateErrorType2;
                                            return;
                                        }
                                        pair = null;
                                        if (!refSym.isStatic() && that.kind == JCTree.JCMemberReference.ReferenceKind.SUPER) {
                                            this.rs.checkNonAbstract(that.pos(), that.sym);
                                        }
                                        if (isTargetSerializable) {
                                            this.chk.checkElemAccessFromSerializableLambda(that);
                                        }
                                    } else {
                                        pair = null;
                                    }
                                    ResultInfo checkInfo = this.resultInfo.dup(newMethodTemplate(desc.mo178getReturnType().hasTag(TypeTag.VOID) ? Type.noType : desc.mo178getReturnType(), that.kind.isUnbound() ? argtypes.tail : argtypes, r23), new FunctionalReturnContext(this.resultInfo.checkContext));
                                    Type refType = checkId(this.noCheckTree, lookupHelper.site, refSym, localEnv, checkInfo);
                                    if (that.kind.isUnbound() && this.resultInfo.checkContext.inferenceContext().free(argtypes.head)) {
                                        exprType2 = currentTarget3;
                                        if (!this.types.isSubtype(this.resultInfo.checkContext.inferenceContext().asUndetVar(argtypes.head), exprType2)) {
                                            Assert.error("Can't get here");
                                        }
                                    } else {
                                        exprType2 = currentTarget3;
                                    }
                                    Type refType2 = !refType.isErroneous() ? this.types.createMethodTypeWithReturn(refType, adjustMethodReturnType(lookupHelper.site, that.name, checkInfo.pt.mo176getParameterTypes(), refType.mo178getReturnType())) : refType;
                                    boolean isSpeculativeRound = this.resultInfo.checkContext.deferredAttrContext().mode == DeferredAttr.AttrMode.SPECULATIVE;
                                    that.type = currentTarget2;
                                    checkReferenceCompatible(that, desc, refType2, this.resultInfo.checkContext, isSpeculativeRound);
                                    if (!isSpeculativeRound) {
                                        checkAccessibleTypes(that, localEnv, this.resultInfo.checkContext.inferenceContext(), desc, currentTarget2);
                                    }
                                    this.result = check(that, currentTarget2, 12, this.resultInfo);
                                    return;
                                }
                                switch (refSym.kind) {
                                    case 129:
                                    case 130:
                                    case 131:
                                    case 132:
                                    case 134:
                                    case 135:
                                    case 138:
                                        targetError = true;
                                        break;
                                    case 133:
                                    case 137:
                                    default:
                                        Assert.error("unexpected result kind " + refSym.kind);
                                        targetError = false;
                                        break;
                                    case 136:
                                        targetError = false;
                                        break;
                                }
                                JCDiagnostic detailsDiag = ((Resolve.ResolveError) refSym.baseSymbol()).getDiagnostic(JCDiagnostic.DiagnosticType.FRAGMENT, that, currentTarget3.tsym, currentTarget3, that.name, argtypes, r23);
                                if (targetError) {
                                    try {
                                        diagnosticType = JCDiagnostic.DiagnosticType.FRAGMENT;
                                    } catch (Types.FunctionDescriptorLookupError e2) {
                                        ex = e2;
                                        r10 = that;
                                    }
                                } else {
                                    diagnosticType = JCDiagnostic.DiagnosticType.ERROR;
                                }
                                JCDiagnostic.DiagnosticType diagKind = diagnosticType;
                                try {
                                    JCDiagnostic diag = this.diags.create(diagKind, this.log.currentSource(), that, "invalid.mref", Kinds.kindName(that.getMode()), detailsDiag);
                                    if (targetError && currentTarget2 == Type.recoveryType) {
                                        that.type = currentTarget2;
                                        this.result = currentTarget2;
                                        return;
                                    }
                                    if (targetError) {
                                        this.resultInfo.checkContext.report(that, diag);
                                    } else {
                                        this.log.report(diag);
                                    }
                                    Type typeCreateErrorType3 = this.types.createErrorType(currentTarget2);
                                    that.type = typeCreateErrorType3;
                                    this.result = typeCreateErrorType3;
                                    return;
                                } catch (Types.FunctionDescriptorLookupError e3) {
                                    ex = e3;
                                    r10 = that;
                                }
                            } catch (Types.FunctionDescriptorLookupError e4) {
                                ex = e4;
                                r10 = that;
                            }
                        } catch (Throwable th2) {
                            th = th2;
                            saved_undet = saved_undet2;
                            this.resultInfo.checkContext.inferenceContext().rollback(saved_undet);
                            throw th;
                        }
                    } catch (Types.FunctionDescriptorLookupError e5) {
                        ex = e5;
                    }
                } catch (Types.FunctionDescriptorLookupError e6) {
                    ex = e6;
                    r10 = that;
                }
                JCDiagnostic cause = ex.getDiagnostic();
                this.resultInfo.checkContext.report(r10, cause);
                Type typeCreateErrorType4 = this.types.createErrorType(pt());
                r10.type = typeCreateErrorType4;
                this.result = typeCreateErrorType4;
                return;
            }
            jCMemberReference = that;
        }
        if (pt().hasTag(TypeTag.NONE)) {
            this.log.error(that.pos(), "unexpected.mref", new Object[0]);
        }
        Type typeCreateErrorType5 = this.types.createErrorType(pt());
        jCMemberReference.type = typeCreateErrorType5;
        this.result = typeCreateErrorType5;
    }

    ResultInfo memberReferenceQualifierResult(JCTree.JCMemberReference tree) {
        return new ResultInfo(this, tree.getMode() == MemberReferenceTree.ReferenceMode.INVOKE ? 14 : 2, Type.noType);
    }

    void checkReferenceCompatible(JCTree.JCMemberReference tree, Type descriptor, Type refType, Check.CheckContext checkContext, boolean speculativeAttr) {
        Type resType;
        Type returnType = checkContext.inferenceContext().asUndetVar(descriptor.mo178getReturnType());
        switch (tree.getMode()) {
            case NEW:
                if (!tree.expr.type.isRaw()) {
                    resType = tree.expr.type;
                    break;
                }
            default:
                resType = refType.mo178getReturnType();
                break;
        }
        Type incompatibleReturnType = resType;
        if (returnType.hasTag(TypeTag.VOID)) {
            incompatibleReturnType = null;
        }
        if (!returnType.hasTag(TypeTag.VOID) && !resType.hasTag(TypeTag.VOID) && (resType.isErroneous() || new FunctionalReturnContext(checkContext).compatible(resType, returnType, this.types.noWarnings))) {
            incompatibleReturnType = null;
        }
        if (incompatibleReturnType != null) {
            checkContext.report(tree, this.diags.fragment("incompatible.ret.type.in.mref", this.diags.fragment("inconvertible.types", resType, descriptor.mo178getReturnType())));
        }
        if (!speculativeAttr) {
            List<Type> thrownTypes = checkContext.inferenceContext().asUndetVars(descriptor.mo179getThrownTypes());
            if (this.chk.unhandled(refType.mo179getThrownTypes(), thrownTypes).nonEmpty()) {
                this.log.error(tree, "incompatible.thrown.types.in.mref", refType.mo179getThrownTypes());
            }
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void setFunctionalInfo(final Env<AttrContext> env, final JCTree.JCFunctionalExpression fExpr, final Type pt, final Type descriptorType, final Type primaryTarget, final Check.CheckContext checkContext) {
        if (checkContext.inferenceContext().free(descriptorType)) {
            checkContext.inferenceContext().addFreeTypeListener(List.of(pt, descriptorType), new Infer.FreeTypeListener() { // from class: com.sun.tools.javac.comp.Attr.10
                @Override // com.sun.tools.javac.comp.Infer.FreeTypeListener
                public void typesInferred(Infer.InferenceContext inferenceContext) {
                    Attr.this.setFunctionalInfo(env, fExpr, pt, inferenceContext.asInstType(descriptorType), inferenceContext.asInstType(primaryTarget), checkContext);
                }
            });
            return;
        }
        ListBuffer<Type> targets = new ListBuffer<>();
        if (pt.hasTag(TypeTag.CLASS)) {
            if (pt.isCompound()) {
                targets.append(this.types.removeWildcards(primaryTarget));
                for (Type t : ((Type.IntersectionClassType) pt()).interfaces_field) {
                    if (t != primaryTarget) {
                        targets.append(this.types.removeWildcards(t));
                    }
                }
            } else {
                targets.append(this.types.removeWildcards(primaryTarget));
            }
        }
        fExpr.targets = targets.toList();
        if (checkContext.deferredAttrContext().mode == DeferredAttr.AttrMode.CHECK && pt != Type.recoveryType) {
            try {
                Symbol.ClassSymbol csym = this.types.makeFunctionalInterfaceClass(env, this.names.empty, List.of(fExpr.targets.head), 1024L);
                if (csym != null) {
                    this.chk.checkImplementations(env.tree, csym, csym);
                }
            } catch (Types.FunctionDescriptorLookupError ex) {
                JCDiagnostic cause = ex.getDiagnostic();
                this.resultInfo.checkContext.report(env.tree, cause);
            }
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitParens(JCTree.JCParens tree) {
        Type owntype = attribTree(tree.expr, this.env, this.resultInfo);
        this.result = check(tree, owntype, pkind(), this.resultInfo);
        Symbol sym = TreeInfo.symbol(tree);
        if (sym != null && (sym.kind & 3) != 0) {
            this.log.error(tree.pos(), "illegal.start.of.type", new Object[0]);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssign(JCTree.JCAssign tree) {
        Type owntype = attribTree(tree.lhs, this.env.dup(tree), this.varInfo);
        Type capturedType = capture(owntype);
        attribExpr(tree.rhs, this.env, owntype);
        this.result = check(tree, capturedType, 12, this.resultInfo);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssignop(JCTree.JCAssignOp tree) {
        Type owntype = attribTree(tree.lhs, this.env, this.varInfo);
        Type operand = attribExpr(tree.rhs, this.env);
        Symbol operator = this.rs.resolveBinaryOperator(tree.pos(), tree.getTag().noAssignOp(), this.env, owntype, operand);
        tree.operator = operator;
        if (operator.kind == 16 && !owntype.isErroneous() && !operand.isErroneous()) {
            this.chk.checkOperator(tree.pos(), (Symbol.OperatorSymbol) operator, tree.getTag().noAssignOp(), owntype, operand);
            this.chk.checkDivZero(tree.rhs.pos(), operator, operand);
            this.chk.checkCastable(tree.rhs.pos(), operator.type.mo178getReturnType(), owntype);
        }
        this.result = check(tree, owntype, 12, this.resultInfo);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitUnary(JCTree.JCUnary tree) {
        Type argtype;
        Type ctype;
        if (tree.getTag().isIncOrDecUnaryOp()) {
            argtype = attribTree(tree.arg, this.env, this.varInfo);
        } else {
            argtype = this.chk.checkNonVoid(tree.arg.pos(), attribExpr(tree.arg, this.env));
        }
        Symbol operator = this.rs.resolveUnaryOperator(tree.pos(), tree.getTag(), this.env, argtype);
        tree.operator = operator;
        Type owntype = this.types.createErrorType(tree.type);
        if (operator.kind == 16 && !argtype.isErroneous()) {
            owntype = tree.getTag().isIncOrDecUnaryOp() ? tree.arg.type : operator.type.mo178getReturnType();
            int opc = ((Symbol.OperatorSymbol) operator).opcode;
            if (argtype.constValue() != null && (ctype = this.cfolder.fold1(opc, argtype)) != null) {
                owntype = this.cfolder.coerce(ctype, owntype);
            }
        }
        this.result = check(tree, owntype, 12, this.resultInfo);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBinary(JCTree.JCBinary tree) {
        Type owntype;
        Type ctype;
        Type left = this.chk.checkNonVoid(tree.lhs.pos(), attribExpr(tree.lhs, this.env));
        Type right = this.chk.checkNonVoid(tree.lhs.pos(), attribExpr(tree.rhs, this.env));
        Symbol operator = this.rs.resolveBinaryOperator(tree.pos(), tree.getTag(), this.env, left, right);
        tree.operator = operator;
        Type owntype2 = this.types.createErrorType(tree.type);
        if (operator.kind == 16 && !left.isErroneous() && !right.isErroneous()) {
            Type owntype3 = operator.type.mo178getReturnType();
            int opc = this.chk.checkOperator(tree.lhs.pos(), (Symbol.OperatorSymbol) operator, tree.getTag(), left, right);
            if (left.constValue() != null && right.constValue() != null && (ctype = this.cfolder.fold2(opc, left, right)) != null) {
                owntype = this.cfolder.coerce(ctype, owntype3);
            } else {
                owntype = owntype3;
            }
            if ((opc == 165 || opc == 166) && !this.types.isEqualityComparable(left, right, new Warner(tree.pos()))) {
                this.log.error(tree.pos(), "incomparable.types", left, right);
            }
            this.chk.checkDivZero(tree.rhs.pos(), operator, right);
            owntype2 = owntype;
        }
        this.result = check(tree, owntype2, 12, this.resultInfo);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeCast(JCTree.JCTypeCast tree) {
        ResultInfo castInfo;
        Type clazztype = attribType(tree.clazz, this.env);
        boolean z = false;
        this.chk.validate(tree.clazz, this.env, false);
        Env<AttrContext> localEnv = this.env.dup(tree);
        JCTree.JCExpression expr = TreeInfo.skipParens(tree.expr);
        if (this.allowPoly && (expr.hasTag(JCTree.Tag.LAMBDA) || expr.hasTag(JCTree.Tag.REFERENCE))) {
            z = true;
        }
        boolean isPoly = z;
        if (isPoly) {
            castInfo = new ResultInfo(12, clazztype, new Check.NestedCheckContext(this.resultInfo.checkContext) { // from class: com.sun.tools.javac.comp.Attr.11
                @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                public boolean compatible(Type found, Type req, Warner warn) {
                    return Attr.this.types.isCastable(found, req, warn);
                }
            });
        } else {
            castInfo = this.unknownExprInfo;
        }
        Type exprtype = attribTree(tree.expr, localEnv, castInfo);
        Type owntype = isPoly ? clazztype : this.chk.checkCastable(tree.expr.pos(), exprtype, clazztype);
        if (exprtype.constValue() != null) {
            owntype = this.cfolder.coerce(exprtype, owntype);
        }
        this.result = check(tree, capture(owntype), 12, this.resultInfo);
        if (!isPoly) {
            this.chk.checkRedundantCast(localEnv, tree);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeTest(JCTree.JCInstanceOf tree) {
        Type exprtype = this.chk.checkNullOrRefType(tree.expr.pos(), attribExpr(tree.expr, this.env));
        Type clazztype = attribType(tree.clazz, this.env);
        if (!clazztype.hasTag(TypeTag.TYPEVAR)) {
            clazztype = this.chk.checkClassOrArrayType(tree.clazz.pos(), clazztype);
        }
        if (!clazztype.isErroneous() && !this.types.isReifiable(clazztype)) {
            this.log.error(tree.clazz.pos(), "illegal.generic.type.for.instof", new Object[0]);
            clazztype = this.types.createErrorType(clazztype);
        }
        this.chk.validate(tree.clazz, this.env, false);
        this.chk.checkCastable(tree.expr.pos(), exprtype, clazztype);
        this.result = check(tree, this.syms.booleanType, 12, this.resultInfo);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIndexed(JCTree.JCArrayAccess tree) {
        Type owntype = this.types.createErrorType(tree.type);
        Type atype = attribExpr(tree.indexed, this.env);
        attribExpr(tree.index, this.env, this.syms.intType);
        if (this.types.isArray(atype)) {
            owntype = this.types.elemtype(atype);
        } else if (!atype.hasTag(TypeTag.ERROR)) {
            this.log.error(tree.pos(), "array.req.but.found", atype);
        }
        if ((pkind() & 4) == 0) {
            owntype = capture(owntype);
        }
        this.result = check(tree, owntype, 4, this.resultInfo);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIdent(JCTree.JCIdent tree) {
        Symbol sym;
        if (pt().hasTag(TypeTag.METHOD) || pt().hasTag(TypeTag.FORALL)) {
            this.env.info.pendingResolutionPhase = null;
            sym = this.rs.resolveMethod(tree.pos(), this.env, tree.name, pt().mo176getParameterTypes(), pt().getTypeArguments());
        } else if (tree.sym != null && tree.sym.kind != 4) {
            sym = tree.sym;
        } else {
            sym = this.rs.resolveIdent(tree.pos(), this.env, tree.name, pkind());
        }
        tree.sym = sym;
        Env env = this.env;
        boolean noOuterThisPath = false;
        if (this.env.enclClass.sym.owner.kind != 1 && (sym.kind & 22) != 0 && sym.owner.kind == 2 && tree.name != this.names._this && tree.name != this.names._super) {
            while (env.outer != null && !sym.isMemberOf(env.enclClass.sym, this.types)) {
                if ((env.enclClass.sym.flags() & 4194304) != 0) {
                    noOuterThisPath = !this.allowAnonOuterThis;
                }
                env = env.outer;
            }
        }
        Env env2 = env;
        boolean noOuterThisPath2 = noOuterThisPath;
        if (sym.kind == 4) {
            Symbol.VarSymbol v = (Symbol.VarSymbol) sym;
            checkInit(tree, this.env, v, false);
            if (pkind() == 4) {
                checkAssignable(tree.pos(), v, null, this.env);
            }
        }
        if ((((AttrContext) env2.info).isSelfCall || noOuterThisPath2) && (sym.kind & 20) != 0 && sym.owner.kind == 2 && (sym.flags() & 8) == 0) {
            this.chk.earlyRefError(tree.pos(), sym.kind == 4 ? sym : thisSym(tree.pos(), this.env));
        }
        Env env3 = this.env;
        if (sym.kind != 63 && sym.kind != 2 && sym.owner != null && sym.owner != env3.enclClass.sym) {
            while (env3.outer != null && !this.rs.isAccessible(this.env, env3.enclClass.sym.type, sym)) {
                env3 = env3.outer;
            }
        }
        if (this.env.info.isSerializable) {
            this.chk.checkElemAccessFromSerializableLambda(tree);
        }
        this.result = checkId(tree, env3.enclClass.sym.type, sym, this.env, this.resultInfo);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSelect(JCTree.JCFieldAccess tree) {
        int skind;
        Type site;
        Symbol sym;
        Symbol sym2;
        Type site1;
        if (tree.name == this.names._this || tree.name == this.names._super || tree.name == this.names._class) {
            skind = 2;
        } else {
            int skind2 = (pkind() & 1) != 0 ? 0 | 1 : 0;
            if ((pkind() & 2) != 0) {
                skind2 = skind2 | 2 | 1;
            }
            if ((pkind() & 28) != 0) {
                skind2 = skind2 | 12 | 2;
            }
            skind = skind2;
        }
        Type site2 = attribTree(tree.selected, this.env, new ResultInfo(this, skind, Infer.anyPoly));
        if ((pkind() & 3) != 0) {
            site = site2;
        } else {
            site = capture(site2);
        }
        if (skind == 2) {
            Type elt = site;
            while (elt.hasTag(TypeTag.ARRAY)) {
                elt = ((Type.ArrayType) elt.unannotatedType()).elemtype;
            }
            if (elt.hasTag(TypeTag.TYPEVAR)) {
                this.log.error(tree.pos(), "type.var.cant.be.deref", new Object[0]);
                Type typeCreateErrorType = this.types.createErrorType(tree.name, site.tsym, site);
                tree.type = typeCreateErrorType;
                this.result = typeCreateErrorType;
                tree.sym = tree.type.tsym;
                return;
            }
        }
        Symbol sitesym = TreeInfo.symbol(tree.selected);
        boolean selectSuperPrev = this.env.info.selectSuper;
        this.env.info.selectSuper = sitesym != null && sitesym.name == this.names._super;
        this.env.info.pendingResolutionPhase = null;
        Symbol sym3 = selectSym(tree, sitesym, site, this.env, this.resultInfo);
        if (sym3.kind == 4 && sym3.name != this.names._super && this.env.info.defaultSuperCallSite != null) {
            this.log.error(tree.selected.pos(), "not.encl.class", site.tsym);
            sym = this.syms.errSymbol;
        } else {
            sym = sym3;
        }
        if (sym.exists() && !isType(sym) && (pkind() & 3) != 0) {
            site = capture(site);
            sym2 = selectSym(tree, sitesym, site, this.env, this.resultInfo);
        } else {
            sym2 = sym;
        }
        this.env.info.lastResolveVarargs();
        tree.sym = sym2;
        if (site.hasTag(TypeTag.TYPEVAR) && !isType(sym2) && sym2.kind != 63) {
            while (site.hasTag(TypeTag.TYPEVAR)) {
                site = site.getUpperBound();
            }
            site = capture(site);
        }
        if (sym2.kind == 4) {
            Symbol.VarSymbol v = (Symbol.VarSymbol) sym2;
            checkInit(tree, this.env, v, true);
            if (pkind() == 4) {
                checkAssignable(tree.pos(), v, tree.selected, this.env);
            }
        }
        if (sitesym != null && sitesym.kind == 4 && ((Symbol.VarSymbol) sitesym).isResourceVariable() && sym2.kind == 16 && sym2.name.equals(this.names.close) && sym2.overrides(this.syms.autoCloseableClose, sitesym.type.tsym, this.types, true) && this.env.info.lint.isEnabled(Lint.LintCategory.TRY)) {
            this.log.warning(Lint.LintCategory.TRY, tree, "try.explicit.close.call", new Object[0]);
        }
        if (isType(sym2) && (sitesym == null || (sitesym.kind & 3) == 0)) {
            tree.type = check(tree.selected, pt(), sitesym == null ? 12 : sitesym.kind, new ResultInfo(this, 3, pt()));
        }
        if (isType(sitesym)) {
            if (sym2.name == this.names._this) {
                if (this.env.info.isSelfCall && site.tsym == this.env.enclClass.sym) {
                    this.chk.earlyRefError(tree.pos(), sym2);
                }
            } else if ((sym2.flags() & 8) == 0 && !this.env.next.tree.hasTag(JCTree.Tag.REFERENCE) && sym2.name != this.names._super && (sym2.kind == 4 || sym2.kind == 16)) {
                Resolve resolve = this.rs;
                Resolve resolve2 = this.rs;
                resolve2.getClass();
                resolve.accessBase(resolve2.new StaticError(sym2), tree.pos(), site, sym2.name, true);
            }
            if (!this.allowStaticInterfaceMethods && sitesym.isInterface() && sym2.isStatic() && sym2.kind == 16) {
                this.log.error(tree.pos(), "static.intf.method.invoke.not.supported.in.source", this.sourceName);
            }
        } else if (sym2.kind != 63 && (sym2.flags() & 8) != 0 && sym2.name != this.names._class) {
            this.chk.warnStatic(tree, "static.not.qualified.by.type", Kinds.kindName(sym2.kind), sym2.owner);
        }
        if (this.env.info.selectSuper && (sym2.flags() & 8) == 0) {
            this.rs.checkNonAbstract(tree.pos(), sym2);
            if (site.isRaw() && (site1 = this.types.asSuper(this.env.enclClass.sym.type, site.tsym)) != null) {
                site = site1;
            }
        }
        if (this.env.info.isSerializable) {
            this.chk.checkElemAccessFromSerializableLambda(tree);
        }
        this.env.info.selectSuper = selectSuperPrev;
        this.result = checkId(tree, site, sym2, this.env, this.resultInfo);
    }

    private Symbol selectSym(JCTree.JCFieldAccess tree, Symbol location, Type site, Env<AttrContext> env, ResultInfo resultInfo) {
        List<Type> typeargs;
        Symbol sym2;
        JCDiagnostic.DiagnosticPosition pos = tree.pos();
        Name name = tree.name;
        switch (site.getTag()) {
            case PACKAGE:
                return this.rs.accessBase(this.rs.findIdentInPackage(env, site.tsym, name, resultInfo.pkind), pos, location, site, name, true);
            case ARRAY:
            case CLASS:
                if (resultInfo.pt.hasTag(TypeTag.METHOD) || resultInfo.pt.hasTag(TypeTag.FORALL)) {
                    return this.rs.resolveQualifiedMethod(pos, env, location, site, name, resultInfo.pt.mo176getParameterTypes(), resultInfo.pt.getTypeArguments());
                }
                if (name == this.names._this || name == this.names._super) {
                    return this.rs.resolveSelf(pos, env, site.tsym, name);
                }
                if (name == this.names._class) {
                    Type t = this.syms.classType;
                    if (this.allowGenerics) {
                        typeargs = List.of(this.types.erasure(site));
                    } else {
                        typeargs = List.nil();
                    }
                    return new Symbol.VarSymbol(25L, this.names._class, new Type.ClassType(t.getEnclosingType(), typeargs, t.tsym), site.tsym);
                }
                Symbol sym = this.rs.findIdentInType(env, site, name, resultInfo.pkind);
                if ((resultInfo.pkind & 128) == 0) {
                    return this.rs.accessBase(sym, pos, location, site, name, true);
                }
                return sym;
            case WILDCARD:
                throw new AssertionError(tree);
            case TYPEVAR:
                Symbol sym3 = site.getUpperBound() != null ? selectSym(tree, location, capture(site.getUpperBound()), env, resultInfo) : null;
                if (sym3 == null) {
                    this.log.error(pos, "type.var.cant.be.deref", new Object[0]);
                    return this.syms.errSymbol;
                }
                if ((sym3.flags() & 2) != 0) {
                    Resolve resolve = this.rs;
                    resolve.getClass();
                    sym2 = resolve.new AccessError(env, site, sym3);
                } else {
                    sym2 = sym3;
                }
                this.rs.accessBase(sym2, pos, location, site, name, true);
                return sym3;
            case ERROR:
                return this.types.createErrorType(name, site.tsym, site).tsym;
            default:
                if (name == this.names._class) {
                    Type t2 = this.syms.classType;
                    Type arg = this.types.boxedClass(site).type;
                    return new Symbol.VarSymbol(25L, this.names._class, new Type.ClassType(t2.getEnclosingType(), List.of(arg), t2.tsym), site.tsym);
                }
                this.log.error(pos, "cant.deref", site);
                return this.syms.errSymbol;
        }
    }

    Type checkId(JCTree tree, Type site, Symbol sym, Env<AttrContext> env, ResultInfo resultInfo) {
        if (resultInfo.pt.hasTag(TypeTag.FORALL) || resultInfo.pt.hasTag(TypeTag.METHOD)) {
            return checkMethodId(tree, site, sym, env, resultInfo);
        }
        return checkIdInternal(tree, site, sym, resultInfo.pt, env, resultInfo);
    }

    Type checkMethodId(JCTree tree, Type site, Symbol sym, Env<AttrContext> env, ResultInfo resultInfo) {
        boolean isPolymorhicSignature = (sym.baseSymbol().flags() & Flags.SIGNATURE_POLYMORPHIC) != 0;
        if (isPolymorhicSignature) {
            return checkSigPolyMethodId(tree, site, sym, env, resultInfo);
        }
        return checkMethodIdInternal(tree, site, sym, env, resultInfo);
    }

    Type checkSigPolyMethodId(JCTree tree, Type site, Symbol sym, Env<AttrContext> env, ResultInfo resultInfo) {
        checkMethodIdInternal(tree, site, sym.baseSymbol(), env, resultInfo);
        env.info.pendingResolutionPhase = Resolve.MethodResolutionPhase.BASIC;
        return sym.type;
    }

    Type checkMethodIdInternal(JCTree tree, Type site, Symbol sym, Env<AttrContext> env, ResultInfo resultInfo) {
        if ((resultInfo.pkind & 32) != 0) {
            Type type = resultInfo.pt;
            DeferredAttr deferredAttr = this.deferredAttr;
            deferredAttr.getClass();
            Type pt = type.map(deferredAttr.new RecoveryDeferredTypeMap(DeferredAttr.AttrMode.SPECULATIVE, sym, env.info.pendingResolutionPhase));
            Type owntype = checkIdInternal(tree, site, sym, pt, env, resultInfo);
            Type type2 = resultInfo.pt;
            DeferredAttr deferredAttr2 = this.deferredAttr;
            deferredAttr2.getClass();
            type2.map(deferredAttr2.new RecoveryDeferredTypeMap(DeferredAttr.AttrMode.CHECK, sym, env.info.pendingResolutionPhase));
            return owntype;
        }
        return checkIdInternal(tree, site, sym, resultInfo.pt, env, resultInfo);
    }

    Type checkIdInternal(JCTree tree, Type site, Symbol sym, Type pt, Env<AttrContext> env, ResultInfo resultInfo) {
        Type owntype;
        Type s;
        if (pt.isErroneous()) {
            return this.types.createErrorType(site);
        }
        switch (sym.kind) {
            case 1:
            case 63:
                owntype = sym.type;
                break;
            case 2:
                owntype = sym.type;
                if (owntype.hasTag(TypeTag.CLASS)) {
                    this.chk.checkForBadAuxiliaryClassAccess(tree.pos(), env, (Symbol.ClassSymbol) sym);
                    Type ownOuter = owntype.getEnclosingType();
                    if (owntype.tsym.type.getTypeArguments().nonEmpty()) {
                        owntype = this.types.erasure(owntype);
                    } else if (ownOuter.hasTag(TypeTag.CLASS) && site != ownOuter) {
                        Type normOuter = site;
                        if (normOuter.hasTag(TypeTag.CLASS)) {
                            normOuter = this.types.asEnclosingSuper(site, ownOuter.tsym);
                        }
                        if (normOuter == null) {
                            normOuter = this.types.erasure(ownOuter);
                        }
                        if (normOuter != ownOuter) {
                            owntype = new Type.ClassType(normOuter, List.nil(), owntype.tsym);
                        }
                    }
                }
                break;
            case 4:
                Symbol.VarSymbol v = (Symbol.VarSymbol) sym;
                if (this.allowGenerics && resultInfo.pkind == 4 && v.owner.kind == 2 && (v.flags() & 8) == 0 && ((site.hasTag(TypeTag.CLASS) || site.hasTag(TypeTag.TYPEVAR)) && (s = this.types.asOuterSuper(site, v.owner)) != null && s.isRaw() && !this.types.isSameType(v.type, v.erasure(this.types)))) {
                    this.chk.warnUnchecked(tree.pos(), "unchecked.assign.to.var", v, s);
                }
                Type owntype2 = (sym.owner.kind != 2 || sym.name == this.names._this || sym.name == this.names._super) ? sym.type : this.types.memberType(site, sym);
                if (v.getConstValue() != null && isStaticReference(tree)) {
                    owntype2 = owntype2.constType(v.getConstValue());
                }
                owntype = resultInfo.pkind != 12 ? owntype2 : capture(owntype2);
                break;
            case 16:
                owntype = checkMethod(site, sym, new ResultInfo(resultInfo.pkind, resultInfo.pt.mo178getReturnType(), resultInfo.checkContext), env, TreeInfo.args(env.tree), resultInfo.pt.mo176getParameterTypes(), resultInfo.pt.getTypeArguments());
                break;
            default:
                throw new AssertionError("unexpected kind: " + sym.kind + " in tree " + tree);
        }
        if (sym.name != this.names.init) {
            this.chk.checkDeprecated(tree.pos(), env.info.scope.owner, sym);
            this.chk.checkSunAPI(tree.pos(), sym);
            this.chk.checkProfile(tree.pos(), sym);
        }
        return check(tree, owntype, sym.kind, resultInfo);
    }

    private void checkInit(JCTree tree, Env<AttrContext> env, Symbol.VarSymbol v, boolean onlyWarning) {
        if ((env.info.enclVar == v || v.pos > tree.pos) && v.owner.kind == 2 && enclosingInitEnv(env) != null && v.owner == env.info.scope.owner.enclClass()) {
            if (((v.flags() & 8) != 0) == Resolve.isStatic(env) && (!env.tree.hasTag(JCTree.Tag.ASSIGN) || TreeInfo.skipParens(((JCTree.JCAssign) env.tree).lhs) != tree)) {
                String suffix = env.info.enclVar == v ? "self.ref" : "forward.ref";
                if (!onlyWarning || isStaticEnumField(v)) {
                    this.log.error(tree.pos(), "illegal." + suffix, new Object[0]);
                } else if (this.useBeforeDeclarationWarning) {
                    this.log.warning(tree.pos(), suffix, v);
                }
            }
        }
        v.getConstValue();
        checkEnumInitializer(tree, env, v);
    }

    /* JADX WARN: Multi-variable type inference failed */
    Env<AttrContext> enclosingInitEnv(Env<AttrContext> env) {
        Env env2 = env;
        while (true) {
            switch (env2.tree.getTag()) {
                case METHODDEF:
                case CLASSDEF:
                case TOPLEVEL:
                    return null;
                case VARDEF:
                    if (((JCTree.JCVariableDecl) env2.tree).sym.owner.kind == 2) {
                        return env2;
                    }
                    break;
                case BLOCK:
                    if (env2.next.tree.hasTag(JCTree.Tag.CLASSDEF)) {
                        return env2;
                    }
                    break;
            }
            Assert.checkNonNull(env2.next);
            env2 = env2.next;
        }
    }

    private void checkEnumInitializer(JCTree tree, Env<AttrContext> env, Symbol.VarSymbol v) {
        Symbol.ClassSymbol enclClass;
        if (!isStaticEnumField(v) || (enclClass = env.info.scope.owner.enclClass()) == null || enclClass.owner == null) {
            return;
        }
        if ((v.owner != enclClass && !this.types.isSubtype(enclClass.type, v.owner.type)) || !Resolve.isInitializer(env)) {
            return;
        }
        this.log.error(tree.pos(), "illegal.enum.static.ref", new Object[0]);
    }

    private boolean isStaticEnumField(Symbol.VarSymbol v) {
        return Flags.isEnum(v.owner) && Flags.isStatic(v) && !Flags.isConstant(v) && v.name != this.names._class;
    }

    public Type checkMethod(Type site, Symbol sym, ResultInfo resultInfo, Env<AttrContext> env, List<JCTree.JCExpression> argtrees, List<Type> argtypes, List<Type> typeargtypes) {
        Resolve.InapplicableMethodException inapplicableMethodException;
        List<Type> argtypes2;
        Env<AttrContext> env2;
        Type type;
        Type s;
        if (this.allowGenerics && (sym.flags() & 8) == 0 && ((site.hasTag(TypeTag.CLASS) || site.hasTag(TypeTag.TYPEVAR)) && (s = this.types.asOuterSuper(site, sym.owner)) != null && s.isRaw() && !this.types.isSameTypes(sym.type.mo176getParameterTypes(), sym.erasure(this.types).mo176getParameterTypes()))) {
            this.chk.warnUnchecked(env.tree.pos(), "unchecked.call.mbr.of.raw.type", sym, s);
        }
        if (env.info.defaultSuperCallSite != null) {
            Iterator<Type> it = this.types.interfaces(env.enclClass.type).prepend(this.types.supertype(env.enclClass.type)).iterator();
            while (true) {
                if (!it.hasNext()) {
                    break;
                }
                Type sup = it.next();
                if (sup.tsym.isSubClass(sym.enclClass(), this.types) && !this.types.isSameType(sup, env.info.defaultSuperCallSite)) {
                    List<Symbol.MethodSymbol> icand_sup = this.types.interfaceCandidates(sup, (Symbol.MethodSymbol) sym);
                    if (icand_sup.nonEmpty() && icand_sup.head != sym && icand_sup.head.overrides(sym, icand_sup.head.enclClass(), this.types, true)) {
                        this.log.error(env.tree.pos(), "illegal.default.super.call", env.info.defaultSuperCallSite, this.diags.fragment("overridden.default", sym, sup));
                        break;
                    }
                }
            }
            env.info.defaultSuperCallSite = null;
        }
        if (sym.isStatic() && site.isInterface() && env.tree.hasTag(JCTree.Tag.APPLY)) {
            JCTree.JCMethodInvocation app = (JCTree.JCMethodInvocation) env.tree;
            if (app.meth.hasTag(JCTree.Tag.SELECT) && !TreeInfo.isStaticSelector(((JCTree.JCFieldAccess) app.meth).selected, this.names)) {
                this.log.error(env.tree.pos(), "illegal.static.intf.meth.call", site);
            }
        }
        this.noteWarner.clear();
        try {
            Type owntype = this.rs.checkMethod(env, site, sym, resultInfo, argtypes, typeargtypes, this.noteWarner);
            DeferredAttr deferredAttr = this.deferredAttr;
            deferredAttr.getClass();
            DeferredAttr.DeferredTypeMap checkDeferredMap = deferredAttr.new DeferredTypeMap(DeferredAttr.AttrMode.CHECK, sym, env.info.pendingResolutionPhase);
            try {
                List<Type> argtypes3 = Type.map(argtypes, checkDeferredMap);
                try {
                    if (this.noteWarner.hasNonSilentLint(Lint.LintCategory.UNCHECKED)) {
                        this.chk.warnUnchecked(env.tree.pos(), "unchecked.meth.invocation.applied", Kinds.kindName(sym), sym.name, this.rs.methodArguments(sym.type.mo176getParameterTypes()), this.rs.methodArguments(Type.map(argtypes3, checkDeferredMap)), Kinds.kindName(sym.location()), sym.location());
                        owntype = new Type.MethodType(owntype.mo176getParameterTypes(), this.types.erasure(owntype.mo178getReturnType()), this.types.erasure(owntype.mo179getThrownTypes()), this.syms.methodClass);
                    }
                    return this.chk.checkMethod(owntype, sym, env, argtrees, argtypes3, env.info.lastResolveVarargs(), resultInfo.checkContext.inferenceContext());
                } catch (Infer.InferenceException e) {
                    ex = e;
                    env2 = env;
                    type = site;
                    resultInfo.checkContext.report(env2.tree.pos(), ex.getDiagnostic());
                    return this.types.createErrorType(type);
                } catch (Resolve.InapplicableMethodException e2) {
                    inapplicableMethodException = e2;
                    argtypes2 = argtypes3;
                    Resolve.InapplicableMethodException ex = inapplicableMethodException;
                    JCDiagnostic diag = ex.getDiagnostic();
                    Resolve resolve = this.rs;
                    resolve.getClass();
                    Resolve.InapplicableSymbolError errSym = new Resolve.InapplicableSymbolError(resolve, null, sym, diag) { // from class: com.sun.tools.javac.comp.Attr.12
                        final /* synthetic */ JCDiagnostic val$diag;
                        final /* synthetic */ Symbol val$sym;

                        /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
                        {
                            super(resolve, context);
                            this.val$sym = sym;
                            this.val$diag = diag;
                            resolve.getClass();
                        }

                        @Override // com.sun.tools.javac.comp.Resolve.InapplicableSymbolError
                        protected Pair<Symbol, JCDiagnostic> errCandidate() {
                            return new Pair<>(this.val$sym, this.val$diag);
                        }
                    };
                    Resolve resolve2 = this.rs;
                    resolve2.getClass();
                    List<Type> argtypes22 = Type.map(argtypes2, resolve2.new ResolveDeferredRecoveryMap(DeferredAttr.AttrMode.CHECK, sym, env.info.pendingResolutionPhase));
                    JCDiagnostic errDiag = errSym.getDiagnostic(JCDiagnostic.DiagnosticType.ERROR, env.tree, sym, site, sym.name, argtypes22, typeargtypes);
                    this.log.report(errDiag);
                    return this.types.createErrorType(site);
                }
            } catch (Infer.InferenceException e3) {
                ex = e3;
                env2 = env;
                type = site;
            } catch (Resolve.InapplicableMethodException e4) {
                inapplicableMethodException = e4;
                argtypes2 = argtypes;
            }
        } catch (Infer.InferenceException e5) {
            ex = e5;
            env2 = env;
            type = site;
        } catch (Resolve.InapplicableMethodException e6) {
            inapplicableMethodException = e6;
            argtypes2 = argtypes;
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLiteral(JCTree.JCLiteral tree) {
        this.result = check(tree, litType(tree.typetag).constType(tree.value), 12, this.resultInfo);
    }

    Type litType(TypeTag tag) {
        return tag == TypeTag.CLASS ? this.syms.stringType : this.syms.typeOfTag[tag.ordinal()];
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeIdent(JCTree.JCPrimitiveTypeTree tree) {
        this.result = check(tree, this.syms.typeOfTag[tree.typetag.ordinal()], 2, this.resultInfo);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeArray(JCTree.JCArrayTypeTree tree) {
        Type etype = attribType(tree.elemtype, this.env);
        Type type = new Type.ArrayType(etype, this.syms.arrayClass);
        this.result = check(tree, type, 2, this.resultInfo);
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r6v11, types: [A, com.sun.tools.javac.code.Type] */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeApply(JCTree.JCTypeApply tree) {
        Type site;
        Type owntype = this.types.createErrorType(tree.type);
        Type clazztype = this.chk.checkClassType(tree.clazz.pos(), attribType(tree.clazz, this.env));
        List<Type> actuals = attribTypes(tree.arguments, this.env);
        if (clazztype.hasTag(TypeTag.CLASS)) {
            List<Type> formals = clazztype.tsym.type.getTypeArguments();
            if (actuals.isEmpty()) {
                actuals = formals;
            }
            if (actuals.length() == formals.length()) {
                List list = actuals;
                List list2 = formals;
                while (list.nonEmpty()) {
                    list.head = ((Type) list.head).withTypeVar((Type) list2.head);
                    list = list.tail;
                    list2 = list2.tail;
                }
                Type clazzOuter = clazztype.getEnclosingType();
                if (clazzOuter.hasTag(TypeTag.CLASS)) {
                    JCTree.JCExpression clazz = TreeInfo.typeIn(tree.clazz);
                    if (clazz.hasTag(JCTree.Tag.IDENT)) {
                        site = this.env.enclClass.sym.type;
                    } else if (clazz.hasTag(JCTree.Tag.SELECT)) {
                        site = ((JCTree.JCFieldAccess) clazz).selected.type;
                    } else {
                        throw new AssertionError("" + tree);
                    }
                    if (clazzOuter.hasTag(TypeTag.CLASS) && site != clazzOuter) {
                        if (site.hasTag(TypeTag.CLASS)) {
                            site = this.types.asOuterSuper(site, clazzOuter.tsym);
                        }
                        if (site == null) {
                            site = this.types.erasure(clazzOuter);
                        }
                        clazzOuter = site;
                    }
                }
                owntype = new Type.ClassType(clazzOuter, actuals, clazztype.tsym);
            } else {
                if (formals.length() != 0) {
                    this.log.error(tree.pos(), "wrong.number.type.args", Integer.toString(formals.length()));
                } else {
                    this.log.error(tree.pos(), "type.doesnt.take.params", clazztype.tsym);
                }
                owntype = this.types.createErrorType(tree.type);
            }
        }
        this.result = check(tree, owntype, 2, this.resultInfo);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeUnion(JCTree.JCTypeUnion tree) {
        Iterator<JCTree.JCExpression> it;
        Iterator<JCTree.JCExpression> it2;
        ListBuffer<Type> multicatchTypes = new ListBuffer<>();
        ListBuffer<Type> all_multicatchTypes = null;
        Iterator<JCTree.JCExpression> it3 = tree.alternatives.iterator();
        while (it3.hasNext()) {
            JCTree.JCExpression typeTree = it3.next();
            Type ctype = this.chk.checkType(typeTree.pos(), this.chk.checkClassType(typeTree.pos(), attribType(typeTree, this.env)), this.syms.throwableType);
            if (!ctype.isErroneous()) {
                if (!this.chk.intersects(ctype, multicatchTypes.toList())) {
                    it = it3;
                } else {
                    for (Type t : multicatchTypes) {
                        boolean sub = this.types.isSubtype(ctype, t);
                        boolean sup = this.types.isSubtype(t, ctype);
                        if (sub || sup) {
                            Type a = sub ? ctype : t;
                            Type b = sub ? t : ctype;
                            it2 = it3;
                            this.log.error(typeTree.pos(), "multicatch.types.must.be.disjoint", a, b);
                        } else {
                            it2 = it3;
                        }
                        it3 = it2;
                    }
                    it = it3;
                }
                multicatchTypes.append(ctype);
                if (all_multicatchTypes != null) {
                    all_multicatchTypes.append(ctype);
                }
            } else {
                it = it3;
                if (all_multicatchTypes == null) {
                    all_multicatchTypes = new ListBuffer<>();
                    all_multicatchTypes.appendList(multicatchTypes);
                }
                all_multicatchTypes.append(ctype);
            }
            it3 = it;
        }
        Type t2 = check(this.noCheckTree, this.types.lub(multicatchTypes.toList()), 2, this.resultInfo);
        if (t2.hasTag(TypeTag.CLASS)) {
            List<Type> alternatives = (all_multicatchTypes == null ? multicatchTypes : all_multicatchTypes).toList();
            t2 = new Type.UnionClassType((Type.ClassType) t2, alternatives);
        }
        this.result = t2;
        tree.type = t2;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeIntersection(JCTree.JCTypeIntersection tree) {
        attribTypes(tree.bounds, this.env);
        Type typeCheckIntersection = checkIntersection(tree, tree.bounds);
        this.result = typeCheckIntersection;
        tree.type = typeCheckIntersection;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeParameter(JCTree.JCTypeParameter tree) {
        Type.TypeVar typeVar = (Type.TypeVar) tree.type;
        if (tree.annotations != null && tree.annotations.nonEmpty()) {
            annotateType(tree, tree.annotations);
        }
        if (!typeVar.bound.isErroneous()) {
            typeVar.bound = checkIntersection(tree, tree.bounds);
        }
    }

    Type checkIntersection(JCTree tree, List<JCTree.JCExpression> bounds) {
        JCTree.JCExpression extending;
        List<JCTree.JCExpression> implementing;
        Set<Type> boundSet = new HashSet<>();
        if (bounds.nonEmpty()) {
            bounds.head.type = checkBase(bounds.head.type, bounds.head, this.env, false, false, false);
            boundSet.add(this.types.erasure(bounds.head.type));
            if (bounds.head.type.isErroneous()) {
                return bounds.head.type;
            }
            if (bounds.head.type.hasTag(TypeTag.TYPEVAR)) {
                if (bounds.tail.nonEmpty()) {
                    this.log.error(bounds.tail.head.pos(), "type.var.may.not.be.followed.by.other.bounds", new Object[0]);
                    return bounds.head.type;
                }
            } else {
                for (JCTree.JCExpression bound : bounds.tail) {
                    bound.type = checkBase(bound.type, bound, this.env, false, true, false);
                    if (bound.type.isErroneous()) {
                        bounds = List.of(bound);
                    } else if (bound.type.hasTag(TypeTag.CLASS)) {
                        this.chk.checkNotRepeated(bound.pos(), this.types.erasure(bound.type), boundSet);
                    }
                }
            }
        }
        if (bounds.length() == 0) {
            return this.syms.objectType;
        }
        if (bounds.length() == 1) {
            return bounds.head.type;
        }
        Type owntype = this.types.makeIntersectionType(TreeInfo.types(bounds));
        if (!bounds.head.type.isInterface()) {
            extending = bounds.head;
            implementing = bounds.tail;
        } else {
            extending = null;
            implementing = bounds;
        }
        JCTree.JCClassDecl cd = this.make.at(tree).ClassDef(this.make.Modifiers(Flags.AnnotationTypeElementMask), this.names.empty, List.nil(), extending, implementing, List.nil());
        Symbol.ClassSymbol c = (Symbol.ClassSymbol) owntype.tsym;
        Assert.check((c.flags() & 16777216) != 0);
        cd.sym = c;
        c.sourcefile = this.env.toplevel.sourcefile;
        c.flags_field |= 268435456;
        Env<AttrContext> cenv = this.enter.classEnv(cd, this.env);
        this.typeEnvs.put(c, cenv);
        attribClass(c);
        return owntype;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWildcard(JCTree.JCWildcard tree) {
        Type type = tree.kind.kind == BoundKind.UNBOUND ? this.syms.objectType : attribType(tree.inner, this.env);
        this.result = check(tree, new Type.WildcardType(this.chk.checkRefType(tree.pos(), type), tree.kind.kind, this.syms.boundClass), 2, this.resultInfo);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotation(JCTree.JCAnnotation tree) {
        Assert.error("should be handled in Annotate");
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
        Type underlyingType = attribType(tree.getUnderlyingType(), this.env);
        attribAnnotationTypes(tree.annotations, this.env);
        annotateType(tree, tree.annotations);
        tree.type = underlyingType;
        this.result = underlyingType;
    }

    public void annotateType(final JCTree tree, final List<JCTree.JCAnnotation> annotations) {
        this.annotate.typeAnnotation(new Annotate.Worker() { // from class: com.sun.tools.javac.comp.Attr.13
            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public String toString() {
                return "annotate " + annotations + " onto " + tree;
            }

            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public void run() {
                List<Attribute.TypeCompound> compounds = Attr.fromAnnotations(annotations);
                if (annotations.size() == compounds.size()) {
                    tree.type = tree.type.unannotatedType().annotatedType(compounds);
                }
            }
        });
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static List<Attribute.TypeCompound> fromAnnotations(List<JCTree.JCAnnotation> annotations) {
        if (annotations.isEmpty()) {
            return List.nil();
        }
        ListBuffer<Attribute.TypeCompound> buf = new ListBuffer<>();
        for (JCTree.JCAnnotation anno : annotations) {
            if (anno.attribute != null) {
                buf.append((Attribute.TypeCompound) anno.attribute);
            }
        }
        return buf.toList();
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitErroneous(JCTree.JCErroneous tree) {
        if (tree.errs != null) {
            for (JCTree err : tree.errs) {
                attribTree(err, this.env, new ResultInfo(this, 63, pt()));
            }
        }
        Type type = this.syms.errType;
        tree.type = type;
        this.result = type;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTree(JCTree tree) {
        throw new AssertionError();
    }

    public void attrib(Env<AttrContext> env) {
        if (env.tree.hasTag(JCTree.Tag.TOPLEVEL)) {
            attribTopLevel(env);
        } else {
            attribClass(env.tree.pos(), env.enclClass.sym);
        }
    }

    public void attribTopLevel(Env<AttrContext> env) {
        JCTree.JCCompilationUnit toplevel = env.toplevel;
        try {
            this.annotate.flush();
        } catch (Symbol.CompletionFailure ex) {
            this.chk.completionError(toplevel.pos(), ex);
        }
    }

    public void attribClass(JCDiagnostic.DiagnosticPosition pos, Symbol.ClassSymbol c) {
        try {
            this.annotate.flush();
            attribClass(c);
        } catch (Symbol.CompletionFailure ex) {
            this.chk.completionError(pos, ex);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void attribClass(Symbol.ClassSymbol c) throws Symbol.CompletionFailure {
        if (c.type.hasTag(TypeTag.ERROR)) {
            return;
        }
        this.chk.checkNonCyclic((JCDiagnostic.DiagnosticPosition) null, c.type);
        Type st = this.types.supertype(c.type);
        if ((c.flags_field & 16777216) == 0) {
            if (st.hasTag(TypeTag.CLASS)) {
                attribClass((Symbol.ClassSymbol) st.tsym);
            }
            if (c.owner.kind == 2 && c.owner.type.hasTag(TypeTag.CLASS)) {
                attribClass((Symbol.ClassSymbol) c.owner);
            }
        }
        if ((c.flags_field & 268435456) != 0) {
            c.flags_field &= -268435457;
            Env<AttrContext> env = this.typeEnvs.get(c);
            Env env2 = env;
            while (((AttrContext) env2.info).lint == null) {
                env2 = env2.next;
            }
            env.info.lint = ((AttrContext) env2.info).lint.augment(c);
            Lint prevLint = this.chk.setLint(env.info.lint);
            JavaFileObject prev = this.log.useSource(c.sourcefile);
            ResultInfo prevReturnRes = env.info.returnResult;
            try {
                this.deferredLintHandler.flush(env.tree);
                env.info.returnResult = null;
                if (st.tsym == this.syms.enumSym && (c.flags_field & 16793600) == 0) {
                    this.log.error(env.tree.pos(), "enum.no.subclassing", new Object[0]);
                }
                if (st.tsym != null && (st.tsym.flags_field & 16384) != 0 && (c.flags_field & 16793600) == 0) {
                    this.log.error(env.tree.pos(), "enum.types.not.extensible", new Object[0]);
                }
                if (isSerializable(c.type)) {
                    env.info.isSerializable = true;
                }
                attribClassBody(env, c);
                this.chk.checkDeprecatedAnnotation(env.tree.pos(), c);
                this.chk.checkClassOverrideEqualsAndHashIfNeeded(env.tree.pos(), c);
                this.chk.checkFunctionalInterface((JCTree.JCClassDecl) env.tree, c);
            } finally {
                env.info.returnResult = prevReturnRes;
                this.log.useSource(prev);
                this.chk.setLint(prevLint);
            }
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitImport(JCTree.JCImport tree) {
    }

    /* JADX WARN: Multi-variable type inference failed */
    private void attribClassBody(Env<AttrContext> env, Symbol.ClassSymbol c) {
        JCTree.JCClassDecl tree = (JCTree.JCClassDecl) env.tree;
        Assert.check(c == tree.sym);
        attribStats(tree.typarams, env);
        if (!c.isAnonymous()) {
            this.chk.validate(tree.typarams, env);
            this.chk.validate(tree.extending, env);
            this.chk.validate(tree.implementing, env);
        }
        c.markAbstractIfNeeded(this.types);
        if ((c.flags() & 1536) == 0 && !this.relax) {
            this.chk.checkAllDefined(tree.pos(), c);
        }
        if ((c.flags() & 8192) != 0) {
            if (tree.implementing.nonEmpty()) {
                this.log.error(tree.implementing.head.pos(), "cant.extend.intf.annotation", new Object[0]);
            }
            if (tree.typarams.nonEmpty()) {
                this.log.error(tree.typarams.head.pos(), "intf.annotation.cant.have.type.params", new Object[0]);
            }
            Attribute.Compound repeatable = c.attribute(this.syms.repeatableType.tsym);
            if (repeatable != null) {
                JCDiagnostic.DiagnosticPosition cbPos = getDiagnosticPosition(tree, repeatable.type);
                Assert.checkNonNull(cbPos);
                this.chk.validateRepeatable(c, repeatable, cbPos);
            }
        } else {
            this.chk.checkCompatibleSupertypes(tree.pos(), c.type);
            if (this.allowDefaultMethods) {
                this.chk.checkDefaultMethodClashes(tree.pos(), c.type);
            }
        }
        this.chk.checkClassBounds(tree.pos(), c.type);
        tree.type = c.type;
        for (List list = tree.typarams; list.nonEmpty(); list = list.tail) {
            Assert.checkNonNull(env.info.scope.lookup(((JCTree.JCTypeParameter) list.head).name).scope);
        }
        if (!c.type.allparams().isEmpty() && this.types.isSubtype(c.type, this.syms.throwableType)) {
            this.log.error(tree.extending.pos(), "generic.throwable", new Object[0]);
        }
        this.chk.checkImplementations(tree);
        checkAutoCloseable(tree.pos(), env, c.type);
        for (List list2 = tree.defs; list2.nonEmpty(); list2 = list2.tail) {
            attribStat((JCTree) list2.head, env);
            if (c.owner.kind != 1 && (((c.flags() & 8) == 0 || c.name == this.names.empty) && (TreeInfo.flags((JCTree) list2.head) & 520) != 0)) {
                Symbol sym = ((JCTree) list2.head).hasTag(JCTree.Tag.VARDEF) ? ((JCTree.JCVariableDecl) list2.head).sym : null;
                if (sym == null || sym.kind != 4 || ((Symbol.VarSymbol) sym).getConstValue() == null) {
                    this.log.error(((JCTree) list2.head).pos(), "icls.cant.have.static.decl", c);
                }
            }
        }
        this.chk.checkCyclicConstructors(tree);
        this.chk.checkNonCyclicElements(tree);
        if (env.info.lint.isEnabled(Lint.LintCategory.SERIAL) && isSerializable(c.type) && (c.flags() & 16384) == 0 && checkForSerial(c)) {
            checkSerialVersionUID(tree, c);
        }
        if (this.allowTypeAnnos) {
            this.typeAnnotations.organizeTypeAnnotationsBodies(tree);
            validateTypeAnnotations(tree, false);
        }
    }

    boolean checkForSerial(Symbol.ClassSymbol c) {
        if ((c.flags() & 1024) == 0) {
            return true;
        }
        return c.members().anyMatch(anyNonAbstractOrDefaultMethod);
    }

    /* JADX WARN: Multi-variable type inference failed */
    private JCDiagnostic.DiagnosticPosition getDiagnosticPosition(JCTree.JCClassDecl tree, Type t) {
        for (List list = tree.mods.annotations; !list.isEmpty(); list = list.tail) {
            if (this.types.isSameType(((JCTree.JCAnnotation) list.head).annotationType.type, t)) {
                return ((JCTree.JCAnnotation) list.head).pos();
            }
        }
        return null;
    }

    boolean isSerializable(Type t) {
        try {
            this.syms.serializableType.complete();
            return this.types.isSubtype(t, this.syms.serializableType);
        } catch (Symbol.CompletionFailure e) {
            return false;
        }
    }

    private void checkSerialVersionUID(JCTree.JCClassDecl tree, Symbol.ClassSymbol c) {
        Scope.Entry e = c.members().lookup(this.names.serialVersionUID);
        while (e.scope != null && e.sym.kind != 4) {
            e = e.next();
        }
        if (e.scope == null) {
            this.log.warning(Lint.LintCategory.SERIAL, tree.pos(), "missing.SVUID", c);
            return;
        }
        Symbol.VarSymbol svuid = (Symbol.VarSymbol) e.sym;
        if ((svuid.flags() & 24) != 24) {
            this.log.warning(Lint.LintCategory.SERIAL, TreeInfo.diagnosticPositionFor(svuid, tree), "improper.SVUID", c);
        } else if (!svuid.type.hasTag(TypeTag.LONG)) {
            this.log.warning(Lint.LintCategory.SERIAL, TreeInfo.diagnosticPositionFor(svuid, tree), "long.SVUID", c);
        } else if (svuid.getConstValue() == null) {
            this.log.warning(Lint.LintCategory.SERIAL, TreeInfo.diagnosticPositionFor(svuid, tree), "constant.SVUID", c);
        }
    }

    private Type capture(Type type) {
        return this.types.capture(type);
    }

    public void validateTypeAnnotations(JCTree tree, boolean sigOnly) {
        tree.accept(new TypeAnnotationsValidator(sigOnly));
    }

    private final class TypeAnnotationsValidator extends TreeScanner {
        private final boolean sigOnly;

        public TypeAnnotationsValidator(boolean sigOnly) {
            this.sigOnly = sigOnly;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAnnotation(JCTree.JCAnnotation tree) {
            Attr.this.chk.validateTypeAnnotation(tree, false);
            super.visitAnnotation(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
            if (!tree.underlyingType.type.isErroneous()) {
                super.visitAnnotatedType(tree);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeParameter(JCTree.JCTypeParameter tree) {
            Attr.this.chk.validateTypeAnnotations(tree.annotations, true);
            scan(tree.bounds);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl tree) {
            if (tree.recvparam != null && !tree.recvparam.vartype.type.isErroneous()) {
                checkForDeclarationAnnotations(tree.recvparam.mods.annotations, tree.recvparam.vartype.type.tsym);
            }
            if (tree.restype != null && tree.restype.type != null) {
                validateAnnotatedType(tree.restype, tree.restype.type);
            }
            if (this.sigOnly) {
                scan(tree.mods);
                scan(tree.restype);
                scan(tree.typarams);
                scan(tree.recvparam);
                scan(tree.params);
                scan(tree.thrown);
                return;
            }
            scan(tree.defaultValue);
            scan(tree.body);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            if (tree.sym != null && tree.sym.type != null) {
                validateAnnotatedType(tree.vartype, tree.sym.type);
            }
            scan(tree.mods);
            scan(tree.vartype);
            if (!this.sigOnly) {
                scan(tree.init);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeCast(JCTree.JCTypeCast tree) {
            if (tree.clazz != null && tree.clazz.type != null) {
                validateAnnotatedType(tree.clazz, tree.clazz.type);
            }
            super.visitTypeCast(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeTest(JCTree.JCInstanceOf tree) {
            if (tree.clazz != null && tree.clazz.type != null) {
                validateAnnotatedType(tree.clazz, tree.clazz.type);
            }
            super.visitTypeTest(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            if (tree.clazz != null && tree.clazz.type != null) {
                if (tree.clazz.hasTag(JCTree.Tag.ANNOTATED_TYPE)) {
                    checkForDeclarationAnnotations(((JCTree.JCAnnotatedType) tree.clazz).annotations, tree.clazz.type.tsym);
                }
                if (tree.def != null) {
                    checkForDeclarationAnnotations(tree.def.mods.annotations, tree.clazz.type.tsym);
                }
                validateAnnotatedType(tree.clazz, tree.clazz.type);
            }
            super.visitNewClass(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewArray(JCTree.JCNewArray tree) {
            if (tree.elemtype != null && tree.elemtype.type != null) {
                if (tree.elemtype.hasTag(JCTree.Tag.ANNOTATED_TYPE)) {
                    checkForDeclarationAnnotations(((JCTree.JCAnnotatedType) tree.elemtype).annotations, tree.elemtype.type.tsym);
                }
                validateAnnotatedType(tree.elemtype, tree.elemtype.type);
            }
            super.visitNewArray(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            if (this.sigOnly) {
                scan(tree.mods);
                scan(tree.typarams);
                scan(tree.extending);
                scan(tree.implementing);
            }
            for (JCTree member : tree.defs) {
                if (!member.hasTag(JCTree.Tag.CLASSDEF)) {
                    scan(member);
                }
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBlock(JCTree.JCBlock tree) {
            if (!this.sigOnly) {
                scan(tree.stats);
            }
        }

        private void validateAnnotatedType(JCTree errtree, Type type) {
            if (type.isPrimitiveOrVoid()) {
                return;
            }
            JCTree enclTr = errtree;
            Type enclTy = type;
            boolean repeat = true;
            while (repeat) {
                if (enclTr.hasTag(JCTree.Tag.TYPEAPPLY)) {
                    List<Type> tyargs = enclTy.getTypeArguments();
                    List<JCTree.JCExpression> trargs = ((JCTree.JCTypeApply) enclTr).getTypeArguments();
                    if (trargs.length() > 0 && tyargs.length() == trargs.length()) {
                        for (int i = 0; i < tyargs.length(); i++) {
                            validateAnnotatedType(trargs.get(i), tyargs.get(i));
                        }
                    }
                    enclTr = ((JCTree.JCTypeApply) enclTr).clazz;
                }
                if (enclTr.hasTag(JCTree.Tag.SELECT)) {
                    enclTr = ((JCTree.JCFieldAccess) enclTr).getExpression();
                    if (enclTy != null && !enclTy.hasTag(TypeTag.NONE)) {
                        enclTy = enclTy.getEnclosingType();
                    }
                } else if (enclTr.hasTag(JCTree.Tag.ANNOTATED_TYPE)) {
                    JCTree.JCAnnotatedType at = (JCTree.JCAnnotatedType) enclTr;
                    if (enclTy == null || enclTy.hasTag(TypeTag.NONE)) {
                        if (at.getAnnotations().size() == 1) {
                            Attr.this.log.error(at.underlyingType.pos(), "cant.type.annotate.scoping.1", at.getAnnotations().head.attribute);
                        } else {
                            ListBuffer<Attribute.Compound> comps = new ListBuffer<>();
                            for (JCTree.JCAnnotation an : at.getAnnotations()) {
                                comps.add(an.attribute);
                            }
                            Attr.this.log.error(at.underlyingType.pos(), "cant.type.annotate.scoping", comps.toList());
                        }
                        repeat = false;
                    }
                    enclTr = at.underlyingType;
                } else if (enclTr.hasTag(JCTree.Tag.IDENT)) {
                    repeat = false;
                } else if (enclTr.hasTag(JCTree.Tag.WILDCARD)) {
                    JCTree.JCWildcard wc = (JCTree.JCWildcard) enclTr;
                    if (wc.getKind() == Tree.Kind.EXTENDS_WILDCARD) {
                        validateAnnotatedType(wc.getBound(), ((Type.WildcardType) enclTy.unannotatedType()).getExtendsBound());
                    } else if (wc.getKind() == Tree.Kind.SUPER_WILDCARD) {
                        validateAnnotatedType(wc.getBound(), ((Type.WildcardType) enclTy.unannotatedType()).getSuperBound());
                    }
                    repeat = false;
                } else if (enclTr.hasTag(JCTree.Tag.TYPEARRAY)) {
                    JCTree.JCArrayTypeTree art = (JCTree.JCArrayTypeTree) enclTr;
                    validateAnnotatedType(art.getType(), ((Type.ArrayType) enclTy.unannotatedType()).getComponentType());
                    repeat = false;
                } else if (enclTr.hasTag(JCTree.Tag.TYPEUNION)) {
                    JCTree.JCTypeUnion ut = (JCTree.JCTypeUnion) enclTr;
                    for (JCTree t : ut.getTypeAlternatives()) {
                        validateAnnotatedType(t, t.type);
                    }
                    repeat = false;
                } else if (enclTr.hasTag(JCTree.Tag.TYPEINTERSECTION)) {
                    JCTree.JCTypeIntersection it = (JCTree.JCTypeIntersection) enclTr;
                    for (JCTree t2 : it.getBounds()) {
                        validateAnnotatedType(t2, t2.type);
                    }
                    repeat = false;
                } else if (enclTr.getKind() == Tree.Kind.PRIMITIVE_TYPE || enclTr.getKind() == Tree.Kind.ERRONEOUS) {
                    repeat = false;
                } else {
                    Assert.error("Unexpected tree: " + enclTr + " with kind: " + enclTr.getKind() + " within: " + errtree + " with kind: " + errtree.getKind());
                }
            }
        }

        private void checkForDeclarationAnnotations(List<? extends JCTree.JCAnnotation> annotations, Symbol sym) {
            for (JCTree.JCAnnotation ai : annotations) {
                if (!ai.type.isErroneous() && Attr.this.typeAnnotations.annotationType(ai.attribute, sym) == TypeAnnotations.AnnotationType.DECLARATION) {
                    Attr.this.log.error(ai.pos(), "annotation.type.not.applicable", new Object[0]);
                }
            }
        }
    }

    public void postAttr(JCTree tree) {
        new PostAttrAnalyzer().scan(tree);
    }

    class PostAttrAnalyzer extends TreeScanner {
        PostAttrAnalyzer() {
        }

        private void initTypeIfNeeded(JCTree that) {
            if (that.type == null) {
                if (that.hasTag(JCTree.Tag.METHODDEF)) {
                    that.type = dummyMethodType((JCTree.JCMethodDecl) that);
                } else {
                    that.type = Attr.this.syms.unknownType;
                }
            }
        }

        private Type dummyMethodType(JCTree.JCMethodDecl md) {
            Type restype = Attr.this.syms.unknownType;
            if (md != null && md.restype.hasTag(JCTree.Tag.TYPEIDENT)) {
                JCTree.JCPrimitiveTypeTree prim = (JCTree.JCPrimitiveTypeTree) md.restype;
                if (prim.typetag == TypeTag.VOID) {
                    restype = Attr.this.syms.voidType;
                }
            }
            return new Type.MethodType(List.nil(), restype, List.nil(), Attr.this.syms.methodClass);
        }

        private Type dummyMethodType() {
            return dummyMethodType(null);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree tree) {
            if (tree == null) {
                return;
            }
            if (tree instanceof JCTree.JCExpression) {
                initTypeIfNeeded(tree);
            }
            super.scan(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent that) {
            if (that.sym == null) {
                that.sym = Attr.this.syms.unknownSymbol;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess that) {
            if (that.sym == null) {
                that.sym = Attr.this.syms.unknownSymbol;
            }
            super.visitSelect(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl that) {
            initTypeIfNeeded(that);
            if (that.sym == null) {
                that.sym = new Symbol.ClassSymbol(0L, that.name, that.type, Attr.this.syms.noSymbol);
            }
            super.visitClassDef(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl that) {
            initTypeIfNeeded(that);
            if (that.sym == null) {
                that.sym = new Symbol.MethodSymbol(0L, that.name, that.type, Attr.this.syms.noSymbol);
            }
            super.visitMethodDef(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl that) {
            initTypeIfNeeded(that);
            if (that.sym == null) {
                that.sym = new Symbol.VarSymbol(0L, that.name, that.type, Attr.this.syms.noSymbol);
                that.sym.adr = 0;
            }
            super.visitVarDef(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass that) {
            if (that.constructor == null) {
                that.constructor = new Symbol.MethodSymbol(0L, Attr.this.names.init, dummyMethodType(), Attr.this.syms.noSymbol);
            }
            if (that.constructorType == null) {
                that.constructorType = Attr.this.syms.unknownType;
            }
            super.visitNewClass(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssignop(JCTree.JCAssignOp that) {
            if (that.operator == null) {
                that.operator = new Symbol.OperatorSymbol(Attr.this.names.empty, dummyMethodType(), -1, Attr.this.syms.noSymbol);
            }
            super.visitAssignop(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBinary(JCTree.JCBinary that) {
            if (that.operator == null) {
                that.operator = new Symbol.OperatorSymbol(Attr.this.names.empty, dummyMethodType(), -1, Attr.this.syms.noSymbol);
            }
            super.visitBinary(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitUnary(JCTree.JCUnary that) {
            if (that.operator == null) {
                that.operator = new Symbol.OperatorSymbol(Attr.this.names.empty, dummyMethodType(), -1, Attr.this.syms.noSymbol);
            }
            super.visitUnary(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda that) {
            super.visitLambda(that);
            if (that.targets == null) {
                that.targets = List.nil();
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReference(JCTree.JCMemberReference that) {
            super.visitReference(that);
            if (that.sym == null) {
                that.sym = new Symbol.MethodSymbol(0L, Attr.this.names.empty, dummyMethodType(), Attr.this.syms.noSymbol);
            }
            if (that.targets == null) {
                that.targets = List.nil();
            }
        }
    }
}
