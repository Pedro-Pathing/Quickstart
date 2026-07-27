package com.sun.tools.javac.comp;

import com.sun.source.tree.LambdaExpressionTree;
import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.api.Formattable;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Kinds;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.comp.Attr.ResultInfo;
import com.sun.tools.javac.comp.Check;
import com.sun.tools.javac.comp.DeferredAttr;
import com.sun.tools.javac.comp.DeferredAttr.DeferredAttrContext;
import com.sun.tools.javac.comp.DeferredAttr.RecoveryDeferredTypeMap;
import com.sun.tools.javac.comp.Infer;
import com.sun.tools.javac.jvm.ClassReader;
import com.sun.tools.javac.jvm.Target;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.DiagnosticSource;
import com.sun.tools.javac.util.FatalError;
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
import java.util.Arrays;
import java.util.Collection;
import java.util.EnumSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import javax.lang.model.element.ElementVisitor;

/* JADX INFO: loaded from: classes.dex */
public class Resolve {
    protected static final Context.Key<Resolve> resolveKey = new Context.Key<>();
    public final boolean allowFunctionalInterfaceMostSpecific;
    public final boolean allowMethodHandles;
    Attr attr;
    public final boolean boxingEnabled;
    public final boolean checkVarargsAccessAfterResolution;
    Check chk;
    private final boolean compactMethodDiags;
    private final boolean debugResolve;
    DeferredAttr deferredAttr;
    JCDiagnostic.Factory diags;
    private final InapplicableMethodException inapplicableMethodException;
    Infer infer;
    Log log;
    private final SymbolNotFoundError methodNotFound;
    private final SymbolNotFoundError methodWithCorrectStaticnessNotFound;
    Names names;
    Scope polymorphicSignatureScope;
    ClassReader reader;
    Symtab syms;
    TreeInfo treeinfo;
    private final SymbolNotFoundError typeNotFound;
    Types types;
    private final SymbolNotFoundError varNotFound;
    public final boolean varargsEnabled;
    final EnumSet<VerboseResolutionMode> verboseResolutionMode;
    Types.SimpleVisitor<Void, Env<AttrContext>> accessibilityChecker = new Types.SimpleVisitor<Void, Env<AttrContext>>() { // from class: com.sun.tools.javac.comp.Resolve.1
        void visit(List<Type> ts, Env<AttrContext> env) {
            for (Type t : ts) {
                visit(t, env);
            }
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Void visitType(Type t, Env<AttrContext> env) {
            return null;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Void visitArrayType(Type.ArrayType t, Env<AttrContext> env) {
            visit(t.elemtype, env);
            return null;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Void visitClassType(Type.ClassType t, Env<AttrContext> env) {
            visit(t.getTypeArguments(), env);
            if (!Resolve.this.isAccessible(env, (Type) t, true)) {
                Resolve.this.accessBase(new AccessError(Resolve.this, t.tsym), env.tree.pos(), env.enclClass.sym, t, t.tsym.name, true);
                return null;
            }
            return null;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Void visitWildcardType(Type.WildcardType t, Env<AttrContext> env) {
            visit(t.type, env);
            return null;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Void visitMethodType(Type.MethodType t, Env<AttrContext> env) {
            visit(t.mo176getParameterTypes(), env);
            visit(t.mo178getReturnType(), env);
            visit(t.mo179getThrownTypes(), env);
            return null;
        }
    };
    MethodCheck nilMethodCheck = new MethodCheck() { // from class: com.sun.tools.javac.comp.Resolve.2
        @Override // com.sun.tools.javac.comp.Resolve.MethodCheck
        public void argumentsAcceptable(Env<AttrContext> env, DeferredAttr.DeferredAttrContext deferredAttrContext, List<Type> argtypes, List<Type> formals, Warner warn) {
        }

        @Override // com.sun.tools.javac.comp.Resolve.MethodCheck
        public MethodCheck mostSpecificCheck(List<Type> actuals, boolean strict) {
            return this;
        }
    };
    MethodCheck arityMethodCheck = new AbstractMethodCheck() { // from class: com.sun.tools.javac.comp.Resolve.3
        @Override // com.sun.tools.javac.comp.Resolve.AbstractMethodCheck
        void checkArg(JCDiagnostic.DiagnosticPosition pos, boolean varargs, Type actual, Type formal, DeferredAttr.DeferredAttrContext deferredAttrContext, Warner warn) {
        }

        public String toString() {
            return "arityMethodCheck";
        }
    };
    MethodCheck resolveMethodCheck = new AbstractMethodCheck() { // from class: com.sun.tools.javac.comp.Resolve.4
        @Override // com.sun.tools.javac.comp.Resolve.AbstractMethodCheck
        void checkArg(JCDiagnostic.DiagnosticPosition pos, boolean varargs, Type actual, Type formal, DeferredAttr.DeferredAttrContext deferredAttrContext, Warner warn) {
            Attr.ResultInfo mresult = methodCheckResult(varargs, formal, deferredAttrContext, warn);
            mresult.check(pos, actual);
        }

        @Override // com.sun.tools.javac.comp.Resolve.AbstractMethodCheck, com.sun.tools.javac.comp.Resolve.MethodCheck
        public void argumentsAcceptable(Env<AttrContext> env, DeferredAttr.DeferredAttrContext deferredAttrContext, List<Type> argtypes, List<Type> formals, Warner warn) {
            super.argumentsAcceptable(env, deferredAttrContext, argtypes, formals, warn);
            if (deferredAttrContext.phase.isVarargsRequired()) {
                if (deferredAttrContext.mode == DeferredAttr.AttrMode.CHECK || !Resolve.this.checkVarargsAccessAfterResolution) {
                    varargsAccessible(env, Resolve.this.types.elemtype(formals.last()), deferredAttrContext.inferenceContext);
                }
            }
        }

        /* JADX INFO: Access modifiers changed from: private */
        public void varargsAccessible(final Env<AttrContext> env, final Type t, Infer.InferenceContext inferenceContext) {
            if (inferenceContext.free(t)) {
                inferenceContext.addFreeTypeListener(List.of(t), new Infer.FreeTypeListener() { // from class: com.sun.tools.javac.comp.Resolve.4.1
                    @Override // com.sun.tools.javac.comp.Infer.FreeTypeListener
                    public void typesInferred(Infer.InferenceContext inferenceContext2) {
                        varargsAccessible(env, inferenceContext2.asInstType(t), inferenceContext2);
                    }
                });
            } else if (!Resolve.this.isAccessible(env, Resolve.this.types.erasure(t))) {
                Symbol location = env.enclClass.sym;
                reportMC(env.tree, MethodCheckDiag.INACCESSIBLE_VARARGS, inferenceContext, t, Kinds.kindName(location), location);
            }
        }

        private Attr.ResultInfo methodCheckResult(final boolean varargsCheck, Type to, DeferredAttr.DeferredAttrContext deferredAttrContext, Warner rsWarner) {
            Check.CheckContext checkContext = new MethodCheckContext(!deferredAttrContext.phase.isBoxingRequired(), deferredAttrContext, rsWarner) { // from class: com.sun.tools.javac.comp.Resolve.4.2
                MethodCheckDiag methodDiag;

                {
                    Resolve resolve = Resolve.this;
                    this.methodDiag = varargsCheck ? MethodCheckDiag.VARARG_MISMATCH : MethodCheckDiag.ARG_MISMATCH;
                }

                @Override // com.sun.tools.javac.comp.Resolve.MethodCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                public void report(JCDiagnostic.DiagnosticPosition pos, JCDiagnostic details) {
                    reportMC(pos, this.methodDiag, this.deferredAttrContext.inferenceContext, details);
                }
            };
            return Resolve.this.new MethodResultInfo(to, checkContext);
        }

        @Override // com.sun.tools.javac.comp.Resolve.AbstractMethodCheck, com.sun.tools.javac.comp.Resolve.MethodCheck
        public MethodCheck mostSpecificCheck(List<Type> actuals, boolean strict) {
            return Resolve.this.new MostSpecificCheck(strict, actuals);
        }

        public String toString() {
            return "resolveMethodCheck";
        }
    };
    Warner noteWarner = new Warner();
    LogResolveHelper basicLogResolveHelper = new LogResolveHelper() { // from class: com.sun.tools.javac.comp.Resolve.6
        @Override // com.sun.tools.javac.comp.Resolve.LogResolveHelper
        public boolean resolveDiagnosticNeeded(Type site, List<Type> argtypes, List<Type> typeargtypes) {
            return !site.isErroneous();
        }

        @Override // com.sun.tools.javac.comp.Resolve.LogResolveHelper
        public List<Type> getArgumentTypes(ResolveError errSym, Symbol accessedSym, Name name, List<Type> argtypes) {
            return argtypes;
        }
    };
    LogResolveHelper methodLogResolveHelper = new LogResolveHelper() { // from class: com.sun.tools.javac.comp.Resolve.7
        @Override // com.sun.tools.javac.comp.Resolve.LogResolveHelper
        public boolean resolveDiagnosticNeeded(Type site, List<Type> argtypes, List<Type> typeargtypes) {
            return (site.isErroneous() || Type.isErroneous(argtypes) || (typeargtypes != null && Type.isErroneous(typeargtypes))) ? false : true;
        }

        @Override // com.sun.tools.javac.comp.Resolve.LogResolveHelper
        public List<Type> getArgumentTypes(ResolveError errSym, Symbol accessedSym, Name name, List<Type> argtypes) {
            return Resolve.this.syms.operatorNames.contains(name) ? argtypes : Type.map(argtypes, Resolve.this.new ResolveDeferredRecoveryMap(DeferredAttr.AttrMode.SPECULATIVE, accessedSym, Resolve.this.currentResolutionContext.step));
        }
    };
    private final Formattable.LocalizedString noArgs = new Formattable.LocalizedString("compiler.misc.no.args");
    final List<MethodResolutionPhase> methodResolutionSteps = List.of(MethodResolutionPhase.BASIC, MethodResolutionPhase.BOX, MethodResolutionPhase.VARARITY);
    MethodResolutionContext currentResolutionContext = null;

    enum InterfaceLookupPhase {
        ABSTRACT_OK { // from class: com.sun.tools.javac.comp.Resolve.InterfaceLookupPhase.1
            @Override // com.sun.tools.javac.comp.Resolve.InterfaceLookupPhase
            InterfaceLookupPhase update(Symbol s, Resolve rs) {
                if ((s.flags() & 17920) != 0) {
                    return this;
                }
                return DEFAULT_OK;
            }
        },
        DEFAULT_OK { // from class: com.sun.tools.javac.comp.Resolve.InterfaceLookupPhase.2
            @Override // com.sun.tools.javac.comp.Resolve.InterfaceLookupPhase
            InterfaceLookupPhase update(Symbol s, Resolve rs) {
                return this;
            }
        };

        abstract InterfaceLookupPhase update(Symbol symbol, Resolve resolve);
    }

    interface LogResolveHelper {
        List<Type> getArgumentTypes(ResolveError resolveError, Symbol symbol, Name name, List<Type> list);

        boolean resolveDiagnosticNeeded(Type type, List<Type> list, List<Type> list2);
    }

    interface MethodCheck {
        void argumentsAcceptable(Env<AttrContext> env, DeferredAttr.DeferredAttrContext deferredAttrContext, List<Type> list, List<Type> list2, Warner warner);

        MethodCheck mostSpecificCheck(List<Type> list, boolean z);
    }

    enum SearchResultKind {
        GOOD_MATCH,
        BAD_MATCH_MORE_SPECIFIC,
        BAD_MATCH,
        NOT_APPLICABLE_MATCH
    }

    protected Resolve(Context context) {
        context.put(resolveKey, this);
        this.syms = Symtab.instance(context);
        this.varNotFound = new SymbolNotFoundError(this, 133);
        this.methodNotFound = new SymbolNotFoundError(this, 136);
        this.methodWithCorrectStaticnessNotFound = new SymbolNotFoundError(138, "method found has incorrect staticness");
        this.typeNotFound = new SymbolNotFoundError(this, 137);
        this.names = Names.instance(context);
        this.log = Log.instance(context);
        this.attr = Attr.instance(context);
        this.deferredAttr = DeferredAttr.instance(context);
        this.chk = Check.instance(context);
        this.infer = Infer.instance(context);
        this.reader = ClassReader.instance(context);
        this.treeinfo = TreeInfo.instance(context);
        this.types = Types.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        Source source = Source.instance(context);
        this.boxingEnabled = source.allowBoxing();
        this.varargsEnabled = source.allowVarargs();
        Options options = Options.instance(context);
        this.debugResolve = options.isSet("debugresolve");
        this.compactMethodDiags = options.isSet(Option.XDIAGS, "compact") || (options.isUnset(Option.XDIAGS) && options.isUnset("rawDiagnostics"));
        this.verboseResolutionMode = VerboseResolutionMode.getVerboseResolutionMode(options);
        Target target = Target.instance(context);
        this.allowMethodHandles = target.hasMethodHandles();
        this.allowFunctionalInterfaceMostSpecific = source.allowFunctionalInterfaceMostSpecific();
        this.checkVarargsAccessAfterResolution = source.allowPostApplicabilityVarargsAccessCheck();
        this.polymorphicSignatureScope = new Scope(this.syms.noSymbol);
        this.inapplicableMethodException = new InapplicableMethodException(this.diags);
    }

    public static Resolve instance(Context context) {
        Resolve instance = (Resolve) context.get(resolveKey);
        if (instance == null) {
            return new Resolve(context);
        }
        return instance;
    }

    enum VerboseResolutionMode {
        SUCCESS("success"),
        FAILURE("failure"),
        APPLICABLE("applicable"),
        INAPPLICABLE("inapplicable"),
        DEFERRED_INST("deferred-inference"),
        PREDEF("predef"),
        OBJECT_INIT("object-init"),
        INTERNAL("internal");

        final String opt;

        VerboseResolutionMode(String opt) {
            this.opt = opt;
        }

        static EnumSet<VerboseResolutionMode> getVerboseResolutionMode(Options opts) {
            String s = opts.get("verboseResolution");
            EnumSet<VerboseResolutionMode> res = EnumSet.noneOf(VerboseResolutionMode.class);
            if (s == null) {
                return res;
            }
            if (s.contains("all")) {
                res = EnumSet.allOf(VerboseResolutionMode.class);
            }
            Collection<String> args = Arrays.asList(s.split(DocLint.TAGS_SEPARATOR));
            for (VerboseResolutionMode mode : values()) {
                if (args.contains(mode.opt)) {
                    res.add(mode);
                } else if (args.contains("-" + mode.opt)) {
                    res.remove(mode);
                }
            }
            return res;
        }
    }

    void reportVerboseResolutionDiagnostic(JCDiagnostic.DiagnosticPosition dpos, Name name, Type site, List<Type> argtypes, List<Type> typeargtypes, Symbol bestSoFar) {
        JCDiagnostic verboseInapplicableCandidateDiag;
        boolean success = bestSoFar.kind < 128;
        if (success && !this.verboseResolutionMode.contains(VerboseResolutionMode.SUCCESS)) {
            return;
        }
        if (success || this.verboseResolutionMode.contains(VerboseResolutionMode.FAILURE)) {
            if (bestSoFar.name == this.names.init && bestSoFar.owner == this.syms.objectType.tsym && !this.verboseResolutionMode.contains(VerboseResolutionMode.OBJECT_INIT)) {
                return;
            }
            if (site == this.syms.predefClass.type && !this.verboseResolutionMode.contains(VerboseResolutionMode.PREDEF)) {
                return;
            }
            if (this.currentResolutionContext.internalResolution && !this.verboseResolutionMode.contains(VerboseResolutionMode.INTERNAL)) {
                return;
            }
            int pos = 0;
            int mostSpecificPos = -1;
            ListBuffer<JCDiagnostic> subDiags = new ListBuffer<>();
            for (MethodResolutionContext.Candidate c : this.currentResolutionContext.candidates) {
                if (this.currentResolutionContext.step == c.step && (!c.isApplicable() || this.verboseResolutionMode.contains(VerboseResolutionMode.APPLICABLE))) {
                    if (c.isApplicable() || this.verboseResolutionMode.contains(VerboseResolutionMode.INAPPLICABLE)) {
                        if (c.isApplicable()) {
                            verboseInapplicableCandidateDiag = getVerboseApplicableCandidateDiag(pos, c.sym, c.mtype);
                        } else {
                            verboseInapplicableCandidateDiag = getVerboseInapplicableCandidateDiag(pos, c.sym, c.details);
                        }
                        subDiags.append(verboseInapplicableCandidateDiag);
                        if (c.sym == bestSoFar) {
                            mostSpecificPos = pos;
                        }
                        pos++;
                    }
                }
            }
            String key = success ? "verbose.resolve.multi" : "verbose.resolve.multi.1";
            DeferredAttr deferredAttr = this.deferredAttr;
            deferredAttr.getClass();
            List<Type> argtypes2 = Type.map(argtypes, deferredAttr.new RecoveryDeferredTypeMap(DeferredAttr.AttrMode.SPECULATIVE, bestSoFar, this.currentResolutionContext.step));
            JCDiagnostic main = this.diags.note(this.log.currentSource(), dpos, key, name, site.tsym, Integer.valueOf(mostSpecificPos), this.currentResolutionContext.step, methodArguments(argtypes2), methodArguments(typeargtypes));
            JCDiagnostic d = new JCDiagnostic.MultilineDiagnostic(main, subDiags.toList());
            this.log.report(d);
        }
    }

    JCDiagnostic getVerboseApplicableCandidateDiag(int pos, Symbol sym, Type inst) {
        JCDiagnostic subDiag = null;
        if (sym.type.hasTag(TypeTag.FORALL)) {
            subDiag = this.diags.fragment("partial.inst.sig", inst);
        }
        String key = subDiag == null ? "applicable.method.found" : "applicable.method.found.1";
        return this.diags.fragment(key, Integer.valueOf(pos), sym, subDiag);
    }

    JCDiagnostic getVerboseInapplicableCandidateDiag(int pos, Symbol sym, JCDiagnostic subDiag) {
        return this.diags.fragment("not.applicable.method.found", Integer.valueOf(pos), sym, subDiag);
    }

    protected static boolean isStatic(Env<AttrContext> env) {
        return env.outer != null && env.info.staticLevel > env.outer.info.staticLevel;
    }

    static boolean isInitializer(Env<AttrContext> env) {
        Symbol owner = env.info.scope.owner;
        return owner.isConstructor() || (owner.owner.kind == 2 && ((owner.kind == 4 || (owner.kind == 16 && (owner.flags() & 1048576) != 0)) && (owner.flags() & 8) == 0));
    }

    public boolean isAccessible(Env<AttrContext> env, Symbol.TypeSymbol c) {
        return isAccessible(env, c, false);
    }

    public boolean isAccessible(Env<AttrContext> env, Symbol.TypeSymbol c, boolean checkInner) {
        boolean isAccessible;
        switch ((short) (c.flags() & 7)) {
            case 0:
                isAccessible = env.toplevel.packge == c.owner || env.toplevel.packge == c.packge() || !(env.enclMethod == null || (env.enclMethod.mods.flags & 536870912) == 0);
                break;
            case 1:
            case 3:
            default:
                isAccessible = true;
                break;
            case 2:
                isAccessible = env.enclClass.sym.outermostClass() == c.owner.outermostClass();
                break;
            case 4:
                isAccessible = env.toplevel.packge == c.owner || env.toplevel.packge == c.packge() || isInnerSubClass(env.enclClass.sym, c.owner);
                break;
        }
        if (!checkInner || c.type.getEnclosingType() == Type.noType) {
            return isAccessible;
        }
        return isAccessible && isAccessible(env, c.type.getEnclosingType(), checkInner);
    }

    private boolean isInnerSubClass(Symbol.ClassSymbol c, Symbol base) {
        while (c != null && !c.isSubClass(base, this.types)) {
            c = c.owner.enclClass();
        }
        return c != null;
    }

    boolean isAccessible(Env<AttrContext> env, Type t) {
        return isAccessible(env, t, false);
    }

    boolean isAccessible(Env<AttrContext> env, Type t, boolean checkInner) {
        if (t.hasTag(TypeTag.ARRAY)) {
            return isAccessible(env, this.types.cvarUpperBound(this.types.elemtype(t)));
        }
        return isAccessible(env, t.tsym, checkInner);
    }

    public boolean isAccessible(Env<AttrContext> env, Type site, Symbol sym) {
        return isAccessible(env, site, sym, false);
    }

    public boolean isAccessible(Env<AttrContext> env, Type site, Symbol sym, boolean checkInner) {
        if (sym.name == this.names.init && sym.owner != site.tsym) {
            return false;
        }
        switch ((short) (sym.flags() & 7)) {
            case 0:
                if ((env.toplevel.packge == sym.owner.owner || env.toplevel.packge == sym.packge()) && isAccessible(env, site, checkInner) && sym.isInheritedIn(site.tsym, this.types) && notOverriddenIn(site, sym)) {
                }
                break;
            case 1:
            case 3:
            default:
                if (isAccessible(env, site, checkInner) && notOverriddenIn(site, sym)) {
                    break;
                }
                break;
            case 2:
                if ((env.enclClass.sym == sym.owner || env.enclClass.sym.outermostClass() == sym.owner.outermostClass()) && sym.isInheritedIn(site.tsym, this.types)) {
                }
                break;
            case 4:
                if ((env.toplevel.packge == sym.owner.owner || env.toplevel.packge == sym.packge() || isProtectedAccessible(sym, env.enclClass.sym, site) || (env.info.selectSuper && (sym.flags() & 8) == 0 && sym.kind != 2)) && isAccessible(env, site, checkInner) && notOverriddenIn(site, sym)) {
                }
                break;
        }
        return false;
    }

    private boolean notOverriddenIn(Type site, Symbol sym) {
        Symbol s2;
        return sym.kind != 16 || sym.isConstructor() || sym.isStatic() || (s2 = ((Symbol.MethodSymbol) sym).implementation(site.tsym, this.types, true)) == null || s2 == sym || sym.owner == s2.owner || !this.types.isSubSignature(this.types.memberType(site, s2), this.types.memberType(site, sym));
    }

    private boolean isProtectedAccessible(Symbol sym, Symbol.ClassSymbol c, Type site) {
        Type newSite = site.hasTag(TypeTag.TYPEVAR) ? site.getUpperBound() : site;
        while (c != null && (!c.isSubClass(sym.owner, this.types) || (c.flags() & 512) != 0 || ((sym.flags() & 8) == 0 && sym.kind != 2 && !newSite.tsym.isSubClass(c, this.types)))) {
            c = c.owner.enclClass();
        }
        return c != null;
    }

    void checkAccessibleType(Env<AttrContext> env, Type t) {
        this.accessibilityChecker.visit(t, env);
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Removed duplicated region for block: B:37:0x00d3  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    com.sun.tools.javac.code.Type rawInstantiate(com.sun.tools.javac.comp.Env<com.sun.tools.javac.comp.AttrContext> r20, com.sun.tools.javac.code.Type r21, com.sun.tools.javac.code.Symbol r22, com.sun.tools.javac.comp.Attr.ResultInfo r23, com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type> r24, com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type> r25, boolean r26, boolean r27, com.sun.tools.javac.util.Warner r28) throws com.sun.tools.javac.comp.Infer.InferenceException {
        /*
            Method dump skipped, instruction units count: 316
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Resolve.rawInstantiate(com.sun.tools.javac.comp.Env, com.sun.tools.javac.code.Type, com.sun.tools.javac.code.Symbol, com.sun.tools.javac.comp.Attr$ResultInfo, com.sun.tools.javac.util.List, com.sun.tools.javac.util.List, boolean, boolean, com.sun.tools.javac.util.Warner):com.sun.tools.javac.code.Type");
    }

    Type checkMethod(Env<AttrContext> env, Type site, Symbol m, Attr.ResultInfo resultInfo, List<Type> argtypes, List<Type> typeargtypes, Warner warn) {
        MethodResolutionContext prevContext = this.currentResolutionContext;
        try {
            this.currentResolutionContext = new MethodResolutionContext();
            this.currentResolutionContext.attrMode = DeferredAttr.AttrMode.CHECK;
            if (env.tree.hasTag(JCTree.Tag.REFERENCE)) {
                try {
                    this.currentResolutionContext.methodCheck = new MethodReferenceCheck(resultInfo.checkContext.inferenceContext());
                } catch (Throwable th) {
                    th = th;
                    this.currentResolutionContext = prevContext;
                    throw th;
                }
            }
            MethodResolutionContext methodResolutionContext = this.currentResolutionContext;
            MethodResolutionPhase step = env.info.pendingResolutionPhase;
            methodResolutionContext.step = step;
            Type typeRawInstantiate = rawInstantiate(env, site, m, resultInfo, argtypes, typeargtypes, step.isBoxingRequired(), step.isVarargsRequired(), warn);
            this.currentResolutionContext = prevContext;
            return typeRawInstantiate;
        } catch (Throwable th2) {
            th = th2;
        }
    }

    Type instantiate(Env<AttrContext> env, Type site, Symbol m, Attr.ResultInfo resultInfo, List<Type> argtypes, List<Type> typeargtypes, boolean allowBoxing, boolean useVarargs, Warner warn) {
        try {
            return rawInstantiate(env, site, m, resultInfo, argtypes, typeargtypes, allowBoxing, useVarargs, warn);
        } catch (InapplicableMethodException e) {
            return null;
        }
    }

    enum MethodCheckDiag {
        ARITY_MISMATCH("arg.length.mismatch", "infer.arg.length.mismatch"),
        ARG_MISMATCH("no.conforming.assignment.exists", "infer.no.conforming.assignment.exists"),
        VARARG_MISMATCH("varargs.argument.mismatch", "infer.varargs.argument.mismatch"),
        INACCESSIBLE_VARARGS("inaccessible.varargs.type", "inaccessible.varargs.type");

        final String basicKey;
        final String inferKey;

        MethodCheckDiag(String basicKey, String inferKey) {
            this.basicKey = basicKey;
            this.inferKey = inferKey;
        }

        String regex() {
            return String.format("([a-z]*\\.)*(%s|%s)", this.basicKey, this.inferKey);
        }
    }

    abstract class AbstractMethodCheck implements MethodCheck {
        abstract void checkArg(JCDiagnostic.DiagnosticPosition diagnosticPosition, boolean z, Type type, Type type2, DeferredAttr.DeferredAttrContext deferredAttrContext, Warner warner);

        AbstractMethodCheck() {
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.comp.Resolve.MethodCheck
        public void argumentsAcceptable(Env<AttrContext> env, DeferredAttr.DeferredAttrContext deferredAttrContext, List<Type> argtypes, List<Type> formals, Warner warn) {
            boolean useVarargs = deferredAttrContext.phase.isVarargsRequired();
            JCTree callTree = treeForDiagnostics(env);
            List<JCTree.JCExpression> trees = TreeInfo.args(callTree);
            Infer.InferenceContext inferenceContext = deferredAttrContext.inferenceContext;
            Type varargsFormal = useVarargs ? formals.last() : null;
            if (varargsFormal == null && argtypes.size() != formals.size()) {
                reportMC(callTree, MethodCheckDiag.ARITY_MISMATCH, inferenceContext, new Object[0]);
            }
            List list = argtypes;
            List list2 = formals;
            List list3 = trees;
            while (list.nonEmpty() && list2.head != varargsFormal) {
                JCDiagnostic.DiagnosticPosition pos = list3 != null ? (JCTree.JCExpression) list3.head : null;
                List list4 = list3;
                checkArg(pos, false, (Type) list.head, (Type) list2.head, deferredAttrContext, warn);
                list = list.tail;
                list2 = list2.tail;
                list3 = list4 != null ? list4.tail : list4;
            }
            List list5 = list3;
            if (list2.head != varargsFormal) {
                reportMC(callTree, MethodCheckDiag.ARITY_MISMATCH, inferenceContext, new Object[0]);
            }
            if (useVarargs) {
                Type elt = Resolve.this.types.elemtype(varargsFormal);
                while (list.nonEmpty()) {
                    JCDiagnostic.DiagnosticPosition pos2 = list5 != null ? (JCTree.JCExpression) list5.head : null;
                    checkArg(pos2, true, (Type) list.head, elt, deferredAttrContext, warn);
                    list = list.tail;
                    list5 = list5 != null ? list5.tail : list5;
                }
            }
        }

        private JCTree treeForDiagnostics(Env<AttrContext> env) {
            return env.info.preferredTreeForDiagnostics != null ? env.info.preferredTreeForDiagnostics : env.tree;
        }

        protected void reportMC(JCDiagnostic.DiagnosticPosition pos, MethodCheckDiag diag, Infer.InferenceContext inferenceContext, Object... args) {
            boolean inferDiag = inferenceContext != Resolve.this.infer.emptyContext;
            Resolve resolve = Resolve.this;
            InapplicableMethodException ex = inferDiag ? resolve.infer.inferenceException : resolve.inapplicableMethodException;
            if (inferDiag && !diag.inferKey.equals(diag.basicKey)) {
                Object[] args2 = new Object[args.length + 1];
                System.arraycopy(args, 0, args2, 1, args.length);
                args2[0] = inferenceContext.inferenceVars();
                args = args2;
            }
            String key = inferDiag ? diag.inferKey : diag.basicKey;
            throw ex.setMessage(Resolve.this.diags.create(JCDiagnostic.DiagnosticType.FRAGMENT, Resolve.this.log.currentSource(), pos, key, args));
        }

        @Override // com.sun.tools.javac.comp.Resolve.MethodCheck
        public MethodCheck mostSpecificCheck(List<Type> actuals, boolean strict) {
            return Resolve.this.nilMethodCheck;
        }
    }

    List<Type> dummyArgs(int length) {
        ListBuffer<Type> buf = new ListBuffer<>();
        for (int i = 0; i < length; i++) {
            buf.append(Type.noType);
        }
        return buf.toList();
    }

    class MethodReferenceCheck extends AbstractMethodCheck {
        Infer.InferenceContext pendingInferenceContext;

        MethodReferenceCheck(Infer.InferenceContext pendingInferenceContext) {
            super();
            this.pendingInferenceContext = pendingInferenceContext;
        }

        @Override // com.sun.tools.javac.comp.Resolve.AbstractMethodCheck
        void checkArg(JCDiagnostic.DiagnosticPosition pos, boolean varargs, Type actual, Type formal, DeferredAttr.DeferredAttrContext deferredAttrContext, Warner warn) {
            Attr.ResultInfo mresult = methodCheckResult(varargs, formal, deferredAttrContext, warn);
            mresult.check(pos, actual);
        }

        private Attr.ResultInfo methodCheckResult(final boolean varargsCheck, Type to, DeferredAttr.DeferredAttrContext deferredAttrContext, Warner rsWarner) {
            Check.CheckContext checkContext = new MethodCheckContext(!deferredAttrContext.phase.isBoxingRequired(), deferredAttrContext, rsWarner) { // from class: com.sun.tools.javac.comp.Resolve.MethodReferenceCheck.1
                MethodCheckDiag methodDiag;

                {
                    Resolve resolve = Resolve.this;
                    this.methodDiag = varargsCheck ? MethodCheckDiag.VARARG_MISMATCH : MethodCheckDiag.ARG_MISMATCH;
                }

                @Override // com.sun.tools.javac.comp.Resolve.MethodCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                public boolean compatible(Type found, Type req, Warner warn) {
                    Type found2 = MethodReferenceCheck.this.pendingInferenceContext.asUndetVar(found);
                    if (found2.hasTag(TypeTag.UNDETVAR) && req.isPrimitive()) {
                        req = Resolve.this.types.boxedClass(req).type;
                    }
                    return super.compatible(found2, req, warn);
                }

                @Override // com.sun.tools.javac.comp.Resolve.MethodCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                public void report(JCDiagnostic.DiagnosticPosition pos, JCDiagnostic details) {
                    MethodReferenceCheck.this.reportMC(pos, this.methodDiag, this.deferredAttrContext.inferenceContext, details);
                }
            };
            return Resolve.this.new MethodResultInfo(to, checkContext);
        }

        @Override // com.sun.tools.javac.comp.Resolve.AbstractMethodCheck, com.sun.tools.javac.comp.Resolve.MethodCheck
        public MethodCheck mostSpecificCheck(List<Type> actuals, boolean strict) {
            return Resolve.this.new MostSpecificCheck(strict, actuals);
        }
    }

    abstract class MethodCheckContext implements Check.CheckContext {
        DeferredAttr.DeferredAttrContext deferredAttrContext;
        Warner rsWarner;
        boolean strict;

        public MethodCheckContext(boolean strict, DeferredAttr.DeferredAttrContext deferredAttrContext, Warner rsWarner) {
            this.strict = strict;
            this.deferredAttrContext = deferredAttrContext;
            this.rsWarner = rsWarner;
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public boolean compatible(Type found, Type req, Warner warn) {
            Infer.InferenceContext inferenceContext = this.deferredAttrContext.inferenceContext;
            if (this.strict) {
                return Resolve.this.types.isSubtypeUnchecked(inferenceContext.asUndetVar(found), inferenceContext.asUndetVar(req), warn);
            }
            return Resolve.this.types.isConvertible(inferenceContext.asUndetVar(found), inferenceContext.asUndetVar(req), warn);
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public void report(JCDiagnostic.DiagnosticPosition pos, JCDiagnostic details) {
            throw Resolve.this.inapplicableMethodException.setMessage(details);
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public Warner checkWarner(JCDiagnostic.DiagnosticPosition pos, Type found, Type req) {
            return this.rsWarner;
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public Infer.InferenceContext inferenceContext() {
            return this.deferredAttrContext.inferenceContext;
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public DeferredAttr.DeferredAttrContext deferredAttrContext() {
            return this.deferredAttrContext;
        }

        public String toString() {
            return "MethodReferenceCheck";
        }
    }

    class MethodResultInfo extends Attr.ResultInfo {
        /* JADX WARN: Illegal instructions before constructor call */
        public MethodResultInfo(Type pt, Check.CheckContext checkContext) {
            Attr attr = Resolve.this.attr;
            attr.getClass();
            super(12, pt, checkContext);
        }

        @Override // com.sun.tools.javac.comp.Attr.ResultInfo
        protected Type check(JCDiagnostic.DiagnosticPosition pos, Type found) {
            Type capturedType;
            if (found.hasTag(TypeTag.DEFERRED)) {
                DeferredAttr.DeferredType dt = (DeferredAttr.DeferredType) found;
                return dt.check(this);
            }
            Type uResult = U(found);
            if (pos == null || pos.getTree() == null) {
                capturedType = Resolve.this.types.capture(uResult);
            } else {
                capturedType = this.checkContext.inferenceContext().cachedCapture(pos.getTree(), uResult, true);
            }
            return super.check(pos, Resolve.this.chk.checkNonVoid(pos, capturedType));
        }

        private Type U(Type found) {
            return found == this.pt ? found : Resolve.this.types.cvarUpperBound(found);
        }

        /* JADX INFO: Access modifiers changed from: protected */
        @Override // com.sun.tools.javac.comp.Attr.ResultInfo
        public MethodResultInfo dup(Type newPt) {
            return Resolve.this.new MethodResultInfo(newPt, this.checkContext);
        }

        @Override // com.sun.tools.javac.comp.Attr.ResultInfo
        protected Attr.ResultInfo dup(Check.CheckContext newContext) {
            return Resolve.this.new MethodResultInfo(this.pt, newContext);
        }
    }

    class MostSpecificCheck implements MethodCheck {
        List<Type> actuals;
        boolean strict;

        MostSpecificCheck(boolean strict, List<Type> actuals) {
            this.strict = strict;
            this.actuals = actuals;
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.comp.Resolve.MethodCheck
        public void argumentsAcceptable(Env<AttrContext> env, DeferredAttr.DeferredAttrContext deferredAttrContext, List<Type> list, List<Type> list2, Warner warner) {
            List listAdjustArgs = Resolve.this.adjustArgs(list2, deferredAttrContext.msym, list.length(), deferredAttrContext.phase.isVarargsRequired());
            List list3 = list;
            while (listAdjustArgs.nonEmpty()) {
                methodCheckResult((Type) listAdjustArgs.head, deferredAttrContext, warner, this.actuals.head).check(null, (Type) list3.head);
                List list4 = list3.tail;
                listAdjustArgs = listAdjustArgs.tail;
                this.actuals = this.actuals.isEmpty() ? this.actuals : this.actuals.tail;
                list3 = list4;
            }
        }

        Attr.ResultInfo methodCheckResult(Type to, DeferredAttr.DeferredAttrContext deferredAttrContext, Warner rsWarner, Type actual) {
            Attr attr = Resolve.this.attr;
            attr.getClass();
            return attr.new ResultInfo(12, to, new MostSpecificCheckContext(this.strict, deferredAttrContext, rsWarner, actual));
        }

        class MostSpecificCheckContext extends MethodCheckContext {
            Type actual;

            public MostSpecificCheckContext(boolean strict, DeferredAttr.DeferredAttrContext deferredAttrContext, Warner rsWarner, Type actual) {
                super(strict, deferredAttrContext, rsWarner);
                this.actual = actual;
            }

            @Override // com.sun.tools.javac.comp.Resolve.MethodCheckContext, com.sun.tools.javac.comp.Check.CheckContext
            public boolean compatible(Type found, Type req, Warner warn) {
                if (Resolve.this.allowFunctionalInterfaceMostSpecific && unrelatedFunctionalInterfaces(found, req) && this.actual != null && this.actual.getTag() == TypeTag.DEFERRED) {
                    DeferredAttr.DeferredType dt = (DeferredAttr.DeferredType) this.actual;
                    DeferredAttr.DeferredType.SpeculativeCache.Entry e = dt.speculativeCache.get(this.deferredAttrContext.msym, this.deferredAttrContext.phase);
                    if (e != null && e.speculativeTree != Resolve.this.deferredAttr.stuckTree) {
                        return functionalInterfaceMostSpecific(found, req, e.speculativeTree, warn);
                    }
                }
                return super.compatible(found, req, warn);
            }

            /* JADX INFO: Access modifiers changed from: private */
            public boolean unrelatedFunctionalInterfaces(Type t, Type s) {
                return Resolve.this.types.isFunctionalInterface(t.tsym) && Resolve.this.types.isFunctionalInterface(s.tsym) && Resolve.this.types.asSuper(t, s.tsym) == null && Resolve.this.types.asSuper(s, t.tsym) == null;
            }

            /* JADX INFO: Access modifiers changed from: private */
            public boolean functionalInterfaceMostSpecific(Type t, Type s, JCTree tree, Warner warn) {
                FunctionalInterfaceMostSpecificChecker msc = new FunctionalInterfaceMostSpecificChecker(t, s, warn);
                msc.scan(tree);
                return msc.result;
            }

            class FunctionalInterfaceMostSpecificChecker extends DeferredAttr.PolyScanner {
                boolean result = true;
                final Type s;
                final Type t;
                final Warner warn;

                FunctionalInterfaceMostSpecificChecker(Type t, Type s, Warner warn) {
                    this.t = t;
                    this.s = s;
                    this.warn = warn;
                }

                @Override // com.sun.tools.javac.comp.DeferredAttr.FilterScanner
                void skip(JCTree tree) {
                    this.result = false;
                }

                @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
                public void visitConditional(JCTree.JCConditional tree) {
                    scan(tree.truepart);
                    scan(tree.falsepart);
                }

                @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
                public void visitReference(JCTree.JCMemberReference tree) {
                    Type desc_t = Resolve.this.types.findDescriptorType(this.t);
                    Type desc_s = Resolve.this.types.findDescriptorType(this.s);
                    boolean z = false;
                    if (!Resolve.this.types.isSameTypes(desc_t.mo176getParameterTypes(), MostSpecificCheckContext.this.inferenceContext().asUndetVars(desc_s.mo176getParameterTypes()))) {
                        this.result = false;
                        return;
                    }
                    Type ret_t = desc_t.mo178getReturnType();
                    Type ret_s = desc_s.mo178getReturnType();
                    if (ret_s.hasTag(TypeTag.VOID)) {
                        this.result &= true;
                        return;
                    }
                    if (ret_t.hasTag(TypeTag.VOID)) {
                        this.result = false;
                        return;
                    }
                    if (ret_t.isPrimitive() == ret_s.isPrimitive()) {
                        this.result &= MostSpecificCheckContext.super.compatible(ret_t, ret_s, this.warn);
                        return;
                    }
                    boolean retValIsPrimitive = tree.refPolyKind == JCTree.JCPolyExpression.PolyKind.STANDALONE && tree.sym.type.mo178getReturnType().isPrimitive();
                    boolean z2 = this.result;
                    if (retValIsPrimitive == ret_t.isPrimitive() && retValIsPrimitive != ret_s.isPrimitive()) {
                        z = true;
                    }
                    this.result = z & z2;
                }

                @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
                public void visitLambda(JCTree.JCLambda tree) {
                    Type desc_t = Resolve.this.types.findDescriptorType(this.t);
                    Type desc_s = Resolve.this.types.findDescriptorType(this.s);
                    if (!Resolve.this.types.isSameTypes(desc_t.mo176getParameterTypes(), MostSpecificCheckContext.this.inferenceContext().asUndetVars(desc_s.mo176getParameterTypes()))) {
                        this.result = false;
                        return;
                    }
                    Type ret_t = desc_t.mo178getReturnType();
                    Type ret_s = desc_s.mo178getReturnType();
                    if (ret_s.hasTag(TypeTag.VOID)) {
                        this.result &= true;
                        return;
                    }
                    if (!ret_t.hasTag(TypeTag.VOID)) {
                        if (MostSpecificCheckContext.this.unrelatedFunctionalInterfaces(ret_t, ret_s)) {
                            Iterator<JCTree.JCExpression> it = lambdaResults(tree).iterator();
                            while (it.hasNext()) {
                                this.result &= MostSpecificCheckContext.this.functionalInterfaceMostSpecific(ret_t, ret_s, it.next(), this.warn);
                            }
                            return;
                        } else {
                            if (ret_t.isPrimitive() == ret_s.isPrimitive()) {
                                this.result &= MostSpecificCheckContext.super.compatible(ret_t, ret_s, this.warn);
                                return;
                            }
                            for (JCTree.JCExpression expr : lambdaResults(tree)) {
                                boolean retValIsPrimitive = expr.isStandalone() && expr.type.isPrimitive();
                                this.result &= retValIsPrimitive == ret_t.isPrimitive() && retValIsPrimitive != ret_s.isPrimitive();
                            }
                            return;
                        }
                    }
                    this.result = false;
                }

                private List<JCTree.JCExpression> lambdaResults(JCTree.JCLambda lambda) {
                    if (lambda.getBodyKind() == LambdaExpressionTree.BodyKind.EXPRESSION) {
                        return List.of((JCTree.JCExpression) lambda.body);
                    }
                    final ListBuffer<JCTree.JCExpression> buffer = new ListBuffer<>();
                    DeferredAttr.LambdaReturnScanner lambdaScanner = new DeferredAttr.LambdaReturnScanner() { // from class: com.sun.tools.javac.comp.Resolve.MostSpecificCheck.MostSpecificCheckContext.FunctionalInterfaceMostSpecificChecker.1
                        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
                        public void visitReturn(JCTree.JCReturn tree) {
                            if (tree.expr != null) {
                                buffer.append(tree.expr);
                            }
                        }
                    };
                    lambdaScanner.scan(lambda.body);
                    return buffer.toList();
                }
            }
        }

        @Override // com.sun.tools.javac.comp.Resolve.MethodCheck
        public MethodCheck mostSpecificCheck(List<Type> actuals, boolean strict) {
            Assert.error("Cannot get here!");
            return null;
        }
    }

    public static class InapplicableMethodException extends RuntimeException {
        private static final long serialVersionUID = 0;
        JCDiagnostic diagnostic = null;
        JCDiagnostic.Factory diags;

        InapplicableMethodException(JCDiagnostic.Factory diags) {
            this.diags = diags;
        }

        InapplicableMethodException setMessage() {
            return setMessage((JCDiagnostic) null);
        }

        InapplicableMethodException setMessage(String key) {
            return setMessage(key != null ? this.diags.fragment(key, new Object[0]) : null);
        }

        InapplicableMethodException setMessage(String key, Object... args) {
            return setMessage(key != null ? this.diags.fragment(key, args) : null);
        }

        InapplicableMethodException setMessage(JCDiagnostic diag) {
            this.diagnostic = diag;
            return this;
        }

        public JCDiagnostic getDiagnostic() {
            return this.diagnostic;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    Symbol findField(Env<AttrContext> env, Type site, Name name, Symbol.TypeSymbol c) {
        while (c.type.hasTag(TypeTag.TYPEVAR)) {
            c = c.type.getUpperBound().tsym;
        }
        Symbol bestSoFar = this.varNotFound;
        for (Scope.Entry e = c.members().lookup(name); e.scope != null; e = e.next()) {
            if (e.sym.kind == 4 && (e.sym.flags_field & 4096) == 0) {
                return isAccessible(env, site, e.sym) ? e.sym : new AccessError(env, site, e.sym);
            }
        }
        Type st = this.types.supertype(c.type);
        if (st != null && (st.hasTag(TypeTag.CLASS) || st.hasTag(TypeTag.TYPEVAR))) {
            Symbol sym = findField(env, site, name, st.tsym);
            if (sym.kind < bestSoFar.kind) {
                bestSoFar = sym;
            }
        }
        for (List listInterfaces = this.types.interfaces(c.type); bestSoFar.kind != 129 && listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
            Symbol sym2 = findField(env, site, name, ((Type) listInterfaces.head).tsym);
            if (bestSoFar.exists() && sym2.exists() && sym2.owner != bestSoFar.owner) {
                bestSoFar = new AmbiguityError(bestSoFar, sym2);
            } else if (sym2.kind < bestSoFar.kind) {
                bestSoFar = sym2;
            }
        }
        return bestSoFar;
    }

    public Symbol.VarSymbol resolveInternalField(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type site, Name name) {
        Symbol sym = findField(env, site, name, site.tsym);
        if (sym.kind == 4) {
            return (Symbol.VarSymbol) sym;
        }
        throw new FatalError(this.diags.fragment("fatal.err.cant.locate.field", name));
    }

    /* JADX WARN: Multi-variable type inference failed */
    Symbol findVar(Env<AttrContext> env, Name name) {
        Symbol bestSoFar = this.varNotFound;
        boolean staticOnly = false;
        for (Env env2 = env; env2.outer != null; env2 = env2.outer) {
            if (isStatic(env2)) {
                staticOnly = true;
            }
            Scope.Entry e = ((AttrContext) env2.info).scope.lookup(name);
            while (e.scope != null && (e.sym.kind != 4 || (e.sym.flags_field & 4096) != 0)) {
                e = e.next();
            }
            Symbol sym = e.scope != null ? e.sym : findField(env2, env2.enclClass.sym.type, name, env2.enclClass.sym);
            if (sym.exists()) {
                if (staticOnly && sym.kind == 4 && sym.owner.kind == 2 && (sym.flags() & 8) == 0) {
                    return new StaticError(sym);
                }
                return sym;
            }
            if (sym.kind < bestSoFar.kind) {
                bestSoFar = sym;
            }
            if ((env2.enclClass.sym.flags() & 8) != 0) {
                staticOnly = true;
            }
        }
        Symbol sym2 = findField(env, this.syms.predefClass.type, name, this.syms.predefClass);
        if (sym2.exists()) {
            return sym2;
        }
        if (bestSoFar.exists()) {
            return bestSoFar;
        }
        Symbol origin = null;
        Scope[] scopeArr = {env.toplevel.namedImportScope, env.toplevel.starImportScope};
        for (int i = 0; i < 2; i++) {
            Scope sc = scopeArr[i];
            for (Scope.Entry e2 = sc.lookup(name); e2.scope != null; e2 = e2.next()) {
                Symbol sym3 = e2.sym;
                if (sym3.kind == 4) {
                    if (bestSoFar.kind < 129 && sym3.owner != bestSoFar.owner) {
                        return new AmbiguityError(bestSoFar, sym3);
                    }
                    if (bestSoFar.kind >= 4) {
                        origin = e2.getOrigin().owner;
                        bestSoFar = isAccessible(env, origin.type, sym3) ? sym3 : new AccessError(env, origin.type, sym3);
                    }
                }
            }
            if (bestSoFar.exists()) {
                break;
            }
        }
        if (bestSoFar.kind == 4 && bestSoFar.owner.type != origin.type) {
            return bestSoFar.clone(origin);
        }
        return bestSoFar;
    }

    Symbol selectBest(Env<AttrContext> env, Type site, List<Type> argtypes, List<Type> typeargtypes, Symbol sym, Symbol bestSoFar, boolean allowBoxing, boolean useVarargs, boolean operator) {
        if (sym.kind == 63 || !sym.isInheritedIn(site.tsym, this.types)) {
            return bestSoFar;
        }
        if (useVarargs && (sym.flags() & Flags.VARARGS) == 0) {
            return bestSoFar.kind >= 128 ? new BadVarargsMethod((ResolveError) bestSoFar.baseSymbol()) : bestSoFar;
        }
        Assert.check(sym.kind < 129);
        try {
            Type mt = rawInstantiate(env, site, sym, null, argtypes, typeargtypes, allowBoxing, useVarargs, this.types.noWarnings);
            if (!operator || this.verboseResolutionMode.contains(VerboseResolutionMode.PREDEF)) {
                this.currentResolutionContext.addApplicableCandidate(sym, mt);
            }
            if (!isAccessible(env, site, sym)) {
                return bestSoFar.kind == 136 ? new AccessError(env, site, sym) : bestSoFar;
            }
            if (bestSoFar.kind > 129) {
                return sym;
            }
            return mostSpecific(argtypes, sym, bestSoFar, env, site, allowBoxing && operator, useVarargs);
        } catch (InapplicableMethodException ex) {
            if (!operator) {
                this.currentResolutionContext.addInapplicableCandidate(sym, ex.getDiagnostic());
            }
            switch (bestSoFar.kind) {
                case 135:
                    if (!operator) {
                        break;
                    }
                    break;
            }
        }
    }

    /* JADX WARN: Incorrect condition in loop: B:8:0x002b */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    com.sun.tools.javac.code.Symbol mostSpecific(com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type> r21, com.sun.tools.javac.code.Symbol r22, com.sun.tools.javac.code.Symbol r23, com.sun.tools.javac.comp.Env<com.sun.tools.javac.comp.AttrContext> r24, com.sun.tools.javac.code.Type r25, boolean r26, boolean r27) {
        /*
            Method dump skipped, instruction units count: 346
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Resolve.mostSpecific(com.sun.tools.javac.util.List, com.sun.tools.javac.code.Symbol, com.sun.tools.javac.code.Symbol, com.sun.tools.javac.comp.Env, com.sun.tools.javac.code.Type, boolean, boolean):com.sun.tools.javac.code.Symbol");
    }

    /* JADX WARN: Removed duplicated region for block: B:12:0x008b  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private boolean signatureMoreSpecific(com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type> r19, com.sun.tools.javac.comp.Env<com.sun.tools.javac.comp.AttrContext> r20, com.sun.tools.javac.code.Type r21, com.sun.tools.javac.code.Symbol r22, com.sun.tools.javac.code.Symbol r23, boolean r24, boolean r25) {
        /*
            r18 = this;
            r11 = r18
            r12 = r22
            com.sun.tools.javac.util.Warner r0 = r11.noteWarner
            r0.clear()
            com.sun.tools.javac.code.Type r0 = r12.type
            com.sun.tools.javac.util.List r0 = r0.mo176getParameterTypes()
            int r0 = r0.length()
            int r1 = r19.length()
            int r0 = java.lang.Math.max(r0, r1)
            r13 = r23
            com.sun.tools.javac.code.Type r1 = r13.type
            com.sun.tools.javac.util.List r1 = r1.mo176getParameterTypes()
            int r1 = r1.length()
            int r14 = java.lang.Math.max(r0, r1)
            com.sun.tools.javac.comp.Resolve$MethodResolutionContext r15 = r11.currentResolutionContext
            com.sun.tools.javac.comp.Resolve$MethodResolutionContext r0 = new com.sun.tools.javac.comp.Resolve$MethodResolutionContext     // Catch: java.lang.Throwable -> L90
            r0.<init>()     // Catch: java.lang.Throwable -> L90
            r11.currentResolutionContext = r0     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.comp.Resolve$MethodResolutionContext r0 = r11.currentResolutionContext     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.comp.Resolve$MethodResolutionPhase r1 = r15.step     // Catch: java.lang.Throwable -> L90
            r0.step = r1     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.comp.Resolve$MethodResolutionContext r0 = r11.currentResolutionContext     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.comp.Resolve$MethodCheck r1 = r15.methodCheck     // Catch: java.lang.Throwable -> L90
            r16 = 1
            r17 = 0
            if (r24 != 0) goto L47
            r2 = r16
            goto L49
        L47:
            r2 = r17
        L49:
            r10 = r19
            com.sun.tools.javac.comp.Resolve$MethodCheck r1 = r1.mostSpecificCheck(r10, r2)     // Catch: java.lang.Throwable -> L90
            r0.methodCheck = r1     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.code.Types r0 = r11.types     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.code.Types r1 = r11.types     // Catch: java.lang.Throwable -> L90
            r9 = r21
            com.sun.tools.javac.code.Type r1 = r1.memberType(r9, r12)     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.util.List r1 = r1.mo176getParameterTypes()     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.util.List r0 = r0.cvarLowerBounds(r1)     // Catch: java.lang.Throwable -> L90
            r8 = r25
            com.sun.tools.javac.util.List r6 = r11.adjustArgs(r0, r12, r14, r8)     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.util.Warner r0 = r11.noteWarner     // Catch: java.lang.Throwable -> L90
            r5 = 0
            r7 = 0
            r1 = r18
            r2 = r20
            r3 = r21
            r4 = r23
            r8 = r24
            r9 = r25
            r10 = r0
            com.sun.tools.javac.code.Type r0 = r1.instantiate(r2, r3, r4, r5, r6, r7, r8, r9, r10)     // Catch: java.lang.Throwable -> L90
            if (r0 == 0) goto L8b
            com.sun.tools.javac.util.Warner r1 = r11.noteWarner     // Catch: java.lang.Throwable -> L90
            com.sun.tools.javac.code.Lint$LintCategory r2 = com.sun.tools.javac.code.Lint.LintCategory.UNCHECKED     // Catch: java.lang.Throwable -> L90
            boolean r1 = r1.hasLint(r2)     // Catch: java.lang.Throwable -> L90
            if (r1 != 0) goto L8b
            goto L8d
        L8b:
            r16 = r17
        L8d:
            r11.currentResolutionContext = r15
            return r16
        L90:
            r0 = move-exception
            r11.currentResolutionContext = r15
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Resolve.signatureMoreSpecific(com.sun.tools.javac.util.List, com.sun.tools.javac.comp.Env, com.sun.tools.javac.code.Type, com.sun.tools.javac.code.Symbol, com.sun.tools.javac.code.Symbol, boolean, boolean):boolean");
    }

    List<Type> adjustArgs(List<Type> args, Symbol msym, int length, boolean allowVarargs) {
        if ((msym.flags() & Flags.VARARGS) != 0 && allowVarargs) {
            Type varargsElem = this.types.elemtype(args.last());
            if (varargsElem == null) {
                Assert.error("Bad varargs = " + args.last() + " " + msym);
            }
            List<Type> newArgs = args.reverse().tail.prepend(varargsElem).reverse();
            while (newArgs.length() < length) {
                newArgs = newArgs.append(newArgs.last());
            }
            return newArgs;
        }
        return args;
    }

    Type mostSpecificReturnType(Type mt1, Type mt2) {
        Type rt1 = mt1.mo178getReturnType();
        Type rt2 = mt2.mo178getReturnType();
        if (mt1.hasTag(TypeTag.FORALL) && mt2.hasTag(TypeTag.FORALL)) {
            rt1 = this.types.subst(rt1, mt1.getTypeArguments(), mt2.getTypeArguments());
        }
        if (this.types.isSubtype(rt1, rt2)) {
            return mt1;
        }
        if (this.types.isSubtype(rt2, rt1)) {
            return mt2;
        }
        if (this.types.returnTypeSubstitutable(mt1, mt2)) {
            return mt1;
        }
        if (this.types.returnTypeSubstitutable(mt2, mt1)) {
            return mt2;
        }
        return null;
    }

    Symbol ambiguityError(Symbol m1, Symbol m2) {
        if (((m1.flags() | m2.flags()) & Flags.CLASH) != 0) {
            return (m1.flags() & Flags.CLASH) == 0 ? m1 : m2;
        }
        return new AmbiguityError(m1, m2);
    }

    Symbol findMethodInScope(Env<AttrContext> env, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes, Scope sc, Symbol bestSoFar, boolean allowBoxing, boolean useVarargs, boolean operator, boolean abstractok) {
        Symbol bestSoFar2 = bestSoFar;
        for (Symbol s : sc.getElementsByName(name, new LookupFilter(abstractok))) {
            bestSoFar2 = selectBest(env, site, argtypes, typeargtypes, s, bestSoFar2, allowBoxing, useVarargs, operator);
        }
        return bestSoFar2;
    }

    class LookupFilter implements Filter<Symbol> {
        boolean abstractOk;

        LookupFilter(boolean abstractOk) {
            this.abstractOk = abstractOk;
        }

        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol s) {
            long flags = s.flags();
            return s.kind == 16 && (4096 & flags) == 0 && (this.abstractOk || (Flags.DEFAULT & flags) != 0 || (1024 & flags) == 0);
        }
    }

    Symbol findMethod(Env<AttrContext> env, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes, boolean allowBoxing, boolean useVarargs, boolean operator) {
        Symbol bestSoFar = this.methodNotFound;
        return findMethod(env, site, name, argtypes, typeargtypes, site.tsym.type, bestSoFar, allowBoxing, useVarargs, operator);
    }

    private Symbol findMethod(Env<AttrContext> env, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes, Type intype, Symbol bestSoFar, boolean allowBoxing, boolean useVarargs, boolean operator) {
        char c;
        List<Type>[] itypes = {List.nil(), List.nil()};
        InterfaceLookupPhase iphase = InterfaceLookupPhase.ABSTRACT_OK;
        Symbol bestSoFar2 = bestSoFar;
        InterfaceLookupPhase iphase2 = iphase;
        for (Symbol.TypeSymbol s : superclasses(intype)) {
            InterfaceLookupPhase iphase3 = iphase2;
            bestSoFar2 = findMethodInScope(env, site, name, argtypes, typeargtypes, s.members(), bestSoFar2, allowBoxing, useVarargs, operator, true);
            if (name == this.names.init) {
                return bestSoFar2;
            }
            InterfaceLookupPhase iphase4 = iphase3 == null ? null : iphase3.update(s, this);
            if (iphase4 != null) {
                Iterator<Type> it = this.types.interfaces(s.type).iterator();
                while (it.hasNext()) {
                    itypes[iphase4.ordinal()] = this.types.union(this.types.closure(it.next()), itypes[iphase4.ordinal()]);
                }
            }
            iphase2 = iphase4;
        }
        char c2 = '?';
        Symbol concrete = (bestSoFar2.kind >= 63 || (bestSoFar2.flags() & 1024) != 0) ? this.methodNotFound : bestSoFar2;
        InterfaceLookupPhase[] interfaceLookupPhaseArrValues = InterfaceLookupPhase.values();
        int length = interfaceLookupPhaseArrValues.length;
        int i = 0;
        while (i < length) {
            InterfaceLookupPhase iphase22 = interfaceLookupPhaseArrValues[i];
            Symbol bestSoFar3 = bestSoFar2;
            for (Type itype : itypes[iphase22.ordinal()]) {
                if (itype.isInterface() && (iphase22 != InterfaceLookupPhase.DEFAULT_OK || (itype.tsym.flags() & Flags.DEFAULT) != 0)) {
                    InterfaceLookupPhase iphase23 = iphase22;
                    int i2 = i;
                    int i3 = length;
                    InterfaceLookupPhase[] interfaceLookupPhaseArr = interfaceLookupPhaseArrValues;
                    Symbol concrete2 = concrete;
                    Symbol bestSoFar4 = findMethodInScope(env, site, name, argtypes, typeargtypes, itype.tsym.members(), bestSoFar3, allowBoxing, useVarargs, operator, true);
                    if (concrete2 == bestSoFar4) {
                        c = '?';
                    } else {
                        c = '?';
                        if (concrete2.kind < 63 && bestSoFar4.kind < 63 && this.types.isSubSignature(concrete2.type, bestSoFar4.type)) {
                            bestSoFar4 = concrete2;
                        }
                    }
                    bestSoFar3 = bestSoFar4;
                    concrete = concrete2;
                    length = i3;
                    iphase22 = iphase23;
                    i = i2;
                    interfaceLookupPhaseArrValues = interfaceLookupPhaseArr;
                    c2 = c;
                }
            }
            i++;
            bestSoFar2 = bestSoFar3;
            c2 = c2;
        }
        return bestSoFar2;
    }

    Iterable<Symbol.TypeSymbol> superclasses(final Type intype) {
        return new Iterable<Symbol.TypeSymbol>() { // from class: com.sun.tools.javac.comp.Resolve.5
            @Override // java.lang.Iterable
            public Iterator<Symbol.TypeSymbol> iterator() {
                return new Iterator<Symbol.TypeSymbol>() { // from class: com.sun.tools.javac.comp.Resolve.5.1
                    Symbol.TypeSymbol currentSym;
                    List<Symbol.TypeSymbol> seen = List.nil();
                    Symbol.TypeSymbol prevSym = null;

                    {
                        this.currentSym = symbolFor(intype);
                    }

                    @Override // java.util.Iterator
                    public boolean hasNext() {
                        if (this.currentSym == Resolve.this.syms.noSymbol) {
                            this.currentSym = symbolFor(Resolve.this.types.supertype(this.prevSym.type));
                        }
                        return this.currentSym != null;
                    }

                    /* JADX WARN: Can't rename method to resolve collision */
                    @Override // java.util.Iterator
                    public Symbol.TypeSymbol next() {
                        this.prevSym = this.currentSym;
                        this.currentSym = Resolve.this.syms.noSymbol;
                        Assert.check((this.prevSym == null && this.prevSym == Resolve.this.syms.noSymbol) ? false : true);
                        return this.prevSym;
                    }

                    @Override // java.util.Iterator
                    public void remove() {
                        throw new UnsupportedOperationException();
                    }

                    Symbol.TypeSymbol symbolFor(Type t) {
                        if (!t.hasTag(TypeTag.CLASS) && !t.hasTag(TypeTag.TYPEVAR)) {
                            return null;
                        }
                        while (t.hasTag(TypeTag.TYPEVAR)) {
                            t = t.getUpperBound();
                        }
                        if (this.seen.contains(t.tsym)) {
                            return null;
                        }
                        this.seen = this.seen.prepend(t.tsym);
                        return t.tsym;
                    }
                };
            }
        };
    }

    /* JADX WARN: Multi-variable type inference failed */
    Symbol findFun(Env<AttrContext> env, Name name, List<Type> argtypes, List<Type> typeargtypes, boolean allowBoxing, boolean useVarargs) throws Throwable {
        Scope.Entry e;
        Scope.Entry e2;
        JCTree jCTree;
        Symbol bestSoFar = this.methodNotFound;
        Symbol bestSoFar2 = bestSoFar;
        Env env2 = env;
        boolean staticOnly = false;
        while (env2.outer != null) {
            if (isStatic(env2)) {
                staticOnly = true;
            }
            boolean staticOnly2 = staticOnly;
            Assert.check(((AttrContext) env2.info).preferredTreeForDiagnostics == null);
            ((AttrContext) env2.info).preferredTreeForDiagnostics = env.tree;
            try {
                try {
                    Symbol sym = findMethod(env2, env2.enclClass.sym.type, name, argtypes, typeargtypes, allowBoxing, useVarargs, false);
                    if (sym.exists()) {
                        if (staticOnly2 && sym.kind == 16 && sym.owner.kind == 2 && (8 & sym.flags()) == 0) {
                            StaticError staticError = new StaticError(sym);
                            ((AttrContext) env2.info).preferredTreeForDiagnostics = null;
                            return staticError;
                        }
                        JCTree jCTree2 = null;
                        ((AttrContext) env2.info).preferredTreeForDiagnostics = jCTree2;
                        return sym;
                    }
                    jCTree = null;
                    try {
                        if (sym.kind < bestSoFar2.kind) {
                            bestSoFar2 = sym;
                        }
                        ((AttrContext) env2.info).preferredTreeForDiagnostics = null;
                        if ((8 & env2.enclClass.sym.flags()) != 0) {
                            staticOnly2 = true;
                        }
                        env2 = env2.outer;
                        staticOnly = staticOnly2;
                    } catch (Throwable th) {
                        th = th;
                    }
                } catch (Throwable th2) {
                    th = th2;
                    jCTree = null;
                }
            } catch (Throwable th3) {
                th = th3;
                jCTree = null;
            }
            ((AttrContext) env2.info).preferredTreeForDiagnostics = jCTree;
            throw th;
        }
        Symbol sym2 = findMethod(env, this.syms.predefClass.type, name, argtypes, typeargtypes, allowBoxing, useVarargs, false);
        if (sym2.exists()) {
            return sym2;
        }
        Symbol bestSoFar3 = bestSoFar2;
        Scope.Entry e3 = env.toplevel.namedImportScope.lookup(name);
        while (e3.scope != null) {
            Symbol sym3 = e3.sym;
            Type origin = e3.getOrigin().owner.type;
            if (sym3.kind == 16) {
                if (e3.sym.owner.type != origin) {
                    sym3 = sym3.clone(e3.getOrigin().owner);
                }
                e2 = e3;
                bestSoFar3 = selectBest(env, origin, argtypes, typeargtypes, !isAccessible(env, origin, sym3) ? new AccessError(env, origin, sym3) : sym3, bestSoFar3, allowBoxing, useVarargs, false);
            } else {
                e2 = e3;
            }
            e3 = e2.next();
        }
        if (bestSoFar3.exists()) {
            return bestSoFar3;
        }
        Scope.Entry e4 = env.toplevel.starImportScope.lookup(name);
        while (e4.scope != null) {
            Symbol sym4 = e4.sym;
            Type origin2 = e4.getOrigin().owner.type;
            if (sym4.kind == 16) {
                if (e4.sym.owner.type != origin2) {
                    sym4 = sym4.clone(e4.getOrigin().owner);
                }
                e = e4;
                bestSoFar3 = selectBest(env, origin2, argtypes, typeargtypes, !isAccessible(env, origin2, sym4) ? new AccessError(env, origin2, sym4) : sym4, bestSoFar3, allowBoxing, useVarargs, false);
            } else {
                e = e4;
            }
            e4 = e.next();
        }
        return bestSoFar3;
    }

    Symbol loadClass(Env<AttrContext> env, Name name) {
        try {
            Symbol.ClassSymbol c = this.reader.loadClass(name);
            return isAccessible(env, c) ? c : new AccessError(this, c);
        } catch (ClassReader.BadClassFile err) {
            throw err;
        } catch (Symbol.CompletionFailure e) {
            return this.typeNotFound;
        }
    }

    Symbol findImmediateMemberType(Env<AttrContext> env, Type site, Name name, Symbol.TypeSymbol c) {
        for (Scope.Entry e = c.members().lookup(name); e.scope != null; e = e.next()) {
            if (e.sym.kind == 2) {
                return isAccessible(env, site, e.sym) ? e.sym : new AccessError(env, site, e.sym);
            }
        }
        return this.typeNotFound;
    }

    /* JADX WARN: Multi-variable type inference failed */
    Symbol findInheritedMemberType(Env<AttrContext> env, Type site, Name name, Symbol.TypeSymbol c) {
        Symbol bestSoFar = this.typeNotFound;
        Type st = this.types.supertype(c.type);
        if (st != null && st.hasTag(TypeTag.CLASS)) {
            Symbol sym = findMemberType(env, site, name, st.tsym);
            if (sym.kind < bestSoFar.kind) {
                bestSoFar = sym;
            }
        }
        for (List listInterfaces = this.types.interfaces(c.type); bestSoFar.kind != 129 && listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
            Symbol sym2 = findMemberType(env, site, name, ((Type) listInterfaces.head).tsym);
            if (bestSoFar.kind < 129 && sym2.kind < 129 && sym2.owner != bestSoFar.owner) {
                bestSoFar = new AmbiguityError(bestSoFar, sym2);
            } else if (sym2.kind < bestSoFar.kind) {
                bestSoFar = sym2;
            }
        }
        return bestSoFar;
    }

    Symbol findMemberType(Env<AttrContext> env, Type site, Name name, Symbol.TypeSymbol c) {
        Symbol sym = findImmediateMemberType(env, site, name, c);
        if (sym != this.typeNotFound) {
            return sym;
        }
        return findInheritedMemberType(env, site, name, c);
    }

    Symbol findGlobalType(Env<AttrContext> env, Scope scope, Name name) {
        Symbol bestSoFar = this.typeNotFound;
        for (Scope.Entry e = scope.lookup(name); e.scope != null; e = e.next()) {
            Symbol sym = loadClass(env, e.sym.flatName());
            if (bestSoFar.kind == 2 && sym.kind == 2 && bestSoFar != sym) {
                return new AmbiguityError(bestSoFar, sym);
            }
            if (sym.kind < bestSoFar.kind) {
                bestSoFar = sym;
            }
        }
        return bestSoFar;
    }

    Symbol findTypeVar(Env<AttrContext> env, Name name, boolean staticOnly) {
        for (Scope.Entry e = env.info.scope.lookup(name); e.scope != null; e = e.next()) {
            if (e.sym.kind == 2) {
                if (staticOnly && e.sym.type.hasTag(TypeTag.TYPEVAR) && e.sym.owner.kind == 2) {
                    return new StaticError(e.sym);
                }
                return e.sym;
            }
        }
        return this.typeNotFound;
    }

    /* JADX WARN: Multi-variable type inference failed */
    Symbol findType(Env<AttrContext> env, Name name) {
        Symbol bestSoFar = this.typeNotFound;
        boolean staticOnly = false;
        for (Env env2 = env; env2.outer != null; env2 = env2.outer) {
            if (isStatic(env2)) {
                staticOnly = true;
            }
            Symbol tyvar = findTypeVar(env2, name, staticOnly);
            Symbol sym = findImmediateMemberType(env2, env2.enclClass.sym.type, name, env2.enclClass.sym);
            if (tyvar != this.typeNotFound && (sym == this.typeNotFound || (tyvar.kind == 2 && tyvar.exists() && tyvar.owner.kind == 16))) {
                return tyvar;
            }
            if (sym == this.typeNotFound) {
                sym = findInheritedMemberType(env2, env2.enclClass.sym.type, name, env2.enclClass.sym);
            }
            if (staticOnly && sym.kind == 2 && sym.type.hasTag(TypeTag.CLASS) && sym.type.getEnclosingType().hasTag(TypeTag.CLASS) && env2.enclClass.sym.type.isParameterized() && sym.type.getEnclosingType().isParameterized()) {
                return new StaticError(sym);
            }
            if (sym.exists()) {
                return sym;
            }
            if (sym.kind < bestSoFar.kind) {
                bestSoFar = sym;
            }
            JCTree.JCClassDecl encl = env2.baseClause ? (JCTree.JCClassDecl) env2.tree : env2.enclClass;
            if ((encl.sym.flags() & 8) != 0) {
                staticOnly = true;
            }
        }
        if (!env.tree.hasTag(JCTree.Tag.IMPORT)) {
            Symbol sym2 = findGlobalType(env, env.toplevel.namedImportScope, name);
            if (sym2.exists()) {
                return sym2;
            }
            if (sym2.kind < bestSoFar.kind) {
                bestSoFar = sym2;
            }
            Symbol sym3 = findGlobalType(env, env.toplevel.packge.members(), name);
            if (sym3.exists()) {
                return sym3;
            }
            if (sym3.kind < bestSoFar.kind) {
                bestSoFar = sym3;
            }
            Symbol sym4 = findGlobalType(env, env.toplevel.starImportScope, name);
            return (!sym4.exists() && sym4.kind >= bestSoFar.kind) ? bestSoFar : sym4;
        }
        return bestSoFar;
    }

    Symbol findIdent(Env<AttrContext> env, Name name, int kind) {
        Symbol bestSoFar = this.typeNotFound;
        if ((kind & 4) != 0) {
            Symbol sym = findVar(env, name);
            if (sym.exists()) {
                return sym;
            }
            if (sym.kind < bestSoFar.kind) {
                bestSoFar = sym;
            }
        }
        if ((kind & 2) != 0) {
            Symbol sym2 = findType(env, name);
            if (sym2.kind == 2) {
                reportDependence(env.enclClass.sym, sym2);
            }
            if (sym2.exists()) {
                return sym2;
            }
            if (sym2.kind < bestSoFar.kind) {
                bestSoFar = sym2;
            }
        }
        return (kind & 1) != 0 ? this.reader.enterPackage(name) : bestSoFar;
    }

    public void reportDependence(Symbol from, Symbol to) {
    }

    Symbol findIdentInPackage(Env<AttrContext> env, Symbol.TypeSymbol pck, Name name, int kind) {
        Name fullname = Symbol.TypeSymbol.formFullName(name, pck);
        Symbol bestSoFar = this.typeNotFound;
        Symbol.PackageSymbol pack = null;
        if ((kind & 1) != 0) {
            pack = this.reader.enterPackage(fullname);
            if (pack.exists()) {
                return pack;
            }
        }
        if ((kind & 2) != 0) {
            Symbol sym = loadClass(env, fullname);
            if (sym.exists()) {
                if (name == sym.name) {
                    return sym;
                }
            } else if (sym.kind < bestSoFar.kind) {
                bestSoFar = sym;
            }
        }
        return pack != null ? pack : bestSoFar;
    }

    Symbol findIdentInType(Env<AttrContext> env, Type site, Name name, int kind) {
        Symbol bestSoFar = this.typeNotFound;
        if ((kind & 4) != 0) {
            Symbol sym = findField(env, site, name, site.tsym);
            if (sym.exists()) {
                return sym;
            }
            if (sym.kind < bestSoFar.kind) {
                bestSoFar = sym;
            }
        }
        if ((kind & 2) != 0) {
            Symbol sym2 = findMemberType(env, site, name, site.tsym);
            return (!sym2.exists() && sym2.kind >= bestSoFar.kind) ? bestSoFar : sym2;
        }
        return bestSoFar;
    }

    Symbol accessInternal(Symbol sym, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, boolean qualified, List<Type> argtypes, List<Type> typeargtypes, LogResolveHelper logResolveHelper) {
        if (sym.kind < 129) {
            return sym;
        }
        ResolveError errSym = (ResolveError) sym.baseSymbol();
        Symbol sym2 = errSym.access(name, qualified ? site.tsym : this.syms.noSymbol);
        List<Type> argtypes2 = logResolveHelper.getArgumentTypes(errSym, sym2, name, argtypes);
        if (logResolveHelper.resolveDiagnosticNeeded(site, argtypes2, typeargtypes)) {
            logResolveError(errSym, pos, location, site, name, argtypes2, typeargtypes);
            return sym2;
        }
        return sym2;
    }

    Symbol accessMethod(Symbol sym, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, boolean qualified, List<Type> argtypes, List<Type> typeargtypes) {
        return accessInternal(sym, pos, location, site, name, qualified, argtypes, typeargtypes, this.methodLogResolveHelper);
    }

    Symbol accessMethod(Symbol sym, JCDiagnostic.DiagnosticPosition pos, Type site, Name name, boolean qualified, List<Type> argtypes, List<Type> typeargtypes) {
        return accessMethod(sym, pos, site.tsym, site, name, qualified, argtypes, typeargtypes);
    }

    Symbol accessBase(Symbol sym, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, boolean qualified) {
        return accessInternal(sym, pos, location, site, name, qualified, List.nil(), null, this.basicLogResolveHelper);
    }

    Symbol accessBase(Symbol sym, JCDiagnostic.DiagnosticPosition pos, Type site, Name name, boolean qualified) {
        return accessBase(sym, pos, site.tsym, site, name, qualified);
    }

    class ResolveDeferredRecoveryMap extends DeferredAttr.RecoveryDeferredTypeMap {
        /* JADX WARN: Illegal instructions before constructor call */
        public ResolveDeferredRecoveryMap(DeferredAttr.AttrMode mode, Symbol msym, MethodResolutionPhase step) {
            DeferredAttr deferredAttr = Resolve.this.deferredAttr;
            deferredAttr.getClass();
            super(mode, msym, step);
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.RecoveryDeferredTypeMap, com.sun.tools.javac.comp.DeferredAttr.DeferredTypeMap
        protected Type typeOf(DeferredAttr.DeferredType dt) {
            Type res = super.typeOf(dt);
            if (!res.isErroneous()) {
                switch (TreeInfo.skipParens(dt.tree).getTag()) {
                    case LAMBDA:
                    case REFERENCE:
                        return dt;
                    case CONDEXPR:
                        return res == Type.recoveryType ? dt : res;
                }
            }
            return res;
        }
    }

    void checkNonAbstract(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
        if ((sym.flags() & 1024) != 0 && (sym.flags() & Flags.DEFAULT) == 0) {
            this.log.error(pos, "abstract.cant.be.accessed.directly", Kinds.kindName(sym), sym, sym.location());
        }
    }

    public void printscopes(Scope s) {
        while (s != null) {
            if (s.owner != null) {
                System.err.print(s.owner + ": ");
            }
            for (Scope.Entry e = s.elems; e != null; e = e.sibling) {
                if ((e.sym.flags() & 1024) != 0) {
                    System.err.print("abstract ");
                }
                System.err.print(e.sym + " ");
            }
            System.err.println();
            s = s.next;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void printscopes(Env<AttrContext> env) {
        for (Env env2 = env; env2.outer != null; env2 = env2.outer) {
            System.err.println("------------------------------");
            printscopes(((AttrContext) env2.info).scope);
        }
    }

    public void printscopes(Type t) {
        while (t.hasTag(TypeTag.CLASS)) {
            printscopes(t.tsym.members());
            t = this.types.supertype(t);
        }
    }

    Symbol resolveIdent(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Name name, int kind) {
        return accessBase(findIdent(env, name, kind), pos, env.enclClass.sym.type, name, false);
    }

    Symbol resolveMethod(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Name name, List<Type> argtypes, List<Type> typeargtypes) {
        return lookupMethod(env, pos, env.enclClass.sym, this.resolveMethodCheck, new BasicLookupHelper(name, env.enclClass.sym.type, argtypes, typeargtypes) { // from class: com.sun.tools.javac.comp.Resolve.8
            @Override // com.sun.tools.javac.comp.Resolve.BasicLookupHelper
            Symbol doLookup(Env<AttrContext> env2, MethodResolutionPhase phase) {
                return Resolve.this.findFun(env2, this.name, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired());
            }
        });
    }

    Symbol resolveQualifiedMethod(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
        return resolveQualifiedMethod(pos, env, site.tsym, site, name, argtypes, typeargtypes);
    }

    Symbol resolveQualifiedMethod(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
        return resolveQualifiedMethod(new MethodResolutionContext(), pos, env, location, site, name, argtypes, typeargtypes);
    }

    private Symbol resolveQualifiedMethod(MethodResolutionContext resolveContext, JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
        return lookupMethod(env, pos, location, resolveContext, new BasicLookupHelper(name, site, argtypes, typeargtypes) { // from class: com.sun.tools.javac.comp.Resolve.9
            @Override // com.sun.tools.javac.comp.Resolve.BasicLookupHelper
            Symbol doLookup(Env<AttrContext> env2, MethodResolutionPhase phase) {
                return Resolve.this.findMethod(env2, this.site, this.name, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired(), false);
            }

            @Override // com.sun.tools.javac.comp.Resolve.BasicLookupHelper, com.sun.tools.javac.comp.Resolve.LookupHelper
            Symbol access(Env<AttrContext> env2, JCDiagnostic.DiagnosticPosition pos2, Symbol location2, Symbol sym) {
                if (sym.kind >= 129) {
                    return super.access(env2, pos2, location2, sym);
                }
                if (Resolve.this.allowMethodHandles) {
                    Symbol.MethodSymbol msym = (Symbol.MethodSymbol) sym;
                    if ((msym.flags() & Flags.SIGNATURE_POLYMORPHIC) != 0) {
                        return Resolve.this.findPolymorphicSignatureInstance(env2, sym, this.argtypes);
                    }
                    return sym;
                }
                return sym;
            }
        });
    }

    Symbol findPolymorphicSignatureInstance(Env<AttrContext> env, final Symbol spMethod, List<Type> argtypes) {
        Type mtype = this.infer.instantiatePolymorphicSignatureInstance(env, (Symbol.MethodSymbol) spMethod, this.currentResolutionContext, argtypes);
        for (Symbol sym : this.polymorphicSignatureScope.getElementsByName(spMethod.name)) {
            if (this.types.isSameType(mtype, sym.type)) {
                return sym;
            }
        }
        long flags = (spMethod.flags() & 7) | 137438954496L;
        Symbol msym = new Symbol.MethodSymbol(flags, spMethod.name, mtype, spMethod.owner) { // from class: com.sun.tools.javac.comp.Resolve.10
            @Override // com.sun.tools.javac.code.Symbol
            public Symbol baseSymbol() {
                return spMethod;
            }
        };
        if (!mtype.isErroneous()) {
            this.polymorphicSignatureScope.enter(msym);
        }
        return msym;
    }

    public Symbol.MethodSymbol resolveInternalMethod(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
        MethodResolutionContext resolveContext = new MethodResolutionContext();
        resolveContext.internalResolution = true;
        Symbol sym = resolveQualifiedMethod(resolveContext, pos, env, site.tsym, site, name, argtypes, typeargtypes);
        if (sym.kind == 16) {
            return (Symbol.MethodSymbol) sym;
        }
        throw new FatalError(this.diags.fragment("fatal.err.cant.locate.meth", name));
    }

    Symbol resolveConstructor(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type site, List<Type> argtypes, List<Type> typeargtypes) {
        return resolveConstructor(new MethodResolutionContext(), pos, env, site, argtypes, typeargtypes);
    }

    private Symbol resolveConstructor(MethodResolutionContext resolveContext, final JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type site, List<Type> argtypes, List<Type> typeargtypes) {
        return lookupMethod(env, pos, site.tsym, resolveContext, new BasicLookupHelper(this.names.init, site, argtypes, typeargtypes) { // from class: com.sun.tools.javac.comp.Resolve.11
            @Override // com.sun.tools.javac.comp.Resolve.BasicLookupHelper
            Symbol doLookup(Env<AttrContext> env2, MethodResolutionPhase phase) {
                return Resolve.this.findConstructor(pos, env2, this.site, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired());
            }
        });
    }

    public Symbol.MethodSymbol resolveInternalConstructor(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type site, List<Type> argtypes, List<Type> typeargtypes) {
        MethodResolutionContext resolveContext = new MethodResolutionContext();
        resolveContext.internalResolution = true;
        Symbol sym = resolveConstructor(resolveContext, pos, env, site, argtypes, typeargtypes);
        if (sym.kind == 16) {
            return (Symbol.MethodSymbol) sym;
        }
        throw new FatalError(this.diags.fragment("fatal.err.cant.locate.ctor", site));
    }

    Symbol findConstructor(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type site, List<Type> argtypes, List<Type> typeargtypes, boolean allowBoxing, boolean useVarargs) {
        Symbol sym = findMethod(env, site, this.names.init, argtypes, typeargtypes, allowBoxing, useVarargs, false);
        this.chk.checkDeprecated(pos, env.info.scope.owner, sym);
        return sym;
    }

    Symbol resolveDiamond(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type site, List<Type> argtypes, List<Type> typeargtypes) {
        return lookupMethod(env, pos, site.tsym, this.resolveMethodCheck, new BasicLookupHelper(this.names.init, site, argtypes, typeargtypes) { // from class: com.sun.tools.javac.comp.Resolve.12
            @Override // com.sun.tools.javac.comp.Resolve.BasicLookupHelper
            Symbol doLookup(Env<AttrContext> env2, MethodResolutionPhase phase) {
                return Resolve.this.findDiamond(env2, this.site, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired());
            }

            @Override // com.sun.tools.javac.comp.Resolve.BasicLookupHelper, com.sun.tools.javac.comp.Resolve.LookupHelper
            Symbol access(Env<AttrContext> env2, JCDiagnostic.DiagnosticPosition pos2, Symbol location, Symbol sym) {
                if (sym.kind < 129) {
                    return sym;
                }
                if (sym.kind != 135 && sym.kind != 134) {
                    return super.access(env2, pos2, location, sym);
                }
                final JCDiagnostic details = sym.kind == 135 ? ((InapplicableSymbolError) sym.baseSymbol()).errCandidate().snd : null;
                Symbol sym2 = Resolve.this.accessMethod(new InapplicableSymbolError(sym.kind, "diamondError", Resolve.this.currentResolutionContext) { // from class: com.sun.tools.javac.comp.Resolve.12.1
                    {
                        Resolve resolve = Resolve.this;
                    }

                    @Override // com.sun.tools.javac.comp.Resolve.InapplicableSymbolError, com.sun.tools.javac.comp.Resolve.ResolveError
                    JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType dkind, JCDiagnostic.DiagnosticPosition pos3, Symbol location2, Type site2, Name name, List<Type> argtypes2, List<Type> typeargtypes2) {
                        String key = details == null ? "cant.apply.diamond" : "cant.apply.diamond.1";
                        return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos3, key, Resolve.this.diags.fragment("diamond", site2.tsym), details);
                    }
                }, pos2, this.site, Resolve.this.names.init, true, this.argtypes, this.typeargtypes);
                env2.info.pendingResolutionPhase = Resolve.this.currentResolutionContext.step;
                return sym2;
            }
        });
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Symbol findDiamond(Env<AttrContext> env, Type site, List<Type> argtypes, List<Type> typeargtypes, boolean allowBoxing, boolean useVarargs) {
        Scope.Entry e;
        Resolve resolve = this;
        Type type = site;
        Symbol bestSoFar = resolve.methodNotFound;
        Symbol bestSoFar2 = bestSoFar;
        Scope.Entry e2 = type.tsym.members().lookup(resolve.names.init);
        while (e2.scope != null) {
            final Symbol sym = e2.sym;
            if (sym.kind == 16 && (sym.flags_field & 4096) == 0) {
                List<Type> oldParams = e2.sym.type.hasTag(TypeTag.FORALL) ? ((Type.ForAll) sym.type).tvars : List.nil();
                Type constrType = new Type.ForAll(type.tsym.type.getTypeArguments().appendList(oldParams), resolve.types.createMethodTypeWithReturn(sym.type.asMethodType(), type));
                Symbol.MethodSymbol newConstr = new Symbol.MethodSymbol(sym.flags(), resolve.names.init, constrType, type.tsym) { // from class: com.sun.tools.javac.comp.Resolve.13
                    @Override // com.sun.tools.javac.code.Symbol
                    public Symbol baseSymbol() {
                        return sym;
                    }
                };
                e = e2;
                bestSoFar2 = selectBest(env, site, argtypes, typeargtypes, newConstr, bestSoFar2, allowBoxing, useVarargs, false);
            } else {
                e = e2;
            }
            e2 = e.next();
            resolve = this;
            type = site;
        }
        return bestSoFar2;
    }

    Symbol resolveOperator(JCDiagnostic.DiagnosticPosition pos, JCTree.Tag optag, Env<AttrContext> env, List<Type> argtypes) {
        MethodResolutionContext prevResolutionContext = this.currentResolutionContext;
        try {
            this.currentResolutionContext = new MethodResolutionContext();
            Name name = this.treeinfo.operatorName(optag);
            return lookupMethod(env, pos, this.syms.predefClass, this.currentResolutionContext, new BasicLookupHelper(name, this.syms.predefClass.type, argtypes, null, MethodResolutionPhase.BOX) { // from class: com.sun.tools.javac.comp.Resolve.14
                @Override // com.sun.tools.javac.comp.Resolve.BasicLookupHelper
                Symbol doLookup(Env<AttrContext> env2, MethodResolutionPhase phase) {
                    return Resolve.this.findMethod(env2, this.site, this.name, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired(), true);
                }

                @Override // com.sun.tools.javac.comp.Resolve.BasicLookupHelper, com.sun.tools.javac.comp.Resolve.LookupHelper
                Symbol access(Env<AttrContext> env2, JCDiagnostic.DiagnosticPosition pos2, Symbol location, Symbol sym) {
                    return Resolve.this.accessMethod(sym, pos2, env2.enclClass.sym.type, this.name, false, this.argtypes, null);
                }
            });
        } finally {
            this.currentResolutionContext = prevResolutionContext;
        }
    }

    Symbol resolveUnaryOperator(JCDiagnostic.DiagnosticPosition pos, JCTree.Tag optag, Env<AttrContext> env, Type arg) {
        return resolveOperator(pos, optag, env, List.of(arg));
    }

    Symbol resolveBinaryOperator(JCDiagnostic.DiagnosticPosition pos, JCTree.Tag optag, Env<AttrContext> env, Type left, Type right) {
        return resolveOperator(pos, optag, env, List.of(left, right));
    }

    Symbol getMemberReference(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, JCTree.JCMemberReference referenceTree, Type site, Name name) {
        Type site2 = this.types.capture(site);
        ReferenceLookupHelper lookupHelper = makeReferenceLookupHelper(referenceTree, site2, name, List.nil(), null, MethodResolutionPhase.VARARITY);
        Env<AttrContext> newEnv = env.dup(env.tree, env.info.dup());
        Symbol sym = lookupMethod(newEnv, env.tree.pos(), site2.tsym, this.nilMethodCheck, lookupHelper);
        env.info.pendingResolutionPhase = newEnv.info.pendingResolutionPhase;
        return sym;
    }

    ReferenceLookupHelper makeReferenceLookupHelper(JCTree.JCMemberReference referenceTree, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes, MethodResolutionPhase maxPhase) {
        if (!name.equals(this.names.init)) {
            ReferenceLookupHelper result = new MethodReferenceLookupHelper(referenceTree, name, site, argtypes, typeargtypes, maxPhase);
            return result;
        }
        if (site.hasTag(TypeTag.ARRAY)) {
            ReferenceLookupHelper result2 = new ArrayConstructorReferenceLookupHelper(referenceTree, site, argtypes, typeargtypes, maxPhase);
            return result2;
        }
        ReferenceLookupHelper result3 = new ConstructorReferenceLookupHelper(referenceTree, site, argtypes, typeargtypes, maxPhase);
        return result3;
    }

    Symbol resolveMemberReferenceByArity(Env<AttrContext> env, JCTree.JCMemberReference referenceTree, Type site, Name name, List<Type> argtypes, Infer.InferenceContext inferenceContext) {
        Symbol boundSym;
        Env<AttrContext> unboundEnv;
        boolean isStaticSelector = TreeInfo.isStaticSelector(referenceTree.expr, this.names);
        Type site2 = this.types.capture(site);
        ReferenceLookupHelper boundLookupHelper = makeReferenceLookupHelper(referenceTree, site2, name, argtypes, null, MethodResolutionPhase.VARARITY);
        Env<AttrContext> boundEnv = env.dup(env.tree, env.info.dup());
        Symbol boundSym2 = lookupMethod(boundEnv, env.tree.pos(), site2.tsym, this.arityMethodCheck, boundLookupHelper);
        if (isStaticSelector && !name.equals(this.names.init) && !boundSym2.isStatic() && boundSym2.kind < 128) {
            boundSym = this.methodNotFound;
        } else {
            boundSym = boundSym2;
        }
        Symbol unboundSym = this.methodNotFound;
        Env<AttrContext> unboundEnv2 = env.dup(env.tree, env.info.dup());
        if (!isStaticSelector) {
            unboundEnv = unboundEnv2;
        } else {
            ReferenceLookupHelper unboundLookupHelper = boundLookupHelper.unboundLookup(inferenceContext);
            unboundEnv = unboundEnv2;
            unboundSym = lookupMethod(unboundEnv2, env.tree.pos(), site2.tsym, this.arityMethodCheck, unboundLookupHelper);
            if (unboundSym.isStatic() && unboundSym.kind < 128) {
                unboundSym = this.methodNotFound;
            }
        }
        Symbol bestSym = choose(boundSym, unboundSym);
        env.info.pendingResolutionPhase = (bestSym == unboundSym ? unboundEnv.info : boundEnv.info).pendingResolutionPhase;
        return bestSym;
    }

    /* JADX WARN: Removed duplicated region for block: B:26:0x00ab  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    com.sun.tools.javac.util.Pair<com.sun.tools.javac.code.Symbol, com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper> resolveMemberReference(com.sun.tools.javac.comp.Env<com.sun.tools.javac.comp.AttrContext> r29, com.sun.tools.javac.tree.JCTree.JCMemberReference r30, com.sun.tools.javac.code.Type r31, com.sun.tools.javac.util.Name r32, com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type> r33, com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type> r34, com.sun.tools.javac.comp.Resolve.MethodCheck r35, com.sun.tools.javac.comp.Infer.InferenceContext r36, com.sun.tools.javac.comp.DeferredAttr.AttrMode r37) {
        /*
            Method dump skipped, instruction units count: 481
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Resolve.resolveMemberReference(com.sun.tools.javac.comp.Env, com.sun.tools.javac.tree.JCTree$JCMemberReference, com.sun.tools.javac.code.Type, com.sun.tools.javac.util.Name, com.sun.tools.javac.util.List, com.sun.tools.javac.util.List, com.sun.tools.javac.comp.Resolve$MethodCheck, com.sun.tools.javac.comp.Infer$InferenceContext, com.sun.tools.javac.comp.DeferredAttr$AttrMode):com.sun.tools.javac.util.Pair");
    }

    boolean hasAnotherApplicableMethod(MethodResolutionContext resolutionContext, Symbol bestSoFar, boolean staticMth) {
        for (MethodResolutionContext.Candidate c : resolutionContext.candidates) {
            if (resolutionContext.step == c.step && c.isApplicable() && c.sym != bestSoFar && c.sym.isStatic() == staticMth) {
                return true;
            }
        }
        return false;
    }

    private Symbol choose(Symbol boundSym, Symbol unboundSym) {
        if (lookupSuccess(boundSym) && lookupSuccess(unboundSym)) {
            return ambiguityError(boundSym, unboundSym);
        }
        if (lookupSuccess(boundSym) || (canIgnore(unboundSym) && !canIgnore(boundSym))) {
            return boundSym;
        }
        if (lookupSuccess(unboundSym) || (canIgnore(boundSym) && !canIgnore(unboundSym))) {
            return unboundSym;
        }
        return boundSym;
    }

    private boolean lookupSuccess(Symbol s) {
        return s.kind == 16 || s.kind == 129;
    }

    private boolean canIgnore(Symbol s) {
        switch (s.kind) {
            case 134:
                InapplicableSymbolsError errSyms = (InapplicableSymbolsError) s.baseSymbol();
                break;
            case 135:
                InapplicableSymbolError errSym = (InapplicableSymbolError) s.baseSymbol();
                break;
        }
        return false;
    }

    abstract class LookupHelper {
        List<Type> argtypes;
        MethodResolutionPhase maxPhase;
        Name name;
        Type site;
        List<Type> typeargtypes;

        abstract Symbol access(Env<AttrContext> env, JCDiagnostic.DiagnosticPosition diagnosticPosition, Symbol symbol, Symbol symbol2);

        abstract Symbol lookup(Env<AttrContext> env, MethodResolutionPhase methodResolutionPhase);

        LookupHelper(Name name, Type site, List<Type> argtypes, List<Type> typeargtypes, MethodResolutionPhase maxPhase) {
            this.name = name;
            this.site = site;
            this.argtypes = argtypes;
            this.typeargtypes = typeargtypes;
            this.maxPhase = maxPhase;
        }

        final boolean shouldStop(Symbol sym, MethodResolutionPhase phase) {
            return phase.ordinal() > this.maxPhase.ordinal() || sym.kind < 128 || sym.kind == 129;
        }

        void debug(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
        }
    }

    abstract class BasicLookupHelper extends LookupHelper {
        abstract Symbol doLookup(Env<AttrContext> env, MethodResolutionPhase methodResolutionPhase);

        BasicLookupHelper(Resolve this$0, Name name, Type site, List<Type> argtypes, List<Type> typeargtypes) {
            this(name, site, argtypes, typeargtypes, MethodResolutionPhase.VARARITY);
        }

        BasicLookupHelper(Name name, Type site, List<Type> argtypes, List<Type> typeargtypes, MethodResolutionPhase maxPhase) {
            super(name, site, argtypes, typeargtypes, maxPhase);
        }

        @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
        final Symbol lookup(Env<AttrContext> env, MethodResolutionPhase phase) {
            Symbol sym = doLookup(env, phase);
            if (sym.kind == 129) {
                AmbiguityError a_err = (AmbiguityError) sym.baseSymbol();
                return a_err.mergeAbstracts(this.site);
            }
            return sym;
        }

        @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
        Symbol access(Env<AttrContext> env, JCDiagnostic.DiagnosticPosition pos, Symbol location, Symbol sym) {
            if (sym.kind >= 129) {
                return Resolve.this.accessMethod(sym, pos, location, this.site, this.name, true, this.argtypes, this.typeargtypes);
            }
            return sym;
        }

        @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
        void debug(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
            Resolve.this.reportVerboseResolutionDiagnostic(pos, this.name, this.site, this.argtypes, this.typeargtypes, sym);
        }
    }

    abstract class ReferenceLookupHelper extends LookupHelper {
        JCTree.JCMemberReference referenceTree;

        abstract JCTree.JCMemberReference.ReferenceKind referenceKind(Symbol symbol);

        ReferenceLookupHelper(JCTree.JCMemberReference referenceTree, Name name, Type site, List<Type> argtypes, List<Type> typeargtypes, MethodResolutionPhase maxPhase) {
            super(name, site, argtypes, typeargtypes, maxPhase);
            this.referenceTree = referenceTree;
        }

        ReferenceLookupHelper unboundLookup(Infer.InferenceContext inferenceContext) {
            return new ReferenceLookupHelper(this.referenceTree, this.name, this.site, this.argtypes, this.typeargtypes, this.maxPhase) { // from class: com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper.1
                {
                    Resolve resolve = Resolve.this;
                }

                @Override // com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper
                ReferenceLookupHelper unboundLookup(Infer.InferenceContext inferenceContext2) {
                    return this;
                }

                @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
                Symbol lookup(Env<AttrContext> env, MethodResolutionPhase phase) {
                    return Resolve.this.methodNotFound;
                }

                @Override // com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper
                JCTree.JCMemberReference.ReferenceKind referenceKind(Symbol sym) {
                    Assert.error();
                    return null;
                }
            };
        }

        @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
        Symbol access(Env<AttrContext> env, JCDiagnostic.DiagnosticPosition pos, Symbol location, Symbol sym) {
            if (sym.kind == 129) {
                AmbiguityError a_err = (AmbiguityError) sym.baseSymbol();
                return a_err.mergeAbstracts(this.site);
            }
            return sym;
        }
    }

    class MethodReferenceLookupHelper extends ReferenceLookupHelper {
        MethodReferenceLookupHelper(JCTree.JCMemberReference referenceTree, Name name, Type site, List<Type> argtypes, List<Type> typeargtypes, MethodResolutionPhase maxPhase) {
            super(referenceTree, name, site, argtypes, typeargtypes, maxPhase);
        }

        @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
        final Symbol lookup(Env<AttrContext> env, MethodResolutionPhase phase) {
            return Resolve.this.findMethod(env, this.site, this.name, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired(), Resolve.this.syms.operatorNames.contains(this.name));
        }

        @Override // com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper
        ReferenceLookupHelper unboundLookup(Infer.InferenceContext inferenceContext) {
            if (TreeInfo.isStaticSelector(this.referenceTree.expr, Resolve.this.names) && this.argtypes.nonEmpty() && (this.argtypes.head.hasTag(TypeTag.NONE) || Resolve.this.types.isSubtypeUnchecked(inferenceContext.asUndetVar(this.argtypes.head), this.site))) {
                return Resolve.this.new UnboundMethodReferenceLookupHelper(this.referenceTree, this.name, this.site, this.argtypes, this.typeargtypes, this.maxPhase);
            }
            return super.unboundLookup(inferenceContext);
        }

        @Override // com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper
        JCTree.JCMemberReference.ReferenceKind referenceKind(Symbol sym) {
            if (sym.isStatic()) {
                return JCTree.JCMemberReference.ReferenceKind.STATIC;
            }
            Name selName = TreeInfo.name(this.referenceTree.getQualifierExpression());
            return (selName == null || selName != Resolve.this.names._super) ? JCTree.JCMemberReference.ReferenceKind.BOUND : JCTree.JCMemberReference.ReferenceKind.SUPER;
        }
    }

    class UnboundMethodReferenceLookupHelper extends MethodReferenceLookupHelper {
        UnboundMethodReferenceLookupHelper(JCTree.JCMemberReference referenceTree, Name name, Type site, List<Type> argtypes, List<Type> typeargtypes, MethodResolutionPhase maxPhase) {
            super(referenceTree, name, site, argtypes.tail, typeargtypes, maxPhase);
            if (site.isRaw() && !argtypes.head.hasTag(TypeTag.NONE)) {
                Type asSuperSite = Resolve.this.types.asSuper(argtypes.head, site.tsym);
                this.site = Resolve.this.types.capture(asSuperSite);
            }
        }

        @Override // com.sun.tools.javac.comp.Resolve.MethodReferenceLookupHelper, com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper
        ReferenceLookupHelper unboundLookup(Infer.InferenceContext inferenceContext) {
            return this;
        }

        @Override // com.sun.tools.javac.comp.Resolve.MethodReferenceLookupHelper, com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper
        JCTree.JCMemberReference.ReferenceKind referenceKind(Symbol sym) {
            return JCTree.JCMemberReference.ReferenceKind.UNBOUND;
        }
    }

    class ArrayConstructorReferenceLookupHelper extends ReferenceLookupHelper {
        ArrayConstructorReferenceLookupHelper(JCTree.JCMemberReference referenceTree, Type site, List<Type> argtypes, List<Type> typeargtypes, MethodResolutionPhase maxPhase) {
            super(referenceTree, Resolve.this.names.init, site, argtypes, typeargtypes, maxPhase);
        }

        @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
        protected Symbol lookup(Env<AttrContext> env, MethodResolutionPhase phase) {
            Scope sc = new Scope(Resolve.this.syms.arrayClass);
            Symbol.MethodSymbol arrayConstr = new Symbol.MethodSymbol(1L, this.name, null, this.site.tsym);
            arrayConstr.type = new Type.MethodType(List.of(Resolve.this.syms.intType), this.site, List.nil(), Resolve.this.syms.methodClass);
            sc.enter(arrayConstr);
            return Resolve.this.findMethodInScope(env, this.site, this.name, this.argtypes, this.typeargtypes, sc, Resolve.this.methodNotFound, phase.isBoxingRequired(), phase.isVarargsRequired(), false, false);
        }

        @Override // com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper
        JCTree.JCMemberReference.ReferenceKind referenceKind(Symbol sym) {
            return JCTree.JCMemberReference.ReferenceKind.ARRAY_CTOR;
        }
    }

    class ConstructorReferenceLookupHelper extends ReferenceLookupHelper {
        boolean needsInference;

        ConstructorReferenceLookupHelper(JCTree.JCMemberReference referenceTree, Type site, List<Type> argtypes, List<Type> typeargtypes, MethodResolutionPhase maxPhase) {
            super(referenceTree, Resolve.this.names.init, site, argtypes, typeargtypes, maxPhase);
            if (site.isRaw()) {
                this.site = new Type.ClassType(site.getEnclosingType(), site.tsym.type.getTypeArguments(), site.tsym);
                this.needsInference = true;
            }
        }

        @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
        protected Symbol lookup(Env<AttrContext> env, MethodResolutionPhase phase) {
            Symbol sym;
            if (this.needsInference) {
                sym = Resolve.this.findDiamond(env, this.site, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired());
            } else {
                sym = Resolve.this.findMethod(env, this.site, this.name, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired(), Resolve.this.syms.operatorNames.contains(this.name));
            }
            return (sym.kind != 16 || this.site.getEnclosingType().hasTag(TypeTag.NONE) || Resolve.this.hasEnclosingInstance(env, this.site)) ? sym : new InvalidSymbolError(132, sym, null) { // from class: com.sun.tools.javac.comp.Resolve.ConstructorReferenceLookupHelper.1
                {
                    Resolve resolve = Resolve.this;
                }

                @Override // com.sun.tools.javac.comp.Resolve.ResolveError
                JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType dkind, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
                    return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, "cant.access.inner.cls.constr", site.tsym.name, argtypes, site.getEnclosingType());
                }
            };
        }

        @Override // com.sun.tools.javac.comp.Resolve.ReferenceLookupHelper
        JCTree.JCMemberReference.ReferenceKind referenceKind(Symbol sym) {
            return this.site.getEnclosingType().hasTag(TypeTag.NONE) ? JCTree.JCMemberReference.ReferenceKind.TOPLEVEL : JCTree.JCMemberReference.ReferenceKind.IMPLICIT_INNER;
        }
    }

    Symbol lookupMethod(Env<AttrContext> env, JCDiagnostic.DiagnosticPosition pos, Symbol location, MethodCheck methodCheck, LookupHelper lookupHelper) {
        MethodResolutionContext resolveContext = new MethodResolutionContext();
        resolveContext.methodCheck = methodCheck;
        return lookupMethod(env, pos, location, resolveContext, lookupHelper);
    }

    Symbol lookupMethod(Env<AttrContext> env, JCDiagnostic.DiagnosticPosition pos, Symbol location, MethodResolutionContext resolveContext, LookupHelper lookupHelper) {
        MethodResolutionContext prevResolutionContext = this.currentResolutionContext;
        try {
            Symbol bestSoFar = this.methodNotFound;
            this.currentResolutionContext = resolveContext;
            for (MethodResolutionPhase phase : this.methodResolutionSteps) {
                if (!phase.isApplicable(this.boxingEnabled, this.varargsEnabled) || lookupHelper.shouldStop(bestSoFar, phase)) {
                    break;
                }
                MethodResolutionPhase prevPhase = this.currentResolutionContext.step;
                Symbol prevBest = bestSoFar;
                this.currentResolutionContext.step = phase;
                Symbol sym = lookupHelper.lookup(env, phase);
                lookupHelper.debug(pos, sym);
                bestSoFar = phase.mergeResults(bestSoFar, sym);
                env.info.pendingResolutionPhase = prevBest == bestSoFar ? prevPhase : phase;
            }
            return lookupHelper.access(env, pos, location, bestSoFar);
        } finally {
            this.currentResolutionContext = prevResolutionContext;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    Symbol resolveSelf(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Symbol.TypeSymbol c, Name name) {
        Symbol sym;
        boolean staticOnly = false;
        for (Env env2 = env; env2.outer != null; env2 = env2.outer) {
            if (isStatic(env2)) {
                staticOnly = true;
            }
            if (env2.enclClass.sym == c && (sym = ((AttrContext) env2.info).scope.lookup(name).sym) != null) {
                if (staticOnly) {
                    sym = new StaticError(sym);
                }
                return accessBase(sym, pos, env.enclClass.sym.type, name, true);
            }
            if ((env2.enclClass.sym.flags() & 8) != 0) {
                staticOnly = true;
            }
        }
        if (c.isInterface() && name == this.names._super && !isStatic(env) && this.types.isDirectSuperInterface(c, env.enclClass.sym)) {
            for (Type t : pruneInterfaces(env.enclClass.type)) {
                if (t.tsym == c) {
                    env.info.defaultSuperCallSite = t;
                    return new Symbol.VarSymbol(0L, this.names._super, this.types.asSuper(env.enclClass.type, c), env.enclClass.sym);
                }
            }
            for (Type i : this.types.interfaces(env.enclClass.type)) {
                if (i.tsym.isSubClass(c, this.types) && i.tsym != c) {
                    this.log.error(pos, "illegal.default.super.call", c, this.diags.fragment("redundant.supertype", c, i));
                    return this.syms.errSymbol;
                }
            }
            Assert.error();
        }
        this.log.error(pos, "not.encl.class", c);
        return this.syms.errSymbol;
    }

    private List<Type> pruneInterfaces(Type t) {
        ListBuffer<Type> result = new ListBuffer<>();
        for (Type t1 : this.types.interfaces(t)) {
            boolean shouldAdd = true;
            for (Type t2 : this.types.interfaces(t)) {
                if (t1 != t2 && this.types.isSubtypeNoCapture(t2, t1)) {
                    shouldAdd = false;
                }
            }
            if (shouldAdd) {
                result.append(t1);
            }
        }
        return result.toList();
    }

    Symbol resolveSelfContaining(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Symbol member, boolean isSuperCall) {
        Symbol sym = resolveSelfContainingInternal(env, member, isSuperCall);
        if (sym == null) {
            this.log.error(pos, "encl.class.required", member);
            return this.syms.errSymbol;
        }
        return accessBase(sym, pos, env.enclClass.sym.type, sym.name, true);
    }

    boolean hasEnclosingInstance(Env<AttrContext> env, Type type) {
        Symbol encl = resolveSelfContainingInternal(env, type.tsym, false);
        return encl != null && encl.kind < 128;
    }

    /* JADX WARN: Multi-variable type inference failed */
    private Symbol resolveSelfContainingInternal(Env<AttrContext> env, Symbol member, boolean isSuperCall) {
        Symbol sym;
        Name name = this.names._this;
        Env env2 = isSuperCall ? env.outer : env;
        boolean staticOnly = false;
        if (env2 != null) {
            while (env2 != null && env2.outer != null) {
                if (isStatic(env2)) {
                    staticOnly = true;
                }
                if (env2.enclClass.sym.isSubClass(member.owner, this.types) && (sym = ((AttrContext) env2.info).scope.lookup(name).sym) != null) {
                    return staticOnly ? new StaticError(sym) : sym;
                }
                if ((env2.enclClass.sym.flags() & 8) != 0) {
                    staticOnly = true;
                }
                env2 = env2.outer;
            }
            return null;
        }
        return null;
    }

    Type resolveImplicitThis(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type t) {
        return resolveImplicitThis(pos, env, t, false);
    }

    Type resolveImplicitThis(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Type t, boolean isSuperCall) {
        Symbol symbolResolveSelfContaining;
        if ((t.tsym.owner.kind & 20) != 0) {
            symbolResolveSelfContaining = resolveSelf(pos, env, t.getEnclosingType().tsym, this.names._this);
        } else {
            symbolResolveSelfContaining = resolveSelfContaining(pos, env, t.tsym, isSuperCall);
        }
        Type thisType = symbolResolveSelfContaining.type;
        if (env.info.isSelfCall && thisType.tsym == env.enclClass.sym) {
            this.log.error(pos, "cant.ref.before.ctor.called", "this");
        }
        return thisType;
    }

    public void logAccessErrorInternal(Env<AttrContext> env, JCTree tree, Type type) {
        AccessError error = new AccessError(env, env.enclClass.type, type.tsym);
        logResolveError(error, tree.pos(), env.enclClass.sym, env.enclClass.type, null, null, null);
    }

    private void logResolveError(ResolveError error, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
        JCDiagnostic d = error.getDiagnostic(JCDiagnostic.DiagnosticType.ERROR, pos, location, site, name, argtypes, typeargtypes);
        if (d != null) {
            d.setFlag(JCDiagnostic.DiagnosticFlag.RESOLVE_ERROR);
            this.log.report(d);
        }
    }

    public Object methodArguments(List<Type> argtypes) {
        if (argtypes == null || argtypes.isEmpty()) {
            return this.noArgs;
        }
        ListBuffer<Object> diagArgs = new ListBuffer<>();
        for (Type t : argtypes) {
            if (t.hasTag(TypeTag.DEFERRED)) {
                diagArgs.append(((DeferredAttr.DeferredType) t).tree);
            } else {
                diagArgs.append(t);
            }
        }
        return diagArgs;
    }

    abstract class ResolveError extends Symbol {
        final String debugName;

        abstract JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType diagnosticType, JCDiagnostic.DiagnosticPosition diagnosticPosition, Symbol symbol, Type type, Name name, List<Type> list, List<Type> list2);

        ResolveError(int kind, String debugName) {
            super(kind, 0L, null, null, null);
            this.debugName = debugName;
        }

        @Override // javax.lang.model.element.Element
        public <R, P> R accept(ElementVisitor<R, P> v, P p) {
            throw new AssertionError();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public String toString() {
            return this.debugName;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean exists() {
            return false;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean isStatic() {
            return false;
        }

        protected Symbol access(Name name, Symbol.TypeSymbol location) {
            return Resolve.this.types.createErrorType(name, location, Resolve.this.syms.errSymbol.type).tsym;
        }
    }

    abstract class InvalidSymbolError extends ResolveError {
        Symbol sym;

        InvalidSymbolError(int kind, Symbol sym, String debugName) {
            super(kind, debugName);
            this.sym = sym;
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError, com.sun.tools.javac.code.Symbol
        public boolean exists() {
            return true;
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError, com.sun.tools.javac.code.Symbol
        public String toString() {
            return super.toString() + " wrongSym=" + this.sym;
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        public Symbol access(Name name, Symbol.TypeSymbol location) {
            if ((this.sym.kind & 128) == 0 && (this.sym.kind & 2) != 0) {
                return Resolve.this.types.createErrorType(name, location, this.sym.type).tsym;
            }
            return this.sym;
        }
    }

    class SymbolNotFoundError extends ResolveError {
        SymbolNotFoundError(Resolve this$0, int kind) {
            this(kind, "symbol not found error");
        }

        SymbolNotFoundError(int kind, String debugName) {
            super(kind, debugName);
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType dkind, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
            Symbol location2;
            List<Type> argtypes2 = argtypes == null ? List.nil() : argtypes;
            List<Type> typeargtypes2 = typeargtypes == null ? List.nil() : typeargtypes;
            if (name == Resolve.this.names.error) {
                return null;
            }
            if (Resolve.this.syms.operatorNames.contains(name)) {
                isConstructor = argtypes2.size() == 1;
                boolean isUnaryOp = isConstructor;
                String key = argtypes2.size() == 1 ? "operator.cant.be.applied" : "operator.cant.be.applied.1";
                Type first = argtypes2.head;
                Type second = isUnaryOp ? null : argtypes2.tail.head;
                return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, key, name, first, second);
            }
            boolean hasLocation = false;
            if (location != null) {
                location2 = location;
            } else {
                location2 = site.tsym;
            }
            if (!location2.name.isEmpty()) {
                if (location2.kind != 1 || site.tsym.exists()) {
                    hasLocation = (location2.name.equals(Resolve.this.names._this) || location2.name.equals(Resolve.this.names._super)) ? false : true;
                } else {
                    return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, "doesnt.exist", location2);
                }
            }
            if ((this.kind == 136 || this.kind == 138) && name == Resolve.this.names.init) {
                isConstructor = true;
            }
            Kinds.KindName kindname = isConstructor ? Kinds.KindName.CONSTRUCTOR : Kinds.absentKind(this.kind);
            Name idname = isConstructor ? site.tsym.name : name;
            String errKey = getErrorKey(kindname, typeargtypes2.nonEmpty(), hasLocation);
            if (hasLocation) {
                return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, errKey, kindname, idname, typeargtypes2, args(argtypes2), getLocationDiag(location2, site));
            }
            return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, errKey, kindname, idname, typeargtypes2, args(argtypes2));
        }

        private Object args(List<Type> args) {
            return args.isEmpty() ? args : Resolve.this.methodArguments(args);
        }

        private String getErrorKey(Kinds.KindName kindname, boolean hasTypeArgs, boolean hasLocation) {
            String suffix = hasLocation ? ".location" : "";
            switch (kindname) {
                case METHOD:
                case CONSTRUCTOR:
                    suffix = (suffix + ".args") + (hasTypeArgs ? ".params" : "");
                    break;
            }
            return "cant.resolve" + suffix;
        }

        private JCDiagnostic getLocationDiag(Symbol location, Type site) {
            if (location.kind == 4) {
                return Resolve.this.diags.fragment("location.1", Kinds.kindName(location), location, location.type);
            }
            return Resolve.this.diags.fragment("location", Kinds.typeKindName(site), site, null);
        }
    }

    class InapplicableSymbolError extends ResolveError {
        protected MethodResolutionContext resolveContext;

        InapplicableSymbolError(Resolve this$0, MethodResolutionContext context) {
            this(135, "inapplicable symbol error", context);
        }

        protected InapplicableSymbolError(int kind, String debugName, MethodResolutionContext context) {
            super(kind, debugName);
            this.resolveContext = context;
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError, com.sun.tools.javac.code.Symbol
        public String toString() {
            return super.toString();
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError, com.sun.tools.javac.code.Symbol
        public boolean exists() {
            return true;
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType dkind, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
            if (name == Resolve.this.names.error) {
                return null;
            }
            if (Resolve.this.syms.operatorNames.contains(name)) {
                boolean isUnaryOp = argtypes.size() == 1;
                String key = argtypes.size() == 1 ? "operator.cant.be.applied" : "operator.cant.be.applied.1";
                Type first = argtypes.head;
                Type second = isUnaryOp ? null : argtypes.tail.head;
                return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, key, name, first, second);
            }
            Pair<Symbol, JCDiagnostic> c = errCandidate();
            if (Resolve.this.compactMethodDiags) {
                for (Map.Entry<MethodResolutionDiagHelper.Template, MethodResolutionDiagHelper.DiagnosticRewriter> _entry : MethodResolutionDiagHelper.rewriters.entrySet()) {
                    if (_entry.getKey().matches(c.snd)) {
                        JCDiagnostic simpleDiag = _entry.getValue().rewriteDiagnostic(Resolve.this.diags, pos, Resolve.this.log.currentSource(), dkind, c.snd);
                        simpleDiag.setFlag(JCDiagnostic.DiagnosticFlag.COMPRESSED);
                        return simpleDiag;
                    }
                }
            }
            Symbol ws = c.fst.asMemberOf(site, Resolve.this.types);
            return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, "cant.apply.symbol", Kinds.kindName(ws), ws.name == Resolve.this.names.init ? ws.owner.name : ws.name, Resolve.this.methodArguments(ws.type.mo176getParameterTypes()), Resolve.this.methodArguments(argtypes), Kinds.kindName(ws.owner), ws.owner.type, c.snd);
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        public Symbol access(Name name, Symbol.TypeSymbol location) {
            return Resolve.this.types.createErrorType(name, location, Resolve.this.syms.errSymbol.type).tsym;
        }

        protected Pair<Symbol, JCDiagnostic> errCandidate() {
            MethodResolutionContext.Candidate bestSoFar = null;
            for (MethodResolutionContext.Candidate c : this.resolveContext.candidates) {
                if (!c.isApplicable()) {
                    bestSoFar = c;
                }
            }
            Assert.checkNonNull(bestSoFar);
            return new Pair<>(bestSoFar.sym, bestSoFar.details);
        }
    }

    class InapplicableSymbolsError extends InapplicableSymbolError {
        InapplicableSymbolsError(MethodResolutionContext context) {
            super(134, "inapplicable symbols", context);
        }

        @Override // com.sun.tools.javac.comp.Resolve.InapplicableSymbolError, com.sun.tools.javac.comp.Resolve.ResolveError
        JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType dkind, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
            Map<Symbol, JCDiagnostic> filteredCandidates;
            Map<Symbol, JCDiagnostic> filteredCandidates2;
            EnumSet enumSetNoneOf;
            Map<Symbol, JCDiagnostic> candidatesMap = mapCandidates();
            if (Resolve.this.compactMethodDiags) {
                filteredCandidates = filterCandidates(candidatesMap);
            } else {
                filteredCandidates = mapCandidates();
            }
            if (!filteredCandidates.isEmpty()) {
                filteredCandidates2 = filteredCandidates;
            } else {
                filteredCandidates2 = candidatesMap;
            }
            boolean truncatedDiag = candidatesMap.size() != filteredCandidates2.size();
            if (filteredCandidates2.size() > 1) {
                JCDiagnostic.Factory factory = Resolve.this.diags;
                if (truncatedDiag) {
                    enumSetNoneOf = EnumSet.of(JCDiagnostic.DiagnosticFlag.COMPRESSED);
                } else {
                    enumSetNoneOf = EnumSet.noneOf(JCDiagnostic.DiagnosticFlag.class);
                }
                JCDiagnostic err = factory.create(dkind, null, enumSetNoneOf, Resolve.this.log.currentSource(), pos, "cant.apply.symbols", name == Resolve.this.names.init ? Kinds.KindName.CONSTRUCTOR : Kinds.absentKind(this.kind), name == Resolve.this.names.init ? site.tsym.name : name, Resolve.this.methodArguments(argtypes));
                return new JCDiagnostic.MultilineDiagnostic(err, candidateDetails(filteredCandidates2, site));
            }
            if (filteredCandidates2.size() == 1) {
                Map.Entry<Symbol, JCDiagnostic> _e = filteredCandidates2.entrySet().iterator().next();
                final Pair<Symbol, JCDiagnostic> p = new Pair<>(_e.getKey(), _e.getValue());
                JCDiagnostic d = new InapplicableSymbolError(this.resolveContext) { // from class: com.sun.tools.javac.comp.Resolve.InapplicableSymbolsError.1
                    {
                        Resolve resolve = Resolve.this;
                    }

                    @Override // com.sun.tools.javac.comp.Resolve.InapplicableSymbolError
                    protected Pair<Symbol, JCDiagnostic> errCandidate() {
                        return p;
                    }
                }.getDiagnostic(dkind, pos, location, site, name, argtypes, typeargtypes);
                if (truncatedDiag) {
                    d.setFlag(JCDiagnostic.DiagnosticFlag.COMPRESSED);
                }
                return d;
            }
            return new SymbolNotFoundError(Resolve.this, 136).getDiagnostic(dkind, pos, location, site, name, argtypes, typeargtypes);
        }

        /* JADX INFO: Access modifiers changed from: private */
        public Map<Symbol, JCDiagnostic> mapCandidates() {
            Map<Symbol, JCDiagnostic> candidates = new LinkedHashMap<>();
            for (MethodResolutionContext.Candidate c : this.resolveContext.candidates) {
                if (!c.isApplicable()) {
                    candidates.put(c.sym, c.details);
                }
            }
            return candidates;
        }

        Map<Symbol, JCDiagnostic> filterCandidates(Map<Symbol, JCDiagnostic> candidatesMap) {
            Map<Symbol, JCDiagnostic> candidates = new LinkedHashMap<>();
            for (Map.Entry<Symbol, JCDiagnostic> _entry : candidatesMap.entrySet()) {
                JCDiagnostic d = _entry.getValue();
                if (!new MethodResolutionDiagHelper.Template(MethodCheckDiag.ARITY_MISMATCH.regex(), new MethodResolutionDiagHelper.Template[0]).matches(d)) {
                    candidates.put(_entry.getKey(), d);
                }
            }
            return candidates;
        }

        private List<JCDiagnostic> candidateDetails(Map<Symbol, JCDiagnostic> candidatesMap, Type site) {
            List<JCDiagnostic> details = List.nil();
            for (Map.Entry<Symbol, JCDiagnostic> _entry : candidatesMap.entrySet()) {
                Symbol sym = _entry.getKey();
                JCDiagnostic detailDiag = Resolve.this.diags.fragment("inapplicable.method", Kinds.kindName(sym), sym.location(site, Resolve.this.types), sym.asMemberOf(site, Resolve.this.types), _entry.getValue());
                details = details.prepend(detailDiag);
            }
            return details;
        }
    }

    class AccessError extends InvalidSymbolError {
        private Env<AttrContext> env;
        private Type site;

        AccessError(Resolve this$0, Symbol sym) {
            this(null, null, sym);
        }

        AccessError(Env<AttrContext> env, Type site, Symbol sym) {
            super(130, sym, "access error");
            this.env = env;
            this.site = site;
            if (Resolve.this.debugResolve) {
                Resolve.this.log.error("proc.messager", sym + " @ " + site + " is inaccessible.");
            }
        }

        @Override // com.sun.tools.javac.comp.Resolve.InvalidSymbolError, com.sun.tools.javac.comp.Resolve.ResolveError, com.sun.tools.javac.code.Symbol
        public boolean exists() {
            return false;
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType dkind, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
            if (!this.sym.owner.type.hasTag(TypeTag.ERROR)) {
                if (this.sym.name == Resolve.this.names.init && this.sym.owner != site.tsym) {
                    return new SymbolNotFoundError(Resolve.this, 136).getDiagnostic(dkind, pos, location, site, name, argtypes, typeargtypes);
                }
                if ((this.sym.flags() & 1) != 0 || (this.env != null && this.site != null && !Resolve.this.isAccessible(this.env, this.site))) {
                    return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, "not.def.access.class.intf.cant.access", this.sym, this.sym.location());
                }
                if ((this.sym.flags() & 6) != 0) {
                    return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, "report.access", this.sym, Flags.asFlagSet(this.sym.flags() & 6), this.sym.location());
                }
                return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, "not.def.public.cant.access", this.sym, this.sym.location());
            }
            return null;
        }
    }

    class StaticError extends InvalidSymbolError {
        StaticError(Symbol sym) {
            super(131, sym, "static error");
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType dkind, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
            Symbol errSym = (this.sym.kind == 2 && this.sym.type.hasTag(TypeTag.CLASS)) ? Resolve.this.types.erasure(this.sym.type).tsym : this.sym;
            return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, "non-static.cant.be.ref", Kinds.kindName(this.sym), errSym);
        }
    }

    class AmbiguityError extends ResolveError {
        List<Symbol> ambiguousSyms;

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError, com.sun.tools.javac.code.Symbol
        public boolean exists() {
            return true;
        }

        AmbiguityError(Symbol sym1, Symbol sym2) {
            super(129, "ambiguity error");
            this.ambiguousSyms = List.nil();
            this.ambiguousSyms = flatten(sym2).appendList(flatten(sym1));
        }

        private List<Symbol> flatten(Symbol sym) {
            if (sym.kind == 129) {
                return ((AmbiguityError) sym.baseSymbol()).ambiguousSyms;
            }
            return List.of(sym);
        }

        AmbiguityError addAmbiguousSymbol(Symbol s) {
            this.ambiguousSyms = this.ambiguousSyms.prepend(s);
            return this;
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType dkind, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
            List<Symbol> diagSyms = this.ambiguousSyms.reverse();
            Symbol s1 = diagSyms.head;
            Symbol s2 = diagSyms.tail.head;
            Name sname = s1.name;
            if (sname == Resolve.this.names.init) {
                sname = s1.owner.name;
            }
            return Resolve.this.diags.create(dkind, Resolve.this.log.currentSource(), pos, "ref.ambiguous", sname, Kinds.kindName(s1), s1, s1.location(site, Resolve.this.types), Kinds.kindName(s2), s2, s2.location(site, Resolve.this.types));
        }

        Symbol mergeAbstracts(Type site) {
            List<Symbol> ambiguousInOrder = this.ambiguousSyms.reverse();
            for (Symbol s : ambiguousInOrder) {
                Type mt = Resolve.this.types.memberType(site, s);
                boolean found = true;
                List<Type> allThrown = mt.mo179getThrownTypes();
                for (Symbol s2 : ambiguousInOrder) {
                    Type mt2 = Resolve.this.types.memberType(site, s2);
                    if ((s2.flags() & 1024) == 0 || !Resolve.this.types.overrideEquivalent(mt, mt2) || !Resolve.this.types.isSameTypes(s.erasure(Resolve.this.types).mo176getParameterTypes(), s2.erasure(Resolve.this.types).mo176getParameterTypes())) {
                        return this;
                    }
                    Type mst = Resolve.this.mostSpecificReturnType(mt, mt2);
                    if (mst == null || mst != mt) {
                        found = false;
                        break;
                    }
                    allThrown = Resolve.this.chk.intersect(allThrown, mt2.mo179getThrownTypes());
                }
                if (found) {
                    return allThrown == mt.mo179getThrownTypes() ? s : new Symbol.MethodSymbol(s.flags(), s.name, Resolve.this.types.createMethodTypeWithThrown(s.type, allThrown), s.owner);
                }
            }
            return this;
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        protected Symbol access(Name name, Symbol.TypeSymbol location) {
            Symbol firstAmbiguity = this.ambiguousSyms.last();
            return firstAmbiguity.kind == 2 ? Resolve.this.types.createErrorType(name, location, firstAmbiguity.type).tsym : firstAmbiguity;
        }
    }

    class BadVarargsMethod extends ResolveError {
        ResolveError delegatedError;

        BadVarargsMethod(ResolveError delegatedError) {
            super(delegatedError.kind, "badVarargs");
            this.delegatedError = delegatedError;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Symbol baseSymbol() {
            return this.delegatedError.baseSymbol();
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        protected Symbol access(Name name, Symbol.TypeSymbol location) {
            return this.delegatedError.access(name, location);
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError, com.sun.tools.javac.code.Symbol
        public boolean exists() {
            return true;
        }

        @Override // com.sun.tools.javac.comp.Resolve.ResolveError
        JCDiagnostic getDiagnostic(JCDiagnostic.DiagnosticType dkind, JCDiagnostic.DiagnosticPosition pos, Symbol location, Type site, Name name, List<Type> argtypes, List<Type> typeargtypes) {
            return this.delegatedError.getDiagnostic(dkind, pos, location, site, name, argtypes, typeargtypes);
        }
    }

    static class MethodResolutionDiagHelper {
        static final Template skip = new Template("", new Template[0]) { // from class: com.sun.tools.javac.comp.Resolve.MethodResolutionDiagHelper.1
            @Override // com.sun.tools.javac.comp.Resolve.MethodResolutionDiagHelper.Template
            boolean matches(Object d) {
                return true;
            }
        };
        static final Map<Template, DiagnosticRewriter> rewriters = new LinkedHashMap();

        interface DiagnosticRewriter {
            JCDiagnostic rewriteDiagnostic(JCDiagnostic.Factory factory, JCDiagnostic.DiagnosticPosition diagnosticPosition, DiagnosticSource diagnosticSource, JCDiagnostic.DiagnosticType diagnosticType, JCDiagnostic jCDiagnostic);
        }

        MethodResolutionDiagHelper() {
        }

        static class Template {
            String regex;
            Template[] subTemplates;

            Template(String key, Template... subTemplates) {
                this.regex = key;
                this.subTemplates = subTemplates;
            }

            boolean matches(Object o) {
                JCDiagnostic d = (JCDiagnostic) o;
                Object[] args = d.getArgs();
                if (!d.getCode().matches(this.regex) || this.subTemplates.length != d.getArgs().length) {
                    return false;
                }
                for (int i = 0; i < args.length; i++) {
                    if (!this.subTemplates[i].matches(args[i])) {
                        return false;
                    }
                }
                return true;
            }
        }

        static {
            String argMismatchRegex = MethodCheckDiag.ARG_MISMATCH.regex();
            rewriters.put(new Template(argMismatchRegex, skip), new DiagnosticRewriter() { // from class: com.sun.tools.javac.comp.Resolve.MethodResolutionDiagHelper.2
                @Override // com.sun.tools.javac.comp.Resolve.MethodResolutionDiagHelper.DiagnosticRewriter
                public JCDiagnostic rewriteDiagnostic(JCDiagnostic.Factory diags, JCDiagnostic.DiagnosticPosition preferedPos, DiagnosticSource preferredSource, JCDiagnostic.DiagnosticType preferredKind, JCDiagnostic d) {
                    JCDiagnostic cause = (JCDiagnostic) d.getArgs()[0];
                    JCDiagnostic.DiagnosticPosition pos = d.getDiagnosticPosition();
                    if (pos == null) {
                        pos = preferedPos;
                    }
                    return diags.create(preferredKind, preferredSource, pos, "prob.found.req", cause);
                }
            });
        }
    }

    /* JADX WARN: Enum visitor error
    jadx.core.utils.exceptions.JadxRuntimeException: Init of enum field 'VARARITY' uses external variables
    	at jadx.core.dex.visitors.EnumVisitor.createEnumFieldByConstructor(EnumVisitor.java:451)
    	at jadx.core.dex.visitors.EnumVisitor.processEnumFieldByField(EnumVisitor.java:372)
    	at jadx.core.dex.visitors.EnumVisitor.processEnumFieldByWrappedInsn(EnumVisitor.java:337)
    	at jadx.core.dex.visitors.EnumVisitor.extractEnumFieldsFromFilledArray(EnumVisitor.java:322)
    	at jadx.core.dex.visitors.EnumVisitor.extractEnumFieldsFromInsn(EnumVisitor.java:262)
    	at jadx.core.dex.visitors.EnumVisitor.convertToEnum(EnumVisitor.java:151)
    	at jadx.core.dex.visitors.EnumVisitor.visit(EnumVisitor.java:100)
     */
    /* JADX WARN: Failed to restore enum class, 'enum' modifier and super class removed */
    static class MethodResolutionPhase {
        private static final /* synthetic */ MethodResolutionPhase[] $VALUES;
        public static final MethodResolutionPhase BASIC = new MethodResolutionPhase("BASIC", 0, false, false);
        public static final MethodResolutionPhase BOX = new MethodResolutionPhase("BOX", 1, true, false);
        public static final MethodResolutionPhase VARARITY;
        final boolean isBoxingRequired;
        final boolean isVarargsRequired;

        public static MethodResolutionPhase valueOf(String name) {
            return (MethodResolutionPhase) Enum.valueOf(MethodResolutionPhase.class, name);
        }

        public static MethodResolutionPhase[] values() {
            return (MethodResolutionPhase[]) $VALUES.clone();
        }

        static {
            boolean z = true;
            VARARITY = new MethodResolutionPhase("VARARITY", 2, z, z) { // from class: com.sun.tools.javac.comp.Resolve.MethodResolutionPhase.1
                @Override // com.sun.tools.javac.comp.Resolve.MethodResolutionPhase
                public Symbol mergeResults(Symbol bestSoFar, Symbol sym) {
                    Assert.check(bestSoFar.kind >= 128 && bestSoFar.kind != 129);
                    if (sym.kind < 128) {
                        return sym;
                    }
                    switch (bestSoFar.kind) {
                        case 134:
                        case 135:
                            switch (sym.kind) {
                                case 135:
                                    return bestSoFar.kind == 134 ? bestSoFar : sym;
                                case 136:
                                    return bestSoFar;
                                default:
                                    return sym;
                            }
                        default:
                            return bestSoFar;
                    }
                }
            };
            $VALUES = new MethodResolutionPhase[]{BASIC, BOX, VARARITY};
        }

        private MethodResolutionPhase(String str, int i, boolean isBoxingRequired, boolean isVarargsRequired) {
            this.isBoxingRequired = isBoxingRequired;
            this.isVarargsRequired = isVarargsRequired;
        }

        public boolean isBoxingRequired() {
            return this.isBoxingRequired;
        }

        public boolean isVarargsRequired() {
            return this.isVarargsRequired;
        }

        public boolean isApplicable(boolean boxingEnabled, boolean varargsEnabled) {
            return (varargsEnabled || !this.isVarargsRequired) && (boxingEnabled || !this.isBoxingRequired);
        }

        public Symbol mergeResults(Symbol prev, Symbol sym) {
            return sym;
        }
    }

    class MethodResolutionContext {
        MethodCheck methodCheck;
        private List<Candidate> candidates = List.nil();
        MethodResolutionPhase step = null;
        private boolean internalResolution = false;
        private DeferredAttr.AttrMode attrMode = DeferredAttr.AttrMode.SPECULATIVE;

        MethodResolutionContext() {
            this.methodCheck = Resolve.this.resolveMethodCheck;
        }

        void addInapplicableCandidate(Symbol sym, JCDiagnostic details) {
            Candidate c = new Candidate(Resolve.this.currentResolutionContext.step, sym, details, null);
            this.candidates = this.candidates.append(c);
        }

        void addApplicableCandidate(Symbol sym, Type mtype) {
            Candidate c = new Candidate(Resolve.this.currentResolutionContext.step, sym, null, mtype);
            this.candidates = this.candidates.append(c);
        }

        DeferredAttr.DeferredAttrContext deferredAttrContext(Symbol sym, Infer.InferenceContext inferenceContext, Attr.ResultInfo pendingResult, Warner warn) {
            DeferredAttr.DeferredAttrContext parent = pendingResult == null ? Resolve.this.deferredAttr.emptyDeferredAttrContext : pendingResult.checkContext.deferredAttrContext();
            DeferredAttr deferredAttr = Resolve.this.deferredAttr;
            deferredAttr.getClass();
            return deferredAttr.new DeferredAttrContext(this.attrMode, sym, this.step, inferenceContext, parent, warn);
        }

        class Candidate {
            final JCDiagnostic details;
            final Type mtype;
            final MethodResolutionPhase step;
            final Symbol sym;

            private Candidate(MethodResolutionPhase step, Symbol sym, JCDiagnostic details, Type mtype) {
                this.step = step;
                this.sym = sym;
                this.details = details;
                this.mtype = mtype;
            }

            public boolean equals(Object o) {
                if (o instanceof Candidate) {
                    Symbol s1 = this.sym;
                    Symbol s2 = ((Candidate) o).sym;
                    if (s1 == s2 || (!s1.overrides(s2, s1.owner.type.tsym, Resolve.this.types, false) && !s2.overrides(s1, s2.owner.type.tsym, Resolve.this.types, false))) {
                        if ((s1.isConstructor() || s2.isConstructor()) && s1.owner != s2.owner) {
                            return true;
                        }
                    } else {
                        return true;
                    }
                }
                return false;
            }

            boolean isApplicable() {
                return this.mtype != null;
            }
        }

        DeferredAttr.AttrMode attrMode() {
            return this.attrMode;
        }

        boolean internal() {
            return this.internalResolution;
        }
    }
}
