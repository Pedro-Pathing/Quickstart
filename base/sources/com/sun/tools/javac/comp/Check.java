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
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.DeferredAttr;
import com.sun.tools.javac.comp.Infer;
import com.sun.tools.javac.jvm.ClassReader;
import com.sun.tools.javac.jvm.Profile;
import com.sun.tools.javac.jvm.Target;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.util.Abort;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.DiagnosticSource;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.MandatoryWarningHandler;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.Pair;
import com.sun.tools.javac.util.Warner;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import javax.tools.JavaFileManager;

/* JADX INFO: loaded from: classes.dex */
public class Check {
    protected static final Context.Key<Check> checkKey = new Context.Key<>();
    private static final boolean ignoreAnnotatedCasts = true;
    boolean allowAnnotations;
    boolean allowCovariantReturns;
    boolean allowDefaultMethods;
    boolean allowGenerics;
    boolean allowSimplifiedVarargs;
    boolean allowStrictMethodClashCheck;
    boolean allowVarargs;
    boolean complexInference;
    private Set<Name> defaultTargets;
    private final DeferredAttr deferredAttr;
    private DeferredLintHandler deferredLintHandler;
    private MandatoryWarningHandler deprecationHandler;
    private final Name[] dfltTargetMeta;
    private final JCDiagnostic.Factory diags;
    private boolean enableSunApiLintControl;
    private final Enter enter;
    private final JavaFileManager fileManager;
    private final Infer infer;
    private Lint lint;
    private final Log log;
    private Symbol.MethodSymbol method;
    private final Names names;
    private final Profile profile;
    private final Resolve rs;
    private MandatoryWarningHandler sunApiHandler;
    private boolean suppressAbortOnBadClassFile;
    private final Symtab syms;
    char syntheticNameChar;
    private final TreeInfo treeinfo;
    private final Types types;
    private MandatoryWarningHandler uncheckedHandler;
    private final boolean warnOnAccessToSensitiveMembers;
    private boolean warnOnSyntheticConflicts;
    public Map<Name, Symbol.ClassSymbol> compiled = new HashMap();
    CheckContext basicHandler = new CheckContext() { // from class: com.sun.tools.javac.comp.Check.1
        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public void report(JCDiagnostic.DiagnosticPosition pos, JCDiagnostic details) {
            Check.this.log.error(pos, "prob.found.req", details);
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public boolean compatible(Type found, Type req, Warner warn) {
            return Check.this.types.isAssignable(found, req, warn);
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public Warner checkWarner(JCDiagnostic.DiagnosticPosition pos, Type found, Type req) {
            return Check.this.convertWarner(pos, found, req);
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public Infer.InferenceContext inferenceContext() {
            return Check.this.infer.emptyContext;
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public DeferredAttr.DeferredAttrContext deferredAttrContext() {
            return Check.this.deferredAttr.emptyDeferredAttrContext;
        }

        public String toString() {
            return "CheckContext: basicHandler";
        }
    };
    Types.UnaryVisitor<Boolean> isTypeArgErroneous = new Types.UnaryVisitor<Boolean>() { // from class: com.sun.tools.javac.comp.Check.5
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Void s) {
            return Boolean.valueOf(t.isErroneous());
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitTypeVar(Type.TypeVar t, Void s) {
            return visit(t.getUpperBound());
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitCapturedType(Type.CapturedType t, Void s) {
            return Boolean.valueOf(visit(t.getUpperBound()).booleanValue() || visit(t.getLowerBound()).booleanValue());
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitWildcardType(Type.WildcardType t, Void s) {
            return visit(t.type);
        }
    };
    Warner overrideWarner = new Warner();
    private Filter<Symbol> equalsHasCodeFilter = new Filter<Symbol>() { // from class: com.sun.tools.javac.comp.Check.6
        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol s) {
            return Symbol.MethodSymbol.implementation_filter.accepts(s) && (s.flags() & 35184372088832L) == 0;
        }
    };

    public interface CheckContext {
        Warner checkWarner(JCDiagnostic.DiagnosticPosition diagnosticPosition, Type type, Type type2);

        boolean compatible(Type type, Type type2, Warner warner);

        DeferredAttr.DeferredAttrContext deferredAttrContext();

        Infer.InferenceContext inferenceContext();

        void report(JCDiagnostic.DiagnosticPosition diagnosticPosition, JCDiagnostic jCDiagnostic);
    }

    public static Check instance(Context context) {
        Check instance = (Check) context.get(checkKey);
        if (instance == null) {
            return new Check(context);
        }
        return instance;
    }

    protected Check(Context context) {
        context.put(checkKey, this);
        this.names = Names.instance(context);
        this.dfltTargetMeta = new Name[]{this.names.PACKAGE, this.names.TYPE, this.names.FIELD, this.names.METHOD, this.names.CONSTRUCTOR, this.names.ANNOTATION_TYPE, this.names.LOCAL_VARIABLE, this.names.PARAMETER};
        this.log = Log.instance(context);
        this.rs = Resolve.instance(context);
        this.syms = Symtab.instance(context);
        this.enter = Enter.instance(context);
        this.deferredAttr = DeferredAttr.instance(context);
        this.infer = Infer.instance(context);
        this.types = Types.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        Options options = Options.instance(context);
        this.lint = Lint.instance(context);
        this.treeinfo = TreeInfo.instance(context);
        this.fileManager = (JavaFileManager) context.get(JavaFileManager.class);
        Source source = Source.instance(context);
        this.allowGenerics = source.allowGenerics();
        this.allowVarargs = source.allowVarargs();
        this.allowAnnotations = source.allowAnnotations();
        this.allowCovariantReturns = source.allowCovariantReturns();
        this.allowSimplifiedVarargs = source.allowSimplifiedVarargs();
        this.allowDefaultMethods = source.allowDefaultMethods();
        this.allowStrictMethodClashCheck = source.allowStrictMethodClashCheck();
        this.complexInference = options.isSet("complexinference");
        this.warnOnSyntheticConflicts = options.isSet("warnOnSyntheticConflicts");
        this.suppressAbortOnBadClassFile = options.isSet("suppressAbortOnBadClassFile");
        this.enableSunApiLintControl = options.isSet("enableSunApiLintControl");
        this.warnOnAccessToSensitiveMembers = options.isSet("warnOnAccessToSensitiveMembers");
        Target target = Target.instance(context);
        this.syntheticNameChar = target.syntheticNameChar();
        this.profile = Profile.instance(context);
        boolean verboseDeprecated = this.lint.isEnabled(Lint.LintCategory.DEPRECATION);
        boolean verboseUnchecked = this.lint.isEnabled(Lint.LintCategory.UNCHECKED);
        boolean verboseSunApi = this.lint.isEnabled(Lint.LintCategory.SUNAPI);
        boolean enforceMandatoryWarnings = source.enforceMandatoryWarnings();
        this.deprecationHandler = new MandatoryWarningHandler(this.log, verboseDeprecated, enforceMandatoryWarnings, "deprecated", Lint.LintCategory.DEPRECATION);
        this.uncheckedHandler = new MandatoryWarningHandler(this.log, verboseUnchecked, enforceMandatoryWarnings, "unchecked", Lint.LintCategory.UNCHECKED);
        this.sunApiHandler = new MandatoryWarningHandler(this.log, verboseSunApi, enforceMandatoryWarnings, "sunapi", null);
        this.deferredLintHandler = DeferredLintHandler.instance(context);
    }

    Lint setLint(Lint newLint) {
        Lint prev = this.lint;
        this.lint = newLint;
        return prev;
    }

    Symbol.MethodSymbol setMethod(Symbol.MethodSymbol newMethod) {
        Symbol.MethodSymbol prev = this.method;
        this.method = newMethod;
        return prev;
    }

    void warnDeprecated(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
        if (!this.lint.isSuppressed(Lint.LintCategory.DEPRECATION)) {
            this.deprecationHandler.report(pos, "has.been.deprecated", sym, sym.location());
        }
    }

    public void warnUnchecked(JCDiagnostic.DiagnosticPosition pos, String msg, Object... args) {
        if (!this.lint.isSuppressed(Lint.LintCategory.UNCHECKED)) {
            this.uncheckedHandler.report(pos, msg, args);
        }
    }

    void warnUnsafeVararg(JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        if (this.lint.isEnabled(Lint.LintCategory.VARARGS) && this.allowSimplifiedVarargs) {
            this.log.warning(Lint.LintCategory.VARARGS, pos, key, args);
        }
    }

    public void warnSunApi(JCDiagnostic.DiagnosticPosition pos, String msg, Object... args) {
        if (!this.lint.isSuppressed(Lint.LintCategory.SUNAPI)) {
            this.sunApiHandler.report(pos, msg, args);
        }
    }

    public void warnStatic(JCDiagnostic.DiagnosticPosition pos, String msg, Object... args) {
        if (this.lint.isEnabled(Lint.LintCategory.STATIC)) {
            this.log.warning(Lint.LintCategory.STATIC, pos, msg, args);
        }
    }

    public void reportDeferredDiagnostics() {
        this.deprecationHandler.reportDeferredDiagnostic();
        this.uncheckedHandler.reportDeferredDiagnostic();
        this.sunApiHandler.reportDeferredDiagnostic();
    }

    public Type completionError(JCDiagnostic.DiagnosticPosition pos, Symbol.CompletionFailure ex) {
        this.log.error(JCDiagnostic.DiagnosticFlag.NON_DEFERRABLE, pos, "cant.access", ex.sym, ex.getDetailValue());
        if ((ex instanceof ClassReader.BadClassFile) && !this.suppressAbortOnBadClassFile) {
            throw new Abort();
        }
        return this.syms.errType;
    }

    Type typeTagError(JCDiagnostic.DiagnosticPosition pos, Object required, Object found) {
        if ((found instanceof Type) && ((Type) found).hasTag(TypeTag.VOID)) {
            this.log.error(pos, "illegal.start.of.type", new Object[0]);
            return this.syms.errType;
        }
        this.log.error(pos, "type.found.req", found, required);
        return this.types.createErrorType(found instanceof Type ? (Type) found : this.syms.errType);
    }

    void earlyRefError(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
        this.log.error(pos, "cant.ref.before.ctor.called", sym);
    }

    void duplicateError(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
        if (!sym.type.isErroneous()) {
            Symbol location = sym.location();
            if (location.kind == 16 && ((Symbol.MethodSymbol) location).isStaticOrInstanceInit()) {
                this.log.error(pos, "already.defined.in.clinit", Kinds.kindName(sym), sym, Kinds.kindName(sym.location()), Kinds.kindName(sym.location().enclClass()), sym.location().enclClass());
            } else {
                this.log.error(pos, "already.defined", Kinds.kindName(sym), sym, Kinds.kindName(sym.location()), sym.location());
            }
        }
    }

    void varargsDuplicateError(JCDiagnostic.DiagnosticPosition pos, Symbol sym1, Symbol sym2) {
        if (!sym1.type.isErroneous() && !sym2.type.isErroneous()) {
            this.log.error(pos, "array.and.varargs", sym1, sym2, sym2.location());
        }
    }

    void checkTransparentVar(JCDiagnostic.DiagnosticPosition pos, Symbol.VarSymbol v, Scope s) {
        if (s.next != null) {
            for (Scope.Entry e = s.next.lookup(v.name); e.scope != null && e.sym.owner == v.owner; e = e.next()) {
                if (e.sym.kind == 4 && (e.sym.owner.kind & 20) != 0 && v.name != this.names.error) {
                    duplicateError(pos, e.sym);
                    return;
                }
            }
        }
    }

    void checkTransparentClass(JCDiagnostic.DiagnosticPosition pos, Symbol.ClassSymbol c, Scope s) {
        if (s.next != null) {
            for (Scope.Entry e = s.next.lookup(c.name); e.scope != null && e.sym.owner == c.owner; e = e.next()) {
                if (e.sym.kind == 2 && !e.sym.type.hasTag(TypeTag.TYPEVAR) && (e.sym.owner.kind & 20) != 0 && c.name != this.names.error) {
                    duplicateError(pos, e.sym);
                    return;
                }
            }
        }
    }

    boolean checkUniqueClassName(JCDiagnostic.DiagnosticPosition pos, Name name, Scope s) {
        for (Scope.Entry e = s.lookup(name); e.scope == s; e = e.next()) {
            if (e.sym.kind == 2 && e.sym.name != this.names.error) {
                duplicateError(pos, e.sym);
                return false;
            }
        }
        for (Symbol sym = s.owner; sym != null; sym = sym.owner) {
            if (sym.kind == 2 && sym.name == name && sym.name != this.names.error) {
                duplicateError(pos, sym);
                return true;
            }
        }
        return true;
    }

    Name localClassName(Symbol.ClassSymbol c) {
        int i = 1;
        while (true) {
            Name flatname = this.names.fromString("" + ((Object) c.owner.enclClass().flatname) + this.syntheticNameChar + i + ((Object) c.name));
            if (this.compiled.get(flatname) == null) {
                return flatname;
            }
            i++;
        }
    }

    static class NestedCheckContext implements CheckContext {
        CheckContext enclosingContext;

        NestedCheckContext(CheckContext enclosingContext) {
            this.enclosingContext = enclosingContext;
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public boolean compatible(Type found, Type req, Warner warn) {
            return this.enclosingContext.compatible(found, req, warn);
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public void report(JCDiagnostic.DiagnosticPosition pos, JCDiagnostic details) {
            this.enclosingContext.report(pos, details);
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public Warner checkWarner(JCDiagnostic.DiagnosticPosition pos, Type found, Type req) {
            return this.enclosingContext.checkWarner(pos, found, req);
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public Infer.InferenceContext inferenceContext() {
            return this.enclosingContext.inferenceContext();
        }

        @Override // com.sun.tools.javac.comp.Check.CheckContext
        public DeferredAttr.DeferredAttrContext deferredAttrContext() {
            return this.enclosingContext.deferredAttrContext();
        }
    }

    Type checkType(JCDiagnostic.DiagnosticPosition pos, Type found, Type req) {
        return checkType(pos, found, req, this.basicHandler);
    }

    Type checkType(final JCDiagnostic.DiagnosticPosition pos, final Type found, final Type req, final CheckContext checkContext) {
        Infer.InferenceContext inferenceContext = checkContext.inferenceContext();
        if (inferenceContext.free(req) || inferenceContext.free(found)) {
            inferenceContext.addFreeTypeListener(List.of(req, found), new Infer.FreeTypeListener() { // from class: com.sun.tools.javac.comp.Check.2
                @Override // com.sun.tools.javac.comp.Infer.FreeTypeListener
                public void typesInferred(Infer.InferenceContext inferenceContext2) {
                    Check.this.checkType(pos, inferenceContext2.asInstType(found), inferenceContext2.asInstType(req), checkContext);
                }
            });
        }
        if (req.hasTag(TypeTag.ERROR)) {
            return req;
        }
        if (req.hasTag(TypeTag.NONE) || checkContext.compatible(found, req, checkContext.checkWarner(pos, found, req))) {
            return found;
        }
        if (found.isNumeric() && req.isNumeric()) {
            checkContext.report(pos, this.diags.fragment("possible.loss.of.precision", found, req));
            return this.types.createErrorType(found);
        }
        checkContext.report(pos, this.diags.fragment("inconvertible.types", found, req));
        return this.types.createErrorType(found);
    }

    Type checkCastable(JCDiagnostic.DiagnosticPosition pos, Type found, Type req) {
        return checkCastable(pos, found, req, this.basicHandler);
    }

    Type checkCastable(JCDiagnostic.DiagnosticPosition pos, Type found, Type req, CheckContext checkContext) {
        if (this.types.isCastable(found, req, castWarner(pos, found, req))) {
            return req;
        }
        checkContext.report(pos, this.diags.fragment("inconvertible.types", found, req));
        return this.types.createErrorType(found);
    }

    public void checkRedundantCast(Env<AttrContext> env, final JCTree.JCTypeCast tree) {
        if (!tree.type.isErroneous() && this.types.isSameType(tree.expr.type, tree.clazz.type) && !TreeInfo.containsTypeAnnotation(tree.clazz) && !is292targetTypeCast(tree)) {
            this.deferredLintHandler.report(new DeferredLintHandler.LintLogger() { // from class: com.sun.tools.javac.comp.Check.3
                @Override // com.sun.tools.javac.code.DeferredLintHandler.LintLogger
                public void report() {
                    if (Check.this.lint.isEnabled(Lint.LintCategory.CAST)) {
                        Check.this.log.warning(Lint.LintCategory.CAST, tree.pos(), "redundant.cast", tree.expr.type);
                    }
                }
            });
        }
    }

    private boolean is292targetTypeCast(JCTree.JCTypeCast tree) {
        JCTree.JCExpression expr = TreeInfo.skipParens(tree.expr);
        if (!expr.hasTag(JCTree.Tag.APPLY)) {
            return false;
        }
        JCTree.JCMethodInvocation apply = (JCTree.JCMethodInvocation) expr;
        Symbol sym = TreeInfo.symbol(apply.meth);
        boolean is292targetTypeCast = (sym == null || sym.kind != 16 || (sym.flags() & Flags.HYPOTHETICAL) == 0) ? false : true;
        return is292targetTypeCast;
    }

    private boolean checkExtends(Type a, Type bound) {
        if (a.isUnbound()) {
            return true;
        }
        if (!a.hasTag(TypeTag.WILDCARD)) {
            return this.types.isSubtype(this.types.cvarUpperBound(a), bound);
        }
        if (a.isExtendsBound()) {
            return this.types.isCastable(bound, this.types.wildUpperBound(a), this.types.noWarnings);
        }
        if (a.isSuperBound()) {
            return !this.types.notSoftSubtype(this.types.wildLowerBound(a), bound);
        }
        return true;
    }

    Type checkNonVoid(JCDiagnostic.DiagnosticPosition pos, Type t) {
        if (t.hasTag(TypeTag.VOID)) {
            this.log.error(pos, "void.not.allowed.here", new Object[0]);
            return this.types.createErrorType(t);
        }
        return t;
    }

    Type checkClassOrArrayType(JCDiagnostic.DiagnosticPosition pos, Type t) {
        if (!t.hasTag(TypeTag.CLASS) && !t.hasTag(TypeTag.ARRAY) && !t.hasTag(TypeTag.ERROR)) {
            return typeTagError(pos, this.diags.fragment("type.req.class.array", new Object[0]), asTypeParam(t));
        }
        return t;
    }

    Type checkClassType(JCDiagnostic.DiagnosticPosition pos, Type t) {
        if (!t.hasTag(TypeTag.CLASS) && !t.hasTag(TypeTag.ERROR)) {
            return typeTagError(pos, this.diags.fragment("type.req.class", new Object[0]), asTypeParam(t));
        }
        return t;
    }

    private Object asTypeParam(Type t) {
        return t.hasTag(TypeTag.TYPEVAR) ? this.diags.fragment("type.parameter", t) : t;
    }

    Type checkConstructorRefType(JCDiagnostic.DiagnosticPosition pos, Type t) {
        Type t2 = checkClassOrArrayType(pos, t);
        if (!t2.hasTag(TypeTag.CLASS)) {
            if (t2.hasTag(TypeTag.ARRAY) && !this.types.isReifiable(((Type.ArrayType) t2).elemtype)) {
                this.log.error(pos, "generic.array.creation", new Object[0]);
                return this.types.createErrorType(t2);
            }
            return t2;
        }
        if ((t2.tsym.flags() & 1536) != 0) {
            this.log.error(pos, "abstract.cant.be.instantiated", t2.tsym);
            return this.types.createErrorType(t2);
        }
        if ((t2.tsym.flags() & 16384) != 0) {
            this.log.error(pos, "enum.cant.be.instantiated", new Object[0]);
            return this.types.createErrorType(t2);
        }
        return checkClassType(pos, t2, true);
    }

    /* JADX WARN: Multi-variable type inference failed */
    Type checkClassType(JCDiagnostic.DiagnosticPosition pos, Type t, boolean noBounds) {
        Type t2 = checkClassType(pos, t);
        if (noBounds && t2.isParameterized()) {
            for (List typeArguments = t2.getTypeArguments(); typeArguments.nonEmpty(); typeArguments = typeArguments.tail) {
                if (((Type) typeArguments.head).hasTag(TypeTag.WILDCARD)) {
                    return typeTagError(pos, this.diags.fragment("type.req.exact", new Object[0]), typeArguments.head);
                }
            }
        }
        return t2;
    }

    Type checkRefType(JCDiagnostic.DiagnosticPosition pos, Type t) {
        if (t.isReference()) {
            return t;
        }
        return typeTagError(pos, this.diags.fragment("type.req.ref", new Object[0]), t);
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r2v4, types: [A, com.sun.tools.javac.code.Type] */
    List<Type> checkRefTypes(List<JCTree.JCExpression> trees, List<Type> types) {
        List list = trees;
        for (List list2 = types; list2.nonEmpty(); list2 = list2.tail) {
            list2.head = checkRefType(((JCTree.JCExpression) list.head).pos(), (Type) list2.head);
            list = list.tail;
        }
        return types;
    }

    Type checkNullOrRefType(JCDiagnostic.DiagnosticPosition pos, Type t) {
        if (t.isReference() || t.hasTag(TypeTag.BOT)) {
            return t;
        }
        return typeTagError(pos, this.diags.fragment("type.req.ref", new Object[0]), t);
    }

    boolean checkDisjoint(JCDiagnostic.DiagnosticPosition pos, long flags, long set1, long set2) {
        if ((flags & set1) != 0 && (flags & set2) != 0) {
            this.log.error(pos, "illegal.combination.of.modifiers", Flags.asFlagSet(TreeInfo.firstFlag(flags & set1)), Flags.asFlagSet(TreeInfo.firstFlag(flags & set2)));
            return false;
        }
        return true;
    }

    Type checkDiamond(JCTree.JCNewClass tree, Type t) {
        if (!TreeInfo.isDiamond(tree) || t.isErroneous()) {
            return checkClassType(tree.clazz.pos(), t, true);
        }
        if (tree.def != null) {
            this.log.error(tree.clazz.pos(), "cant.apply.diamond.1", t, this.diags.fragment("diamond.and.anon.class", t));
            return this.types.createErrorType(t);
        }
        if (t.tsym.type.getTypeArguments().isEmpty()) {
            this.log.error(tree.clazz.pos(), "cant.apply.diamond.1", t, this.diags.fragment("diamond.non.generic", t));
            return this.types.createErrorType(t);
        }
        if (tree.typeargs != null && tree.typeargs.nonEmpty()) {
            this.log.error(tree.clazz.pos(), "cant.apply.diamond.1", t, this.diags.fragment("diamond.and.explicit.params", t));
            return this.types.createErrorType(t);
        }
        return t;
    }

    void checkVarargsMethodDecl(Env<AttrContext> env, JCTree.JCMethodDecl tree) {
        Symbol.MethodSymbol m = tree.sym;
        if (this.allowSimplifiedVarargs) {
            boolean hasTrustMeAnno = m.attribute(this.syms.trustMeType.tsym) != null;
            Type varargElemType = null;
            if (m.isVarArgs()) {
                varargElemType = this.types.elemtype(tree.params.last().type);
            }
            if (hasTrustMeAnno && !isTrustMeAllowedOnMethod(m)) {
                if (varargElemType != null) {
                    this.log.error(tree, "varargs.invalid.trustme.anno", this.syms.trustMeType.tsym, this.diags.fragment("varargs.trustme.on.virtual.varargs", m));
                    return;
                } else {
                    this.log.error(tree, "varargs.invalid.trustme.anno", this.syms.trustMeType.tsym, this.diags.fragment("varargs.trustme.on.non.varargs.meth", m));
                    return;
                }
            }
            if (hasTrustMeAnno && varargElemType != null && this.types.isReifiable(varargElemType)) {
                warnUnsafeVararg(tree, "varargs.redundant.trustme.anno", this.syms.trustMeType.tsym, this.diags.fragment("varargs.trustme.on.reifiable.varargs", varargElemType));
            } else if (!hasTrustMeAnno && varargElemType != null && !this.types.isReifiable(varargElemType)) {
                warnUnchecked(tree.params.head.pos(), "unchecked.varargs.non.reifiable.type", varargElemType);
            }
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public boolean isTrustMeAllowedOnMethod(Symbol s) {
        return (s.flags() & Flags.VARARGS) != 0 && (s.isConstructor() || (s.flags() & 24) != 0);
    }

    /* JADX WARN: Multi-variable type inference failed */
    Type checkMethod(final Type mtype, final Symbol sym, final Env<AttrContext> env, final List<JCTree.JCExpression> argtrees, final List<Type> argtypes, final boolean useVarargs, Infer.InferenceContext inferenceContext) {
        if (inferenceContext.free(mtype)) {
            inferenceContext.addFreeTypeListener(List.of(mtype), new Infer.FreeTypeListener() { // from class: com.sun.tools.javac.comp.Check.4
                @Override // com.sun.tools.javac.comp.Infer.FreeTypeListener
                public void typesInferred(Infer.InferenceContext inferenceContext2) {
                    Check.this.checkMethod(inferenceContext2.asInstType(mtype), sym, env, argtrees, argtypes, useVarargs, inferenceContext2);
                }
            });
            return mtype;
        }
        List listMo176getParameterTypes = mtype.mo176getParameterTypes();
        List listMo176getParameterTypes2 = sym.type.mo176getParameterTypes();
        if (listMo176getParameterTypes2.length() != listMo176getParameterTypes.length()) {
            listMo176getParameterTypes2 = listMo176getParameterTypes;
        }
        Type last = useVarargs ? (Type) listMo176getParameterTypes.last() : null;
        if (sym.name == this.names.init && sym.owner == this.syms.enumSym) {
            listMo176getParameterTypes = listMo176getParameterTypes.tail.tail;
            listMo176getParameterTypes2 = listMo176getParameterTypes2.tail.tail;
        }
        List list = argtrees;
        if (list != null) {
            while (listMo176getParameterTypes.head != last) {
                JCTree arg = (JCTree) list.head;
                Warner warn = convertWarner(arg.pos(), arg.type, (Type) listMo176getParameterTypes2.head);
                assertConvertible(arg, arg.type, (Type) listMo176getParameterTypes.head, warn);
                list = list.tail;
                listMo176getParameterTypes = listMo176getParameterTypes.tail;
                listMo176getParameterTypes2 = listMo176getParameterTypes2.tail;
            }
            if (!useVarargs) {
                if ((sym.flags() & 70385924046848L) == Flags.VARARGS && this.allowVarargs) {
                    Type varParam = mtype.mo176getParameterTypes().last();
                    Type lastArg = argtypes.last();
                    if (this.types.isSubtypeUnchecked(lastArg, this.types.elemtype(varParam)) && !this.types.isSameType(this.types.erasure(varParam), this.types.erasure(lastArg))) {
                        this.log.warning(argtrees.last().pos(), "inexact.non-varargs.call", this.types.elemtype(varParam), varParam);
                    }
                }
            } else {
                Type varArg = this.types.elemtype(last);
                while (list.tail != null) {
                    JCTree arg2 = (JCTree) list.head;
                    Warner warn2 = convertWarner(arg2.pos(), arg2.type, varArg);
                    assertConvertible(arg2, arg2.type, varArg, warn2);
                    list = list.tail;
                }
            }
        }
        if (useVarargs) {
            Type argtype = mtype.mo176getParameterTypes().last();
            if (!this.types.isReifiable(argtype) && (!this.allowSimplifiedVarargs || sym.attribute(this.syms.trustMeType.tsym) == null || !isTrustMeAllowedOnMethod(sym))) {
                warnUnchecked(env.tree.pos(), "unchecked.generic.array.creation", argtype);
            }
            if ((sym.baseSymbol().flags() & Flags.SIGNATURE_POLYMORPHIC) == 0) {
                TreeInfo.setVarargsElement(env.tree, this.types.elemtype(argtype));
            }
        }
        JCTree.JCPolyExpression.PolyKind pkind = (sym.type.hasTag(TypeTag.FORALL) && sym.type.mo178getReturnType().containsAny(((Type.ForAll) sym.type).tvars)) ? JCTree.JCPolyExpression.PolyKind.POLY : JCTree.JCPolyExpression.PolyKind.STANDALONE;
        TreeInfo.setPolyKind(env.tree, pkind);
        return mtype;
    }

    private void assertConvertible(JCTree tree, Type actual, Type formal, Warner warn) {
        if (!this.types.isConvertible(actual, formal, warn) && formal.isCompound() && this.types.isSubtype(actual, this.types.supertype(formal)) && this.types.isSubtypeUnchecked(actual, this.types.interfaces(formal), warn)) {
        }
    }

    public boolean checkValidGenericType(Type t) {
        return firstIncompatibleTypeArg(t) == null;
    }

    /* JADX INFO: Access modifiers changed from: private */
    /* JADX WARN: Multi-variable type inference failed */
    public Type firstIncompatibleTypeArg(Type type) {
        List<Type> formals = type.tsym.type.allparams();
        List<Type> actuals = type.allparams();
        List typeArguments = type.getTypeArguments();
        ListBuffer<Type> bounds_buf = new ListBuffer<>();
        for (List typeArguments2 = type.tsym.type.getTypeArguments(); typeArguments.nonEmpty() && typeArguments2.nonEmpty(); typeArguments2 = typeArguments2.tail) {
            bounds_buf.append(this.types.subst(((Type) typeArguments2.head).getUpperBound(), formals, actuals));
            typeArguments = typeArguments.tail;
        }
        List typeArguments3 = type.getTypeArguments();
        for (List listSubstBounds = this.types.substBounds(formals, formals, this.types.capture(type).allparams()); typeArguments3.nonEmpty() && listSubstBounds.nonEmpty(); listSubstBounds = listSubstBounds.tail) {
            ((Type) typeArguments3.head).withTypeVar((Type.TypeVar) listSubstBounds.head);
            typeArguments3 = typeArguments3.tail;
        }
        List typeArguments4 = type.getTypeArguments();
        for (List list = bounds_buf.toList(); typeArguments4.nonEmpty() && list.nonEmpty(); list = list.tail) {
            Type actual = (Type) typeArguments4.head;
            if (!isTypeArgErroneous(actual) && !((Type) list.head).isErroneous() && !checkExtends(actual, (Type) list.head)) {
                return (Type) typeArguments4.head;
            }
            typeArguments4 = typeArguments4.tail;
        }
        List typeArguments5 = type.getTypeArguments();
        List list2 = bounds_buf.toList();
        for (Type arg : this.types.capture(type).getTypeArguments()) {
            if (arg.hasTag(TypeTag.TYPEVAR) && arg.getUpperBound().isErroneous() && !((Type) list2.head).isErroneous() && !isTypeArgErroneous((Type) typeArguments5.head)) {
                return (Type) typeArguments5.head;
            }
            list2 = list2.tail;
            typeArguments5 = typeArguments5.tail;
        }
        return null;
    }

    boolean isTypeArgErroneous(Type t) {
        return this.isTypeArgErroneous.visit(t).booleanValue();
    }

    long checkFlags(JCDiagnostic.DiagnosticPosition pos, long flags, Symbol sym, JCTree tree) {
        long mask;
        long implicit;
        long mask2;
        long mask3;
        long implicit2 = 0;
        switch (sym.kind) {
            case 2:
                if (sym.isLocal()) {
                    long mask4 = 23568;
                    if (sym.name.isEmpty()) {
                        mask4 = 23568 | 8;
                        implicit2 = 0 | 16;
                    }
                    if ((8 & sym.owner.flags_field) == 0 && (flags & 16384) != 0) {
                        this.log.error(pos, "enums.must.be.static", new Object[0]);
                    }
                    mask = mask4;
                } else if (sym.owner.kind == 2) {
                    if (sym.owner.owner.kind == 1 || (sym.owner.flags_field & 8) != 0) {
                        mask = 24087 | 8;
                    } else {
                        if ((flags & 16384) != 0) {
                            this.log.error(pos, "enums.must.be.static", new Object[0]);
                        }
                        mask = 24087;
                    }
                    if ((flags & 16896) != 0) {
                        implicit2 = 8;
                    }
                } else {
                    mask = 32273;
                }
                if ((flags & 512) != 0) {
                    implicit2 |= 1024;
                }
                if ((flags & 16384) != 0) {
                    mask &= -1041;
                    implicit2 |= implicitEnumFinalFlag(tree);
                }
                implicit = implicit2 | (sym.owner.flags_field & 2048);
                mask2 = mask;
                break;
            case 4:
                if (TreeInfo.isReceiverParam(tree)) {
                    implicit = 0;
                    mask2 = 8589934592L;
                } else if (sym.owner.kind != 2) {
                    implicit = 0;
                    mask2 = 8589934608L;
                } else if ((sym.owner.flags_field & 512) != 0) {
                    implicit = 25;
                    mask2 = 25;
                } else {
                    implicit = 0;
                    mask2 = 16607;
                }
                break;
            case 16:
                if (sym.name == this.names.init) {
                    if ((sym.owner.flags_field & 16384) != 0) {
                        implicit2 = 2;
                        mask3 = 2;
                    } else {
                        mask3 = 7;
                    }
                } else if ((sym.owner.flags_field & 512) == 0) {
                    mask3 = 3391;
                } else if ((sym.owner.flags_field & 8192) != 0) {
                    mask3 = Flags.AnnotationTypeElementMask;
                    implicit2 = Flags.AnnotationTypeElementMask;
                } else if ((flags & 8796093022216L) != 0) {
                    mask3 = Flags.InterfaceMethodMask;
                    implicit2 = 1;
                    if ((flags & Flags.DEFAULT) != 0) {
                        implicit2 = 1 | 1024;
                    }
                } else {
                    mask3 = Flags.AnnotationTypeElementMask;
                    implicit2 = 1025;
                }
                if (((flags | implicit2) & 1024) != 0 && (flags & Flags.DEFAULT) == 0) {
                    implicit = implicit2;
                    mask2 = mask3;
                } else {
                    implicit = implicit2 | (sym.owner.flags_field & 2048);
                    mask2 = mask3;
                }
                break;
            default:
                throw new AssertionError();
        }
        long illegal = flags & Flags.ExtendedStandardFlags & (~mask2);
        if (illegal != 0) {
            if ((illegal & 512) != 0) {
                this.log.error(pos, "intf.not.allowed.here", new Object[0]);
                mask2 |= 512;
            } else {
                this.log.error(pos, "mod.not.allowed.here", Flags.asFlagSet(illegal));
            }
        } else if ((sym.kind == 2 || checkDisjoint(pos, flags, 1024L, 8796093022218L)) && checkDisjoint(pos, flags, 8L, Flags.DEFAULT) && checkDisjoint(pos, flags, 1536L, 304L) && checkDisjoint(pos, flags, 1L, 6L) && checkDisjoint(pos, flags, 2L, 5L) && checkDisjoint(pos, flags, 16L, 64L) && sym.kind != 2) {
            checkDisjoint(pos, flags, 1280L, 2048L);
        }
        return (flags & ((-8796093026304L) | mask2)) | implicit;
    }

    private long implicitEnumFinalFlag(JCTree tree) {
        if (!tree.hasTag(JCTree.Tag.CLASSDEF)) {
            return 0L;
        }
        C1SpecialTreeVisitor sts = new C1SpecialTreeVisitor();
        JCTree.JCClassDecl cdef = (JCTree.JCClassDecl) tree;
        for (JCTree defs : cdef.defs) {
            defs.accept(sts);
            if (sts.specialized) {
                return 0L;
            }
        }
        return 16L;
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.comp.Check$1SpecialTreeVisitor, reason: invalid class name */
    class C1SpecialTreeVisitor extends JCTree.Visitor {
        boolean specialized = false;

        C1SpecialTreeVisitor() {
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTree(JCTree tree) {
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            if ((tree.mods.flags & 16384) != 0 && (tree.init instanceof JCTree.JCNewClass) && ((JCTree.JCNewClass) tree.init).def != null) {
                this.specialized = true;
            }
        }
    }

    void validate(JCTree tree, Env<AttrContext> env) {
        validate(tree, env, true);
    }

    void validate(JCTree tree, Env<AttrContext> env, boolean checkRaw) {
        new Validator(env).validateTree(tree, checkRaw, true);
    }

    /* JADX WARN: Multi-variable type inference failed */
    void validate(List<? extends JCTree> trees, Env<AttrContext> env) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            validate((JCTree) list.head, env);
        }
    }

    class Validator extends JCTree.Visitor {
        boolean checkRaw;
        Env<AttrContext> env;
        boolean isOuter;

        Validator(Env<AttrContext> env) {
            this.env = env;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeArray(JCTree.JCArrayTypeTree tree) {
            validateTree(tree.elemtype, this.checkRaw, this.isOuter);
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeApply(JCTree.JCTypeApply tree) {
            if (tree.type.hasTag(TypeTag.CLASS)) {
                List list = tree.arguments;
                List typeArguments = tree.type.tsym.type.getTypeArguments();
                Type incompatibleArg = Check.this.firstIncompatibleTypeArg(tree.type);
                if (incompatibleArg != null) {
                    for (JCTree arg : tree.arguments) {
                        if (arg.type == incompatibleArg) {
                            Check.this.log.error(arg, "not.within.bounds", incompatibleArg, typeArguments.head);
                        }
                        typeArguments = typeArguments.tail;
                    }
                }
                boolean is_java_lang_Class = tree.type.tsym.flatName() == Check.this.names.java_lang_Class;
                for (List typeArguments2 = tree.type.tsym.type.getTypeArguments(); list.nonEmpty() && typeArguments2.nonEmpty(); typeArguments2 = typeArguments2.tail) {
                    validateTree((JCTree) list.head, (this.isOuter && is_java_lang_Class) ? false : true, false);
                    list = list.tail;
                }
                if (tree.type.getEnclosingType().isRaw()) {
                    Check.this.log.error(tree.pos(), "improperly.formed.type.inner.raw.param", new Object[0]);
                }
                if (tree.clazz.hasTag(JCTree.Tag.SELECT)) {
                    visitSelectInternal((JCTree.JCFieldAccess) tree.clazz);
                }
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeParameter(JCTree.JCTypeParameter tree) {
            validateTrees(tree.bounds, true, this.isOuter);
            Check.this.checkClassBounds(tree.pos(), tree.type);
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitWildcard(JCTree.JCWildcard tree) {
            if (tree.inner != null) {
                validateTree(tree.inner, true, this.isOuter);
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            if (tree.type.hasTag(TypeTag.CLASS)) {
                visitSelectInternal(tree);
                if (tree.selected.type.isParameterized() && tree.type.tsym.type.getTypeArguments().nonEmpty()) {
                    Check.this.log.error(tree.pos(), "improperly.formed.type.param.missing", new Object[0]);
                }
            }
        }

        public void visitSelectInternal(JCTree.JCFieldAccess tree) {
            if (tree.type.tsym.isStatic() && tree.selected.type.isParameterized()) {
                Check.this.log.error(tree.pos(), "cant.select.static.class.from.param.type", new Object[0]);
            } else {
                tree.selected.accept(this);
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
            tree.underlyingType.accept(this);
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeIdent(JCTree.JCPrimitiveTypeTree that) {
            if (that.type.hasTag(TypeTag.VOID)) {
                Check.this.log.error(that.pos(), "void.not.allowed.here", new Object[0]);
            }
            super.visitTypeIdent(that);
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTree(JCTree tree) {
        }

        public void validateTree(JCTree tree, boolean checkRaw, boolean isOuter) {
            if (tree != null) {
                boolean prevCheckRaw = this.checkRaw;
                this.checkRaw = checkRaw;
                this.isOuter = isOuter;
                try {
                    try {
                        tree.accept(this);
                        if (checkRaw) {
                            Check.this.checkRaw(tree, this.env);
                        }
                    } catch (Symbol.CompletionFailure ex) {
                        Check.this.completionError(tree.pos(), ex);
                    }
                } finally {
                    this.checkRaw = prevCheckRaw;
                }
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        public void validateTrees(List<? extends JCTree> trees, boolean checkRaw, boolean isOuter) {
            for (List list = trees; list.nonEmpty(); list = list.tail) {
                validateTree((JCTree) list.head, checkRaw, isOuter);
            }
        }
    }

    void checkRaw(JCTree tree, Env<AttrContext> env) {
        if (this.lint.isEnabled(Lint.LintCategory.RAW) && tree.type.hasTag(TypeTag.CLASS) && !TreeInfo.isDiamond(tree) && !withinAnonConstr(env) && tree.type.isRaw()) {
            this.log.warning(Lint.LintCategory.RAW, tree.pos(), "raw.class.use", tree.type, tree.type.tsym.type);
        }
    }

    private boolean withinAnonConstr(Env<AttrContext> env) {
        return env.enclClass.name.isEmpty() && env.enclMethod != null && env.enclMethod.name == this.names.init;
    }

    /* JADX WARN: Multi-variable type inference failed */
    boolean subset(Type t, List<Type> ts) {
        for (List list = ts; list.nonEmpty(); list = list.tail) {
            if (this.types.isSubtype(t, (Type) list.head)) {
                return true;
            }
        }
        return false;
    }

    /* JADX WARN: Multi-variable type inference failed */
    boolean intersects(Type t, List<Type> ts) {
        for (List list = ts; list.nonEmpty(); list = list.tail) {
            if (this.types.isSubtype(t, (Type) list.head) || this.types.isSubtype((Type) list.head, t)) {
                return true;
            }
        }
        return false;
    }

    List<Type> incl(Type t, List<Type> ts) {
        return subset(t, ts) ? ts : excl(t, ts).prepend(t);
    }

    List<Type> excl(Type t, List<Type> ts) {
        if (ts.isEmpty()) {
            return ts;
        }
        List<Type> ts1 = excl(t, ts.tail);
        return this.types.isSubtype(ts.head, t) ? ts1 : ts1 == ts.tail ? ts : ts1.prepend(ts.head);
    }

    /* JADX WARN: Multi-variable type inference failed */
    List<Type> union(List<Type> ts1, List<Type> ts2) {
        List<Type> ts = ts1;
        for (List list = ts2; list.nonEmpty(); list = list.tail) {
            ts = incl((Type) list.head, ts);
        }
        return ts;
    }

    /* JADX WARN: Multi-variable type inference failed */
    List<Type> diff(List<Type> ts1, List<Type> ts2) {
        List<Type> ts = ts1;
        for (List list = ts2; list.nonEmpty(); list = list.tail) {
            ts = excl((Type) list.head, ts);
        }
        return ts;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public List<Type> intersect(List<Type> ts1, List<Type> ts2) {
        List<Type> ts = List.nil();
        for (List list = ts1; list.nonEmpty(); list = list.tail) {
            if (subset((Type) list.head, ts2)) {
                ts = incl((Type) list.head, ts);
            }
        }
        for (List list2 = ts2; list2.nonEmpty(); list2 = list2.tail) {
            if (subset((Type) list2.head, ts1)) {
                ts = incl((Type) list2.head, ts);
            }
        }
        return ts;
    }

    boolean isUnchecked(Symbol.ClassSymbol exc) {
        return exc.kind == 63 || exc.isSubClass(this.syms.errorType.tsym, this.types) || exc.isSubClass(this.syms.runtimeExceptionType.tsym, this.types);
    }

    boolean isUnchecked(Type exc) {
        return exc.hasTag(TypeTag.TYPEVAR) ? isUnchecked(this.types.supertype(exc)) : exc.hasTag(TypeTag.CLASS) ? isUnchecked((Symbol.ClassSymbol) exc.tsym) : exc.hasTag(TypeTag.BOT);
    }

    boolean isUnchecked(JCDiagnostic.DiagnosticPosition pos, Type exc) {
        try {
            return isUnchecked(exc);
        } catch (Symbol.CompletionFailure ex) {
            completionError(pos, ex);
            return true;
        }
    }

    boolean isHandled(Type exc, List<Type> handled) {
        return isUnchecked(exc) || subset(exc, handled);
    }

    /* JADX WARN: Multi-variable type inference failed */
    List<Type> unhandled(List<Type> list, List<Type> list2) {
        List list3 = list;
        List listPrepend = List.nil();
        while (list3.nonEmpty()) {
            if (!isHandled((Type) list3.head, list2)) {
                listPrepend = listPrepend.prepend(list3.head);
            }
            list3 = list3.tail;
            listPrepend = listPrepend;
        }
        return listPrepend;
    }

    static int protection(long flags) {
        switch ((short) (7 & flags)) {
            case 0:
                return 2;
            case 1:
            case 3:
            default:
                return 0;
            case 2:
                return 3;
            case 4:
                return 1;
        }
    }

    Object cannotOverride(Symbol.MethodSymbol m, Symbol.MethodSymbol other) {
        String key;
        if ((other.owner.flags() & 512) == 0) {
            key = "cant.override";
        } else if ((m.owner.flags() & 512) == 0) {
            key = "cant.implement";
        } else {
            key = "clashes.with";
        }
        return this.diags.fragment(key, m, m.location(), other, other.location());
    }

    Object uncheckedOverrides(Symbol.MethodSymbol m, Symbol.MethodSymbol other) {
        String key;
        if ((other.owner.flags() & 512) == 0) {
            key = "unchecked.override";
        } else if ((m.owner.flags() & 512) == 0) {
            key = "unchecked.implement";
        } else {
            key = "unchecked.clash.with";
        }
        return this.diags.fragment(key, m, m.location(), other, other.location());
    }

    Object varargsOverrides(Symbol.MethodSymbol m, Symbol.MethodSymbol other) {
        String key;
        if ((other.owner.flags() & 512) == 0) {
            key = "varargs.override";
        } else if ((m.owner.flags() & 512) == 0) {
            key = "varargs.implement";
        } else {
            key = "varargs.clash.with";
        }
        return this.diags.fragment(key, m, m.location(), other, other.location());
    }

    void checkOverride(JCTree tree, Symbol.MethodSymbol m, Symbol.MethodSymbol other, Symbol.ClassSymbol origin) {
        if ((m.flags() & 2147487744L) == 0 && (other.flags() & 4096) == 0) {
            if ((m.flags() & 8) != 0 && (other.flags() & 8) == 0) {
                this.log.error(TreeInfo.diagnosticPositionFor(m, tree), "override.static", cannotOverride(m, other));
                m.flags_field = 35184372088832L | m.flags_field;
                return;
            }
            if ((other.flags() & 16) != 0 || ((m.flags() & 8) == 0 && (8 & other.flags()) != 0)) {
                this.log.error(TreeInfo.diagnosticPositionFor(m, tree), "override.meth", cannotOverride(m, other), Flags.asFlagSet(other.flags() & 24));
                m.flags_field |= 35184372088832L;
                return;
            }
            if ((m.owner.flags() & 8192) != 0) {
                return;
            }
            if ((origin.flags() & 512) == 0 && protection(m.flags()) > protection(other.flags())) {
                this.log.error(TreeInfo.diagnosticPositionFor(m, tree), "override.weaker.access", cannotOverride(m, other), other.flags() == 0 ? "package" : Flags.asFlagSet(other.flags() & 7));
                m.flags_field = 35184372088832L | m.flags_field;
                return;
            }
            Type mt = this.types.memberType(origin.type, m);
            Type ot = this.types.memberType(origin.type, other);
            List<Type> mtvars = mt.getTypeArguments();
            List<Type> otvars = ot.getTypeArguments();
            Type mtres = mt.mo178getReturnType();
            Type otres = this.types.subst(ot.mo178getReturnType(), otvars, mtvars);
            this.overrideWarner.clear();
            boolean resultTypesOK = this.types.returnTypeSubstitutable(mt, ot, otres, this.overrideWarner);
            if (!resultTypesOK) {
                if (this.allowCovariantReturns || m.owner == origin || !m.owner.isSubClass(other.owner, this.types)) {
                    this.log.error(TreeInfo.diagnosticPositionFor(m, tree), "override.incompatible.ret", cannotOverride(m, other), mtres, otres);
                    m.flags_field |= 35184372088832L;
                    return;
                }
            } else if (this.overrideWarner.hasNonSilentLint(Lint.LintCategory.UNCHECKED)) {
                warnUnchecked(TreeInfo.diagnosticPositionFor(m, tree), "override.unchecked.ret", uncheckedOverrides(m, other), mtres, otres);
            }
            List<Type> otthrown = this.types.subst(ot.mo179getThrownTypes(), otvars, mtvars);
            List<Type> unhandledErased = unhandled(mt.mo179getThrownTypes(), this.types.erasure(otthrown));
            List<Type> unhandledUnerased = unhandled(mt.mo179getThrownTypes(), otthrown);
            if (unhandledErased.nonEmpty()) {
                this.log.error(TreeInfo.diagnosticPositionFor(m, tree), "override.meth.doesnt.throw", cannotOverride(m, other), unhandledUnerased.head);
                m.flags_field |= 35184372088832L;
                return;
            }
            if (unhandledUnerased.nonEmpty()) {
                warnUnchecked(TreeInfo.diagnosticPositionFor(m, tree), "override.unchecked.thrown", cannotOverride(m, other), unhandledUnerased.head);
                return;
            }
            if (((m.flags() ^ other.flags()) & Flags.VARARGS) != 0 && this.lint.isEnabled(Lint.LintCategory.OVERRIDES)) {
                this.log.warning(TreeInfo.diagnosticPositionFor(m, tree), (m.flags() & Flags.VARARGS) != 0 ? "override.varargs.missing" : "override.varargs.extra", varargsOverrides(m, other));
            }
            if ((other.flags() & Flags.BRIDGE) != 0) {
                this.log.warning(TreeInfo.diagnosticPositionFor(m, tree), "override.bridge", uncheckedOverrides(m, other));
            }
            if (!isDeprecatedOverrideIgnorable(other, origin)) {
                Lint prevLint = setLint(this.lint.augment(m));
                try {
                    checkDeprecated(TreeInfo.diagnosticPositionFor(m, tree), m, other);
                } finally {
                    setLint(prevLint);
                }
            }
        }
    }

    private boolean isDeprecatedOverrideIgnorable(Symbol.MethodSymbol m, Symbol.ClassSymbol origin) {
        Symbol.ClassSymbol mc = m.enclClass();
        Type st = this.types.supertype(origin.type);
        if (!st.hasTag(TypeTag.CLASS)) {
            return true;
        }
        Symbol.MethodSymbol stimpl = m.implementation((Symbol.ClassSymbol) st.tsym, this.types, false);
        if (mc == null || (mc.flags() & 512) == 0) {
            return stimpl != m;
        }
        List<Type> intfs = this.types.interfaces(origin.type);
        return (intfs.contains(mc.type) || stimpl == null) ? false : true;
    }

    public void checkCompatibleConcretes(JCDiagnostic.DiagnosticPosition pos, Type site) {
        long j;
        Type sup = this.types.supertype(site);
        if (sup.hasTag(TypeTag.CLASS)) {
            Type t1 = sup;
            while (t1.hasTag(TypeTag.CLASS) && t1.tsym.type.isParameterized()) {
                for (Scope.Entry e1 = t1.tsym.members().elems; e1 != null; e1 = e1.sibling) {
                    Symbol s1 = e1.sym;
                    int i = 16;
                    if (s1.kind == 16) {
                        long j2 = 2147487752L;
                        long j3 = 0;
                        if ((s1.flags() & 2147487752L) == 0 && s1.isInheritedIn(site.tsym, this.types) && ((Symbol.MethodSymbol) s1).implementation(site.tsym, this.types, true) == s1) {
                            Type st1 = this.types.memberType(t1, s1);
                            int s1ArgsLength = st1.mo176getParameterTypes().length();
                            if (st1 != s1.type) {
                                Type t2 = sup;
                                while (t2.hasTag(TypeTag.CLASS)) {
                                    Scope.Entry e2 = t2.tsym.members().lookup(s1.name);
                                    while (e2.scope != null) {
                                        Symbol s2 = e2.sym;
                                        if (s2 == s1) {
                                            j = j3;
                                        } else if (s2.kind == i) {
                                            j = 0;
                                            if ((s2.flags() & j2) == 0 && s2.type.mo176getParameterTypes().length() == s1ArgsLength && s2.isInheritedIn(site.tsym, this.types) && ((Symbol.MethodSymbol) s2).implementation(site.tsym, this.types, true) == s2) {
                                                Type st2 = this.types.memberType(t2, s2);
                                                if (this.types.overrideEquivalent(st1, st2)) {
                                                    this.log.error(pos, "concrete.inheritance.conflict", s1, t1, s2, t2, sup);
                                                }
                                            }
                                        } else {
                                            j = 0;
                                        }
                                        e2 = e2.next();
                                        j3 = j;
                                        i = 16;
                                        j2 = 2147487752L;
                                    }
                                    t2 = this.types.supertype(t2);
                                    i = 16;
                                    j2 = 2147487752L;
                                }
                            }
                        }
                    }
                }
                t1 = this.types.supertype(t1);
            }
        }
    }

    public boolean checkCompatibleAbstracts(JCDiagnostic.DiagnosticPosition pos, Type t1, Type t2) {
        return checkCompatibleAbstracts(pos, t1, t2, this.types.makeIntersectionType(t1, t2));
    }

    public boolean checkCompatibleAbstracts(JCDiagnostic.DiagnosticPosition pos, Type t1, Type t2, Type site) {
        if ((site.tsym.flags() & 16777216) != 0) {
            t1 = this.types.capture(t1);
            t2 = this.types.capture(t2);
        }
        return firstIncompatibility(pos, t1, t2, site) == null;
    }

    private Symbol firstIncompatibility(JCDiagnostic.DiagnosticPosition pos, Type t1, Type t2, Type site) {
        Map<Symbol.TypeSymbol, Type> interfaces2;
        Map<Symbol.TypeSymbol, Type> interfaces1 = new HashMap<>();
        closure(t1, interfaces1);
        if (t1 == t2) {
            interfaces2 = interfaces1;
        } else {
            interfaces2 = new HashMap<>();
            closure(t2, interfaces1, interfaces2);
        }
        for (Type t3 : interfaces1.values()) {
            for (Type t4 : interfaces2.values()) {
                Symbol s = firstDirectIncompatibility(pos, t3, t4, site);
                if (s != null) {
                    return s;
                }
            }
        }
        return null;
    }

    private void closure(Type t, Map<Symbol.TypeSymbol, Type> typeMap) {
        if (t.hasTag(TypeTag.CLASS) && typeMap.put(t.tsym, t) == null) {
            closure(this.types.supertype(t), typeMap);
            for (Type i : this.types.interfaces(t)) {
                closure(i, typeMap);
            }
        }
    }

    private void closure(Type t, Map<Symbol.TypeSymbol, Type> typesSkip, Map<Symbol.TypeSymbol, Type> typeMap) {
        if (t.hasTag(TypeTag.CLASS) && typesSkip.get(t.tsym) == null && typeMap.put(t.tsym, t) == null) {
            closure(this.types.supertype(t), typesSkip, typeMap);
            for (Type i : this.types.interfaces(t)) {
                closure(i, typesSkip, typeMap);
            }
        }
    }

    /* JADX WARN: Removed duplicated region for block: B:55:0x0133  */
    /* JADX WARN: Removed duplicated region for block: B:71:0x00fa A[SYNTHETIC] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private com.sun.tools.javac.code.Symbol firstDirectIncompatibility(com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition r26, com.sun.tools.javac.code.Type r27, com.sun.tools.javac.code.Type r28, com.sun.tools.javac.code.Type r29) {
        /*
            Method dump skipped, instruction units count: 381
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Check.firstDirectIncompatibility(com.sun.tools.javac.util.JCDiagnostic$DiagnosticPosition, com.sun.tools.javac.code.Type, com.sun.tools.javac.code.Type, com.sun.tools.javac.code.Type):com.sun.tools.javac.code.Symbol");
    }

    boolean checkCommonOverriderIn(Symbol s1, Symbol s2, Type site) {
        Map<Symbol.TypeSymbol, Type> supertypes = new HashMap<>();
        Type st1 = this.types.memberType(site, s1);
        Type st2 = this.types.memberType(site, s2);
        closure(site, supertypes);
        for (Type t : supertypes.values()) {
            for (Scope.Entry e = t.tsym.members().lookup(s1.name); e.scope != null; e = e.next()) {
                Symbol s3 = e.sym;
                if (s3 != s1 && s3 != s2 && s3.kind == 16 && (s3.flags() & 2147487744L) == 0) {
                    Type st3 = this.types.memberType(site, s3);
                    if (this.types.overrideEquivalent(st3, st1) && this.types.overrideEquivalent(st3, st2) && this.types.returnTypeSubstitutable(st3, st1) && this.types.returnTypeSubstitutable(st3, st2)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void checkOverride(JCTree.JCMethodDecl tree, Symbol.MethodSymbol m) {
        Symbol.ClassSymbol origin = (Symbol.ClassSymbol) m.owner;
        if ((origin.flags() & 16384) != 0 && this.names.finalize.equals(m.name) && m.overrides(this.syms.enumFinalFinalize, origin, this.types, false)) {
            this.log.error(tree.pos(), "enum.no.finalize", new Object[0]);
            return;
        }
        Type t = origin.type;
        while (t.hasTag(TypeTag.CLASS)) {
            if (t != origin.type) {
                checkOverride(tree, t, origin, m);
            }
            for (Type t2 : this.types.interfaces(t)) {
                checkOverride(tree, t2, origin, m);
            }
            t = this.types.supertype(t);
        }
        if (m.attribute(this.syms.overrideType.tsym) != null && !isOverrider(m)) {
            JCDiagnostic.DiagnosticPosition pos = tree.pos();
            Iterator<JCTree.JCAnnotation> it = tree.getModifiers().annotations.iterator();
            while (true) {
                if (!it.hasNext()) {
                    break;
                }
                JCTree.JCAnnotation a = it.next();
                if (a.annotationType.type.tsym == this.syms.overrideType.tsym) {
                    pos = a.pos();
                    break;
                }
            }
            this.log.error(pos, "method.does.not.override.superclass", new Object[0]);
        }
    }

    void checkOverride(JCTree tree, Type site, Symbol.ClassSymbol origin, Symbol.MethodSymbol m) {
        Symbol.TypeSymbol c = site.tsym;
        for (Scope.Entry e = c.members().lookup(m.name); e.scope != null; e = e.next()) {
            if (m.overrides(e.sym, origin, this.types, false) && (e.sym.flags() & 1024) == 0) {
                checkOverride(tree, m, (Symbol.MethodSymbol) e.sym, origin);
            }
        }
    }

    public void checkClassOverrideEqualsAndHashIfNeeded(JCDiagnostic.DiagnosticPosition pos, Symbol.ClassSymbol someClass) {
        List<Type> interfaces;
        if (someClass == ((Symbol.ClassSymbol) this.syms.objectType.tsym) || someClass.isInterface() || someClass.isEnum() || (someClass.flags() & 8192) != 0 || (someClass.flags() & 1024) != 0) {
            return;
        }
        if (someClass.isAnonymous() && (interfaces = this.types.interfaces(someClass.type)) != null && !interfaces.isEmpty() && interfaces.head.tsym == this.syms.comparatorType.tsym) {
            return;
        }
        checkClassOverrideEqualsAndHash(pos, someClass);
    }

    private void checkClassOverrideEqualsAndHash(JCDiagnostic.DiagnosticPosition pos, Symbol.ClassSymbol someClass) {
        if (this.lint.isEnabled(Lint.LintCategory.OVERRIDES)) {
            Symbol.MethodSymbol equalsAtObject = (Symbol.MethodSymbol) this.syms.objectType.tsym.members().lookup(this.names.equals).sym;
            Symbol.MethodSymbol hashCodeAtObject = (Symbol.MethodSymbol) this.syms.objectType.tsym.members().lookup(this.names.hashCode).sym;
            boolean overridesEquals = this.types.implementation(equalsAtObject, someClass, false, this.equalsHasCodeFilter).owner == someClass;
            boolean overridesHashCode = this.types.implementation(hashCodeAtObject, someClass, false, this.equalsHasCodeFilter) != hashCodeAtObject;
            if (overridesEquals && !overridesHashCode) {
                this.log.warning(Lint.LintCategory.OVERRIDES, pos, "override.equals.but.not.hashcode", someClass);
            }
        }
    }

    private boolean checkNameClash(Symbol.ClassSymbol origin, Symbol s1, Symbol s2) {
        ClashFilter cf = new ClashFilter(origin.type);
        return cf.accepts(s1) && cf.accepts(s2) && this.types.hasSameArgs(s1.erasure(this.types), s2.erasure(this.types));
    }

    void checkAllDefined(JCDiagnostic.DiagnosticPosition pos, Symbol.ClassSymbol c) {
        Symbol.MethodSymbol undef = this.types.firstUnimplementedAbstract(c);
        if (undef != null) {
            Symbol.MethodSymbol undef1 = new Symbol.MethodSymbol(undef.flags(), undef.name, this.types.memberType(c.type, undef), undef.owner);
            this.log.error(pos, "does.not.override.abstract", c, undef1, undef1.location());
        }
    }

    void checkNonCyclicDecl(JCTree.JCClassDecl tree) {
        CycleChecker cc = new CycleChecker();
        cc.scan(tree);
        if (!cc.errorFound && !cc.partialCheck) {
            tree.sym.flags_field |= 1073741824;
        }
    }

    class CycleChecker extends TreeScanner {
        List<Symbol> seenClasses = List.nil();
        boolean errorFound = false;
        boolean partialCheck = false;

        CycleChecker() {
        }

        private void checkSymbol(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
            if (sym != null && sym.kind == 2) {
                Env<AttrContext> classEnv = Check.this.enter.getEnv((Symbol.TypeSymbol) sym);
                if (classEnv != null) {
                    DiagnosticSource prevSource = Check.this.log.currentSource();
                    try {
                        Check.this.log.useSource(classEnv.toplevel.sourcefile);
                        scan(classEnv.tree);
                        return;
                    } finally {
                        Check.this.log.useSource(prevSource.getFile());
                    }
                }
                if (sym.kind == 2) {
                    checkClass(pos, sym, List.nil());
                    return;
                }
                return;
            }
            this.partialCheck = true;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            super.visitSelect(tree);
            checkSymbol(tree.pos(), tree.sym);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            checkSymbol(tree.pos(), tree.sym);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeApply(JCTree.JCTypeApply tree) {
            scan(tree.clazz);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeArray(JCTree.JCArrayTypeTree tree) {
            scan(tree.elemtype);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            List<JCTree> supertypes = List.nil();
            if (tree.getExtendsClause() != null) {
                supertypes = supertypes.prepend(tree.getExtendsClause());
            }
            if (tree.getImplementsClause() != null) {
                for (JCTree intf : tree.getImplementsClause()) {
                    supertypes = supertypes.prepend(intf);
                }
            }
            checkClass(tree.pos(), tree.sym, supertypes);
        }

        void checkClass(JCDiagnostic.DiagnosticPosition pos, Symbol c, List<JCTree> supertypes) {
            if ((c.flags_field & 1073741824) != 0) {
                return;
            }
            if (this.seenClasses.contains(c)) {
                this.errorFound = true;
                Check.this.noteCyclic(pos, (Symbol.ClassSymbol) c);
                return;
            }
            if (!c.type.isErroneous()) {
                try {
                    this.seenClasses = this.seenClasses.prepend(c);
                    if (c.type.hasTag(TypeTag.CLASS)) {
                        if (supertypes.nonEmpty()) {
                            scan(supertypes);
                        } else {
                            Type.ClassType ct = (Type.ClassType) c.type;
                            if (ct.supertype_field != null && ct.interfaces_field != null) {
                                checkSymbol(pos, ct.supertype_field.tsym);
                                for (Type intf : ct.interfaces_field) {
                                    checkSymbol(pos, intf.tsym);
                                }
                            }
                            this.partialCheck = true;
                            return;
                        }
                        if (c.owner.kind == 2) {
                            checkSymbol(pos, c.owner);
                        }
                    }
                } finally {
                    this.seenClasses = this.seenClasses.tail;
                }
            }
        }
    }

    void checkNonCyclic(JCDiagnostic.DiagnosticPosition pos, Type t) {
        checkNonCyclicInternal(pos, t);
    }

    void checkNonCyclic(JCDiagnostic.DiagnosticPosition pos, Type.TypeVar t) {
        checkNonCyclic1(pos, t, List.nil());
    }

    private void checkNonCyclic1(JCDiagnostic.DiagnosticPosition pos, Type t, List<Type.TypeVar> seen) {
        if (t.hasTag(TypeTag.TYPEVAR) && (t.tsym.flags() & 268435456) != 0) {
            return;
        }
        if (seen.contains(t)) {
            ((Type.TypeVar) t.unannotatedType()).bound = this.types.createErrorType(t);
            this.log.error(pos, "cyclic.inheritance", t);
        } else if (t.hasTag(TypeTag.TYPEVAR)) {
            Type.TypeVar tv = (Type.TypeVar) t.unannotatedType();
            List<Type.TypeVar> seen2 = seen.prepend(tv);
            for (Type b : this.types.getBounds(tv)) {
                checkNonCyclic1(pos, b, seen2);
            }
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    private boolean checkNonCyclicInternal(JCDiagnostic.DiagnosticPosition pos, Type t) {
        Type st;
        boolean complete = true;
        Symbol c = t.tsym;
        if ((c.flags_field & 1073741824) != 0) {
            return true;
        }
        if ((c.flags_field & 134217728) != 0) {
            noteCyclic(pos, (Symbol.ClassSymbol) c);
        } else if (!c.type.isErroneous()) {
            try {
                c.flags_field |= 134217728;
                if (c.type.hasTag(TypeTag.CLASS)) {
                    Type.ClassType clazz = (Type.ClassType) c.type;
                    if (clazz.interfaces_field != null) {
                        for (List list = clazz.interfaces_field; list.nonEmpty(); list = list.tail) {
                            complete &= checkNonCyclicInternal(pos, (Type) list.head);
                        }
                    }
                    if (clazz.supertype_field != null && (st = clazz.supertype_field) != null && st.hasTag(TypeTag.CLASS)) {
                        complete &= checkNonCyclicInternal(pos, st);
                    }
                    if (c.owner.kind == 2) {
                        complete &= checkNonCyclicInternal(pos, c.owner.type);
                    }
                }
            } finally {
                c.flags_field &= -134217729;
            }
        }
        if (complete) {
            complete = (c.flags_field & 268435456) == 0 && c.completer == null;
        }
        if (complete) {
            c.flags_field |= 1073741824;
        }
        return complete;
    }

    /* JADX INFO: Access modifiers changed from: private */
    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v13, types: [A, com.sun.tools.javac.code.Type] */
    public void noteCyclic(JCDiagnostic.DiagnosticPosition pos, Symbol.ClassSymbol c) {
        this.log.error(pos, "cyclic.inheritance", c);
        for (List listInterfaces = this.types.interfaces(c.type); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
            listInterfaces.head = this.types.createErrorType((Symbol.ClassSymbol) ((Type) listInterfaces.head).tsym, Type.noType);
        }
        Type st = this.types.supertype(c.type);
        if (st.hasTag(TypeTag.CLASS)) {
            ((Type.ClassType) c.type).supertype_field = this.types.createErrorType((Symbol.ClassSymbol) st.tsym, Type.noType);
        }
        c.type = this.types.createErrorType(c, c.type);
        c.flags_field |= 1073741824;
    }

    void checkImplementations(JCTree.JCClassDecl tree) {
        checkImplementations(tree, tree.sym, tree.sym);
    }

    /* JADX WARN: Multi-variable type inference failed */
    void checkImplementations(JCTree tree, Symbol.ClassSymbol origin, Symbol.ClassSymbol ic) {
        for (List listClosure = this.types.closure(ic.type); listClosure.nonEmpty(); listClosure = listClosure.tail) {
            Symbol.ClassSymbol lc = (Symbol.ClassSymbol) ((Type) listClosure.head).tsym;
            if ((this.allowGenerics || origin != lc) && (lc.flags() & 1024) != 0) {
                for (Scope.Entry e = lc.members().elems; e != null; e = e.sibling) {
                    if (e.sym.kind == 16 && (e.sym.flags() & 1032) == 1024) {
                        Symbol.MethodSymbol absmeth = (Symbol.MethodSymbol) e.sym;
                        Symbol.MethodSymbol implmeth = absmeth.implementation(origin, this.types, false);
                        if (implmeth != null && implmeth != absmeth) {
                            if ((implmeth.owner.flags() & 512) == (512 & origin.flags())) {
                                checkOverride(tree, implmeth, absmeth, origin);
                            }
                        }
                    }
                }
            }
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void checkCompatibleSupertypes(JCDiagnostic.DiagnosticPosition pos, Type c) {
        List<Type> supertypes = this.types.interfaces(c);
        Type supertype = this.types.supertype(c);
        if (supertype.hasTag(TypeTag.CLASS) && (supertype.tsym.flags() & 1024) != 0) {
            supertypes = supertypes.prepend(supertype);
        }
        for (List list = supertypes; list.nonEmpty(); list = list.tail) {
            if (this.allowGenerics && !((Type) list.head).getTypeArguments().isEmpty() && !checkCompatibleAbstracts(pos, (Type) list.head, (Type) list.head, c)) {
                return;
            }
            for (List list2 = supertypes; list2 != list; list2 = list2.tail) {
                if (!checkCompatibleAbstracts(pos, (Type) list.head, (Type) list2.head, c)) {
                    return;
                }
            }
        }
        checkCompatibleConcretes(pos, c);
    }

    void checkConflicts(JCDiagnostic.DiagnosticPosition pos, Symbol sym, Symbol.TypeSymbol c) {
        Type ct = c.type;
        while (ct != Type.noType) {
            for (Scope.Entry e = ct.tsym.members().lookup(sym.name); e.scope == ct.tsym.members(); e = e.next()) {
                if (sym.kind == e.sym.kind && this.types.isSameType(this.types.erasure(sym.type), this.types.erasure(e.sym.type)) && sym != e.sym && (sym.flags() & 4096) != (e.sym.flags() & 4096) && (sym.flags() & 2097152) == 0 && (e.sym.flags() & 2097152) == 0 && (sym.flags() & Flags.BRIDGE) == 0 && (e.sym.flags() & Flags.BRIDGE) == 0) {
                    syntheticError(pos, (e.sym.flags() & 4096) == 0 ? e.sym : sym);
                    return;
                }
            }
            ct = this.types.supertype(ct);
        }
    }

    void checkOverrideClashes(JCDiagnostic.DiagnosticPosition pos, Type site, Symbol.MethodSymbol sym) {
        List<Symbol.MethodSymbol> potentiallyAmbiguousList;
        boolean overridesAny;
        ClashFilter cf = new ClashFilter(site);
        List<Symbol.MethodSymbol> potentiallyAmbiguousList2 = List.nil();
        boolean overridesAny2 = false;
        Iterator<Symbol> it = this.types.membersClosure(site, false).getElementsByName(sym.name, cf).iterator();
        while (it.hasNext()) {
            Symbol m1 = it.next();
            if (!sym.overrides(m1, site.tsym, this.types, false)) {
                if (m1 != sym && !overridesAny2) {
                    potentiallyAmbiguousList2 = potentiallyAmbiguousList2.prepend((Symbol.MethodSymbol) m1);
                }
            } else {
                if (m1 == sym) {
                    potentiallyAmbiguousList = potentiallyAmbiguousList2;
                    overridesAny = overridesAny2;
                } else {
                    List<Symbol.MethodSymbol> potentiallyAmbiguousList3 = List.nil();
                    potentiallyAmbiguousList = potentiallyAmbiguousList3;
                    overridesAny = true;
                }
                for (Symbol m2 : this.types.membersClosure(site, false).getElementsByName(sym.name, cf)) {
                    if (m2 != m1) {
                        if (!this.types.isSubSignature(sym.type, this.types.memberType(site, m2), this.allowStrictMethodClashCheck) && this.types.hasSameArgs(m2.erasure(this.types), m1.erasure(this.types))) {
                            sym.flags_field |= Flags.CLASH;
                            String key = m1 == sym ? "name.clash.same.erasure.no.override" : "name.clash.same.erasure.no.override.1";
                            this.log.error(pos, key, sym, sym.location(), m2, m2.location(), m1, m1.location());
                            return;
                        }
                        cf = cf;
                    }
                }
                potentiallyAmbiguousList2 = potentiallyAmbiguousList;
                overridesAny2 = overridesAny;
            }
        }
        if (!overridesAny2) {
            for (Symbol.MethodSymbol m : potentiallyAmbiguousList2) {
                checkPotentiallyAmbiguousOverloads(pos, site, sym, m);
            }
        }
    }

    void checkHideClashes(JCDiagnostic.DiagnosticPosition pos, Type site, Symbol.MethodSymbol sym) {
        ClashFilter cf = new ClashFilter(site);
        for (Symbol s : this.types.membersClosure(site, true).getElementsByName(sym.name, cf)) {
            if (!this.types.isSubSignature(sym.type, this.types.memberType(site, s), this.allowStrictMethodClashCheck)) {
                if (this.types.hasSameArgs(s.erasure(this.types), sym.erasure(this.types))) {
                    this.log.error(pos, "name.clash.same.erasure.no.hide", sym, sym.location(), s, s.location());
                    return;
                }
                checkPotentiallyAmbiguousOverloads(pos, site, sym, (Symbol.MethodSymbol) s);
            }
        }
    }

    private class ClashFilter implements Filter<Symbol> {
        Type site;

        ClashFilter(Type site) {
            this.site = site;
        }

        boolean shouldSkip(Symbol s) {
            return (s.flags() & Flags.CLASH) != 0 && s.owner == this.site.tsym;
        }

        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol s) {
            return s.kind == 16 && (s.flags() & 4096) == 0 && !shouldSkip(s) && s.isInheritedIn(this.site.tsym, Check.this.types) && !s.isConstructor();
        }
    }

    void checkDefaultMethodClashes(JCDiagnostic.DiagnosticPosition pos, Type site) {
        ListBuffer<Symbol> abstracts;
        ListBuffer<Symbol> defaults;
        String errKey;
        Symbol s2;
        Check check = this;
        DefaultMethodClashFilter dcf = check.new DefaultMethodClashFilter(site);
        boolean z = false;
        for (Symbol m : check.types.membersClosure(site, false).getElements(dcf)) {
            Assert.check(m.kind == 16 ? true : z);
            List<Symbol.MethodSymbol> prov = check.types.interfaceCandidates(site, (Symbol.MethodSymbol) m);
            if (prov.size() > 1) {
                ListBuffer<Symbol> abstracts2 = new ListBuffer<>();
                ListBuffer<Symbol> defaults2 = new ListBuffer<>();
                Iterator<Symbol.MethodSymbol> it = prov.iterator();
                while (true) {
                    if (it.hasNext()) {
                        Symbol.MethodSymbol provSym = it.next();
                        if ((provSym.flags() & Flags.DEFAULT) != 0) {
                            abstracts = abstracts2;
                            defaults = defaults2.append(provSym);
                        } else if ((provSym.flags() & 1024) == 0) {
                            abstracts = abstracts2;
                            defaults = defaults2;
                        } else {
                            abstracts = abstracts2.append(provSym);
                            defaults = defaults2;
                        }
                        if (!defaults.nonEmpty() || defaults.size() + abstracts.size() < 2) {
                            check = this;
                            abstracts2 = abstracts;
                            defaults2 = defaults;
                        } else {
                            Symbol s1 = defaults.first();
                            if (defaults.size() > 1) {
                                errKey = "types.incompatible.unrelated.defaults";
                                s2 = defaults.toList().tail.head;
                            } else {
                                errKey = "types.incompatible.abstract.default";
                                s2 = abstracts.first();
                            }
                            check.log.error(pos, errKey, Kinds.kindName(site.tsym), site, m.name, check.types.memberType(site, m).mo176getParameterTypes(), s1.location(), s2.location());
                        }
                    }
                }
            }
            z = false;
            check = this;
        }
    }

    private class DefaultMethodClashFilter implements Filter<Symbol> {
        Type site;

        DefaultMethodClashFilter(Type site) {
            this.site = site;
        }

        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol s) {
            return s.kind == 16 && (s.flags() & Flags.DEFAULT) != 0 && s.isInheritedIn(this.site.tsym, Check.this.types) && !s.isConstructor();
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void checkPotentiallyAmbiguousOverloads(JCDiagnostic.DiagnosticPosition pos, Type site, Symbol.MethodSymbol msym1, Symbol.MethodSymbol msym2) {
        if (msym1 != msym2 && this.allowDefaultMethods) {
            if (this.lint.isEnabled(Lint.LintCategory.OVERLOADS) && (msym1.flags() & Flags.POTENTIALLY_AMBIGUOUS) == 0 && (msym2.flags() & Flags.POTENTIALLY_AMBIGUOUS) == 0) {
                Type mt1 = this.types.memberType(site, msym1);
                Type mt2 = this.types.memberType(site, msym2);
                if (mt1.hasTag(TypeTag.FORALL) && mt2.hasTag(TypeTag.FORALL) && this.types.hasSameBounds((Type.ForAll) mt1, (Type.ForAll) mt2)) {
                    mt2 = this.types.subst(mt2, ((Type.ForAll) mt2).tvars, ((Type.ForAll) mt1).tvars);
                }
                int maxLength = Math.max(mt1.mo176getParameterTypes().length(), mt2.mo176getParameterTypes().length());
                List listAdjustArgs = this.rs.adjustArgs(mt1.mo176getParameterTypes(), msym1, maxLength, true);
                List listAdjustArgs2 = this.rs.adjustArgs(mt2.mo176getParameterTypes(), msym2, maxLength, true);
                if (listAdjustArgs.length() != listAdjustArgs2.length()) {
                    return;
                }
                boolean potentiallyAmbiguous = false;
                while (listAdjustArgs.nonEmpty() && listAdjustArgs2.nonEmpty()) {
                    Type s = (Type) listAdjustArgs.head;
                    Type t = (Type) listAdjustArgs2.head;
                    if (!this.types.isSubtype(t, s) && !this.types.isSubtype(s, t)) {
                        if (!this.types.isFunctionalInterface(s) || !this.types.isFunctionalInterface(t) || this.types.findDescriptorType(s).mo176getParameterTypes().length() <= 0 || this.types.findDescriptorType(s).mo176getParameterTypes().length() != this.types.findDescriptorType(t).mo176getParameterTypes().length()) {
                            break;
                        } else {
                            potentiallyAmbiguous = true;
                        }
                    }
                    listAdjustArgs = listAdjustArgs.tail;
                    listAdjustArgs2 = listAdjustArgs2.tail;
                }
                if (potentiallyAmbiguous) {
                    msym1.flags_field |= Flags.POTENTIALLY_AMBIGUOUS;
                    msym2.flags_field = Flags.POTENTIALLY_AMBIGUOUS | msym2.flags_field;
                    this.log.warning(Lint.LintCategory.OVERLOADS, pos, "potentially.ambiguous.overload", msym1, msym1.location(), msym2, msym2.location());
                }
            }
        }
    }

    void checkElemAccessFromSerializableLambda(JCTree tree) {
        if (this.warnOnAccessToSensitiveMembers) {
            Symbol sym = TreeInfo.symbol(tree);
            if ((sym.kind & 20) == 0) {
                return;
            }
            if ((sym.kind != 4 || ((sym.flags() & 8589934592L) == 0 && !sym.isLocal() && sym.name != this.names._this && sym.name != this.names._super)) && !this.types.isSubtype(sym.owner.type, this.syms.serializableType) && isEffectivelyNonPublic(sym)) {
                this.log.warning(tree.pos(), "access.to.sensitive.member.from.serializable.element", sym);
            }
        }
    }

    private boolean isEffectivelyNonPublic(Symbol sym) {
        if (sym.packge() == this.syms.rootPackage) {
            return false;
        }
        while (sym.kind != 1) {
            if ((sym.flags() & 1) == 0) {
                return true;
            }
            sym = sym.owner;
        }
        return false;
    }

    private void syntheticError(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
        if (!sym.type.isErroneous()) {
            if (this.warnOnSyntheticConflicts) {
                this.log.warning(pos, "synthetic.name.conflict", sym, sym.location());
            } else {
                this.log.error(pos, "synthetic.name.conflict", sym, sym.location());
            }
        }
    }

    void checkClassBounds(JCDiagnostic.DiagnosticPosition pos, Type type) {
        checkClassBounds(pos, new HashMap(), type);
    }

    /* JADX WARN: Multi-variable type inference failed */
    void checkClassBounds(JCDiagnostic.DiagnosticPosition pos, Map<Symbol.TypeSymbol, Type> seensofar, Type type) {
        if (type.isErroneous()) {
            return;
        }
        for (List listInterfaces = this.types.interfaces(type); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
            Type it = (Type) listInterfaces.head;
            Type oldit = seensofar.put(it.tsym, it);
            if (oldit != null) {
                List<Type> oldparams = oldit.allparams();
                List<Type> newparams = it.allparams();
                if (!this.types.containsTypeEquivalent(oldparams, newparams)) {
                    this.log.error(pos, "cant.inherit.diff.arg", it.tsym, Type.toString(oldparams), Type.toString(newparams));
                }
            }
            checkClassBounds(pos, seensofar, it);
        }
        Type st = this.types.supertype(type);
        if (st != Type.noType) {
            checkClassBounds(pos, seensofar, st);
        }
    }

    void checkNotRepeated(JCDiagnostic.DiagnosticPosition pos, Type it, Set<Type> its) {
        if (its.contains(it)) {
            this.log.error(pos, "repeated.interface", new Object[0]);
        } else {
            its.add(it);
        }
    }

    void validateAnnotationTree(JCTree tree) {
        tree.accept(new TreeScanner() { // from class: com.sun.tools.javac.comp.Check.1AnnotationValidator
            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitAnnotation(JCTree.JCAnnotation tree2) {
                if (!tree2.type.isErroneous()) {
                    super.visitAnnotation(tree2);
                    Check.this.validateAnnotation(tree2);
                }
            }
        });
    }

    void validateAnnotationType(JCTree restype) {
        if (restype != null) {
            validateAnnotationType(restype.pos(), restype.type);
        }
    }

    void validateAnnotationType(JCDiagnostic.DiagnosticPosition pos, Type type) {
        if (type.isPrimitive() || this.types.isSameType(type, this.syms.stringType) || (type.tsym.flags() & 16384) != 0 || (type.tsym.flags() & 8192) != 0 || this.types.cvarLowerBound(type).tsym == this.syms.classType.tsym) {
            return;
        }
        if (this.types.isArray(type) && !this.types.isArray(this.types.elemtype(type))) {
            validateAnnotationType(pos, this.types.elemtype(type));
        } else {
            this.log.error(pos, "invalid.annotation.member.type", new Object[0]);
        }
    }

    void validateAnnotationMethod(JCDiagnostic.DiagnosticPosition pos, Symbol.MethodSymbol m) {
        Type sup = this.syms.annotationType;
        while (sup.hasTag(TypeTag.CLASS)) {
            Scope s = sup.tsym.members();
            for (Scope.Entry e = s.lookup(m.name); e.scope != null; e = e.next()) {
                if (e.sym.kind == 16 && (e.sym.flags() & 5) != 0 && this.types.overrideEquivalent(m.type, e.sym.type)) {
                    this.log.error(pos, "intf.annotation.member.clash", e.sym, sup);
                }
            }
            sup = this.types.supertype(sup);
        }
    }

    public void validateAnnotations(List<JCTree.JCAnnotation> annotations, Symbol s) {
        for (JCTree.JCAnnotation a : annotations) {
            validateAnnotation(a, s);
        }
    }

    public void validateTypeAnnotations(List<JCTree.JCAnnotation> annotations, boolean isTypeParameter) {
        for (JCTree.JCAnnotation a : annotations) {
            validateTypeAnnotation(a, isTypeParameter);
        }
    }

    private void validateAnnotation(JCTree.JCAnnotation a, Symbol s) {
        validateAnnotationTree(a);
        if (!annotationApplicable(a, s)) {
            this.log.error(a.pos(), "annotation.type.not.applicable", new Object[0]);
        }
        if (a.annotationType.type.tsym == this.syms.functionalInterfaceType.tsym) {
            if (s.kind != 2) {
                this.log.error(a.pos(), "bad.functional.intf.anno", new Object[0]);
            } else if (!s.isInterface() || (s.flags() & 8192) != 0) {
                this.log.error(a.pos(), "bad.functional.intf.anno.1", this.diags.fragment("not.a.functional.intf", s));
            }
        }
    }

    public void validateTypeAnnotation(JCTree.JCAnnotation a, boolean isTypeParameter) {
        Assert.checkNonNull(a.type, "annotation tree hasn't been attributed yet: " + a);
        validateAnnotationTree(a);
        if (a.hasTag(JCTree.Tag.TYPE_ANNOTATION) && !a.annotationType.type.isErroneous() && !isTypeAnnotation(a, isTypeParameter)) {
            this.log.error(a.pos(), "annotation.type.not.applicable", new Object[0]);
        }
    }

    public void validateRepeatable(Symbol.TypeSymbol s, Attribute.Compound repeatable, JCDiagnostic.DiagnosticPosition pos) {
        Assert.check(this.types.isSameType(repeatable.type, this.syms.repeatableType));
        Type t = null;
        List<Pair<Symbol.MethodSymbol, Attribute>> l = repeatable.values;
        if (!l.isEmpty()) {
            Assert.check(l.head.fst.name == this.names.value);
            t = ((Attribute.Class) l.head.snd).getValue();
        }
        if (t == null) {
            return;
        }
        validateValue(t.tsym, s, pos);
        validateRetention(t.tsym, s, pos);
        validateDocumented(t.tsym, s, pos);
        validateInherited(t.tsym, s, pos);
        validateTarget(t.tsym, s, pos);
        validateDefault(t.tsym, pos);
    }

    private void validateValue(Symbol.TypeSymbol container, Symbol.TypeSymbol contained, JCDiagnostic.DiagnosticPosition pos) {
        Scope.Entry e = container.members().lookup(this.names.value);
        if (e.scope != null && e.sym.kind == 16) {
            Symbol.MethodSymbol m = (Symbol.MethodSymbol) e.sym;
            Type ret = m.getReturnType();
            if (!ret.hasTag(TypeTag.ARRAY) || !this.types.isSameType(((Type.ArrayType) ret).elemtype, contained.type)) {
                this.log.error(pos, "invalid.repeatable.annotation.value.return", container, ret, this.types.makeArrayType(contained.type));
                return;
            }
            return;
        }
        this.log.error(pos, "invalid.repeatable.annotation.no.value", container);
    }

    private void validateRetention(Symbol container, Symbol contained, JCDiagnostic.DiagnosticPosition pos) {
        Attribute.RetentionPolicy containerRetention = this.types.getRetention(container);
        Attribute.RetentionPolicy containedRetention = this.types.getRetention(contained);
        boolean error = false;
        switch (containedRetention) {
            case RUNTIME:
                if (containerRetention != Attribute.RetentionPolicy.RUNTIME) {
                    error = true;
                }
                break;
            case CLASS:
                if (containerRetention == Attribute.RetentionPolicy.SOURCE) {
                    error = true;
                }
                break;
        }
        if (error) {
            this.log.error(pos, "invalid.repeatable.annotation.retention", container, containerRetention, contained, containedRetention);
        }
    }

    private void validateDocumented(Symbol container, Symbol contained, JCDiagnostic.DiagnosticPosition pos) {
        if (contained.attribute(this.syms.documentedType.tsym) != null && container.attribute(this.syms.documentedType.tsym) == null) {
            this.log.error(pos, "invalid.repeatable.annotation.not.documented", container, contained);
        }
    }

    private void validateInherited(Symbol container, Symbol contained, JCDiagnostic.DiagnosticPosition pos) {
        if (contained.attribute(this.syms.inheritedType.tsym) != null && container.attribute(this.syms.inheritedType.tsym) == null) {
            this.log.error(pos, "invalid.repeatable.annotation.not.inherited", container, contained);
        }
    }

    private void validateTarget(Symbol container, Symbol contained, JCDiagnostic.DiagnosticPosition pos) {
        Set<Name> containerTargets;
        Set<Name> containedTargets;
        Attribute.Array containerTarget = getAttributeTargetAttribute(container);
        if (containerTarget == null) {
            containerTargets = getDefaultTargetSet();
        } else {
            containerTargets = new HashSet<>();
            for (Attribute app : containerTarget.values) {
                if (app instanceof Attribute.Enum) {
                    Attribute.Enum e = (Attribute.Enum) app;
                    containerTargets.add(e.value.name);
                }
            }
        }
        Attribute.Array containedTarget = getAttributeTargetAttribute(contained);
        if (containedTarget == null) {
            containedTargets = getDefaultTargetSet();
        } else {
            Set<Name> containedTargets2 = new HashSet<>();
            for (Attribute app2 : containedTarget.values) {
                if (app2 instanceof Attribute.Enum) {
                    Attribute.Enum e2 = (Attribute.Enum) app2;
                    containedTargets2.add(e2.value.name);
                }
            }
            containedTargets = containedTargets2;
        }
        if (!isTargetSubsetOf(containerTargets, containedTargets)) {
            this.log.error(pos, "invalid.repeatable.annotation.incompatible.target", container, contained);
        }
    }

    private Set<Name> getDefaultTargetSet() {
        if (this.defaultTargets == null) {
            Set<Name> targets = new HashSet<>();
            targets.add(this.names.ANNOTATION_TYPE);
            targets.add(this.names.CONSTRUCTOR);
            targets.add(this.names.FIELD);
            targets.add(this.names.LOCAL_VARIABLE);
            targets.add(this.names.METHOD);
            targets.add(this.names.PACKAGE);
            targets.add(this.names.PARAMETER);
            targets.add(this.names.TYPE);
            this.defaultTargets = Collections.unmodifiableSet(targets);
        }
        return this.defaultTargets;
    }

    private boolean isTargetSubsetOf(Set<Name> s, Set<Name> t) {
        for (Name n2 : s) {
            boolean currentElementOk = false;
            Iterator<Name> it = t.iterator();
            while (true) {
                if (!it.hasNext()) {
                    break;
                }
                Name n1 = it.next();
                if (n1 == n2) {
                    currentElementOk = true;
                    break;
                }
                if (n1 == this.names.TYPE && n2 == this.names.ANNOTATION_TYPE) {
                    currentElementOk = true;
                    break;
                }
                if (n1 == this.names.TYPE_USE && (n2 == this.names.TYPE || n2 == this.names.ANNOTATION_TYPE || n2 == this.names.TYPE_PARAMETER)) {
                    break;
                }
            }
            currentElementOk = true;
            if (!currentElementOk) {
                return false;
            }
        }
        return true;
    }

    private void validateDefault(Symbol container, JCDiagnostic.DiagnosticPosition pos) {
        Scope scope = container.members();
        for (Symbol elm : scope.getElements()) {
            if (elm.name != this.names.value && elm.kind == 16 && ((Symbol.MethodSymbol) elm).defaultValue == null) {
                this.log.error(pos, "invalid.repeatable.annotation.elem.nondefault", container, elm);
            }
        }
    }

    boolean isOverrider(Symbol s) {
        if (s.kind != 16 || s.isStatic()) {
            return false;
        }
        Symbol.MethodSymbol m = (Symbol.MethodSymbol) s;
        Symbol.TypeSymbol owner = (Symbol.TypeSymbol) m.owner;
        for (Type sup : this.types.closure(owner.type)) {
            if (sup != owner.type) {
                Scope scope = sup.tsym.members();
                for (Scope.Entry e = scope.lookup(m.name); e.scope != null; e = e.next()) {
                    if (!e.sym.isStatic() && m.overrides(e.sym, owner, this.types, true)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    protected boolean isTypeAnnotation(JCTree.JCAnnotation a, boolean isTypeParameter) {
        Attribute.Compound atTarget = a.annotationType.type.tsym.attribute(this.syms.annotationTargetType.tsym);
        if (atTarget == null) {
            return false;
        }
        Attribute atValue = atTarget.member(this.names.value);
        if (!(atValue instanceof Attribute.Array)) {
            return false;
        }
        Attribute.Array arr = (Attribute.Array) atValue;
        for (Attribute app : arr.values) {
            if (!(app instanceof Attribute.Enum)) {
                return false;
            }
            Attribute.Enum e = (Attribute.Enum) app;
            if (e.value.name == this.names.TYPE_USE) {
                return true;
            }
            if (isTypeParameter && e.value.name == this.names.TYPE_PARAMETER) {
                return true;
            }
        }
        return false;
    }

    boolean annotationApplicable(JCTree.JCAnnotation a, Symbol s) {
        Name[] targets;
        Attribute.Array arr = getAttributeTargetAttribute(a.annotationType.type.tsym);
        if (arr == null) {
            targets = defaultTargetMetaInfo(a, s);
        } else {
            targets = new Name[arr.values.length];
            for (int i = 0; i < arr.values.length; i++) {
                Attribute app = arr.values[i];
                if (!(app instanceof Attribute.Enum)) {
                    return true;
                }
                Attribute.Enum e = (Attribute.Enum) app;
                targets[i] = e.value.name;
            }
        }
        for (Name target : targets) {
            if (target == this.names.TYPE) {
                if (s.kind == 2) {
                    return true;
                }
            } else if (target == this.names.FIELD) {
                if (s.kind == 4 && s.owner.kind != 16) {
                    return true;
                }
            } else if (target == this.names.METHOD) {
                if (s.kind == 16 && !s.isConstructor()) {
                    return true;
                }
            } else if (target == this.names.PARAMETER) {
                if (s.kind == 4 && s.owner.kind == 16 && (s.flags() & 8589934592L) != 0) {
                    return true;
                }
            } else if (target == this.names.CONSTRUCTOR) {
                if (s.kind == 16 && s.isConstructor()) {
                    return true;
                }
            } else if (target == this.names.LOCAL_VARIABLE) {
                if (s.kind == 4 && s.owner.kind == 16 && (s.flags() & 8589934592L) == 0) {
                    return true;
                }
            } else if (target == this.names.ANNOTATION_TYPE) {
                if (s.kind == 2 && (s.flags() & 8192) != 0) {
                    return true;
                }
            } else if (target == this.names.PACKAGE) {
                if (s.kind == 1) {
                    return true;
                }
            } else if (target == this.names.TYPE_USE) {
                if (s.kind == 2 || s.kind == 4 || ((s.kind == 16 && !s.isConstructor() && !s.type.mo178getReturnType().hasTag(TypeTag.VOID)) || (s.kind == 16 && s.isConstructor()))) {
                    return true;
                }
            } else {
                if (target != this.names.TYPE_PARAMETER) {
                    return true;
                }
                if (s.kind == 2 && s.type.hasTag(TypeTag.TYPEVAR)) {
                    return true;
                }
            }
        }
        return false;
    }

    Attribute.Array getAttributeTargetAttribute(Symbol s) {
        Attribute.Compound atTarget = s.attribute(this.syms.annotationTargetType.tsym);
        if (atTarget == null) {
            return null;
        }
        Attribute atValue = atTarget.member(this.names.value);
        if (!(atValue instanceof Attribute.Array)) {
            return null;
        }
        return (Attribute.Array) atValue;
    }

    private Name[] defaultTargetMetaInfo(JCTree.JCAnnotation a, Symbol s) {
        return this.dfltTargetMeta;
    }

    public boolean validateAnnotationDeferErrors(JCTree.JCAnnotation a) {
        Log.DiagnosticHandler diagHandler = new Log.DiscardDiagnosticHandler(this.log);
        try {
            boolean res = validateAnnotation(a);
            return res;
        } finally {
            this.log.popDiagnosticHandler(diagHandler);
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public boolean validateAnnotation(JCTree.JCAnnotation a) {
        Check check = this;
        boolean isValid = true;
        Set<Symbol.MethodSymbol> members = new LinkedHashSet<>();
        for (Scope.Entry e = a.annotationType.type.tsym.members().elems; e != null; e = e.sibling) {
            if (e.sym.kind == 16 && e.sym.name != check.names.clinit && (e.sym.flags() & 4096) == 0) {
                members.add((Symbol.MethodSymbol) e.sym);
            }
        }
        for (JCTree arg : a.args) {
            if (arg.hasTag(JCTree.Tag.ASSIGN)) {
                JCTree.JCAssign assign = (JCTree.JCAssign) arg;
                Symbol m = TreeInfo.symbol(assign.lhs);
                if (m != null && !m.type.isErroneous() && !members.remove(m)) {
                    isValid = false;
                    check.log.error(assign.lhs.pos(), "duplicate.annotation.member.value", m.name, a.type);
                }
            }
        }
        List<Name> missingDefaults = List.nil();
        for (Symbol.MethodSymbol m2 : members) {
            if (m2.defaultValue == null && !m2.type.isErroneous()) {
                missingDefaults = missingDefaults.append(m2.name);
            }
        }
        List<Name> missingDefaults2 = missingDefaults.reverse();
        if (missingDefaults2.nonEmpty()) {
            isValid = false;
            String key = missingDefaults2.size() > 1 ? "annotation.missing.default.value.1" : "annotation.missing.default.value";
            check.log.error(a.pos(), key, a.type, missingDefaults2);
        }
        if (a.annotationType.type.tsym != check.syms.annotationTargetType.tsym || a.args.tail == null) {
            return isValid;
        }
        if (!a.args.head.hasTag(JCTree.Tag.ASSIGN)) {
            return false;
        }
        JCTree.JCAssign assign2 = (JCTree.JCAssign) a.args.head;
        if (TreeInfo.symbol(assign2.lhs).name != check.names.value) {
            return false;
        }
        JCTree rhs = assign2.rhs;
        if (!rhs.hasTag(JCTree.Tag.NEWARRAY)) {
            return false;
        }
        JCTree.JCNewArray na = (JCTree.JCNewArray) rhs;
        Set<Symbol> targets = new HashSet<>();
        for (JCTree elem : na.elems) {
            if (!targets.add(TreeInfo.symbol(elem))) {
                isValid = false;
                check.log.error(elem.pos(), "repeated.annotation.target", new Object[0]);
            }
            check = this;
        }
        return isValid;
    }

    void checkDeprecatedAnnotation(JCDiagnostic.DiagnosticPosition pos, Symbol s) {
        if (this.allowAnnotations && this.lint.isEnabled(Lint.LintCategory.DEP_ANN) && (s.flags() & 131072) != 0 && !this.syms.deprecatedType.isErroneous() && s.attribute(this.syms.deprecatedType.tsym) == null) {
            this.log.warning(Lint.LintCategory.DEP_ANN, pos, "missing.deprecated.annotation", new Object[0]);
        }
    }

    void checkDeprecated(final JCDiagnostic.DiagnosticPosition pos, Symbol other, final Symbol s) {
        if ((s.flags() & 131072) != 0 && (other.flags() & 131072) == 0 && s.outermostClass() != other.outermostClass()) {
            this.deferredLintHandler.report(new DeferredLintHandler.LintLogger() { // from class: com.sun.tools.javac.comp.Check.7
                @Override // com.sun.tools.javac.code.DeferredLintHandler.LintLogger
                public void report() {
                    Check.this.warnDeprecated(pos, s);
                }
            });
        }
    }

    void checkSunAPI(final JCDiagnostic.DiagnosticPosition pos, final Symbol s) {
        if ((s.flags() & Flags.PROPRIETARY) != 0) {
            this.deferredLintHandler.report(new DeferredLintHandler.LintLogger() { // from class: com.sun.tools.javac.comp.Check.8
                @Override // com.sun.tools.javac.code.DeferredLintHandler.LintLogger
                public void report() {
                    if (!Check.this.enableSunApiLintControl) {
                        Check.this.log.mandatoryWarning(pos, "sun.proprietary", s);
                    } else {
                        Check.this.warnSunApi(pos, "sun.proprietary", s);
                    }
                }
            });
        }
    }

    void checkProfile(JCDiagnostic.DiagnosticPosition pos, Symbol s) {
        if (this.profile != Profile.DEFAULT && (s.flags() & 35184372088832L) != 0) {
            this.log.error(pos, "not.in.profile", s, this.profile);
        }
    }

    void checkNonCyclicElements(JCTree.JCClassDecl tree) {
        if ((tree.sym.flags_field & 8192) == 0) {
            return;
        }
        Assert.check((tree.sym.flags_field & 134217728) == 0);
        try {
            Symbol.ClassSymbol classSymbol = tree.sym;
            classSymbol.flags_field = 134217728 | classSymbol.flags_field;
            for (JCTree def : tree.defs) {
                if (def.hasTag(JCTree.Tag.METHODDEF)) {
                    JCTree.JCMethodDecl meth = (JCTree.JCMethodDecl) def;
                    checkAnnotationResType(meth.pos(), meth.restype.type);
                }
            }
        } finally {
            Symbol.ClassSymbol classSymbol2 = tree.sym;
            classSymbol2.flags_field = (-134217729) & classSymbol2.flags_field;
            Symbol.ClassSymbol classSymbol3 = tree.sym;
            classSymbol3.flags_field = Flags.ACYCLIC_ANN | classSymbol3.flags_field;
        }
    }

    void checkNonCyclicElementsInternal(JCDiagnostic.DiagnosticPosition pos, Symbol.TypeSymbol tsym) {
        if ((tsym.flags_field & Flags.ACYCLIC_ANN) != 0) {
            return;
        }
        if ((tsym.flags_field & 134217728) != 0) {
            this.log.error(pos, "cyclic.annotation.element", new Object[0]);
            return;
        }
        try {
            tsym.flags_field |= 134217728;
            for (Scope.Entry e = tsym.members().elems; e != null; e = e.sibling) {
                Symbol s = e.sym;
                if (s.kind == 16) {
                    checkAnnotationResType(pos, ((Symbol.MethodSymbol) s).type.mo178getReturnType());
                }
            }
        } finally {
            tsym.flags_field = (-134217729) & tsym.flags_field;
            tsym.flags_field |= Flags.ACYCLIC_ANN;
        }
    }

    void checkAnnotationResType(JCDiagnostic.DiagnosticPosition pos, Type type) {
        switch (type.getTag()) {
            case CLASS:
                if ((type.tsym.flags() & 8192) != 0) {
                    checkNonCyclicElementsInternal(pos, type.tsym);
                }
                break;
            case ARRAY:
                checkAnnotationResType(pos, this.types.elemtype(type));
                break;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void checkCyclicConstructors(JCTree.JCClassDecl tree) {
        Map<Symbol, Symbol> callMap = new HashMap<>();
        for (List list = tree.defs; list.nonEmpty(); list = list.tail) {
            JCTree.JCMethodInvocation app = TreeInfo.firstConstructorCall((JCTree) list.head);
            if (app != null) {
                JCTree.JCMethodDecl meth = (JCTree.JCMethodDecl) list.head;
                if (TreeInfo.name(app.meth) == this.names._this) {
                    callMap.put(meth.sym, TreeInfo.symbol(app.meth));
                } else {
                    meth.sym.flags_field |= 1073741824;
                }
            }
        }
        Symbol[] ctors = (Symbol[]) callMap.keySet().toArray(new Symbol[0]);
        for (Symbol caller : ctors) {
            checkCyclicConstructor(tree, caller, callMap);
        }
    }

    private void checkCyclicConstructor(JCTree.JCClassDecl tree, Symbol ctor, Map<Symbol, Symbol> callMap) {
        if (ctor != null && (ctor.flags_field & 1073741824) == 0) {
            if ((ctor.flags_field & 134217728) != 0) {
                this.log.error(TreeInfo.diagnosticPositionFor(ctor, tree), "recursive.ctor.invocation", new Object[0]);
            } else {
                ctor.flags_field |= 134217728;
                checkCyclicConstructor(tree, callMap.remove(ctor), callMap);
                ctor.flags_field &= -134217729;
            }
            ctor.flags_field |= 1073741824;
        }
    }

    int checkOperator(JCDiagnostic.DiagnosticPosition pos, Symbol.OperatorSymbol operator, JCTree.Tag tag, Type left, Type right) {
        if (operator.opcode == 277) {
            this.log.error(pos, "operator.cant.be.applied.1", this.treeinfo.operatorName(tag), left, right);
        }
        return operator.opcode;
    }

    void checkDivZero(JCDiagnostic.DiagnosticPosition pos, Symbol operator, Type operand) {
        if (operand.constValue() != null && this.lint.isEnabled(Lint.LintCategory.DIVZERO) && operand.getTag().isSubRangeOf(TypeTag.LONG) && ((Number) operand.constValue()).longValue() == 0) {
            int opc = ((Symbol.OperatorSymbol) operator).opcode;
            if (opc == 108 || opc == 112 || opc == 109 || opc == 113) {
                this.log.warning(Lint.LintCategory.DIVZERO, pos, "div.zero", new Object[0]);
            }
        }
    }

    void checkEmptyIf(JCTree.JCIf tree) {
        if (tree.thenpart.hasTag(JCTree.Tag.SKIP) && tree.elsepart == null && this.lint.isEnabled(Lint.LintCategory.EMPTY)) {
            this.log.warning(Lint.LintCategory.EMPTY, tree.thenpart.pos(), "empty.if", new Object[0]);
        }
    }

    boolean checkUnique(JCDiagnostic.DiagnosticPosition pos, Symbol sym, Scope s) {
        if (sym.type.isErroneous()) {
            return true;
        }
        if (sym.owner.name == this.names.any) {
            return false;
        }
        for (Scope.Entry e = s.lookup(sym.name); e.scope == s; e = e.next()) {
            if (sym != e.sym && (e.sym.flags() & Flags.CLASH) == 0 && sym.kind == e.sym.kind && sym.name != this.names.error && (sym.kind != 16 || this.types.hasSameArgs(sym.type, e.sym.type) || this.types.hasSameArgs(this.types.erasure(sym.type), this.types.erasure(e.sym.type)))) {
                if ((sym.flags() & Flags.VARARGS) != (e.sym.flags() & Flags.VARARGS)) {
                    varargsDuplicateError(pos, sym, e.sym);
                    return true;
                }
                if (sym.kind != 16 || this.types.hasSameArgs(sym.type, e.sym.type, false)) {
                    duplicateError(pos, e.sym);
                    return false;
                }
                duplicateErasureError(pos, sym, e.sym);
                sym.flags_field |= Flags.CLASH;
                return true;
            }
        }
        return true;
    }

    void duplicateErasureError(JCDiagnostic.DiagnosticPosition pos, Symbol sym1, Symbol sym2) {
        if (!sym1.type.isErroneous() && !sym2.type.isErroneous()) {
            this.log.error(pos, "name.clash.same.erasure", sym1, sym2);
        }
    }

    boolean checkUniqueImport(JCDiagnostic.DiagnosticPosition pos, Symbol sym, Scope s) {
        return checkUniqueImport(pos, sym, s, false);
    }

    boolean checkUniqueStaticImport(JCDiagnostic.DiagnosticPosition pos, Symbol sym, Scope s) {
        return checkUniqueImport(pos, sym, s, true);
    }

    private boolean checkUniqueImport(JCDiagnostic.DiagnosticPosition pos, Symbol sym, Scope s, boolean staticImport) {
        boolean isClassDecl;
        Scope.Entry e = s.lookup(sym.name);
        while (true) {
            if (e.scope == null) {
                return true;
            }
            isClassDecl = e.scope == s;
            if ((isClassDecl || sym != e.sym) && sym.kind == e.sym.kind && sym.name != this.names.error && !(staticImport && e.isStaticallyImported())) {
                break;
            }
            e = e.next();
        }
        if (!e.sym.type.isErroneous()) {
            if (!isClassDecl) {
                if (staticImport) {
                    this.log.error(pos, "already.defined.static.single.import", e.sym);
                } else {
                    this.log.error(pos, "already.defined.single.import", e.sym);
                }
            } else if (sym != e.sym) {
                this.log.error(pos, "already.defined.this.unit", e.sym);
            }
        }
        return false;
    }

    public void checkCanonical(JCTree tree) {
        if (!isCanonical(tree)) {
            this.log.error(tree.pos(), "import.requires.canonical", TreeInfo.symbol(tree));
        }
    }

    private boolean isCanonical(JCTree tree) {
        while (tree.hasTag(JCTree.Tag.SELECT)) {
            JCTree.JCFieldAccess s = (JCTree.JCFieldAccess) tree;
            if (s.sym.owner != TreeInfo.symbol(s.selected)) {
                return false;
            }
            tree = s.selected;
        }
        return true;
    }

    void checkForBadAuxiliaryClassAccess(JCDiagnostic.DiagnosticPosition pos, Env<AttrContext> env, Symbol.ClassSymbol c) {
        if (this.lint.isEnabled(Lint.LintCategory.AUXILIARYCLASS) && (c.flags() & Flags.AUXILIARY) != 0 && this.rs.isAccessible(env, c) && !this.fileManager.isSameFile(c.sourcefile, env.toplevel.sourcefile)) {
            this.log.warning(pos, "auxiliary.class.accessed.from.outside.of.its.source.file", c, c.sourcefile);
        }
    }

    private class ConversionWarner extends Warner {
        final Type expected;
        final Type found;
        final String uncheckedKey;

        public ConversionWarner(JCDiagnostic.DiagnosticPosition pos, String uncheckedKey, Type found, Type expected) {
            super(pos);
            this.uncheckedKey = uncheckedKey;
            this.found = found;
            this.expected = expected;
        }

        @Override // com.sun.tools.javac.util.Warner
        public void warn(Lint.LintCategory lint) {
            boolean warned = this.warned;
            super.warn(lint);
            if (warned) {
                return;
            }
            switch (lint) {
                case UNCHECKED:
                    Check.this.warnUnchecked(pos(), "prob.found.req", Check.this.diags.fragment(this.uncheckedKey, new Object[0]), this.found, this.expected);
                    return;
                case VARARGS:
                    if (Check.this.method != null && Check.this.method.attribute(Check.this.syms.trustMeType.tsym) != null && Check.this.isTrustMeAllowedOnMethod(Check.this.method) && !Check.this.types.isReifiable(Check.this.method.type.mo176getParameterTypes().last())) {
                        Check.this.warnUnsafeVararg(pos(), "varargs.unsafe.use.varargs.param", Check.this.method.params.last());
                        return;
                    }
                    return;
                default:
                    throw new AssertionError("Unexpected lint: " + lint);
            }
        }
    }

    public Warner castWarner(JCDiagnostic.DiagnosticPosition pos, Type found, Type expected) {
        return new ConversionWarner(pos, "unchecked.cast.to.type", found, expected);
    }

    public Warner convertWarner(JCDiagnostic.DiagnosticPosition pos, Type found, Type expected) {
        return new ConversionWarner(pos, "unchecked.assign", found, expected);
    }

    public void checkFunctionalInterface(JCTree.JCClassDecl tree, Symbol.ClassSymbol cs) {
        Attribute.Compound functionalType = cs.attribute(this.syms.functionalInterfaceType.tsym);
        if (functionalType != null) {
            try {
                this.types.findDescriptorSymbol(cs);
            } catch (Types.FunctionDescriptorLookupError ex) {
                JCDiagnostic.DiagnosticPosition pos = tree.pos();
                Iterator<JCTree.JCAnnotation> it = tree.getModifiers().annotations.iterator();
                while (true) {
                    if (!it.hasNext()) {
                        break;
                    }
                    JCTree.JCAnnotation a = it.next();
                    if (a.annotationType.type.tsym == this.syms.functionalInterfaceType.tsym) {
                        pos = a.pos();
                        break;
                    }
                }
                this.log.error(pos, "bad.functional.intf.anno.1", ex.getDiagnostic());
            }
        }
    }
}
