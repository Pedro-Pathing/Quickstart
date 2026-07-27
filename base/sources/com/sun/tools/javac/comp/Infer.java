package com.sun.tools.javac.comp;

import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.comp.Check;
import com.sun.tools.javac.comp.DeferredAttr;
import com.sun.tools.javac.comp.Resolve;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.GraphUtils;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.Pair;
import com.sun.tools.javac.util.Warner;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

/* JADX INFO: loaded from: classes.dex */
public class Infer {
    static final int MAX_INCORPORATION_STEPS = 100;
    boolean allowGraphInference;
    Check chk;
    JCDiagnostic.Factory diags;
    protected final InferenceException inferenceException;
    Log log;
    Resolve rs;
    Symtab syms;
    Types types;
    protected static final Context.Key<Infer> inferKey = new Context.Key<>();
    public static final Type anyPoly = new Type.JCNoType();
    EnumSet<IncorporationStep> incorporationStepsLegacy = EnumSet.of(IncorporationStep.EQ_CHECK_LEGACY);
    EnumSet<IncorporationStep> incorporationStepsGraph = EnumSet.complementOf(EnumSet.of(IncorporationStep.EQ_CHECK_LEGACY));
    Map<IncorporationBinaryOp, Boolean> incorporationCache = new HashMap();
    final InferenceContext emptyContext = new InferenceContext(List.nil());

    enum BoundErrorKind {
        BAD_UPPER { // from class: com.sun.tools.javac.comp.Infer.BoundErrorKind.1
            @Override // com.sun.tools.javac.comp.Infer.BoundErrorKind
            Resolve.InapplicableMethodException setMessage(InferenceException ex, Type.UndetVar uv) {
                return ex.setMessage("incompatible.upper.bounds", uv.qtype, uv.getBounds(Type.UndetVar.InferenceBound.UPPER));
            }
        },
        BAD_EQ_UPPER { // from class: com.sun.tools.javac.comp.Infer.BoundErrorKind.2
            @Override // com.sun.tools.javac.comp.Infer.BoundErrorKind
            Resolve.InapplicableMethodException setMessage(InferenceException ex, Type.UndetVar uv) {
                return ex.setMessage("incompatible.eq.upper.bounds", uv.qtype, uv.getBounds(Type.UndetVar.InferenceBound.EQ), uv.getBounds(Type.UndetVar.InferenceBound.UPPER));
            }
        },
        BAD_EQ_LOWER { // from class: com.sun.tools.javac.comp.Infer.BoundErrorKind.3
            @Override // com.sun.tools.javac.comp.Infer.BoundErrorKind
            Resolve.InapplicableMethodException setMessage(InferenceException ex, Type.UndetVar uv) {
                return ex.setMessage("incompatible.eq.lower.bounds", uv.qtype, uv.getBounds(Type.UndetVar.InferenceBound.EQ), uv.getBounds(Type.UndetVar.InferenceBound.LOWER));
            }
        },
        UPPER { // from class: com.sun.tools.javac.comp.Infer.BoundErrorKind.4
            @Override // com.sun.tools.javac.comp.Infer.BoundErrorKind
            Resolve.InapplicableMethodException setMessage(InferenceException ex, Type.UndetVar uv) {
                return ex.setMessage("inferred.do.not.conform.to.upper.bounds", uv.inst, uv.getBounds(Type.UndetVar.InferenceBound.UPPER));
            }
        },
        LOWER { // from class: com.sun.tools.javac.comp.Infer.BoundErrorKind.5
            @Override // com.sun.tools.javac.comp.Infer.BoundErrorKind
            Resolve.InapplicableMethodException setMessage(InferenceException ex, Type.UndetVar uv) {
                return ex.setMessage("inferred.do.not.conform.to.lower.bounds", uv.inst, uv.getBounds(Type.UndetVar.InferenceBound.LOWER));
            }
        },
        EQ { // from class: com.sun.tools.javac.comp.Infer.BoundErrorKind.6
            @Override // com.sun.tools.javac.comp.Infer.BoundErrorKind
            Resolve.InapplicableMethodException setMessage(InferenceException ex, Type.UndetVar uv) {
                return ex.setMessage("inferred.do.not.conform.to.eq.bounds", uv.inst, uv.getBounds(Type.UndetVar.InferenceBound.EQ));
            }
        };

        abstract Resolve.InapplicableMethodException setMessage(InferenceException inferenceException, Type.UndetVar undetVar);
    }

    interface FreeTypeListener {
        void typesInferred(InferenceContext inferenceContext);
    }

    enum IncorporationBinaryOpKind {
        IS_SUBTYPE { // from class: com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind.1
            @Override // com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind
            boolean apply(Type op1, Type op2, Warner warn, Types types) {
                return types.isSubtypeUnchecked(op1, op2, warn);
            }
        },
        IS_SAME_TYPE { // from class: com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind.2
            @Override // com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind
            boolean apply(Type op1, Type op2, Warner warn, Types types) {
                return types.isSameType(op1, op2);
            }
        },
        ADD_UPPER_BOUND { // from class: com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind.3
            @Override // com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind
            boolean apply(Type op1, Type op2, Warner warn, Types types) {
                Type.UndetVar uv = (Type.UndetVar) op1;
                uv.addBound(Type.UndetVar.InferenceBound.UPPER, op2, types);
                return true;
            }
        },
        ADD_LOWER_BOUND { // from class: com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind.4
            @Override // com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind
            boolean apply(Type op1, Type op2, Warner warn, Types types) {
                Type.UndetVar uv = (Type.UndetVar) op1;
                uv.addBound(Type.UndetVar.InferenceBound.LOWER, op2, types);
                return true;
            }
        },
        ADD_EQ_BOUND { // from class: com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind.5
            @Override // com.sun.tools.javac.comp.Infer.IncorporationBinaryOpKind
            boolean apply(Type op1, Type op2, Warner warn, Types types) {
                Type.UndetVar uv = (Type.UndetVar) op1;
                uv.addBound(Type.UndetVar.InferenceBound.EQ, op2, types);
                return true;
            }
        };

        abstract boolean apply(Type type, Type type2, Warner warner, Types types);
    }

    public static Infer instance(Context context) {
        Infer instance = (Infer) context.get(inferKey);
        if (instance == null) {
            return new Infer(context);
        }
        return instance;
    }

    protected Infer(Context context) {
        context.put(inferKey, this);
        this.rs = Resolve.instance(context);
        this.chk = Check.instance(context);
        this.syms = Symtab.instance(context);
        this.types = Types.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        this.log = Log.instance(context);
        this.inferenceException = new InferenceException(this.diags);
        Options options = Options.instance(context);
        this.allowGraphInference = Source.instance(context).allowGraphInference() && options.isUnset("useLegacyInference");
    }

    public static class InferenceException extends Resolve.InapplicableMethodException {
        private static final long serialVersionUID = 0;
        List<JCDiagnostic> messages;

        InferenceException(JCDiagnostic.Factory diags) {
            super(diags);
            this.messages = List.nil();
        }

        @Override // com.sun.tools.javac.comp.Resolve.InapplicableMethodException
        Resolve.InapplicableMethodException setMessage() {
            return this;
        }

        @Override // com.sun.tools.javac.comp.Resolve.InapplicableMethodException
        Resolve.InapplicableMethodException setMessage(JCDiagnostic diag) {
            this.messages = this.messages.append(diag);
            return this;
        }

        @Override // com.sun.tools.javac.comp.Resolve.InapplicableMethodException
        public JCDiagnostic getDiagnostic() {
            return this.messages.head;
        }

        void clear() {
            this.messages = List.nil();
        }
    }

    /* JADX WARN: Removed duplicated region for block: B:24:0x008b A[Catch: all -> 0x0106, TryCatch #1 {all -> 0x0106, blocks: (B:22:0x0084, B:24:0x008b, B:26:0x0097, B:25:0x008f), top: B:66:0x0084 }] */
    /* JADX WARN: Removed duplicated region for block: B:25:0x008f A[Catch: all -> 0x0106, TryCatch #1 {all -> 0x0106, blocks: (B:22:0x0084, B:24:0x008b, B:26:0x0097, B:25:0x008f), top: B:66:0x0084 }] */
    /* JADX WARN: Removed duplicated region for block: B:46:0x00f9  */
    /* JADX WARN: Removed duplicated region for block: B:48:0x00fe  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    com.sun.tools.javac.code.Type instantiateMethod(com.sun.tools.javac.comp.Env<com.sun.tools.javac.comp.AttrContext> r17, com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type> r18, com.sun.tools.javac.code.Type.MethodType r19, com.sun.tools.javac.comp.Attr.ResultInfo r20, com.sun.tools.javac.code.Symbol.MethodSymbol r21, com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type> r22, boolean r23, boolean r24, com.sun.tools.javac.comp.Resolve.MethodResolutionContext r25, com.sun.tools.javac.util.Warner r26) throws java.lang.Throwable {
        /*
            Method dump skipped, instruction units count: 293
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Infer.instantiateMethod(com.sun.tools.javac.comp.Env, com.sun.tools.javac.util.List, com.sun.tools.javac.code.Type$MethodType, com.sun.tools.javac.comp.Attr$ResultInfo, com.sun.tools.javac.code.Symbol$MethodSymbol, com.sun.tools.javac.util.List, boolean, boolean, com.sun.tools.javac.comp.Resolve$MethodResolutionContext, com.sun.tools.javac.util.Warner):com.sun.tools.javac.code.Type");
    }

    Type generateReturnConstraints(JCTree tree, Attr.ResultInfo resultInfo, Type.MethodType mt, InferenceContext inferenceContext) {
        InferenceContext rsInfoInfContext = resultInfo.checkContext.inferenceContext();
        Type from = mt.mo178getReturnType();
        if (mt.mo178getReturnType().containsAny(inferenceContext.inferencevars) && rsInfoInfContext != this.emptyContext) {
            from = this.types.capture(from);
            for (Type t : from.getTypeArguments()) {
                if (t.hasTag(TypeTag.TYPEVAR) && ((Type.TypeVar) t).isCaptured()) {
                    inferenceContext.addVar((Type.TypeVar) t);
                }
            }
        }
        Type qtype = inferenceContext.asUndetVar(from);
        Type to = resultInfo.pt;
        if (qtype.hasTag(TypeTag.VOID)) {
            to = this.syms.voidType;
        } else if (to.hasTag(TypeTag.NONE)) {
            to = from.isPrimitive() ? from : this.syms.objectType;
        } else if (qtype.hasTag(TypeTag.UNDETVAR)) {
            if (resultInfo.pt.isReference()) {
                to = generateReturnConstraintsUndetVarToReference(tree, (Type.UndetVar) qtype, to, resultInfo, inferenceContext);
            } else if (to.isPrimitive()) {
                to = generateReturnConstraintsPrimitive(tree, (Type.UndetVar) qtype, to, resultInfo, inferenceContext);
            }
        }
        Assert.check(this.allowGraphInference || !rsInfoInfContext.free(to), "legacy inference engine cannot handle constraints on both sides of a subtyping assertion");
        Warner retWarn = new Warner();
        if (!resultInfo.checkContext.compatible(qtype, rsInfoInfContext.asUndetVar(to), retWarn) || (!this.allowGraphInference && retWarn.hasLint(Lint.LintCategory.UNCHECKED))) {
            throw this.inferenceException.setMessage("infer.no.conforming.instance.exists", inferenceContext.restvars(), mt.mo178getReturnType(), to);
        }
        return from;
    }

    private Type generateReturnConstraintsPrimitive(JCTree tree, Type.UndetVar from, Type to, Attr.ResultInfo resultInfo, InferenceContext inferenceContext) {
        if (!this.allowGraphInference) {
            return this.types.boxedClass(to).type;
        }
        for (Type t : from.getBounds(Type.UndetVar.InferenceBound.EQ, Type.UndetVar.InferenceBound.UPPER, Type.UndetVar.InferenceBound.LOWER)) {
            Type boundAsPrimitive = this.types.unboxedType(t);
            if (boundAsPrimitive != null && !boundAsPrimitive.hasTag(TypeTag.NONE)) {
                return generateReferenceToTargetConstraint(tree, from, to, resultInfo, inferenceContext);
            }
        }
        return this.types.boxedClass(to).type;
    }

    private Type generateReturnConstraintsUndetVarToReference(JCTree tree, Type.UndetVar from, Type to, Attr.ResultInfo resultInfo, InferenceContext inferenceContext) {
        Type captureOfTo = this.types.capture(to);
        if (captureOfTo == to) {
            for (Type t : from.getBounds(Type.UndetVar.InferenceBound.EQ, Type.UndetVar.InferenceBound.LOWER)) {
                Type captureOfBound = this.types.capture(t);
                if (captureOfBound != t) {
                    return generateReferenceToTargetConstraint(tree, from, to, resultInfo, inferenceContext);
                }
            }
            for (Type aLowerBound : from.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                for (Type anotherLowerBound : from.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                    if (aLowerBound != anotherLowerBound && !inferenceContext.free(aLowerBound) && !inferenceContext.free(anotherLowerBound) && commonSuperWithDiffParameterization(aLowerBound, anotherLowerBound)) {
                        return generateReferenceToTargetConstraint(tree, from, to, resultInfo, inferenceContext);
                    }
                }
            }
        }
        if (to.isParameterized()) {
            Iterator<Type> it = from.getBounds(Type.UndetVar.InferenceBound.EQ, Type.UndetVar.InferenceBound.LOWER).iterator();
            while (it.hasNext()) {
                Type sup = this.types.asSuper(it.next(), to.tsym);
                if (sup != null && sup.isRaw()) {
                    return generateReferenceToTargetConstraint(tree, from, to, resultInfo, inferenceContext);
                }
            }
        }
        return to;
    }

    private boolean commonSuperWithDiffParameterization(Type t, Type s) {
        Pair<Type, Type> supers = getParameterizedSupers(t, s);
        return (supers == null || this.types.isSameType(supers.fst, supers.snd)) ? false : true;
    }

    private Type generateReferenceToTargetConstraint(JCTree tree, Type.UndetVar from, Type to, Attr.ResultInfo resultInfo, InferenceContext inferenceContext) {
        inferenceContext.solve(List.of(from.qtype), new Warner());
        inferenceContext.notifyChange();
        Type capturedType = resultInfo.checkContext.inferenceContext().cachedCapture(tree, from.inst, false);
        if (this.types.isConvertible(capturedType, resultInfo.checkContext.inferenceContext().asUndetVar(to))) {
            return this.syms.objectType;
        }
        return to;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void instantiateAsUninferredVars(List<Type> vars, InferenceContext inferenceContext) {
        ListBuffer<Type> todo = new ListBuffer<>();
        for (Type t : vars) {
            Type.UndetVar uv = (Type.UndetVar) inferenceContext.asUndetVar(t);
            List<Type> upperBounds = uv.getBounds(Type.UndetVar.InferenceBound.UPPER);
            if (Type.containsAny(upperBounds, vars)) {
                Symbol.TypeSymbol fresh_tvar = new Symbol.TypeVariableSymbol(4096L, uv.qtype.tsym.name, null, uv.qtype.tsym.owner);
                fresh_tvar.type = new Type.TypeVar(fresh_tvar, this.types.makeIntersectionType(uv.getBounds(Type.UndetVar.InferenceBound.UPPER)), (Type) null);
                todo.append(uv);
                uv.inst = fresh_tvar.type;
            } else if (upperBounds.nonEmpty()) {
                uv.inst = this.types.glb(upperBounds);
            } else {
                uv.inst = this.syms.objectType;
            }
        }
        List list = vars;
        for (Type t2 : todo) {
            Type.UndetVar uv2 = (Type.UndetVar) t2;
            Type.TypeVar ct = (Type.TypeVar) uv2.inst;
            ct.bound = this.types.glb(inferenceContext.asInstTypes(this.types.getBounds(ct)));
            if (ct.bound.isErroneous()) {
                reportBoundError(uv2, BoundErrorKind.BAD_UPPER);
            }
            list = list.tail;
        }
    }

    Type instantiatePolymorphicSignatureInstance(Env<AttrContext> env, Symbol.MethodSymbol spMethod, Resolve.MethodResolutionContext resolveContext, List<Type> argtypes) {
        Type restype;
        List<Type> exType;
        switch (env.next.tree.getTag()) {
            case TYPECAST:
                JCTree.JCTypeCast castTree = (JCTree.JCTypeCast) env.next.tree;
                restype = TreeInfo.skipParens(castTree.expr) != env.tree ? this.syms.objectType : castTree.clazz.type;
                break;
            case EXEC:
                JCTree.JCExpressionStatement execTree = (JCTree.JCExpressionStatement) env.next.tree;
                restype = TreeInfo.skipParens(execTree.expr) != env.tree ? this.syms.objectType : this.syms.voidType;
                break;
            default:
                restype = this.syms.objectType;
                break;
        }
        List<Type> paramtypes = Type.map(argtypes, new ImplicitArgType(spMethod, resolveContext.step));
        if (spMethod != null) {
            exType = spMethod.getThrownTypes();
        } else {
            exType = List.of(this.syms.throwableType);
        }
        Type.MethodType mtype = new Type.MethodType(paramtypes, restype, exType, this.syms.methodClass);
        return mtype;
    }

    class ImplicitArgType extends DeferredAttr.DeferredTypeMap {
        /* JADX WARN: Illegal instructions before constructor call */
        public ImplicitArgType(Symbol msym, Resolve.MethodResolutionPhase phase) {
            DeferredAttr deferredAttr = Infer.this.rs.deferredAttr;
            deferredAttr.getClass();
            super(DeferredAttr.AttrMode.SPECULATIVE, msym, phase);
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredTypeMap, com.sun.tools.javac.code.Type.Mapping
        public Type apply(Type t) {
            Type t2 = Infer.this.types.erasure(super.apply(t));
            if (t2.hasTag(TypeTag.BOT)) {
                Type t3 = Infer.this.types.boxedClass(Infer.this.syms.voidType).type;
                return t3;
            }
            return t2;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public Type instantiateFunctionalInterface(JCDiagnostic.DiagnosticPosition diagnosticPosition, Type type, List<Type> list, Check.CheckContext checkContext) {
        if (this.types.capture(type) == type) {
            return type;
        }
        Type type2 = type.tsym.type;
        InferenceContext inferenceContext = new InferenceContext(type.tsym.type.getTypeArguments());
        Assert.check(list != null);
        List<Type> listMo176getParameterTypes = this.types.findDescriptorType(type2).mo176getParameterTypes();
        if (listMo176getParameterTypes.size() != list.size()) {
            checkContext.report(diagnosticPosition, this.diags.fragment("incompatible.arg.types.in.lambda", new Object[0]));
            return this.types.createErrorType(type);
        }
        Iterator<Type> it = listMo176getParameterTypes.iterator();
        List list2 = list;
        while (it.hasNext()) {
            if (!this.types.isSameType(inferenceContext.asUndetVar(it.next()), (Type) list2.head)) {
                checkContext.report(diagnosticPosition, this.diags.fragment("no.suitable.functional.intf.inst", type));
                return this.types.createErrorType(type);
            }
            list2 = list2.tail;
        }
        try {
            inferenceContext.solve(inferenceContext.boundedVars(), this.types.noWarnings);
        } catch (InferenceException e) {
            checkContext.report(diagnosticPosition, this.diags.fragment("no.suitable.functional.intf.inst", type));
        }
        List typeArguments = type.getTypeArguments();
        Iterator<Type> it2 = inferenceContext.undetvars.iterator();
        while (it2.hasNext()) {
            Type.UndetVar undetVar = (Type.UndetVar) it2.next();
            if (undetVar.inst == null) {
                undetVar.inst = (Type) typeArguments.head;
            }
            typeArguments = typeArguments.tail;
        }
        Type typeAsInstType = inferenceContext.asInstType(type2);
        if (!this.chk.checkValidGenericType(typeAsInstType)) {
            checkContext.report(diagnosticPosition, this.diags.fragment("no.suitable.functional.intf.inst", type));
        }
        checkContext.compatible(typeAsInstType, type, this.types.noWarnings);
        return typeAsInstType;
    }

    void checkWithinBounds(InferenceContext inferenceContext, Warner warn) throws InferenceException {
        MultiUndetVarListener mlistener = new MultiUndetVarListener(inferenceContext.undetvars);
        List<Type> saved_undet = inferenceContext.save();
        do {
            try {
                mlistener.reset();
                if (!this.allowGraphInference) {
                    for (Type t : inferenceContext.undetvars) {
                        IncorporationStep.CHECK_BOUNDS.apply((Type.UndetVar) t, inferenceContext, warn);
                    }
                }
                for (Type t2 : inferenceContext.undetvars) {
                    Type.UndetVar uv = (Type.UndetVar) t2;
                    EnumSet<IncorporationStep> incorporationSteps = this.allowGraphInference ? this.incorporationStepsGraph : this.incorporationStepsLegacy;
                    for (IncorporationStep is : incorporationSteps) {
                        if (is.accepts(uv, inferenceContext)) {
                            is.apply(uv, inferenceContext, warn);
                        }
                    }
                }
                if (!mlistener.changed) {
                    break;
                }
            } finally {
                mlistener.detach();
                if (this.incorporationCache.size() == 100) {
                    inferenceContext.rollback(saved_undet);
                }
                this.incorporationCache.clear();
            }
        } while (this.allowGraphInference);
    }

    class MultiUndetVarListener implements Type.UndetVar.UndetVarListener {
        boolean changed;
        List<Type> undetvars;

        public MultiUndetVarListener(List<Type> undetvars) {
            this.undetvars = undetvars;
            for (Type t : undetvars) {
                Type.UndetVar uv = (Type.UndetVar) t;
                uv.listener = this;
            }
        }

        @Override // com.sun.tools.javac.code.Type.UndetVar.UndetVarListener
        public void varChanged(Type.UndetVar uv, Set<Type.UndetVar.InferenceBound> ibs) {
            if (Infer.this.incorporationCache.size() < 100) {
                this.changed = true;
            }
        }

        void reset() {
            this.changed = false;
        }

        void detach() {
            for (Type t : this.undetvars) {
                Type.UndetVar uv = (Type.UndetVar) t;
                uv.listener = null;
            }
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Pair<Type, Type> getParameterizedSupers(Type t, Type s) {
        Type lubResult = this.types.lub(t, s);
        if (lubResult == this.syms.errType || lubResult == this.syms.botType || !lubResult.isParameterized()) {
            return null;
        }
        Type asSuperOfT = this.types.asSuper(t, lubResult.tsym);
        Type asSuperOfS = this.types.asSuper(s, lubResult.tsym);
        return new Pair<>(asSuperOfT, asSuperOfS);
    }

    enum IncorporationStep {
        CHECK_BOUNDS { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.1
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) throws Throwable {
                Infer infer = inferenceContext.infer();
                uv.substBounds(inferenceContext.inferenceVars(), inferenceContext.instTypes(), infer.types);
                infer.checkCompatibleUpperBounds(uv, inferenceContext);
                if (uv.inst != null) {
                    Type inst = uv.inst;
                    for (Type u : uv.getBounds(Type.UndetVar.InferenceBound.UPPER)) {
                        if (!isSubtype(inst, inferenceContext.asUndetVar(u), warn, infer)) {
                            infer.reportBoundError(uv, BoundErrorKind.UPPER);
                        }
                    }
                    for (Type l : uv.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                        if (!isSubtype(inferenceContext.asUndetVar(l), inst, warn, infer)) {
                            infer.reportBoundError(uv, BoundErrorKind.LOWER);
                        }
                    }
                    for (Type e : uv.getBounds(Type.UndetVar.InferenceBound.EQ)) {
                        if (!isSameType(inst, inferenceContext.asUndetVar(e), infer)) {
                            infer.reportBoundError(uv, BoundErrorKind.EQ);
                        }
                    }
                }
            }

            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            boolean accepts(Type.UndetVar uv, InferenceContext inferenceContext) {
                return true;
            }
        },
        EQ_CHECK_LEGACY { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.2
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                Type eq = null;
                for (Type e : uv.getBounds(Type.UndetVar.InferenceBound.EQ)) {
                    Assert.check(!inferenceContext.free(e));
                    if (eq != null && !isSameType(e, eq, infer)) {
                        infer.reportBoundError(uv, BoundErrorKind.EQ);
                    }
                    eq = e;
                    for (Type l : uv.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                        Assert.check(!inferenceContext.free(l));
                        if (!isSubtype(l, e, warn, infer)) {
                            infer.reportBoundError(uv, BoundErrorKind.BAD_EQ_LOWER);
                        }
                    }
                    for (Type u : uv.getBounds(Type.UndetVar.InferenceBound.UPPER)) {
                        if (!inferenceContext.free(u) && !isSubtype(e, u, warn, infer)) {
                            infer.reportBoundError(uv, BoundErrorKind.BAD_EQ_UPPER);
                        }
                    }
                }
            }
        },
        EQ_CHECK { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.3
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                for (Type e : uv.getBounds(Type.UndetVar.InferenceBound.EQ)) {
                    if (!e.containsAny(inferenceContext.inferenceVars())) {
                        for (Type u : uv.getBounds(Type.UndetVar.InferenceBound.UPPER)) {
                            if (!isSubtype(e, inferenceContext.asUndetVar(u), warn, infer)) {
                                infer.reportBoundError(uv, BoundErrorKind.BAD_EQ_UPPER);
                            }
                        }
                        for (Type l : uv.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                            if (!isSubtype(inferenceContext.asUndetVar(l), e, warn, infer)) {
                                infer.reportBoundError(uv, BoundErrorKind.BAD_EQ_LOWER);
                            }
                        }
                    }
                }
            }
        },
        CROSS_UPPER_LOWER { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.4
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                for (Type b1 : uv.getBounds(Type.UndetVar.InferenceBound.UPPER)) {
                    for (Type b2 : uv.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                        isSubtype(inferenceContext.asUndetVar(b2), inferenceContext.asUndetVar(b1), warn, infer);
                    }
                }
            }
        },
        CROSS_UPPER_EQ { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.5
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                for (Type b1 : uv.getBounds(Type.UndetVar.InferenceBound.UPPER)) {
                    for (Type b2 : uv.getBounds(Type.UndetVar.InferenceBound.EQ)) {
                        isSubtype(inferenceContext.asUndetVar(b2), inferenceContext.asUndetVar(b1), warn, infer);
                    }
                }
            }
        },
        CROSS_EQ_LOWER { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.6
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                for (Type b1 : uv.getBounds(Type.UndetVar.InferenceBound.EQ)) {
                    for (Type b2 : uv.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                        isSubtype(inferenceContext.asUndetVar(b2), inferenceContext.asUndetVar(b1), warn, infer);
                    }
                }
            }
        },
        CROSS_UPPER_UPPER { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.7
            /* JADX WARN: Multi-variable type inference failed */
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                List bounds = uv.getBounds(Type.UndetVar.InferenceBound.UPPER);
                List list = bounds.tail;
                while (bounds.nonEmpty()) {
                    for (List list2 = list; list2.nonEmpty(); list2 = list2.tail) {
                        Type b1 = (Type) bounds.head;
                        Type b2 = (Type) list2.head;
                        if (b1 != b2 && !b1.hasTag(TypeTag.WILDCARD) && !b2.hasTag(TypeTag.WILDCARD)) {
                            Pair<Type, Type> commonSupers = infer.getParameterizedSupers(b1, b2);
                            if (commonSupers != null) {
                                List listAllparams = commonSupers.fst.allparams();
                                List listAllparams2 = commonSupers.snd.allparams();
                                while (listAllparams.nonEmpty() && listAllparams2.nonEmpty()) {
                                    if (!((Type) listAllparams.head).hasTag(TypeTag.WILDCARD) && !((Type) listAllparams2.head).hasTag(TypeTag.WILDCARD)) {
                                        isSameType(inferenceContext.asUndetVar((Type) listAllparams.head), inferenceContext.asUndetVar((Type) listAllparams2.head), infer);
                                    }
                                    listAllparams = listAllparams.tail;
                                    listAllparams2 = listAllparams2.tail;
                                }
                                Assert.check(listAllparams.isEmpty() && listAllparams2.isEmpty());
                            }
                        }
                    }
                    bounds = bounds.tail;
                    list = bounds.tail;
                }
            }

            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            boolean accepts(Type.UndetVar uv, InferenceContext inferenceContext) {
                return !uv.isCaptured() && uv.getBounds(Type.UndetVar.InferenceBound.UPPER).nonEmpty();
            }
        },
        CROSS_EQ_EQ { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.8
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                for (Type b1 : uv.getBounds(Type.UndetVar.InferenceBound.EQ)) {
                    for (Type b2 : uv.getBounds(Type.UndetVar.InferenceBound.EQ)) {
                        if (b1 != b2) {
                            isSameType(inferenceContext.asUndetVar(b2), inferenceContext.asUndetVar(b1), infer);
                        }
                    }
                }
            }
        },
        PROP_UPPER { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.9
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                for (Type b : uv.getBounds(Type.UndetVar.InferenceBound.UPPER)) {
                    if (inferenceContext.inferenceVars().contains(b)) {
                        Type.UndetVar uv2 = (Type.UndetVar) inferenceContext.asUndetVar(b);
                        if (!uv2.isCaptured()) {
                            addBound(Type.UndetVar.InferenceBound.LOWER, uv2, inferenceContext.asInstType(uv.qtype), infer);
                            for (Type l : uv.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                                addBound(Type.UndetVar.InferenceBound.LOWER, uv2, inferenceContext.asInstType(l), infer);
                            }
                            for (Type u : uv2.getBounds(Type.UndetVar.InferenceBound.UPPER)) {
                                addBound(Type.UndetVar.InferenceBound.UPPER, uv, inferenceContext.asInstType(u), infer);
                            }
                        }
                    }
                }
            }
        },
        PROP_LOWER { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.10
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                for (Type b : uv.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                    if (inferenceContext.inferenceVars().contains(b)) {
                        Type.UndetVar uv2 = (Type.UndetVar) inferenceContext.asUndetVar(b);
                        if (!uv2.isCaptured()) {
                            addBound(Type.UndetVar.InferenceBound.UPPER, uv2, inferenceContext.asInstType(uv.qtype), infer);
                            for (Type u : uv.getBounds(Type.UndetVar.InferenceBound.UPPER)) {
                                addBound(Type.UndetVar.InferenceBound.UPPER, uv2, inferenceContext.asInstType(u), infer);
                            }
                            for (Type l : uv2.getBounds(Type.UndetVar.InferenceBound.LOWER)) {
                                addBound(Type.UndetVar.InferenceBound.LOWER, uv, inferenceContext.asInstType(l), infer);
                            }
                        }
                    }
                }
            }
        },
        PROP_EQ { // from class: com.sun.tools.javac.comp.Infer.IncorporationStep.11
            @Override // com.sun.tools.javac.comp.Infer.IncorporationStep
            public void apply(Type.UndetVar uv, InferenceContext inferenceContext, Warner warn) {
                Infer infer = inferenceContext.infer();
                for (Type b : uv.getBounds(Type.UndetVar.InferenceBound.EQ)) {
                    if (inferenceContext.inferenceVars().contains(b)) {
                        Type.UndetVar uv2 = (Type.UndetVar) inferenceContext.asUndetVar(b);
                        if (!uv2.isCaptured()) {
                            addBound(Type.UndetVar.InferenceBound.EQ, uv2, inferenceContext.asInstType(uv.qtype), infer);
                            for (Type.UndetVar.InferenceBound ib : Type.UndetVar.InferenceBound.values()) {
                                for (Type b2 : uv.getBounds(ib)) {
                                    if (b2 != uv2) {
                                        addBound(ib, uv2, inferenceContext.asInstType(b2), infer);
                                    }
                                }
                            }
                            for (Type.UndetVar.InferenceBound ib2 : Type.UndetVar.InferenceBound.values()) {
                                for (Type b22 : uv2.getBounds(ib2)) {
                                    if (b22 != uv) {
                                        addBound(ib2, uv, inferenceContext.asInstType(b22), infer);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        };

        abstract void apply(Type.UndetVar undetVar, InferenceContext inferenceContext, Warner warner);

        boolean accepts(Type.UndetVar uv, InferenceContext inferenceContext) {
            return !uv.isCaptured();
        }

        boolean isSubtype(Type s, Type t, Warner warn, Infer infer) {
            return doIncorporationOp(IncorporationBinaryOpKind.IS_SUBTYPE, s, t, warn, infer);
        }

        boolean isSameType(Type s, Type t, Infer infer) {
            return doIncorporationOp(IncorporationBinaryOpKind.IS_SAME_TYPE, s, t, null, infer);
        }

        void addBound(Type.UndetVar.InferenceBound ib, Type.UndetVar uv, Type b, Infer infer) {
            doIncorporationOp(opFor(ib), uv, b, null, infer);
        }

        IncorporationBinaryOpKind opFor(Type.UndetVar.InferenceBound boundKind) {
            switch (boundKind) {
                case EQ:
                    return IncorporationBinaryOpKind.ADD_EQ_BOUND;
                case LOWER:
                    return IncorporationBinaryOpKind.ADD_LOWER_BOUND;
                case UPPER:
                    return IncorporationBinaryOpKind.ADD_UPPER_BOUND;
                default:
                    Assert.error("Can't get here!");
                    return null;
            }
        }

        boolean doIncorporationOp(IncorporationBinaryOpKind opKind, Type op1, Type op2, Warner warn, Infer infer) {
            infer.getClass();
            IncorporationBinaryOp newOp = infer.new IncorporationBinaryOp(opKind, op1, op2);
            Boolean res = infer.incorporationCache.get(newOp);
            if (res == null) {
                Map<IncorporationBinaryOp, Boolean> map = infer.incorporationCache;
                Boolean boolValueOf = Boolean.valueOf(newOp.apply(warn));
                res = boolValueOf;
                map.put(newOp, boolValueOf);
            }
            return res.booleanValue();
        }
    }

    class IncorporationBinaryOp {
        Type op1;
        Type op2;
        IncorporationBinaryOpKind opKind;

        IncorporationBinaryOp(IncorporationBinaryOpKind opKind, Type op1, Type op2) {
            this.opKind = opKind;
            this.op1 = op1;
            this.op2 = op2;
        }

        public boolean equals(Object o) {
            if (!(o instanceof IncorporationBinaryOp)) {
                return false;
            }
            IncorporationBinaryOp that = (IncorporationBinaryOp) o;
            return this.opKind == that.opKind && Infer.this.types.isSameType(this.op1, that.op1, true) && Infer.this.types.isSameType(this.op2, that.op2, true);
        }

        public int hashCode() {
            int result = this.opKind.hashCode();
            return (((result * 127) + Infer.this.types.hashCode(this.op1)) * 127) + Infer.this.types.hashCode(this.op2);
        }

        boolean apply(Warner warn) {
            return this.opKind.apply(this.op1, this.op2, warn, Infer.this.types);
        }
    }

    void checkCompatibleUpperBounds(Type.UndetVar uv, InferenceContext inferenceContext) {
        Type hb;
        List<Type> hibounds = Type.filter(uv.getBounds(Type.UndetVar.InferenceBound.UPPER), new BoundFilter(inferenceContext));
        if (hibounds.isEmpty()) {
            hb = this.syms.objectType;
        } else if (hibounds.tail.isEmpty()) {
            Type hb2 = hibounds.head;
            hb = hb2;
        } else {
            hb = this.types.glb(hibounds);
        }
        if (hb == null || hb.isErroneous()) {
            reportBoundError(uv, BoundErrorKind.BAD_UPPER);
        }
    }

    protected static class BoundFilter implements Filter<Type> {
        InferenceContext inferenceContext;

        public BoundFilter(InferenceContext inferenceContext) {
            this.inferenceContext = inferenceContext;
        }

        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Type t) {
            return (t.isErroneous() || this.inferenceContext.free(t) || t.hasTag(TypeTag.BOT)) ? false : true;
        }
    }

    void reportBoundError(Type.UndetVar uv, BoundErrorKind bk) {
        throw bk.setMessage(this.inferenceException, uv);
    }

    interface GraphStrategy {
        boolean done();

        GraphSolver.InferenceGraph.Node pickNode(GraphSolver.InferenceGraph inferenceGraph) throws NodeNotFoundException;

        public static class NodeNotFoundException extends RuntimeException {
            private static final long serialVersionUID = 0;
            GraphSolver.InferenceGraph graph;

            public NodeNotFoundException(GraphSolver.InferenceGraph graph) {
                this.graph = graph;
            }
        }
    }

    abstract class LeafSolver implements GraphStrategy {
        LeafSolver() {
        }

        @Override // com.sun.tools.javac.comp.Infer.GraphStrategy
        public GraphSolver.InferenceGraph.Node pickNode(GraphSolver.InferenceGraph g) {
            if (g.nodes.isEmpty()) {
                throw new GraphStrategy.NodeNotFoundException(g);
            }
            return g.nodes.get(0);
        }

        boolean isSubtype(Type s, Type t, Warner warn, Infer infer) {
            return doIncorporationOp(IncorporationBinaryOpKind.IS_SUBTYPE, s, t, warn, infer);
        }

        boolean isSameType(Type s, Type t, Infer infer) {
            return doIncorporationOp(IncorporationBinaryOpKind.IS_SAME_TYPE, s, t, null, infer);
        }

        void addBound(Type.UndetVar.InferenceBound ib, Type.UndetVar uv, Type b, Infer infer) {
            doIncorporationOp(opFor(ib), uv, b, null, infer);
        }

        IncorporationBinaryOpKind opFor(Type.UndetVar.InferenceBound boundKind) {
            switch (boundKind) {
                case EQ:
                    return IncorporationBinaryOpKind.ADD_EQ_BOUND;
                case LOWER:
                    return IncorporationBinaryOpKind.ADD_LOWER_BOUND;
                case UPPER:
                    return IncorporationBinaryOpKind.ADD_UPPER_BOUND;
                default:
                    Assert.error("Can't get here!");
                    return null;
            }
        }

        boolean doIncorporationOp(IncorporationBinaryOpKind opKind, Type op1, Type op2, Warner warn, Infer infer) {
            infer.getClass();
            IncorporationBinaryOp newOp = infer.new IncorporationBinaryOp(opKind, op1, op2);
            Boolean res = infer.incorporationCache.get(newOp);
            if (res == null) {
                Map<IncorporationBinaryOp, Boolean> map = infer.incorporationCache;
                Boolean boolValueOf = Boolean.valueOf(newOp.apply(warn));
                res = boolValueOf;
                map.put(newOp, boolValueOf);
            }
            return res.booleanValue();
        }
    }

    abstract class BestLeafSolver extends LeafSolver {
        final Pair<List<GraphSolver.InferenceGraph.Node>, Integer> noPath;
        final Map<GraphSolver.InferenceGraph.Node, Pair<List<GraphSolver.InferenceGraph.Node>, Integer>> treeCache;
        List<Type> varsToSolve;

        BestLeafSolver(List<Type> varsToSolve) {
            super();
            this.treeCache = new HashMap();
            this.noPath = new Pair<>(null, Integer.MAX_VALUE);
            this.varsToSolve = varsToSolve;
        }

        /* JADX WARN: Multi-variable type inference failed */
        Pair<List<GraphSolver.InferenceGraph.Node>, Integer> computeTreeToLeafs(GraphSolver.InferenceGraph.Node n) {
            Pair<List<GraphSolver.InferenceGraph.Node>, Integer> cachedPath = this.treeCache.get(n);
            if (cachedPath == null) {
                if (n.isLeaf()) {
                    cachedPath = new Pair<>(List.of(n), Integer.valueOf(((ListBuffer) n.data).length()));
                } else {
                    Pair<List<GraphSolver.InferenceGraph.Node>, Integer> path = new Pair<>(List.of(n), Integer.valueOf(((ListBuffer) n.data).length()));
                    for (GraphSolver.InferenceGraph.Node n2 : n.getAllDependencies()) {
                        if (n2 != n) {
                            Pair<List<GraphSolver.InferenceGraph.Node>, Integer> subpath = computeTreeToLeafs(n2);
                            path = new Pair<>(path.fst.prependList(subpath.fst), Integer.valueOf(path.snd.intValue() + subpath.snd.intValue()));
                        }
                    }
                    cachedPath = path;
                }
                this.treeCache.put(n, cachedPath);
            }
            return cachedPath;
        }

        @Override // com.sun.tools.javac.comp.Infer.LeafSolver, com.sun.tools.javac.comp.Infer.GraphStrategy
        public GraphSolver.InferenceGraph.Node pickNode(GraphSolver.InferenceGraph g) {
            this.treeCache.clear();
            Pair<List<GraphSolver.InferenceGraph.Node>, Integer> bestPath = this.noPath;
            for (GraphSolver.InferenceGraph.Node n : g.nodes) {
                if (!Collections.disjoint((Collection) n.data, this.varsToSolve)) {
                    Pair<List<GraphSolver.InferenceGraph.Node>, Integer> path = computeTreeToLeafs(n);
                    if (path.snd.intValue() < bestPath.snd.intValue()) {
                        bestPath = path;
                    }
                }
            }
            if (bestPath == this.noPath) {
                throw new GraphStrategy.NodeNotFoundException(g);
            }
            return bestPath.fst.head;
        }
    }

    enum InferenceStep {
        EQ(Type.UndetVar.InferenceBound.EQ) { // from class: com.sun.tools.javac.comp.Infer.InferenceStep.1
            @Override // com.sun.tools.javac.comp.Infer.InferenceStep
            Type solve(Type.UndetVar uv, InferenceContext inferenceContext) {
                return filterBounds(uv, inferenceContext).head;
            }
        },
        LOWER(Type.UndetVar.InferenceBound.LOWER) { // from class: com.sun.tools.javac.comp.Infer.InferenceStep.2
            @Override // com.sun.tools.javac.comp.Infer.InferenceStep
            Type solve(Type.UndetVar uv, InferenceContext inferenceContext) {
                Infer infer = inferenceContext.infer();
                List<Type> lobounds = filterBounds(uv, inferenceContext);
                Type owntype = lobounds.tail.tail == null ? lobounds.head : infer.types.lub(lobounds);
                if (owntype.isPrimitive() || owntype.hasTag(TypeTag.ERROR)) {
                    throw infer.inferenceException.setMessage("no.unique.minimal.instance.exists", uv.qtype, lobounds);
                }
                return owntype;
            }
        },
        THROWS(Type.UndetVar.InferenceBound.UPPER) { // from class: com.sun.tools.javac.comp.Infer.InferenceStep.3
            @Override // com.sun.tools.javac.comp.Infer.InferenceStep
            public boolean accepts(Type.UndetVar t, InferenceContext inferenceContext) {
                if ((t.qtype.tsym.flags() & Flags.THROWS) == 0 || t.getBounds(Type.UndetVar.InferenceBound.EQ, Type.UndetVar.InferenceBound.LOWER, Type.UndetVar.InferenceBound.UPPER).diff(t.getDeclaredBounds()).nonEmpty()) {
                    return false;
                }
                Infer infer = inferenceContext.infer();
                for (Type db : t.getDeclaredBounds()) {
                    if (!t.isInterface() && infer.types.asSuper(infer.syms.runtimeExceptionType, db.tsym) != null) {
                        return true;
                    }
                }
                return false;
            }

            @Override // com.sun.tools.javac.comp.Infer.InferenceStep
            Type solve(Type.UndetVar uv, InferenceContext inferenceContext) {
                return inferenceContext.infer().syms.runtimeExceptionType;
            }
        },
        UPPER(Type.UndetVar.InferenceBound.UPPER) { // from class: com.sun.tools.javac.comp.Infer.InferenceStep.4
            @Override // com.sun.tools.javac.comp.Infer.InferenceStep
            Type solve(Type.UndetVar uv, InferenceContext inferenceContext) {
                Infer infer = inferenceContext.infer();
                List<Type> hibounds = filterBounds(uv, inferenceContext);
                Type owntype = hibounds.tail.tail == null ? hibounds.head : infer.types.glb(hibounds);
                if (owntype.isPrimitive() || owntype.hasTag(TypeTag.ERROR)) {
                    throw infer.inferenceException.setMessage("no.unique.maximal.instance.exists", uv.qtype, hibounds);
                }
                return owntype;
            }
        },
        UPPER_LEGACY(Type.UndetVar.InferenceBound.UPPER) { // from class: com.sun.tools.javac.comp.Infer.InferenceStep.5
            @Override // com.sun.tools.javac.comp.Infer.InferenceStep
            public boolean accepts(Type.UndetVar t, InferenceContext inferenceContext) {
                return (inferenceContext.free(t.getBounds(this.ib)) || t.isCaptured()) ? false : true;
            }

            @Override // com.sun.tools.javac.comp.Infer.InferenceStep
            Type solve(Type.UndetVar uv, InferenceContext inferenceContext) {
                return UPPER.solve(uv, inferenceContext);
            }
        },
        CAPTURED(Type.UndetVar.InferenceBound.UPPER) { // from class: com.sun.tools.javac.comp.Infer.InferenceStep.6
            @Override // com.sun.tools.javac.comp.Infer.InferenceStep
            public boolean accepts(Type.UndetVar t, InferenceContext inferenceContext) {
                return t.isCaptured() && !inferenceContext.free(t.getBounds(Type.UndetVar.InferenceBound.UPPER, Type.UndetVar.InferenceBound.LOWER));
            }

            @Override // com.sun.tools.javac.comp.Infer.InferenceStep
            Type solve(Type.UndetVar uv, InferenceContext inferenceContext) {
                Infer infer = inferenceContext.infer();
                Type upper = UPPER.filterBounds(uv, inferenceContext).nonEmpty() ? UPPER.solve(uv, inferenceContext) : infer.syms.objectType;
                Type lower = LOWER.filterBounds(uv, inferenceContext).nonEmpty() ? LOWER.solve(uv, inferenceContext) : infer.syms.botType;
                Type.CapturedType prevCaptured = (Type.CapturedType) uv.qtype;
                return new Type.CapturedType(prevCaptured.tsym.name, prevCaptured.tsym.owner, upper, lower, prevCaptured.wildcard);
            }
        };

        final Type.UndetVar.InferenceBound ib;

        abstract Type solve(Type.UndetVar undetVar, InferenceContext inferenceContext);

        InferenceStep(Type.UndetVar.InferenceBound ib) {
            this.ib = ib;
        }

        public boolean accepts(Type.UndetVar t, InferenceContext inferenceContext) {
            return filterBounds(t, inferenceContext).nonEmpty() && !t.isCaptured();
        }

        List<Type> filterBounds(Type.UndetVar uv, InferenceContext inferenceContext) {
            return Type.filter(uv.getBounds(this.ib), new BoundFilter(inferenceContext));
        }
    }

    enum LegacyInferenceSteps {
        EQ_LOWER(EnumSet.of(InferenceStep.EQ, InferenceStep.LOWER)),
        EQ_UPPER(EnumSet.of(InferenceStep.EQ, InferenceStep.UPPER_LEGACY));

        final EnumSet<InferenceStep> steps;

        LegacyInferenceSteps(EnumSet enumSet) {
            this.steps = enumSet;
        }
    }

    enum GraphInferenceSteps {
        EQ(EnumSet.of(InferenceStep.EQ)),
        EQ_LOWER(EnumSet.of(InferenceStep.EQ, InferenceStep.LOWER)),
        EQ_LOWER_THROWS_UPPER_CAPTURED(EnumSet.of(InferenceStep.EQ, InferenceStep.LOWER, InferenceStep.UPPER, InferenceStep.THROWS, InferenceStep.CAPTURED));

        final EnumSet<InferenceStep> steps;

        GraphInferenceSteps(EnumSet enumSet) {
            this.steps = enumSet;
        }
    }

    enum DependencyKind implements GraphUtils.DependencyKind {
        BOUND("dotted"),
        STUCK("dashed");

        final String dotSyle;

        DependencyKind(String dotSyle) {
            this.dotSyle = dotSyle;
        }

        @Override // com.sun.tools.javac.util.GraphUtils.DependencyKind
        public String getDotStyle() {
            return this.dotSyle;
        }
    }

    class GraphSolver {
        InferenceContext inferenceContext;
        Map<Type, Set<Type>> stuckDeps;
        Warner warn;

        GraphSolver(InferenceContext inferenceContext, Map<Type, Set<Type>> stuckDeps, Warner warn) {
            this.inferenceContext = inferenceContext;
            this.stuckDeps = stuckDeps;
            this.warn = warn;
        }

        /* JADX WARN: Code restructure failed: missing block: B:12:0x0048, code lost:
        
            r10.this$0.checkWithinBounds(r10.inferenceContext, r10.warn);
         */
        /*
            Code decompiled incorrectly, please refer to instructions dump.
            To view partially-correct code enable 'Show inconsistent code' option in preferences
        */
        void solve(com.sun.tools.javac.comp.Infer.GraphStrategy r11) {
            /*
                r10 = this;
                com.sun.tools.javac.comp.Infer r0 = com.sun.tools.javac.comp.Infer.this
                com.sun.tools.javac.comp.Infer$InferenceContext r1 = r10.inferenceContext
                com.sun.tools.javac.util.Warner r2 = r10.warn
                r0.checkWithinBounds(r1, r2)
                com.sun.tools.javac.comp.Infer$GraphSolver$InferenceGraph r0 = new com.sun.tools.javac.comp.Infer$GraphSolver$InferenceGraph
                java.util.Map<com.sun.tools.javac.code.Type, java.util.Set<com.sun.tools.javac.code.Type>> r1 = r10.stuckDeps
                r0.<init>(r1)
            L10:
                boolean r1 = r11.done()
                if (r1 != 0) goto L79
                com.sun.tools.javac.comp.Infer$GraphSolver$InferenceGraph$Node r1 = r11.pickNode(r0)
                D r2 = r1.data
                java.lang.Iterable r2 = (java.lang.Iterable) r2
                com.sun.tools.javac.util.List r2 = com.sun.tools.javac.util.List.from(r2)
                com.sun.tools.javac.comp.Infer$InferenceContext r3 = r10.inferenceContext
                com.sun.tools.javac.util.List r3 = r3.save()
            L28:
                com.sun.tools.javac.comp.Infer$InferenceContext r4 = r10.inferenceContext     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                com.sun.tools.javac.util.List r4 = r4.restvars()     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                boolean r4 = com.sun.tools.javac.code.Type.containsAny(r4, r2)     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                if (r4 == 0) goto L5e
                com.sun.tools.javac.comp.Infer$GraphInferenceSteps[] r4 = com.sun.tools.javac.comp.Infer.GraphInferenceSteps.values()     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                int r5 = r4.length     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                r6 = 0
            L3a:
                if (r6 >= r5) goto L55
                r7 = r4[r6]     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                com.sun.tools.javac.comp.Infer$InferenceContext r8 = r10.inferenceContext     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                java.util.EnumSet<com.sun.tools.javac.comp.Infer$InferenceStep> r9 = r7.steps     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                boolean r8 = com.sun.tools.javac.comp.Infer.InferenceContext.access$600(r8, r2, r9)     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                if (r8 == 0) goto L52
                com.sun.tools.javac.comp.Infer r4 = com.sun.tools.javac.comp.Infer.this     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                com.sun.tools.javac.comp.Infer$InferenceContext r5 = r10.inferenceContext     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                com.sun.tools.javac.util.Warner r6 = r10.warn     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                r4.checkWithinBounds(r5, r6)     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                goto L28
            L52:
                int r6 = r6 + 1
                goto L3a
            L55:
                com.sun.tools.javac.comp.Infer r4 = com.sun.tools.javac.comp.Infer.this     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                com.sun.tools.javac.comp.Infer$InferenceException r4 = r4.inferenceException     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                com.sun.tools.javac.comp.Resolve$InapplicableMethodException r4 = r4.setMessage()     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
                throw r4     // Catch: com.sun.tools.javac.comp.Infer.InferenceException -> L5f
            L5e:
                goto L75
            L5f:
                r4 = move-exception
                com.sun.tools.javac.comp.Infer$InferenceContext r5 = r10.inferenceContext
                r5.rollback(r3)
                com.sun.tools.javac.comp.Infer r5 = com.sun.tools.javac.comp.Infer.this
                com.sun.tools.javac.comp.Infer$InferenceContext r6 = r10.inferenceContext
                com.sun.tools.javac.comp.Infer.access$700(r5, r2, r6)
                com.sun.tools.javac.comp.Infer r5 = com.sun.tools.javac.comp.Infer.this
                com.sun.tools.javac.comp.Infer$InferenceContext r6 = r10.inferenceContext
                com.sun.tools.javac.util.Warner r7 = r10.warn
                r5.checkWithinBounds(r6, r7)
            L75:
                r0.deleteNode(r1)
                goto L10
            L79:
                return
            */
            throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Infer.GraphSolver.solve(com.sun.tools.javac.comp.Infer$GraphStrategy):void");
        }

        class InferenceGraph {
            ArrayList<Node> nodes;

            class Node extends GraphUtils.TarjanNode<ListBuffer<Type>> {
                EnumMap<DependencyKind, Set<Node>> deps;

                Node(Type ivar) {
                    super(ListBuffer.of(ivar));
                    this.deps = new EnumMap<>(DependencyKind.class);
                }

                @Override // com.sun.tools.javac.util.GraphUtils.Node
                public GraphUtils.DependencyKind[] getSupportedDependencyKinds() {
                    return DependencyKind.values();
                }

                /* JADX WARN: Multi-variable type inference failed */
                @Override // com.sun.tools.javac.util.GraphUtils.Node
                public String getDependencyName(GraphUtils.Node<ListBuffer<Type>> to, GraphUtils.DependencyKind dk2) {
                    if (dk2 == DependencyKind.STUCK) {
                        return "";
                    }
                    StringBuilder buf = new StringBuilder();
                    String sep = "";
                    for (Type from : (ListBuffer) this.data) {
                        Type.UndetVar uv = (Type.UndetVar) GraphSolver.this.inferenceContext.asUndetVar(from);
                        for (Type bound : uv.getBounds(Type.UndetVar.InferenceBound.values())) {
                            if (bound.containsAny(List.from(to.data))) {
                                buf.append(sep);
                                buf.append(bound);
                                sep = DocLint.TAGS_SEPARATOR;
                            }
                        }
                    }
                    return buf.toString();
                }

                @Override // com.sun.tools.javac.util.GraphUtils.TarjanNode, com.sun.tools.javac.util.GraphUtils.Node
                public Iterable<? extends Node> getAllDependencies() {
                    return getDependencies(DependencyKind.values());
                }

                @Override // com.sun.tools.javac.util.GraphUtils.TarjanNode
                public Iterable<? extends GraphUtils.TarjanNode<ListBuffer<Type>>> getDependenciesByKind(GraphUtils.DependencyKind dk2) {
                    return getDependencies((DependencyKind) dk2);
                }

                protected Set<Node> getDependencies(DependencyKind... depKinds) {
                    Set<Node> buf = new LinkedHashSet<>();
                    for (DependencyKind dk2 : depKinds) {
                        Set<Node> depsByKind = this.deps.get(dk2);
                        if (depsByKind != null) {
                            buf.addAll(depsByKind);
                        }
                    }
                    return buf;
                }

                protected void addDependency(DependencyKind dk2, Node depToAdd) {
                    Set<Node> depsByKind = this.deps.get(dk2);
                    if (depsByKind == null) {
                        depsByKind = new LinkedHashSet();
                        this.deps.put(dk2, depsByKind);
                    }
                    depsByKind.add(depToAdd);
                }

                protected void addDependencies(DependencyKind dk2, Set<Node> depsToAdd) {
                    for (Node n : depsToAdd) {
                        addDependency(dk2, n);
                    }
                }

                protected Set<DependencyKind> removeDependency(Node n) {
                    Set<DependencyKind> removedKinds = new HashSet<>();
                    for (DependencyKind dk2 : DependencyKind.values()) {
                        Set<Node> depsByKind = this.deps.get(dk2);
                        if (depsByKind != null && depsByKind.remove(n)) {
                            removedKinds.add(dk2);
                        }
                    }
                    return removedKinds;
                }

                protected Set<Node> closure(DependencyKind... depKinds) {
                    boolean progress = true;
                    Set<Node> closure = new HashSet<>();
                    closure.add(this);
                    while (progress) {
                        progress = false;
                        for (Node n1 : new HashSet(closure)) {
                            progress = closure.addAll(n1.getDependencies(depKinds));
                        }
                    }
                    return closure;
                }

                protected boolean isLeaf() {
                    Set<Node> allDeps = getDependencies(DependencyKind.BOUND, DependencyKind.STUCK);
                    if (allDeps.isEmpty()) {
                        return true;
                    }
                    for (Node n : allDeps) {
                        if (n != this) {
                            return false;
                        }
                    }
                    return true;
                }

                /* JADX WARN: Multi-variable type inference failed */
                protected void mergeWith(List<? extends Node> nodes) {
                    for (Node n : nodes) {
                        Assert.check(((ListBuffer) n.data).length() == 1, "Attempt to merge a compound node!");
                        ((ListBuffer) this.data).appendList((ListBuffer) n.data);
                        for (DependencyKind dk2 : DependencyKind.values()) {
                            addDependencies(dk2, n.getDependencies(dk2));
                        }
                    }
                    EnumMap<DependencyKind, Set<Node>> deps2 = new EnumMap<>(DependencyKind.class);
                    for (DependencyKind dk3 : DependencyKind.values()) {
                        for (Node d : getDependencies(dk3)) {
                            Set<Node> depsByKind = deps2.get(dk3);
                            if (depsByKind == null) {
                                depsByKind = new LinkedHashSet();
                                deps2.put(dk3, depsByKind);
                            }
                            if (((ListBuffer) this.data).contains(((ListBuffer) d.data).first())) {
                                depsByKind.add(this);
                            } else {
                                depsByKind.add(d);
                            }
                        }
                    }
                    this.deps = deps2;
                }

                /* JADX INFO: Access modifiers changed from: private */
                public void graphChanged(Node from, Node to) {
                    for (DependencyKind dk2 : removeDependency(from)) {
                        if (to != null) {
                            addDependency(dk2, to);
                        }
                    }
                }
            }

            InferenceGraph(Map<Type, Set<Type>> optDeps) {
                initNodes(optDeps);
            }

            /* JADX WARN: Multi-variable type inference failed */
            public Node findNode(Type t) {
                for (Node n : this.nodes) {
                    if (((ListBuffer) n.data).contains(t)) {
                        return n;
                    }
                }
                return null;
            }

            public void deleteNode(Node n) {
                Assert.check(this.nodes.contains(n));
                this.nodes.remove(n);
                notifyUpdate(n, null);
            }

            void notifyUpdate(Node from, Node to) {
                for (Node n : this.nodes) {
                    n.graphChanged(from, to);
                }
            }

            /* JADX WARN: Multi-variable type inference failed */
            void initNodes(Map<Type, Set<Type>> stuckDeps) {
                this.nodes = new ArrayList<>();
                for (Type t : GraphSolver.this.inferenceContext.restvars()) {
                    this.nodes.add(new Node(t));
                }
                for (Node n_i : this.nodes) {
                    Type i = (Type) ((ListBuffer) n_i.data).first();
                    Set<Type> optDepsByNode = stuckDeps.get(i);
                    for (Node n_j : this.nodes) {
                        Type j = (Type) ((ListBuffer) n_j.data).first();
                        Type.UndetVar uv_i = (Type.UndetVar) GraphSolver.this.inferenceContext.asUndetVar(i);
                        if (Type.containsAny(uv_i.getBounds(Type.UndetVar.InferenceBound.values()), List.of(j))) {
                            n_i.addDependency(DependencyKind.BOUND, n_j);
                        }
                        if (optDepsByNode != null && optDepsByNode.contains(j)) {
                            n_i.addDependency(DependencyKind.STUCK, n_j);
                        }
                    }
                }
                ArrayList<Node> arrayList = new ArrayList<>();
                for (List<? extends Node> conSubGraph : GraphUtils.tarjan(this.nodes)) {
                    if (conSubGraph.length() > 1) {
                        Node root = (Node) conSubGraph.head;
                        root.mergeWith(conSubGraph.tail);
                        for (Node n : conSubGraph) {
                            notifyUpdate(n, root);
                        }
                    }
                    arrayList.add(conSubGraph.head);
                }
                this.nodes = arrayList;
            }

            String toDot() {
                StringBuilder buf = new StringBuilder();
                for (Type t : GraphSolver.this.inferenceContext.undetvars) {
                    Type.UndetVar uv = (Type.UndetVar) t;
                    buf.append(String.format("var %s - upper bounds = %s, lower bounds = %s, eq bounds = %s\\n", uv.qtype, uv.getBounds(Type.UndetVar.InferenceBound.UPPER), uv.getBounds(Type.UndetVar.InferenceBound.LOWER), uv.getBounds(Type.UndetVar.InferenceBound.EQ)));
                }
                return GraphUtils.toDot(this.nodes, "inferenceGraph" + hashCode(), buf.toString());
            }
        }
    }

    class InferenceContext {
        List<Type> inferencevars;
        List<Type> undetvars;
        Map<FreeTypeListener, List<Type>> freeTypeListeners = new HashMap();
        List<FreeTypeListener> freetypeListeners = List.nil();
        Type.Mapping fromTypeVarFun = new Type.Mapping("fromTypeVarFunWithBounds") { // from class: com.sun.tools.javac.comp.Infer.InferenceContext.1
            @Override // com.sun.tools.javac.code.Type.Mapping
            public Type apply(Type t) {
                if (t.hasTag(TypeTag.TYPEVAR)) {
                    Type.TypeVar tv = (Type.TypeVar) t;
                    if (tv.isCaptured()) {
                        return new Type.CapturedUndetVar((Type.CapturedType) tv, Infer.this.types);
                    }
                    return new Type.UndetVar(tv, Infer.this.types);
                }
                return t.map(this);
            }
        };
        Map<JCTree, Type> captureTypeCache = new HashMap();

        public InferenceContext(List<Type> inferencevars) {
            this.undetvars = Type.map(inferencevars, this.fromTypeVarFun);
            this.inferencevars = inferencevars;
        }

        void addVar(Type.TypeVar t) {
            this.undetvars = this.undetvars.prepend(this.fromTypeVarFun.apply(t));
            this.inferencevars = this.inferencevars.prepend(t);
        }

        List<Type> inferenceVars() {
            return this.inferencevars;
        }

        List<Type> restvars() {
            return filterVars(new Filter<Type.UndetVar>() { // from class: com.sun.tools.javac.comp.Infer.InferenceContext.2
                @Override // com.sun.tools.javac.util.Filter
                public boolean accepts(Type.UndetVar uv) {
                    return uv.inst == null;
                }
            });
        }

        List<Type> instvars() {
            return filterVars(new Filter<Type.UndetVar>() { // from class: com.sun.tools.javac.comp.Infer.InferenceContext.3
                @Override // com.sun.tools.javac.util.Filter
                public boolean accepts(Type.UndetVar uv) {
                    return uv.inst != null;
                }
            });
        }

        final List<Type> boundedVars() {
            return filterVars(new Filter<Type.UndetVar>() { // from class: com.sun.tools.javac.comp.Infer.InferenceContext.4
                @Override // com.sun.tools.javac.util.Filter
                public boolean accepts(Type.UndetVar uv) {
                    return uv.getBounds(Type.UndetVar.InferenceBound.UPPER).diff(uv.getDeclaredBounds()).appendList(uv.getBounds(Type.UndetVar.InferenceBound.EQ, Type.UndetVar.InferenceBound.LOWER)).nonEmpty();
                }
            });
        }

        private List<Type> filterVars(Filter<Type.UndetVar> fu) {
            ListBuffer<Type> res = new ListBuffer<>();
            for (Type t : this.undetvars) {
                Type.UndetVar uv = (Type.UndetVar) t;
                if (fu.accepts(uv)) {
                    res.append(uv.qtype);
                }
            }
            return res.toList();
        }

        final boolean free(Type t) {
            return t.containsAny(this.inferencevars);
        }

        final boolean free(List<Type> ts) {
            for (Type t : ts) {
                if (free(t)) {
                    return true;
                }
            }
            return false;
        }

        final List<Type> freeVarsIn(Type t) {
            ListBuffer<Type> buf = new ListBuffer<>();
            for (Type iv : inferenceVars()) {
                if (t.contains(iv)) {
                    buf.add(iv);
                }
            }
            return buf.toList();
        }

        final List<Type> freeVarsIn(List<Type> ts) {
            ListBuffer<Type> buf = new ListBuffer<>();
            Iterator<Type> it = ts.iterator();
            while (it.hasNext()) {
                buf.appendList(freeVarsIn(it.next()));
            }
            ListBuffer<Type> buf2 = new ListBuffer<>();
            for (Type t : buf) {
                if (!buf2.contains(t)) {
                    buf2.add(t);
                }
            }
            return buf2.toList();
        }

        final Type asUndetVar(Type t) {
            return Infer.this.types.subst(t, this.inferencevars, this.undetvars);
        }

        final List<Type> asUndetVars(List<Type> ts) {
            ListBuffer<Type> buf = new ListBuffer<>();
            for (Type t : ts) {
                buf.append(asUndetVar(t));
            }
            return buf.toList();
        }

        List<Type> instTypes() {
            ListBuffer<Type> buf = new ListBuffer<>();
            for (Type t : this.undetvars) {
                Type.UndetVar uv = (Type.UndetVar) t;
                buf.append(uv.inst != null ? uv.inst : uv.qtype);
            }
            return buf.toList();
        }

        Type asInstType(Type t) {
            return Infer.this.types.subst(t, this.inferencevars, instTypes());
        }

        List<Type> asInstTypes(List<Type> ts) {
            ListBuffer<Type> buf = new ListBuffer<>();
            for (Type t : ts) {
                buf.append(asInstType(t));
            }
            return buf.toList();
        }

        void addFreeTypeListener(List<Type> types, FreeTypeListener ftl) {
            this.freeTypeListeners.put(ftl, freeVarsIn(types));
        }

        void notifyChange() {
            notifyChange(this.inferencevars.diff(restvars()));
        }

        void notifyChange(List<Type> inferredVars) {
            InferenceException thrownEx = null;
            for (Map.Entry<FreeTypeListener, List<Type>> entry : new HashMap(this.freeTypeListeners).entrySet()) {
                if (!Type.containsAny(entry.getValue(), this.inferencevars.diff(inferredVars))) {
                    try {
                        entry.getKey().typesInferred(this);
                        this.freeTypeListeners.remove(entry.getKey());
                    } catch (InferenceException ex) {
                        if (thrownEx == null) {
                            thrownEx = ex;
                        }
                    }
                }
            }
            if (thrownEx != null) {
                throw thrownEx;
            }
        }

        List<Type> save() {
            ListBuffer<Type> buf = new ListBuffer<>();
            for (Type t : this.undetvars) {
                Type.UndetVar uv = (Type.UndetVar) t;
                Type.UndetVar uv2 = new Type.UndetVar((Type.TypeVar) uv.qtype, Infer.this.types);
                for (Type.UndetVar.InferenceBound ib : Type.UndetVar.InferenceBound.values()) {
                    for (Type b : uv.getBounds(ib)) {
                        uv2.addBound(ib, b, Infer.this.types);
                    }
                }
                uv2.inst = uv.inst;
                buf.add(uv2);
            }
            return buf.toList();
        }

        /* JADX WARN: Multi-variable type inference failed */
        void rollback(List<Type> list) {
            Assert.check(list != null && list.length() == this.undetvars.length());
            Iterator<Type> it = this.undetvars.iterator();
            List list2 = list;
            while (it.hasNext()) {
                Type.UndetVar undetVar = (Type.UndetVar) it.next();
                Type.UndetVar undetVar2 = (Type.UndetVar) list2.head;
                for (Type.UndetVar.InferenceBound inferenceBound : Type.UndetVar.InferenceBound.values()) {
                    undetVar.setBounds(inferenceBound, undetVar2.getBounds(inferenceBound));
                }
                undetVar.inst = undetVar2.inst;
                list2 = list2.tail;
            }
        }

        void dupTo(InferenceContext that) {
            that.inferencevars = that.inferencevars.appendList(this.inferencevars.diff(that.inferencevars));
            that.undetvars = that.undetvars.appendList(this.undetvars.diff(that.undetvars));
            for (Type t : this.inferencevars) {
                that.freeTypeListeners.put(new FreeTypeListener() { // from class: com.sun.tools.javac.comp.Infer.InferenceContext.5
                    @Override // com.sun.tools.javac.comp.Infer.FreeTypeListener
                    public void typesInferred(InferenceContext inferenceContext) {
                        InferenceContext.this.notifyChange();
                    }
                }, List.of(t));
            }
        }

        private void solve(GraphStrategy ss, Warner warn) {
            solve(ss, new HashMap(), warn);
        }

        private void solve(GraphStrategy ss, Map<Type, Set<Type>> stuckDeps, Warner warn) {
            GraphSolver s = Infer.this.new GraphSolver(this, stuckDeps, warn);
            s.solve(ss);
        }

        public void solve(Warner warn) {
            solve(new LeafSolver() { // from class: com.sun.tools.javac.comp.Infer.InferenceContext.6
                {
                    Infer infer = Infer.this;
                }

                @Override // com.sun.tools.javac.comp.Infer.GraphStrategy
                public boolean done() {
                    return InferenceContext.this.restvars().isEmpty();
                }
            }, warn);
        }

        public void solve(final List<Type> vars, Warner warn) {
            solve(new BestLeafSolver(vars) { // from class: com.sun.tools.javac.comp.Infer.InferenceContext.7
                {
                    Infer infer = Infer.this;
                }

                @Override // com.sun.tools.javac.comp.Infer.GraphStrategy
                public boolean done() {
                    return !InferenceContext.this.free(InferenceContext.this.asInstTypes(vars));
                }
            }, warn);
        }

        public void solveAny(List<Type> varsToSolve, Map<Type, Set<Type>> optDeps, Warner warn) {
            solve(new BestLeafSolver(varsToSolve.intersect(restvars())) { // from class: com.sun.tools.javac.comp.Infer.InferenceContext.8
                {
                    Infer infer = Infer.this;
                }

                @Override // com.sun.tools.javac.comp.Infer.GraphStrategy
                public boolean done() {
                    return InferenceContext.this.instvars().intersect(this.varsToSolve).nonEmpty();
                }
            }, optDeps, warn);
        }

        private boolean solveBasic(EnumSet<InferenceStep> steps) {
            return solveBasic(this.inferencevars, steps);
        }

        /* JADX INFO: Access modifiers changed from: private */
        public boolean solveBasic(List<Type> varsToSolve, EnumSet<InferenceStep> steps) {
            boolean changed = false;
            for (Type t : varsToSolve.intersect(restvars())) {
                Type.UndetVar uv = (Type.UndetVar) asUndetVar(t);
                Iterator it = steps.iterator();
                while (true) {
                    if (it.hasNext()) {
                        InferenceStep step = (InferenceStep) it.next();
                        if (step.accepts(uv, this)) {
                            uv.inst = step.solve(uv, this);
                            changed = true;
                            break;
                        }
                    }
                }
            }
            return changed;
        }

        public void solveLegacy(boolean partial, Warner warn, EnumSet<InferenceStep> steps) throws Throwable {
            while (true) {
                boolean stuck = !solveBasic(steps);
                if (restvars().isEmpty() || partial) {
                    break;
                }
                if (stuck) {
                    Infer.this.instantiateAsUninferredVars(restvars(), this);
                    break;
                }
                for (Type t : this.undetvars) {
                    Type.UndetVar uv = (Type.UndetVar) t;
                    uv.substBounds(inferenceVars(), instTypes(), Infer.this.types);
                }
            }
            Infer.this.checkWithinBounds(this, warn);
        }

        /* JADX INFO: Access modifiers changed from: private */
        public Infer infer() {
            return Infer.this;
        }

        public String toString() {
            return "Inference vars: " + this.inferencevars + "\nUndet vars: " + this.undetvars;
        }

        Type cachedCapture(JCTree tree, Type t, boolean readOnly) {
            Type captured = this.captureTypeCache.get(tree);
            if (captured != null) {
                return captured;
            }
            Type result = Infer.this.types.capture(t);
            if (result != t && !readOnly) {
                this.captureTypeCache.put(tree, result);
            }
            return result;
        }
    }
}
