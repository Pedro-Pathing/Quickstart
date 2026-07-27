package com.sun.tools.javac.comp;

import com.sun.source.tree.LambdaExpressionTree;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.comp.Check;
import com.sun.tools.javac.comp.Infer;
import com.sun.tools.javac.comp.Resolve;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeCopier;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Warner;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;

/* JADX INFO: loaded from: classes.dex */
public class DeferredAttr extends JCTree.Visitor {
    protected static final Context.Key<DeferredAttr> deferredAttrKey = new Context.Key<>();
    final Attr attr;
    final Check chk;
    final JCDiagnostic.Factory diags;
    final DeferredAttrContext emptyDeferredAttrContext;
    final Enter enter;
    final Flow flow;
    final Infer infer;
    final Log log;
    final TreeMaker make;
    final Names names;
    final Resolve rs;
    final JCTree stuckTree;
    final Symtab syms;
    final TypeEnvs typeEnvs;
    final Types types;
    DeferredTypeCompleter basicCompleter = new DeferredTypeCompleter() { // from class: com.sun.tools.javac.comp.DeferredAttr.2
        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredTypeCompleter
        public Type complete(DeferredType dt, Attr.ResultInfo resultInfo, DeferredAttrContext deferredAttrContext) {
            switch (AnonymousClass6.$SwitchMap$com$sun$tools$javac$comp$DeferredAttr$AttrMode[deferredAttrContext.mode.ordinal()]) {
                case 1:
                    Assert.check(dt.mode == null || dt.mode == AttrMode.SPECULATIVE);
                    JCTree speculativeTree = DeferredAttr.this.attribSpeculative(dt.tree, dt.env, resultInfo);
                    dt.speculativeCache.put(speculativeTree, resultInfo);
                    return speculativeTree.type;
                case 2:
                    Assert.check(dt.mode != null);
                    return DeferredAttr.this.attr.attribTree(dt.tree, dt.env, resultInfo);
                default:
                    Assert.error();
                    return null;
            }
        }
    };
    DeferredTypeCompleter dummyCompleter = new DeferredTypeCompleter() { // from class: com.sun.tools.javac.comp.DeferredAttr.3
        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredTypeCompleter
        public Type complete(DeferredType dt, Attr.ResultInfo resultInfo, DeferredAttrContext deferredAttrContext) {
            Assert.check(deferredAttrContext.mode == AttrMode.CHECK);
            JCTree.JCExpression jCExpression = dt.tree;
            Type.JCNoType jCNoType = Type.stuckType;
            jCExpression.type = jCNoType;
            return jCNoType;
        }
    };
    DeferredStuckPolicy dummyStuckPolicy = new DeferredStuckPolicy() { // from class: com.sun.tools.javac.comp.DeferredAttr.4
        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredStuckPolicy
        public boolean isStuck() {
            return false;
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredStuckPolicy
        public Set<Type> stuckVars() {
            return Collections.emptySet();
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredStuckPolicy
        public Set<Type> depVars() {
            return Collections.emptySet();
        }
    };
    protected UnenterScanner unenterScanner = new UnenterScanner();
    private EnumSet<JCTree.Tag> deferredCheckerTags = EnumSet.of(JCTree.Tag.LAMBDA, JCTree.Tag.REFERENCE, JCTree.Tag.PARENS, JCTree.Tag.TYPECAST, JCTree.Tag.CONDEXPR, JCTree.Tag.NEWCLASS, JCTree.Tag.APPLY, JCTree.Tag.LITERAL);

    public enum AttrMode {
        SPECULATIVE,
        CHECK
    }

    interface DeferredStuckPolicy {
        Set<Type> depVars();

        boolean isStuck();

        Set<Type> stuckVars();
    }

    interface DeferredTypeCompleter {
        Type complete(DeferredType deferredType, Attr.ResultInfo resultInfo, DeferredAttrContext deferredAttrContext);
    }

    interface MethodAnalyzer<E> {
        E process(Symbol.MethodSymbol methodSymbol);

        E reduce(E e, E e2);

        boolean shouldStop(E e);
    }

    public static DeferredAttr instance(Context context) {
        DeferredAttr instance = (DeferredAttr) context.get(deferredAttrKey);
        if (instance == null) {
            return new DeferredAttr(context);
        }
        return instance;
    }

    protected DeferredAttr(Context context) {
        context.put(deferredAttrKey, this);
        this.attr = Attr.instance(context);
        this.chk = Check.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        this.enter = Enter.instance(context);
        this.infer = Infer.instance(context);
        this.rs = Resolve.instance(context);
        this.log = Log.instance(context);
        this.syms = Symtab.instance(context);
        this.make = TreeMaker.instance(context);
        this.types = Types.instance(context);
        this.flow = Flow.instance(context);
        this.names = Names.instance(context);
        this.stuckTree = this.make.Ident(this.names.empty).setType((Type) Type.stuckType);
        this.typeEnvs = TypeEnvs.instance(context);
        this.emptyDeferredAttrContext = new DeferredAttrContext(AttrMode.CHECK, null, Resolve.MethodResolutionPhase.BOX, this.infer.emptyContext, null, null) { // from class: com.sun.tools.javac.comp.DeferredAttr.1
            @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredAttrContext
            void addDeferredAttrNode(DeferredType dt, Attr.ResultInfo ri, DeferredStuckPolicy deferredStuckPolicy) {
                Assert.error("Empty deferred context!");
            }

            @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredAttrContext
            void complete() {
                Assert.error("Empty deferred context!");
            }

            public String toString() {
                return "Empty deferred context!";
            }
        };
    }

    public class DeferredType extends Type {
        Env<AttrContext> env;
        AttrMode mode;
        SpeculativeCache speculativeCache;
        public JCTree.JCExpression tree;

        DeferredType(JCTree.JCExpression tree, Env<AttrContext> env) {
            super(null);
            this.tree = tree;
            this.env = DeferredAttr.this.attr.copyEnv(env);
            this.speculativeCache = new SpeculativeCache();
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.DEFERRED;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return "DeferredType";
        }

        class SpeculativeCache {
            private Map<Symbol, List<Entry>> cache = new WeakHashMap();

            SpeculativeCache() {
            }

            class Entry {
                Attr.ResultInfo resultInfo;
                JCTree speculativeTree;

                public Entry(JCTree speculativeTree, Attr.ResultInfo resultInfo) {
                    this.speculativeTree = speculativeTree;
                    this.resultInfo = resultInfo;
                }

                boolean matches(Resolve.MethodResolutionPhase phase) {
                    return this.resultInfo.checkContext.deferredAttrContext().phase == phase;
                }
            }

            Entry get(Symbol msym, Resolve.MethodResolutionPhase phase) {
                List<Entry> entries = this.cache.get(msym);
                if (entries == null) {
                    return null;
                }
                for (Entry e : entries) {
                    if (e.matches(phase)) {
                        return e;
                    }
                }
                return null;
            }

            void put(JCTree speculativeTree, Attr.ResultInfo resultInfo) {
                Symbol msym = resultInfo.checkContext.deferredAttrContext().msym;
                List<Entry> entries = this.cache.get(msym);
                if (entries == null) {
                    entries = List.nil();
                }
                this.cache.put(msym, entries.prepend(new Entry(speculativeTree, resultInfo)));
            }
        }

        Type speculativeType(Symbol msym, Resolve.MethodResolutionPhase phase) {
            SpeculativeCache.Entry e = this.speculativeCache.get(msym, phase);
            return e != null ? e.speculativeTree.type : Type.noType;
        }

        Type check(Attr.ResultInfo resultInfo) {
            DeferredStuckPolicy deferredStuckPolicy;
            if (resultInfo.pt.hasTag(TypeTag.NONE) || resultInfo.pt.isErroneous()) {
                deferredStuckPolicy = DeferredAttr.this.dummyStuckPolicy;
            } else if (resultInfo.checkContext.deferredAttrContext().mode == AttrMode.SPECULATIVE || resultInfo.checkContext.deferredAttrContext().insideOverloadPhase()) {
                deferredStuckPolicy = DeferredAttr.this.new OverloadStuckPolicy(resultInfo, this);
            } else {
                deferredStuckPolicy = DeferredAttr.this.new CheckStuckPolicy(resultInfo, this);
            }
            return check(resultInfo, deferredStuckPolicy, DeferredAttr.this.basicCompleter);
        }

        /* JADX INFO: Access modifiers changed from: private */
        public Type check(Attr.ResultInfo resultInfo, DeferredStuckPolicy deferredStuckPolicy, DeferredTypeCompleter deferredTypeCompleter) {
            DeferredAttrContext deferredAttrContext = resultInfo.checkContext.deferredAttrContext();
            Assert.check(deferredAttrContext != DeferredAttr.this.emptyDeferredAttrContext);
            if (deferredStuckPolicy.isStuck()) {
                deferredAttrContext.addDeferredAttrNode(this, resultInfo, deferredStuckPolicy);
                return Type.noType;
            }
            try {
                return deferredTypeCompleter.complete(this, resultInfo, deferredAttrContext);
            } finally {
                this.mode = deferredAttrContext.mode;
            }
        }
    }

    JCTree attribSpeculative(JCTree tree, Env<AttrContext> env, Attr.ResultInfo resultInfo) {
        final JCTree newTree = new TreeCopier(this.make).copy(tree);
        Env<AttrContext> speculativeEnv = env.dup(newTree, env.info.dup(env.info.scope.dupUnshared()));
        speculativeEnv.info.scope.owner = env.info.scope.owner;
        Log.DeferredDiagnosticHandler deferredDiagnosticHandler = new Log.DeferredDiagnosticHandler(this.log, new Filter<JCDiagnostic>() { // from class: com.sun.tools.javac.comp.DeferredAttr.5

            /* JADX INFO: renamed from: com.sun.tools.javac.comp.DeferredAttr$5$1PosScanner, reason: invalid class name */
            class C1PosScanner extends TreeScanner {
                boolean found = false;
                final /* synthetic */ JCDiagnostic val$d;

                C1PosScanner(JCDiagnostic jCDiagnostic) {
                    this.val$d = jCDiagnostic;
                }

                @Override // com.sun.tools.javac.tree.TreeScanner
                public void scan(JCTree tree) {
                    if (tree != null && tree.pos() == this.val$d.getDiagnosticPosition()) {
                        this.found = true;
                    }
                    super.scan(tree);
                }
            }

            @Override // com.sun.tools.javac.util.Filter
            public boolean accepts(JCDiagnostic d) {
                C1PosScanner posScanner = new C1PosScanner(d);
                posScanner.scan(newTree);
                return posScanner.found;
            }
        });
        try {
            this.attr.attribTree(newTree, speculativeEnv, resultInfo);
            this.unenterScanner.scan(newTree);
            return newTree;
        } finally {
            this.unenterScanner.scan(newTree);
            this.log.popDiagnosticHandler(deferredDiagnosticHandler);
        }
    }

    class UnenterScanner extends TreeScanner {
        UnenterScanner() {
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            Symbol.ClassSymbol csym = tree.sym;
            if (csym == null) {
                return;
            }
            DeferredAttr.this.typeEnvs.remove(csym);
            DeferredAttr.this.chk.compiled.remove(csym.flatname);
            DeferredAttr.this.syms.classes.remove(csym.flatname);
            super.visitClassDef(tree);
        }
    }

    class DeferredAttrContext {
        ArrayList<DeferredAttrNode> deferredAttrNodes = new ArrayList<>();
        final Infer.InferenceContext inferenceContext;
        final AttrMode mode;
        final Symbol msym;
        final DeferredAttrContext parent;
        final Resolve.MethodResolutionPhase phase;
        final Warner warn;

        DeferredAttrContext(AttrMode mode, Symbol msym, Resolve.MethodResolutionPhase phase, Infer.InferenceContext inferenceContext, DeferredAttrContext parent, Warner warn) {
            this.mode = mode;
            this.msym = msym;
            this.phase = phase;
            this.parent = parent;
            this.warn = warn;
            this.inferenceContext = inferenceContext;
        }

        void addDeferredAttrNode(DeferredType dt, Attr.ResultInfo resultInfo, DeferredStuckPolicy deferredStuckPolicy) {
            this.deferredAttrNodes.add(DeferredAttr.this.new DeferredAttrNode(dt, resultInfo, deferredStuckPolicy));
        }

        void complete() {
            while (!this.deferredAttrNodes.isEmpty()) {
                Map<Type, Set<Type>> depVarsMap = new LinkedHashMap<>();
                List<Type> stuckVars = List.nil();
                boolean progress = false;
                for (DeferredAttrNode deferredAttrNode : List.from(this.deferredAttrNodes)) {
                    if (!deferredAttrNode.process(this)) {
                        List<Type> restStuckVars = List.from(deferredAttrNode.deferredStuckPolicy.stuckVars()).intersect(this.inferenceContext.restvars());
                        stuckVars = stuckVars.prependList(restStuckVars);
                        for (Type t : List.from(deferredAttrNode.deferredStuckPolicy.depVars()).intersect(this.inferenceContext.restvars())) {
                            Set<Type> prevDeps = depVarsMap.get(t);
                            if (prevDeps == null) {
                                prevDeps = new LinkedHashSet<>();
                                depVarsMap.put(t, prevDeps);
                            }
                            prevDeps.addAll(restStuckVars);
                        }
                    } else {
                        this.deferredAttrNodes.remove(deferredAttrNode);
                        progress = true;
                    }
                }
                if (!progress) {
                    if (insideOverloadPhase()) {
                        for (DeferredAttrNode deferredNode : this.deferredAttrNodes) {
                            deferredNode.dt.tree.type = Type.noType;
                        }
                        return;
                    }
                    try {
                        this.inferenceContext.solveAny(stuckVars, depVarsMap, this.warn);
                        this.inferenceContext.notifyChange();
                    } catch (Infer.GraphStrategy.NodeNotFoundException e) {
                        return;
                    }
                }
            }
        }

        /* JADX INFO: Access modifiers changed from: private */
        public boolean insideOverloadPhase() {
            if (this == DeferredAttr.this.emptyDeferredAttrContext) {
                return false;
            }
            if (this.mode == AttrMode.SPECULATIVE) {
                return true;
            }
            return this.parent.insideOverloadPhase();
        }
    }

    class DeferredAttrNode {
        DeferredStuckPolicy deferredStuckPolicy;
        DeferredType dt;
        Attr.ResultInfo resultInfo;

        DeferredAttrNode(DeferredType dt, Attr.ResultInfo resultInfo, DeferredStuckPolicy deferredStuckPolicy) {
            this.dt = dt;
            this.resultInfo = resultInfo;
            this.deferredStuckPolicy = deferredStuckPolicy;
        }

        boolean process(final DeferredAttrContext deferredAttrContext) {
            switch (deferredAttrContext.mode) {
                case SPECULATIVE:
                    if (this.deferredStuckPolicy.isStuck()) {
                        this.dt.check(this.resultInfo, DeferredAttr.this.dummyStuckPolicy, new StructuralStuckChecker());
                        return true;
                    }
                    Assert.error("Cannot get here");
                    break;
                    break;
                case CHECK:
                    break;
                default:
                    throw new AssertionError("Bad mode");
            }
            if (!this.deferredStuckPolicy.isStuck()) {
                Assert.check(!deferredAttrContext.insideOverloadPhase(), "attribution shouldn't be happening here");
                Attr.ResultInfo instResultInfo = this.resultInfo.dup(deferredAttrContext.inferenceContext.asInstType(this.resultInfo.pt));
                this.dt.check(instResultInfo, DeferredAttr.this.dummyStuckPolicy, DeferredAttr.this.basicCompleter);
                return true;
            }
            if (deferredAttrContext.parent != DeferredAttr.this.emptyDeferredAttrContext && Type.containsAny(deferredAttrContext.parent.inferenceContext.inferencevars, List.from(this.deferredStuckPolicy.stuckVars()))) {
                deferredAttrContext.parent.addDeferredAttrNode(this.dt, this.resultInfo.dup(new Check.NestedCheckContext(this.resultInfo.checkContext) { // from class: com.sun.tools.javac.comp.DeferredAttr.DeferredAttrNode.1
                    @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                    public Infer.InferenceContext inferenceContext() {
                        return deferredAttrContext.parent.inferenceContext;
                    }

                    @Override // com.sun.tools.javac.comp.Check.NestedCheckContext, com.sun.tools.javac.comp.Check.CheckContext
                    public DeferredAttrContext deferredAttrContext() {
                        return deferredAttrContext.parent;
                    }
                }), this.deferredStuckPolicy);
                this.dt.tree.type = Type.stuckType;
                return true;
            }
            return false;
        }

        class StructuralStuckChecker extends TreeScanner implements DeferredTypeCompleter {
            Env<AttrContext> env;
            Infer.InferenceContext inferenceContext;
            Attr.ResultInfo resultInfo;

            StructuralStuckChecker() {
            }

            @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredTypeCompleter
            public Type complete(DeferredType dt, Attr.ResultInfo resultInfo, DeferredAttrContext deferredAttrContext) {
                this.resultInfo = resultInfo;
                this.inferenceContext = deferredAttrContext.inferenceContext;
                this.env = dt.env;
                dt.tree.accept(this);
                dt.speculativeCache.put(DeferredAttr.this.stuckTree, resultInfo);
                return Type.noType;
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitLambda(JCTree.JCLambda tree) {
                Check.CheckContext checkContext = this.resultInfo.checkContext;
                Type pt = this.resultInfo.pt;
                if (!this.inferenceContext.inferencevars.contains(pt)) {
                    Type descriptorType = null;
                    try {
                        descriptorType = DeferredAttr.this.types.findDescriptorType(pt);
                    } catch (Types.FunctionDescriptorLookupError ex) {
                        checkContext.report(null, ex.getDiagnostic());
                    }
                    if (descriptorType.mo176getParameterTypes().length() != tree.params.length()) {
                        checkContext.report(tree, DeferredAttr.this.diags.fragment("incompatible.arg.types.in.lambda", new Object[0]));
                    }
                    Type currentReturnType = descriptorType.mo178getReturnType();
                    boolean returnTypeIsVoid = currentReturnType.hasTag(TypeTag.VOID);
                    if (tree.getBodyKind() == LambdaExpressionTree.BodyKind.EXPRESSION) {
                        boolean isExpressionCompatible = !returnTypeIsVoid || TreeInfo.isExpressionStatement((JCTree.JCExpression) tree.getBody());
                        if (!isExpressionCompatible) {
                            this.resultInfo.checkContext.report(tree.pos(), DeferredAttr.this.diags.fragment("incompatible.ret.type.in.lambda", DeferredAttr.this.diags.fragment("missing.ret.val", currentReturnType)));
                            return;
                        }
                        return;
                    }
                    LambdaBodyStructChecker lambdaBodyChecker = DeferredAttrNode.this.new LambdaBodyStructChecker();
                    tree.body.accept(lambdaBodyChecker);
                    boolean isVoidCompatible = lambdaBodyChecker.isVoidCompatible;
                    if (returnTypeIsVoid) {
                        if (!isVoidCompatible) {
                            this.resultInfo.checkContext.report(tree.pos(), DeferredAttr.this.diags.fragment("unexpected.ret.val", new Object[0]));
                            return;
                        }
                        return;
                    }
                    boolean isValueCompatible = lambdaBodyChecker.isPotentiallyValueCompatible && !canLambdaBodyCompleteNormally(tree);
                    if (!isValueCompatible && !isVoidCompatible) {
                        DeferredAttr.this.log.error(tree.body.pos(), "lambda.body.neither.value.nor.void.compatible", new Object[0]);
                    }
                    if (!isValueCompatible) {
                        this.resultInfo.checkContext.report(tree.pos(), DeferredAttr.this.diags.fragment("incompatible.ret.type.in.lambda", DeferredAttr.this.diags.fragment("missing.ret.val", currentReturnType)));
                    }
                }
            }

            /* JADX WARN: Multi-variable type inference failed */
            boolean canLambdaBodyCompleteNormally(JCTree.JCLambda tree) {
                JCTree.JCLambda newTree = (JCTree.JCLambda) new TreeCopier(DeferredAttr.this.make).copy(tree);
                Env<AttrContext> localEnv = DeferredAttr.this.attr.lambdaEnv(newTree, this.env);
                try {
                    for (List list = newTree.params; list.nonEmpty(); list = list.tail) {
                        ((JCTree.JCVariableDecl) list.head).vartype = DeferredAttr.this.make.at((JCDiagnostic.DiagnosticPosition) list.head).Type(DeferredAttr.this.syms.errType);
                    }
                    DeferredAttr.this.attr.attribStats(newTree.params, localEnv);
                    Attr attr = DeferredAttr.this.attr;
                    attr.getClass();
                    Attr.ResultInfo bodyResultInfo = new Attr.ResultInfo(attr, 12, Type.noType);
                    localEnv.info.returnResult = bodyResultInfo;
                    Log.DiagnosticHandler diagHandler = new Log.DiscardDiagnosticHandler(DeferredAttr.this.log);
                    try {
                        JCTree.JCBlock body = (JCTree.JCBlock) newTree.body;
                        DeferredAttr.this.attr.attribStats(body.stats, localEnv);
                        DeferredAttr.this.attr.preFlow(newTree);
                        DeferredAttr.this.flow.analyzeLambda(localEnv, newTree, DeferredAttr.this.make, true);
                        DeferredAttr.this.log.popDiagnosticHandler(diagHandler);
                        return newTree.canCompleteNormally;
                    } catch (Throwable th) {
                        DeferredAttr.this.log.popDiagnosticHandler(diagHandler);
                        throw th;
                    }
                } finally {
                    JCTree.JCBlock body2 = (JCTree.JCBlock) newTree.body;
                    DeferredAttr.this.unenterScanner.scan(body2.stats);
                    localEnv.info.scope.leave();
                }
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitNewClass(JCTree.JCNewClass tree) {
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitApply(JCTree.JCMethodInvocation tree) {
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitReference(JCTree.JCMemberReference tree) {
                Check.CheckContext checkContext = this.resultInfo.checkContext;
                Type pt = this.resultInfo.pt;
                if (!this.inferenceContext.inferencevars.contains(pt)) {
                    try {
                        DeferredAttr.this.types.findDescriptorType(pt);
                    } catch (Types.FunctionDescriptorLookupError ex) {
                        checkContext.report(null, ex.getDiagnostic());
                    }
                    Env<AttrContext> localEnv = this.env.dup(tree);
                    JCTree.JCExpression exprTree = (JCTree.JCExpression) DeferredAttr.this.attribSpeculative(tree.getQualifierExpression(), localEnv, DeferredAttr.this.attr.memberReferenceQualifierResult(tree));
                    ListBuffer<Type> argtypes = new ListBuffer<>();
                    for (Type type : DeferredAttr.this.types.findDescriptorType(pt).mo176getParameterTypes()) {
                        argtypes.append(Type.noType);
                    }
                    JCTree.JCMemberReference mref2 = (JCTree.JCMemberReference) new TreeCopier(DeferredAttr.this.make).copy(tree);
                    mref2.expr = exprTree;
                    Symbol lookupSym = DeferredAttr.this.rs.resolveMemberReferenceByArity(localEnv, mref2, exprTree.type, tree.name, argtypes.toList(), this.inferenceContext);
                    switch (lookupSym.kind) {
                        case 134:
                        case 135:
                        case 136:
                        case 138:
                            checkContext.report(tree, DeferredAttr.this.diags.fragment("incompatible.arg.types.in.mref", new Object[0]));
                            break;
                    }
                }
            }
        }

        class LambdaBodyStructChecker extends TreeScanner {
            boolean isVoidCompatible = true;
            boolean isPotentiallyValueCompatible = true;

            LambdaBodyStructChecker() {
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitClassDef(JCTree.JCClassDecl tree) {
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitLambda(JCTree.JCLambda tree) {
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitNewClass(JCTree.JCNewClass tree) {
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitReturn(JCTree.JCReturn tree) {
                if (tree.expr != null) {
                    this.isVoidCompatible = false;
                } else {
                    this.isPotentiallyValueCompatible = false;
                }
            }
        }
    }

    class DeferredTypeMap extends Type.Mapping {
        DeferredAttrContext deferredAttrContext;

        protected DeferredTypeMap(AttrMode mode, Symbol msym, Resolve.MethodResolutionPhase phase) {
            super(String.format("deferredTypeMap[%s]", mode));
            this.deferredAttrContext = DeferredAttr.this.new DeferredAttrContext(mode, msym, phase, DeferredAttr.this.infer.emptyContext, DeferredAttr.this.emptyDeferredAttrContext, DeferredAttr.this.types.noWarnings);
        }

        @Override // com.sun.tools.javac.code.Type.Mapping
        public Type apply(Type t) {
            if (!t.hasTag(TypeTag.DEFERRED)) {
                return t.map(this);
            }
            DeferredType dt = (DeferredType) t;
            return typeOf(dt);
        }

        protected Type typeOf(DeferredType dt) {
            switch (this.deferredAttrContext.mode) {
                case SPECULATIVE:
                    return dt.speculativeType(this.deferredAttrContext.msym, this.deferredAttrContext.phase);
                case CHECK:
                    return dt.tree.type == null ? Type.noType : dt.tree.type;
                default:
                    Assert.error();
                    return null;
            }
        }
    }

    public class RecoveryDeferredTypeMap extends DeferredTypeMap {
        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredTypeMap, com.sun.tools.javac.code.Type.Mapping
        public /* bridge */ /* synthetic */ Type apply(Type type) {
            return super.apply(type);
        }

        public RecoveryDeferredTypeMap(AttrMode mode, Symbol msym, Resolve.MethodResolutionPhase phase) {
            super(mode, msym, phase != null ? phase : Resolve.MethodResolutionPhase.BOX);
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredTypeMap
        protected Type typeOf(DeferredType dt) {
            Type owntype = super.typeOf(dt);
            return owntype == Type.noType ? recover(dt) : owntype;
        }

        private Type recover(DeferredType dt) {
            Attr attr = DeferredAttr.this.attr;
            attr.getClass();
            dt.check(new Attr.RecoveryInfo(attr, this.deferredAttrContext) { // from class: com.sun.tools.javac.comp.DeferredAttr.RecoveryDeferredTypeMap.1
                /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
                {
                    super(deferredAttrContext);
                    attr.getClass();
                }

                @Override // com.sun.tools.javac.comp.Attr.ResultInfo
                protected Type check(JCDiagnostic.DiagnosticPosition pos, Type found) {
                    return DeferredAttr.this.chk.checkNonVoid(pos, super.check(pos, found));
                }
            });
            return super.apply(dt);
        }
    }

    static abstract class FilterScanner extends TreeScanner {
        final Filter<JCTree> treeFilter;

        FilterScanner(final Set<JCTree.Tag> validTags) {
            this.treeFilter = new Filter<JCTree>() { // from class: com.sun.tools.javac.comp.DeferredAttr.FilterScanner.1
                @Override // com.sun.tools.javac.util.Filter
                public boolean accepts(JCTree t) {
                    return validTags.contains(t.getTag());
                }
            };
        }

        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree tree) {
            if (tree != null) {
                if (this.treeFilter.accepts(tree)) {
                    super.scan(tree);
                } else {
                    skip(tree);
                }
            }
        }

        void skip(JCTree tree) {
        }
    }

    static class PolyScanner extends FilterScanner {
        PolyScanner() {
            super(EnumSet.of(JCTree.Tag.CONDEXPR, JCTree.Tag.PARENS, JCTree.Tag.LAMBDA, JCTree.Tag.REFERENCE));
        }
    }

    static class LambdaReturnScanner extends FilterScanner {
        LambdaReturnScanner() {
            super(EnumSet.of(JCTree.Tag.BLOCK, JCTree.Tag.CASE, JCTree.Tag.CATCH, JCTree.Tag.DOLOOP, JCTree.Tag.FOREACHLOOP, JCTree.Tag.FORLOOP, JCTree.Tag.IF, JCTree.Tag.RETURN, JCTree.Tag.SYNCHRONIZED, JCTree.Tag.SWITCH, JCTree.Tag.TRY, JCTree.Tag.WHILELOOP));
        }
    }

    class CheckStuckPolicy extends PolyScanner implements DeferredStuckPolicy, Infer.FreeTypeListener {
        Infer.InferenceContext inferenceContext;
        Type pt;
        Set<Type> stuckVars = new LinkedHashSet();
        Set<Type> depVars = new LinkedHashSet();

        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredStuckPolicy
        public boolean isStuck() {
            return !this.stuckVars.isEmpty();
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredStuckPolicy
        public Set<Type> stuckVars() {
            return this.stuckVars;
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.DeferredStuckPolicy
        public Set<Type> depVars() {
            return this.depVars;
        }

        public CheckStuckPolicy(Attr.ResultInfo resultInfo, DeferredType dt) {
            this.pt = resultInfo.pt;
            this.inferenceContext = resultInfo.checkContext.inferenceContext();
            scan(dt.tree);
            if (!this.stuckVars.isEmpty()) {
                resultInfo.checkContext.inferenceContext().addFreeTypeListener(List.from(this.stuckVars), this);
            }
        }

        @Override // com.sun.tools.javac.comp.Infer.FreeTypeListener
        public void typesInferred(Infer.InferenceContext inferenceContext) {
            this.stuckVars.clear();
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            if (this.inferenceContext.inferenceVars().contains(this.pt)) {
                this.stuckVars.add(this.pt);
            }
            if (!DeferredAttr.this.types.isFunctionalInterface(this.pt)) {
                return;
            }
            Type descType = DeferredAttr.this.types.findDescriptorType(this.pt);
            List<Type> freeArgVars = this.inferenceContext.freeVarsIn(descType.mo176getParameterTypes());
            if (tree.paramKind == JCTree.JCLambda.ParameterKind.IMPLICIT && freeArgVars.nonEmpty()) {
                this.stuckVars.addAll(freeArgVars);
                this.depVars.addAll(this.inferenceContext.freeVarsIn(descType.mo178getReturnType()));
            }
            scanLambdaBody(tree, descType.mo178getReturnType());
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReference(JCTree.JCMemberReference tree) {
            scan(tree.expr);
            if (this.inferenceContext.inferenceVars().contains(this.pt)) {
                this.stuckVars.add(this.pt);
                return;
            }
            if (!DeferredAttr.this.types.isFunctionalInterface(this.pt)) {
                return;
            }
            Type descType = DeferredAttr.this.types.findDescriptorType(this.pt);
            List<Type> freeArgVars = this.inferenceContext.freeVarsIn(descType.mo176getParameterTypes());
            if (freeArgVars.nonEmpty() && tree.overloadKind == JCTree.JCMemberReference.OverloadKind.OVERLOADED) {
                this.stuckVars.addAll(freeArgVars);
                this.depVars.addAll(this.inferenceContext.freeVarsIn(descType.mo178getReturnType()));
            }
        }

        void scanLambdaBody(JCTree.JCLambda lambda, final Type pt) {
            if (lambda.getBodyKind() == LambdaExpressionTree.BodyKind.EXPRESSION) {
                Type prevPt = this.pt;
                try {
                    this.pt = pt;
                    scan(lambda.body);
                    return;
                } finally {
                    this.pt = prevPt;
                }
            }
            LambdaReturnScanner lambdaScanner = new LambdaReturnScanner() { // from class: com.sun.tools.javac.comp.DeferredAttr.CheckStuckPolicy.1
                @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
                public void visitReturn(JCTree.JCReturn tree) {
                    if (tree.expr != null) {
                        Type prevPt2 = CheckStuckPolicy.this.pt;
                        try {
                            CheckStuckPolicy.this.pt = pt;
                            CheckStuckPolicy.this.scan(tree.expr);
                        } finally {
                            CheckStuckPolicy.this.pt = prevPt2;
                        }
                    }
                }
            };
            lambdaScanner.scan(lambda.body);
        }
    }

    class OverloadStuckPolicy extends CheckStuckPolicy implements DeferredStuckPolicy {
        boolean stuck;

        @Override // com.sun.tools.javac.comp.DeferredAttr.CheckStuckPolicy, com.sun.tools.javac.comp.DeferredAttr.DeferredStuckPolicy
        public boolean isStuck() {
            return super.isStuck() || this.stuck;
        }

        public OverloadStuckPolicy(Attr.ResultInfo resultInfo, DeferredType dt) {
            super(resultInfo, dt);
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.CheckStuckPolicy, com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            super.visitLambda(tree);
            if (tree.paramKind == JCTree.JCLambda.ParameterKind.IMPLICIT) {
                this.stuck = true;
            }
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.CheckStuckPolicy, com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReference(JCTree.JCMemberReference tree) {
            super.visitReference(tree);
            if (tree.overloadKind == JCTree.JCMemberReference.OverloadKind.OVERLOADED) {
                this.stuck = true;
            }
        }
    }

    boolean isDeferred(Env<AttrContext> env, JCTree.JCExpression expr) {
        DeferredChecker dc = new DeferredChecker(env);
        dc.scan(expr);
        return dc.result.isPoly();
    }

    enum ArgumentExpressionKind {
        POLY,
        NO_POLY,
        PRIMITIVE;

        public final boolean isPoly() {
            return this == POLY;
        }

        public final boolean isPrimitive() {
            return this == PRIMITIVE;
        }

        static ArgumentExpressionKind standaloneKind(Type type, Types types) {
            return types.unboxedTypeOrType(type).isPrimitive() ? PRIMITIVE : NO_POLY;
        }

        static ArgumentExpressionKind methodKind(Symbol sym, Types types) {
            Type restype = sym.type.mo178getReturnType();
            if (sym.type.hasTag(TypeTag.FORALL) && restype.containsAny(((Type.ForAll) sym.type).tvars)) {
                return POLY;
            }
            return standaloneKind(restype, types);
        }
    }

    final class DeferredChecker extends FilterScanner {
        MethodAnalyzer<ArgumentExpressionKind> argumentKindAnalyzer;
        Env<AttrContext> env;
        ArgumentExpressionKind result;
        MethodAnalyzer<Symbol> returnSymbolAnalyzer;

        public DeferredChecker(Env<AttrContext> env) {
            super(DeferredAttr.this.deferredCheckerTags);
            this.argumentKindAnalyzer = new MethodAnalyzer<ArgumentExpressionKind>() { // from class: com.sun.tools.javac.comp.DeferredAttr.DeferredChecker.1
                /* JADX WARN: Can't rename method to resolve collision */
                @Override // com.sun.tools.javac.comp.DeferredAttr.MethodAnalyzer
                public ArgumentExpressionKind process(Symbol.MethodSymbol ms) {
                    return ArgumentExpressionKind.methodKind(ms, DeferredAttr.this.types);
                }

                @Override // com.sun.tools.javac.comp.DeferredAttr.MethodAnalyzer
                public ArgumentExpressionKind reduce(ArgumentExpressionKind kind1, ArgumentExpressionKind kind2) {
                    switch (kind1) {
                        case PRIMITIVE:
                            return kind2;
                        case NO_POLY:
                            return kind2.isPoly() ? kind2 : kind1;
                        case POLY:
                            return kind1;
                        default:
                            Assert.error();
                            return null;
                    }
                }

                @Override // com.sun.tools.javac.comp.DeferredAttr.MethodAnalyzer
                public boolean shouldStop(ArgumentExpressionKind result) {
                    return result.isPoly();
                }
            };
            this.returnSymbolAnalyzer = new MethodAnalyzer<Symbol>() { // from class: com.sun.tools.javac.comp.DeferredAttr.DeferredChecker.3
                /* JADX WARN: Can't rename method to resolve collision */
                @Override // com.sun.tools.javac.comp.DeferredAttr.MethodAnalyzer
                public Symbol process(Symbol.MethodSymbol ms) {
                    ArgumentExpressionKind kind = ArgumentExpressionKind.methodKind(ms, DeferredAttr.this.types);
                    if (kind == ArgumentExpressionKind.POLY || ms.getReturnType().hasTag(TypeTag.TYPEVAR)) {
                        return null;
                    }
                    return ms.getReturnType().tsym;
                }

                @Override // com.sun.tools.javac.comp.DeferredAttr.MethodAnalyzer
                public Symbol reduce(Symbol s1, Symbol s2) {
                    if (s1 == DeferredAttr.this.syms.errSymbol) {
                        return s2;
                    }
                    if (s1 == s2) {
                        return s1;
                    }
                    return null;
                }

                @Override // com.sun.tools.javac.comp.DeferredAttr.MethodAnalyzer
                public boolean shouldStop(Symbol result) {
                    return result == null;
                }
            };
            this.env = env;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            this.result = ArgumentExpressionKind.POLY;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReference(JCTree.JCMemberReference tree) {
            Env<AttrContext> localEnv = this.env.dup(tree);
            JCTree.JCExpression exprTree = (JCTree.JCExpression) DeferredAttr.this.attribSpeculative(tree.getQualifierExpression(), localEnv, DeferredAttr.this.attr.memberReferenceQualifierResult(tree));
            JCTree.JCMemberReference mref2 = (JCTree.JCMemberReference) new TreeCopier(DeferredAttr.this.make).copy(tree);
            mref2.expr = exprTree;
            Symbol res = DeferredAttr.this.rs.getMemberReference(tree, localEnv, mref2, exprTree.type, tree.name);
            tree.sym = res;
            if (res.kind >= 128 || res.type.hasTag(TypeTag.FORALL) || (res.flags() & Flags.VARARGS) != 0 || (TreeInfo.isStaticSelector(exprTree, tree.name.table.names) && exprTree.type.isRaw())) {
                tree.overloadKind = JCTree.JCMemberReference.OverloadKind.OVERLOADED;
            } else {
                tree.overloadKind = JCTree.JCMemberReference.OverloadKind.UNOVERLOADED;
            }
            this.result = ArgumentExpressionKind.POLY;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeCast(JCTree.JCTypeCast tree) {
            this.result = ArgumentExpressionKind.NO_POLY;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitConditional(JCTree.JCConditional tree) {
            scan(tree.truepart);
            if (!this.result.isPrimitive()) {
                this.result = ArgumentExpressionKind.POLY;
            } else {
                scan(tree.falsepart);
                this.result = reduce(ArgumentExpressionKind.PRIMITIVE);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            this.result = (TreeInfo.isDiamond(tree) || DeferredAttr.this.attr.findDiamonds) ? ArgumentExpressionKind.POLY : ArgumentExpressionKind.NO_POLY;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation tree) {
            Name name = TreeInfo.name(tree.meth);
            if (tree.typeargs.nonEmpty() || name == name.table.names._this || name == name.table.names._super) {
                this.result = ArgumentExpressionKind.NO_POLY;
                return;
            }
            Symbol sym = quicklyResolveMethod(this.env, tree);
            if (sym == null) {
                this.result = ArgumentExpressionKind.POLY;
            } else {
                this.result = (ArgumentExpressionKind) analyzeCandidateMethods(sym, ArgumentExpressionKind.PRIMITIVE, this.argumentKindAnalyzer);
            }
        }

        private boolean isSimpleReceiver(JCTree rec) {
            switch (rec.getTag()) {
                case IDENT:
                    return true;
                case SELECT:
                    return isSimpleReceiver(((JCTree.JCFieldAccess) rec).selected);
                case TYPEAPPLY:
                case TYPEARRAY:
                    return true;
                case ANNOTATED_TYPE:
                    return isSimpleReceiver(((JCTree.JCAnnotatedType) rec).underlyingType);
                case APPLY:
                    return true;
                case NEWCLASS:
                    JCTree.JCNewClass nc = (JCTree.JCNewClass) rec;
                    return nc.encl == null && nc.def == null && !TreeInfo.isDiamond(nc);
                default:
                    return false;
            }
        }

        private ArgumentExpressionKind reduce(ArgumentExpressionKind kind) {
            return this.argumentKindAnalyzer.reduce(this.result, kind);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLiteral(JCTree.JCLiteral tree) {
            Type litType = DeferredAttr.this.attr.litType(tree.typetag);
            this.result = ArgumentExpressionKind.standaloneKind(litType, DeferredAttr.this.types);
        }

        @Override // com.sun.tools.javac.comp.DeferredAttr.FilterScanner
        void skip(JCTree tree) {
            this.result = ArgumentExpressionKind.NO_POLY;
        }

        private Symbol quicklyResolveMethod(Env<AttrContext> env, JCTree.JCMethodInvocation tree) {
            Type site;
            Symbol resolvedReturnType;
            JCTree.JCExpression rec = tree.meth.hasTag(JCTree.Tag.SELECT) ? ((JCTree.JCFieldAccess) tree.meth).selected : null;
            if (rec != null && !isSimpleReceiver(rec)) {
                return null;
            }
            if (rec != null) {
                switch (rec.getTag()) {
                    case APPLY:
                        Symbol recSym = quicklyResolveMethod(env, (JCTree.JCMethodInvocation) rec);
                        if (recSym == null || (resolvedReturnType = (Symbol) analyzeCandidateMethods(recSym, DeferredAttr.this.syms.errSymbol, this.returnSymbolAnalyzer)) == null) {
                            return null;
                        }
                        site = resolvedReturnType.type;
                        break;
                    case NEWCLASS:
                        JCTree.JCNewClass nc = (JCTree.JCNewClass) rec;
                        site = DeferredAttr.this.attribSpeculative(nc.clazz, env, DeferredAttr.this.attr.unknownTypeExprInfo).type;
                        break;
                    default:
                        site = DeferredAttr.this.attribSpeculative(rec, env, DeferredAttr.this.attr.unknownTypeExprInfo).type;
                        break;
                }
            } else {
                site = env.enclClass.sym.type;
            }
            while (site.hasTag(TypeTag.TYPEVAR)) {
                site = site.getUpperBound();
            }
            Type site2 = DeferredAttr.this.types.capture(site);
            List<Type> args = DeferredAttr.this.rs.dummyArgs(tree.args.length());
            Name name = TreeInfo.name(tree.meth);
            Resolve resolve = DeferredAttr.this.rs;
            resolve.getClass();
            Resolve.LookupHelper lh = new Resolve.LookupHelper(resolve, name, site2, args, List.nil(), Resolve.MethodResolutionPhase.VARARITY, rec) { // from class: com.sun.tools.javac.comp.DeferredAttr.DeferredChecker.2
                final /* synthetic */ JCTree.JCExpression val$rec;

                /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
                {
                    super(name, site2, args, list, maxPhase);
                    this.val$rec = rec;
                    resolve.getClass();
                }

                @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
                Symbol lookup(Env<AttrContext> env2, Resolve.MethodResolutionPhase phase) {
                    if (this.val$rec == null) {
                        return DeferredAttr.this.rs.findFun(env2, this.name, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired());
                    }
                    return DeferredAttr.this.rs.findMethod(env2, this.site, this.name, this.argtypes, this.typeargtypes, phase.isBoxingRequired(), phase.isVarargsRequired(), false);
                }

                @Override // com.sun.tools.javac.comp.Resolve.LookupHelper
                Symbol access(Env<AttrContext> env2, JCDiagnostic.DiagnosticPosition pos, Symbol location, Symbol sym) {
                    return sym;
                }
            };
            return DeferredAttr.this.rs.lookupMethod(env, tree, site2.tsym, DeferredAttr.this.rs.arityMethodCheck, lh);
        }

        <E> E analyzeCandidateMethods(Symbol sym, E defaultValue, MethodAnalyzer<E> analyzer) {
            switch (sym.kind) {
                case 16:
                    return analyzer.process((Symbol.MethodSymbol) sym);
                case 129:
                    Resolve.AmbiguityError err = (Resolve.AmbiguityError) sym.baseSymbol();
                    E res = defaultValue;
                    for (Symbol s : err.ambiguousSyms) {
                        if (s.kind == 16) {
                            res = analyzer.reduce(res, analyzer.process((Symbol.MethodSymbol) s));
                            if (analyzer.shouldStop(res)) {
                                return res;
                            }
                        }
                    }
                    return res;
                default:
                    return defaultValue;
            }
        }
    }
}
