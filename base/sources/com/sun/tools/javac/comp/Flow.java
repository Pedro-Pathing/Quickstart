package com.sun.tools.javac.comp;

import com.sun.source.tree.LambdaExpressionTree;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.util.ArrayUtils;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Bits;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Names;
import java.util.AbstractCollection;
import java.util.HashMap;
import java.util.Iterator;

/* JADX INFO: loaded from: classes.dex */
public class Flow {
    protected static final Context.Key<Flow> flowKey = new Context.Key<>();
    private final boolean allowEffectivelyFinalInInnerClasses;
    private final boolean allowImprovedCatchAnalysis;
    private final boolean allowImprovedRethrowAnalysis;
    private Env<AttrContext> attrEnv;
    private final Check chk;
    private final JCDiagnostic.Factory diags;
    private final boolean enforceThisDotInit;
    private Lint lint;
    private final Log log;
    private TreeMaker make;
    private final Names names;
    private final Resolve rs;
    private final Symtab syms;
    private final Types types;

    public static Flow instance(Context context) {
        Flow instance = (Flow) context.get(flowKey);
        if (instance == null) {
            return new Flow(context);
        }
        return instance;
    }

    public void analyzeTree(Env<AttrContext> env, TreeMaker make) {
        new AliveAnalyzer().analyzeTree(env, make);
        new AssignAnalyzer().analyzeTree(env);
        new FlowAnalyzer().analyzeTree(env, make);
        new CaptureAnalyzer().analyzeTree(env, make);
    }

    public void analyzeLambda(Env<AttrContext> env, JCTree.JCLambda that, TreeMaker make, boolean speculative) {
        Log.DiagnosticHandler diagHandler = null;
        if (!speculative) {
            diagHandler = new Log.DiscardDiagnosticHandler(this.log);
        }
        try {
            new AliveAnalyzer().analyzeTree(env, that, make);
        } finally {
            if (!speculative) {
                this.log.popDiagnosticHandler(diagHandler);
            }
        }
    }

    public List<Type> analyzeLambdaThrownTypes(final Env<AttrContext> env, JCTree.JCLambda that, TreeMaker make) {
        Log.DiagnosticHandler diagHandler = new Log.DiscardDiagnosticHandler(this.log);
        try {
            new AssignAnalyzer() { // from class: com.sun.tools.javac.comp.Flow.1
                Scope enclosedSymbols;

                /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
                {
                    super();
                    this.enclosedSymbols = new Scope(env.enclClass.sym);
                }

                @Override // com.sun.tools.javac.comp.Flow.AssignAnalyzer, com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
                public void visitVarDef(JCTree.JCVariableDecl tree) {
                    this.enclosedSymbols.enter(tree.sym);
                    super.visitVarDef(tree);
                }

                @Override // com.sun.tools.javac.comp.Flow.AssignAnalyzer
                protected boolean trackable(Symbol.VarSymbol sym) {
                    return this.enclosedSymbols.includes(sym) && sym.owner.kind == 16;
                }
            }.analyzeTree(env, that);
            LambdaFlowAnalyzer flowAnalyzer = new LambdaFlowAnalyzer();
            flowAnalyzer.analyzeTree(env, that, make);
            return flowAnalyzer.inferredThrownTypes;
        } finally {
            this.log.popDiagnosticHandler(diagHandler);
        }
    }

    enum FlowKind {
        NORMAL("var.might.already.be.assigned", false),
        SPECULATIVE_LOOP("var.might.be.assigned.in.loop", true);

        final String errKey;
        final boolean isFinal;

        FlowKind(String errKey, boolean isFinal) {
            this.errKey = errKey;
            this.isFinal = isFinal;
        }

        boolean isFinal() {
            return this.isFinal;
        }
    }

    protected Flow(Context context) {
        context.put(flowKey, this);
        this.names = Names.instance(context);
        this.log = Log.instance(context);
        this.syms = Symtab.instance(context);
        this.types = Types.instance(context);
        this.chk = Check.instance(context);
        this.lint = Lint.instance(context);
        this.rs = Resolve.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        Source source = Source.instance(context);
        this.allowImprovedRethrowAnalysis = source.allowImprovedRethrowAnalysis();
        this.allowImprovedCatchAnalysis = source.allowImprovedCatchAnalysis();
        this.allowEffectivelyFinalInInnerClasses = source.allowEffectivelyFinalInInnerClasses();
        this.enforceThisDotInit = source.enforceThisDotInit();
    }

    static abstract class BaseAnalyzer<P extends PendingExit> extends TreeScanner {
        ListBuffer<P> pendingExits;

        abstract void markDead();

        BaseAnalyzer() {
        }

        enum JumpKind {
            BREAK(JCTree.Tag.BREAK) { // from class: com.sun.tools.javac.comp.Flow.BaseAnalyzer.JumpKind.1
                @Override // com.sun.tools.javac.comp.Flow.BaseAnalyzer.JumpKind
                JCTree getTarget(JCTree tree) {
                    return ((JCTree.JCBreak) tree).target;
                }
            },
            CONTINUE(JCTree.Tag.CONTINUE) { // from class: com.sun.tools.javac.comp.Flow.BaseAnalyzer.JumpKind.2
                @Override // com.sun.tools.javac.comp.Flow.BaseAnalyzer.JumpKind
                JCTree getTarget(JCTree tree) {
                    return ((JCTree.JCContinue) tree).target;
                }
            };

            final JCTree.Tag treeTag;

            abstract JCTree getTarget(JCTree jCTree);

            JumpKind(JCTree.Tag treeTag) {
                this.treeTag = treeTag;
            }
        }

        static class PendingExit {
            JCTree tree;

            PendingExit(JCTree tree) {
                this.tree = tree;
            }

            void resolveJump() {
            }
        }

        void recordExit(P pe) {
            this.pendingExits.append(pe);
            markDead();
        }

        /* JADX WARN: Type inference incomplete: some casts might be missing */
        private boolean resolveJump(JCTree jCTree, ListBuffer<P> listBuffer, JumpKind jumpKind) {
            boolean z = false;
            this.pendingExits = listBuffer;
            for (List list = this.pendingExits.toList(); list.nonEmpty(); list = list.tail) {
                PendingExit pendingExit = (PendingExit) list.head;
                if (pendingExit.tree.hasTag(jumpKind.treeTag) && jumpKind.getTarget(pendingExit.tree) == jCTree) {
                    pendingExit.resolveJump();
                    z = true;
                } else {
                    this.pendingExits.append((P) pendingExit);
                }
            }
            return z;
        }

        boolean resolveContinues(JCTree tree) {
            return resolveJump(tree, new ListBuffer<>(), JumpKind.CONTINUE);
        }

        boolean resolveBreaks(JCTree tree, ListBuffer<P> oldPendingExits) {
            return resolveJump(tree, oldPendingExits, JumpKind.BREAK);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree tree) {
            if (tree != null) {
                if (tree.type == null || tree.type != Type.stuckType) {
                    super.scan(tree);
                }
            }
        }
    }

    class AliveAnalyzer extends BaseAnalyzer<BaseAnalyzer.PendingExit> {
        private boolean alive;

        AliveAnalyzer() {
        }

        @Override // com.sun.tools.javac.comp.Flow.BaseAnalyzer
        void markDead() {
            this.alive = false;
        }

        void scanDef(JCTree tree) {
            scanStat(tree);
            if (tree != null && tree.hasTag(JCTree.Tag.BLOCK) && !this.alive) {
                Flow.this.log.error(tree.pos(), "initializer.must.be.able.to.complete.normally", new Object[0]);
            }
        }

        void scanStat(JCTree tree) {
            if (!this.alive && tree != null) {
                Flow.this.log.error(tree.pos(), "unreachable.stmt", new Object[0]);
                if (!tree.hasTag(JCTree.Tag.SKIP)) {
                    this.alive = true;
                }
            }
            scan(tree);
        }

        /* JADX WARN: Multi-variable type inference failed */
        void scanStats(List<? extends JCTree.JCStatement> trees) {
            if (trees != null) {
                for (List list = trees; list.nonEmpty(); list = list.tail) {
                    scanStat((JCTree) list.head);
                }
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            if (tree.sym == null) {
                return;
            }
            boolean alivePrev = this.alive;
            ListBuffer<P> listBuffer = this.pendingExits;
            Lint lintPrev = Flow.this.lint;
            this.pendingExits = new ListBuffer<>();
            Flow.this.lint = Flow.this.lint.augment(tree.sym);
            try {
                for (List list = tree.defs; list.nonEmpty(); list = list.tail) {
                    if (!((JCTree) list.head).hasTag(JCTree.Tag.METHODDEF) && (8 & TreeInfo.flags((JCTree) list.head)) != 0) {
                        scanDef((JCTree) list.head);
                    }
                }
                for (List list2 = tree.defs; list2.nonEmpty(); list2 = list2.tail) {
                    if (!((JCTree) list2.head).hasTag(JCTree.Tag.METHODDEF) && (TreeInfo.flags((JCTree) list2.head) & 8) == 0) {
                        scanDef((JCTree) list2.head);
                    }
                }
                for (List list3 = tree.defs; list3.nonEmpty(); list3 = list3.tail) {
                    if (((JCTree) list3.head).hasTag(JCTree.Tag.METHODDEF)) {
                        scan((JCTree) list3.head);
                    }
                }
            } finally {
                this.pendingExits = listBuffer;
                this.alive = alivePrev;
                Flow.this.lint = lintPrev;
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl tree) {
            if (tree.body == null) {
                return;
            }
            Lint lintPrev = Flow.this.lint;
            Flow.this.lint = Flow.this.lint.augment(tree.sym);
            Assert.check(this.pendingExits.isEmpty());
            try {
                this.alive = true;
                scanStat(tree.body);
                if (this.alive && !tree.sym.type.mo178getReturnType().hasTag(TypeTag.VOID)) {
                    Flow.this.log.error(TreeInfo.diagEndPos(tree.body), "missing.ret.stmt", new Object[0]);
                }
                List list = this.pendingExits.toList();
                this.pendingExits = new ListBuffer<>();
                while (list.nonEmpty()) {
                    BaseAnalyzer.PendingExit exit = (BaseAnalyzer.PendingExit) list.head;
                    list = list.tail;
                    Assert.check(exit.tree.hasTag(JCTree.Tag.RETURN));
                }
            } finally {
                Flow.this.lint = lintPrev;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            if (tree.init != null) {
                Lint lintPrev = Flow.this.lint;
                Flow.this.lint = Flow.this.lint.augment(tree.sym);
                try {
                    scan(tree.init);
                } finally {
                    Flow.this.lint = lintPrev;
                }
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBlock(JCTree.JCBlock tree) {
            scanStats(tree.stats);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
            ListBuffer<BaseAnalyzer.PendingExit> prevPendingExits = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            scanStat(tree.body);
            this.alive |= resolveContinues(tree);
            scan(tree.cond);
            this.alive = this.alive && !tree.cond.type.isTrue();
            this.alive |= resolveBreaks(tree, prevPendingExits);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitWhileLoop(JCTree.JCWhileLoop tree) {
            ListBuffer<BaseAnalyzer.PendingExit> prevPendingExits = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            scan(tree.cond);
            boolean z = true;
            this.alive = !tree.cond.type.isFalse();
            scanStat(tree.body);
            this.alive |= resolveContinues(tree);
            if (!resolveBreaks(tree, prevPendingExits) && tree.cond.type.isTrue()) {
                z = false;
            }
            this.alive = z;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitForLoop(JCTree.JCForLoop tree) {
            ListBuffer<BaseAnalyzer.PendingExit> prevPendingExits = this.pendingExits;
            scanStats(tree.init);
            this.pendingExits = new ListBuffer<>();
            boolean z = true;
            if (tree.cond != null) {
                scan(tree.cond);
                this.alive = !tree.cond.type.isFalse();
            } else {
                this.alive = true;
            }
            scanStat(tree.body);
            this.alive |= resolveContinues(tree);
            scan(tree.step);
            if (!resolveBreaks(tree, prevPendingExits) && (tree.cond == null || tree.cond.type.isTrue())) {
                z = false;
            }
            this.alive = z;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
            visitVarDef(tree.var);
            ListBuffer<BaseAnalyzer.PendingExit> prevPendingExits = this.pendingExits;
            scan(tree.expr);
            this.pendingExits = new ListBuffer<>();
            scanStat(tree.body);
            this.alive |= resolveContinues(tree);
            resolveBreaks(tree, prevPendingExits);
            this.alive = true;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLabelled(JCTree.JCLabeledStatement tree) {
            ListBuffer<BaseAnalyzer.PendingExit> prevPendingExits = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            scanStat(tree.body);
            this.alive |= resolveBreaks(tree, prevPendingExits);
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSwitch(JCTree.JCSwitch tree) {
            ListBuffer<BaseAnalyzer.PendingExit> prevPendingExits = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            scan(tree.selector);
            boolean hasDefault = false;
            for (List list = tree.cases; list.nonEmpty(); list = list.tail) {
                this.alive = true;
                JCTree.JCCase c = (JCTree.JCCase) list.head;
                if (c.pat == null) {
                    hasDefault = true;
                } else {
                    scan(c.pat);
                }
                scanStats(c.stats);
                if (this.alive && Flow.this.lint.isEnabled(Lint.LintCategory.FALLTHROUGH) && c.stats.nonEmpty() && list.tail.nonEmpty()) {
                    Flow.this.log.warning(Lint.LintCategory.FALLTHROUGH, ((JCTree.JCCase) list.tail.head).pos(), "possible.fall-through.into.case", new Object[0]);
                }
            }
            if (!hasDefault) {
                this.alive = true;
            }
            this.alive |= resolveBreaks(tree, prevPendingExits);
        }

        /* JADX WARN: Type inference incomplete: some casts might be missing */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTry(JCTree.JCTry jCTry) {
            ListBuffer<P> listBuffer = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            for (JCTree jCTree : jCTry.resources) {
                if (jCTree instanceof JCTree.JCVariableDecl) {
                    visitVarDef((JCTree.JCVariableDecl) jCTree);
                } else if (jCTree instanceof JCTree.JCExpression) {
                    scan((JCTree.JCExpression) jCTree);
                } else {
                    throw new AssertionError(jCTry);
                }
            }
            scanStat(jCTry.body);
            boolean z = this.alive;
            for (List list = jCTry.catchers; list.nonEmpty(); list = list.tail) {
                this.alive = true;
                scan(((JCTree.JCCatch) list.head).param);
                scanStat(((JCTree.JCCatch) list.head).body);
                z |= this.alive;
            }
            if (jCTry.finalizer != null) {
                ListBuffer<P> listBuffer2 = this.pendingExits;
                this.pendingExits = listBuffer;
                this.alive = true;
                scanStat(jCTry.finalizer);
                jCTry.finallyCanCompleteNormally = this.alive;
                if (!this.alive) {
                    if (Flow.this.lint.isEnabled(Lint.LintCategory.FINALLY)) {
                        Flow.this.log.warning(Lint.LintCategory.FINALLY, TreeInfo.diagEndPos(jCTry.finalizer), "finally.cannot.complete", new Object[0]);
                        return;
                    }
                    return;
                } else {
                    while (listBuffer2.nonEmpty()) {
                        this.pendingExits.append((P) listBuffer2.next());
                    }
                    this.alive = z;
                    return;
                }
            }
            this.alive = z;
            ListBuffer<P> listBuffer3 = this.pendingExits;
            this.pendingExits = listBuffer;
            while (listBuffer3.nonEmpty()) {
                this.pendingExits.append((P) listBuffer3.next());
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIf(JCTree.JCIf tree) {
            scan(tree.cond);
            scanStat(tree.thenpart);
            if (tree.elsepart != null) {
                boolean aliveAfterThen = this.alive;
                this.alive = true;
                scanStat(tree.elsepart);
                this.alive |= aliveAfterThen;
                return;
            }
            this.alive = true;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBreak(JCTree.JCBreak tree) {
            recordExit(new BaseAnalyzer.PendingExit(tree));
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitContinue(JCTree.JCContinue tree) {
            recordExit(new BaseAnalyzer.PendingExit(tree));
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReturn(JCTree.JCReturn tree) {
            scan(tree.expr);
            recordExit(new BaseAnalyzer.PendingExit(tree));
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitThrow(JCTree.JCThrow tree) {
            scan(tree.expr);
            markDead();
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation tree) {
            scan(tree.meth);
            scan(tree.args);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            scan(tree.encl);
            scan(tree.args);
            if (tree.def != null) {
                scan(tree.def);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            if (tree.type != null && tree.type.isErroneous()) {
                return;
            }
            ListBuffer<P> listBuffer = this.pendingExits;
            boolean prevAlive = this.alive;
            try {
                this.pendingExits = new ListBuffer<>();
                this.alive = true;
                scanStat(tree.body);
                tree.canCompleteNormally = this.alive;
            } finally {
                this.pendingExits = listBuffer;
                this.alive = prevAlive;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTopLevel(JCTree.JCCompilationUnit tree) {
        }

        public void analyzeTree(Env<AttrContext> env, TreeMaker make) {
            analyzeTree(env, env.tree, make);
        }

        /* JADX WARN: Multi-variable type inference failed */
        public void analyzeTree(Env<AttrContext> env, JCTree jCTree, TreeMaker treeMaker) {
            try {
                Flow.this.attrEnv = env;
                Flow.this.make = treeMaker;
                this.pendingExits = new ListBuffer<>();
                this.alive = true;
                scan(jCTree);
            } finally {
                this.pendingExits = null;
                Flow.this.make = null;
            }
        }
    }

    class FlowAnalyzer extends BaseAnalyzer<FlowPendingExit> {
        List<Type> caught;
        JCTree.JCClassDecl classDef;
        HashMap<Symbol, List<Type>> preciseRethrowTypes;
        List<Type> thrown;

        FlowAnalyzer() {
        }

        class FlowPendingExit extends BaseAnalyzer.PendingExit {
            Type thrown;

            FlowPendingExit(JCTree tree, Type thrown) {
                super(tree);
                this.thrown = thrown;
            }
        }

        @Override // com.sun.tools.javac.comp.Flow.BaseAnalyzer
        void markDead() {
        }

        void errorUncaught() {
            FlowPendingExit exit = (FlowPendingExit) this.pendingExits.next();
            while (exit != null) {
                if (this.classDef != null && this.classDef.pos == exit.tree.pos) {
                    Flow.this.log.error(exit.tree.pos(), "unreported.exception.default.constructor", exit.thrown);
                } else if (!exit.tree.hasTag(JCTree.Tag.VARDEF) || !((JCTree.JCVariableDecl) exit.tree).sym.isResourceVariable()) {
                    Flow.this.log.error(exit.tree.pos(), "unreported.exception.need.to.catch.or.throw", exit.thrown);
                } else {
                    Flow.this.log.error(exit.tree.pos(), "unreported.exception.implicit.close", exit.thrown, ((JCTree.JCVariableDecl) exit.tree).sym.name);
                }
                exit = (FlowPendingExit) this.pendingExits.next();
            }
        }

        /* JADX WARN: Type inference incomplete: some casts might be missing */
        void markThrown(JCTree jCTree, Type type) {
            if (!Flow.this.chk.isUnchecked(jCTree.pos(), type)) {
                if (!Flow.this.chk.isHandled(type, this.caught)) {
                    this.pendingExits.append(new FlowPendingExit(jCTree, type));
                }
                this.thrown = Flow.this.chk.incl(type, this.thrown);
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            if (tree.sym == null) {
                return;
            }
            JCTree.JCClassDecl classDefPrev = this.classDef;
            List<Type> thrownPrev = this.thrown;
            List<Type> caughtPrev = this.caught;
            ListBuffer<P> listBuffer = this.pendingExits;
            Lint lintPrev = Flow.this.lint;
            this.pendingExits = new ListBuffer<>();
            if (tree.name != Flow.this.names.empty) {
                this.caught = List.nil();
            }
            this.classDef = tree;
            this.thrown = List.nil();
            Flow.this.lint = Flow.this.lint.augment(tree.sym);
            try {
                for (List list = tree.defs; list.nonEmpty(); list = list.tail) {
                    if (!((JCTree) list.head).hasTag(JCTree.Tag.METHODDEF) && (8 & TreeInfo.flags((JCTree) list.head)) != 0) {
                        scan((JCTree) list.head);
                        errorUncaught();
                    }
                }
                if (tree.name != Flow.this.names.empty) {
                    boolean firstConstructor = true;
                    for (List list2 = tree.defs; list2.nonEmpty(); list2 = list2.tail) {
                        if (TreeInfo.isInitialConstructor((JCTree) list2.head)) {
                            List<Type> mthrown = ((JCTree.JCMethodDecl) list2.head).sym.type.mo179getThrownTypes();
                            if (!firstConstructor) {
                                this.caught = Flow.this.chk.intersect(mthrown, this.caught);
                            } else {
                                this.caught = mthrown;
                                firstConstructor = false;
                            }
                        }
                    }
                }
                for (List list3 = tree.defs; list3.nonEmpty(); list3 = list3.tail) {
                    if (!((JCTree) list3.head).hasTag(JCTree.Tag.METHODDEF) && (TreeInfo.flags((JCTree) list3.head) & 8) == 0) {
                        scan((JCTree) list3.head);
                        errorUncaught();
                    }
                }
                if (tree.name == Flow.this.names.empty) {
                    for (List list4 = tree.defs; list4.nonEmpty(); list4 = list4.tail) {
                        if (TreeInfo.isInitialConstructor((JCTree) list4.head)) {
                            JCTree.JCMethodDecl mdef = (JCTree.JCMethodDecl) list4.head;
                            mdef.thrown = Flow.this.make.Types(this.thrown);
                            mdef.sym.type = Flow.this.types.createMethodTypeWithThrown(mdef.sym.type, this.thrown);
                        }
                    }
                    thrownPrev = Flow.this.chk.union(this.thrown, thrownPrev);
                }
                for (List list5 = tree.defs; list5.nonEmpty(); list5 = list5.tail) {
                    if (((JCTree) list5.head).hasTag(JCTree.Tag.METHODDEF)) {
                        scan((JCTree) list5.head);
                        errorUncaught();
                    }
                }
                this.thrown = thrownPrev;
            } finally {
                this.pendingExits = listBuffer;
                this.caught = caughtPrev;
                this.classDef = classDefPrev;
                Flow.this.lint = lintPrev;
            }
        }

        /* JADX WARN: Type inference incomplete: some casts might be missing */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl jCMethodDecl) {
            if (jCMethodDecl.body == null) {
                return;
            }
            List<Type> list = this.caught;
            List<Type> listMo179getThrownTypes = jCMethodDecl.sym.type.mo179getThrownTypes();
            Lint lint = Flow.this.lint;
            Flow.this.lint = Flow.this.lint.augment(jCMethodDecl.sym);
            Assert.check(this.pendingExits.isEmpty());
            try {
                for (List list2 = jCMethodDecl.params; list2.nonEmpty(); list2 = list2.tail) {
                    scan((JCTree.JCVariableDecl) list2.head);
                }
                if (TreeInfo.isInitialConstructor(jCMethodDecl)) {
                    this.caught = Flow.this.chk.union(this.caught, listMo179getThrownTypes);
                } else if ((jCMethodDecl.sym.flags() & 1048584) != 1048576) {
                    this.caught = listMo179getThrownTypes;
                }
                scan(jCMethodDecl.body);
                List list3 = this.pendingExits.toList();
                this.pendingExits = new ListBuffer<>();
                while (list3.nonEmpty()) {
                    FlowPendingExit flowPendingExit = (FlowPendingExit) list3.head;
                    list3 = list3.tail;
                    if (flowPendingExit.thrown == null) {
                        Assert.check(flowPendingExit.tree.hasTag(JCTree.Tag.RETURN));
                    } else {
                        this.pendingExits.append(flowPendingExit);
                    }
                }
            } finally {
                this.caught = list;
                Flow.this.lint = lint;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            if (tree.init != null) {
                Lint lintPrev = Flow.this.lint;
                Flow.this.lint = Flow.this.lint.augment(tree.sym);
                try {
                    scan(tree.init);
                } finally {
                    Flow.this.lint = lintPrev;
                }
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBlock(JCTree.JCBlock tree) {
            scan(tree.stats);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            scan(tree.body);
            resolveContinues(tree);
            scan(tree.cond);
            resolveBreaks(tree, abstractCollection);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitWhileLoop(JCTree.JCWhileLoop tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            scan(tree.cond);
            scan(tree.body);
            resolveContinues(tree);
            resolveBreaks(tree, abstractCollection);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitForLoop(JCTree.JCForLoop tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            scan(tree.init);
            this.pendingExits = new ListBuffer<>();
            if (tree.cond != null) {
                scan(tree.cond);
            }
            scan(tree.body);
            resolveContinues(tree);
            scan(tree.step);
            resolveBreaks(tree, abstractCollection);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
            visitVarDef(tree.var);
            AbstractCollection abstractCollection = this.pendingExits;
            scan(tree.expr);
            this.pendingExits = new ListBuffer<>();
            scan(tree.body);
            resolveContinues(tree);
            resolveBreaks(tree, abstractCollection);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLabelled(JCTree.JCLabeledStatement tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            scan(tree.body);
            resolveBreaks(tree, abstractCollection);
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSwitch(JCTree.JCSwitch tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            scan(tree.selector);
            for (List list = tree.cases; list.nonEmpty(); list = list.tail) {
                JCTree.JCCase c = (JCTree.JCCase) list.head;
                if (c.pat != null) {
                    scan(c.pat);
                }
                scan(c.stats);
            }
            resolveBreaks(tree, abstractCollection);
        }

        /* JADX WARN: Type inference incomplete: some casts might be missing */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTry(JCTree.JCTry jCTry) {
            List<Type> listUnion;
            List<JCTree.JCExpression> list;
            List<Type> listOf;
            List<Type> list2 = this.caught;
            List<Type> list3 = this.thrown;
            this.thrown = List.nil();
            for (List list4 = jCTry.catchers; list4.nonEmpty(); list4 = list4.tail) {
                Iterator<JCTree.JCExpression> it = (TreeInfo.isMultiCatch((JCTree.JCCatch) list4.head) ? ((JCTree.JCTypeUnion) ((JCTree.JCCatch) list4.head).param.vartype).alternatives : List.of(((JCTree.JCCatch) list4.head).param.vartype)).iterator();
                while (it.hasNext()) {
                    this.caught = Flow.this.chk.incl(it.next().type, this.caught);
                }
            }
            ListBuffer<P> listBuffer = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            for (JCTree jCTree : jCTry.resources) {
                if (jCTree instanceof JCTree.JCVariableDecl) {
                    visitVarDef((JCTree.JCVariableDecl) jCTree);
                } else if (jCTree instanceof JCTree.JCExpression) {
                    scan((JCTree.JCExpression) jCTree);
                } else {
                    throw new AssertionError(jCTry);
                }
            }
            for (JCTree jCTree2 : jCTry.resources) {
                if (jCTree2.type.isCompound()) {
                    listOf = Flow.this.types.interfaces(jCTree2.type).prepend(Flow.this.types.supertype(jCTree2.type));
                } else {
                    listOf = List.of(jCTree2.type);
                }
                for (Type type : listOf) {
                    if (Flow.this.types.asSuper(type, Flow.this.syms.autoCloseableType.tsym) != null) {
                        Symbol symbolResolveQualifiedMethod = Flow.this.rs.resolveQualifiedMethod(jCTry, Flow.this.attrEnv, type, Flow.this.names.close, List.nil(), List.nil());
                        Type typeMemberType = Flow.this.types.memberType(jCTree2.type, symbolResolveQualifiedMethod);
                        if (symbolResolveQualifiedMethod.kind == 16) {
                            Iterator<Type> it2 = typeMemberType.mo179getThrownTypes().iterator();
                            while (it2.hasNext()) {
                                markThrown(jCTree2, it2.next());
                            }
                        }
                    }
                }
            }
            scan(jCTry.body);
            if (Flow.this.allowImprovedCatchAnalysis) {
                listUnion = Flow.this.chk.union(this.thrown, List.of(Flow.this.syms.runtimeExceptionType, Flow.this.syms.errorType));
            } else {
                listUnion = this.thrown;
            }
            this.thrown = list3;
            this.caught = list2;
            List<Type> listNil = List.nil();
            for (List list5 = jCTry.catchers; list5.nonEmpty(); list5 = list5.tail) {
                JCTree.JCVariableDecl jCVariableDecl = ((JCTree.JCCatch) list5.head).param;
                List<JCTree.JCExpression> listOf2 = TreeInfo.isMultiCatch((JCTree.JCCatch) list5.head) ? ((JCTree.JCTypeUnion) ((JCTree.JCCatch) list5.head).param.vartype).alternatives : List.of(((JCTree.JCCatch) list5.head).param.vartype);
                List<Type> listNil2 = List.nil();
                List<Type> listDiff = Flow.this.chk.diff(listUnion, listNil);
                Iterator<JCTree.JCExpression> it3 = listOf2.iterator();
                while (it3.hasNext()) {
                    Type type2 = it3.next().type;
                    if (type2 == Flow.this.syms.unknownType) {
                        list = listOf2;
                    } else {
                        listNil2 = listNil2.append(type2);
                        list = listOf2;
                        if (!Flow.this.types.isSameType(type2, Flow.this.syms.objectType)) {
                            checkCaughtType(((JCTree.JCCatch) list5.head).pos(), type2, listUnion, listNil);
                            listNil = Flow.this.chk.incl(type2, listNil);
                        } else {
                            listOf2 = list;
                        }
                    }
                    listOf2 = list;
                }
                scan(jCVariableDecl);
                this.preciseRethrowTypes.put(jCVariableDecl.sym, Flow.this.chk.intersect(listNil2, listDiff));
                scan(((JCTree.JCCatch) list5.head).body);
                this.preciseRethrowTypes.remove(jCVariableDecl.sym);
            }
            if (jCTry.finalizer == null) {
                this.thrown = Flow.this.chk.union(this.thrown, Flow.this.chk.diff(listUnion, listNil));
                ListBuffer<P> listBuffer2 = this.pendingExits;
                this.pendingExits = listBuffer;
                while (listBuffer2.nonEmpty()) {
                    this.pendingExits.append((P) listBuffer2.next());
                }
                return;
            }
            List<Type> list6 = this.thrown;
            this.thrown = List.nil();
            ListBuffer<P> listBuffer3 = this.pendingExits;
            this.pendingExits = listBuffer;
            scan(jCTry.finalizer);
            if (!jCTry.finallyCanCompleteNormally) {
                this.thrown = Flow.this.chk.union(this.thrown, list3);
                return;
            }
            this.thrown = Flow.this.chk.union(this.thrown, Flow.this.chk.diff(listUnion, listNil));
            this.thrown = Flow.this.chk.union(this.thrown, list6);
            while (listBuffer3.nonEmpty()) {
                this.pendingExits.append((P) listBuffer3.next());
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIf(JCTree.JCIf tree) {
            scan(tree.cond);
            scan(tree.thenpart);
            if (tree.elsepart != null) {
                scan(tree.elsepart);
            }
        }

        void checkCaughtType(JCDiagnostic.DiagnosticPosition pos, Type exc, List<Type> thrownInTry, List<Type> caughtInTry) {
            if (Flow.this.chk.subset(exc, caughtInTry)) {
                Flow.this.log.error(pos, "except.already.caught", exc);
                return;
            }
            if (Flow.this.chk.isUnchecked(pos, exc) || isExceptionOrThrowable(exc) || Flow.this.chk.intersects(exc, thrownInTry)) {
                if (Flow.this.allowImprovedCatchAnalysis) {
                    List<Type> catchableThrownTypes = Flow.this.chk.intersect(List.of(exc), thrownInTry);
                    if (Flow.this.chk.diff(catchableThrownTypes, caughtInTry).isEmpty() && !isExceptionOrThrowable(exc)) {
                        String key = catchableThrownTypes.length() == 1 ? "unreachable.catch" : "unreachable.catch.1";
                        Flow.this.log.warning(pos, key, catchableThrownTypes);
                        return;
                    }
                    return;
                }
                return;
            }
            Flow.this.log.error(pos, "except.never.thrown.in.try", exc);
        }

        private boolean isExceptionOrThrowable(Type exc) {
            return exc.tsym == Flow.this.syms.throwableType.tsym || exc.tsym == Flow.this.syms.exceptionType.tsym;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBreak(JCTree.JCBreak tree) {
            recordExit(new FlowPendingExit(tree, null));
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitContinue(JCTree.JCContinue tree) {
            recordExit(new FlowPendingExit(tree, null));
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReturn(JCTree.JCReturn tree) {
            scan(tree.expr);
            recordExit(new FlowPendingExit(tree, null));
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitThrow(JCTree.JCThrow tree) {
            scan(tree.expr);
            Symbol sym = TreeInfo.symbol(tree.expr);
            if (sym != null && sym.kind == 4 && (sym.flags() & 2199023255568L) != 0 && this.preciseRethrowTypes.get(sym) != null && Flow.this.allowImprovedRethrowAnalysis) {
                for (Type t : this.preciseRethrowTypes.get(sym)) {
                    markThrown(tree, t);
                }
            } else {
                markThrown(tree, tree.expr.type);
            }
            markDead();
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation tree) {
            scan(tree.meth);
            scan(tree.args);
            for (List listMo179getThrownTypes = tree.meth.type.mo179getThrownTypes(); listMo179getThrownTypes.nonEmpty(); listMo179getThrownTypes = listMo179getThrownTypes.tail) {
                markThrown(tree, (Type) listMo179getThrownTypes.head);
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            scan(tree.encl);
            scan(tree.args);
            for (List listMo179getThrownTypes = tree.constructorType.mo179getThrownTypes(); listMo179getThrownTypes.nonEmpty(); listMo179getThrownTypes = listMo179getThrownTypes.tail) {
                markThrown(tree, (Type) listMo179getThrownTypes.head);
            }
            List<Type> l = this.caught;
            try {
                if (tree.def != null) {
                    for (List listMo179getThrownTypes2 = tree.constructor.type.mo179getThrownTypes(); listMo179getThrownTypes2.nonEmpty(); listMo179getThrownTypes2 = listMo179getThrownTypes2.tail) {
                        this.caught = Flow.this.chk.incl((Type) listMo179getThrownTypes2.head, this.caught);
                    }
                }
                scan(tree.def);
            } finally {
                this.caught = l;
            }
        }

        /* JADX WARN: Type inference incomplete: some casts might be missing */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda jCLambda) {
            if (jCLambda.type != null && jCLambda.type.isErroneous()) {
                return;
            }
            List<Type> list = this.caught;
            List<Type> list2 = this.thrown;
            ListBuffer<P> listBuffer = this.pendingExits;
            try {
                this.pendingExits = new ListBuffer<>();
                this.caught = jCLambda.getDescriptorType(Flow.this.types).mo179getThrownTypes();
                this.thrown = List.nil();
                scan(jCLambda.body);
                List list3 = this.pendingExits.toList();
                this.pendingExits = new ListBuffer<>();
                while (list3.nonEmpty()) {
                    FlowPendingExit flowPendingExit = (FlowPendingExit) list3.head;
                    list3 = list3.tail;
                    if (flowPendingExit.thrown == null) {
                        Assert.check(flowPendingExit.tree.hasTag(JCTree.Tag.RETURN));
                    } else {
                        this.pendingExits.append(flowPendingExit);
                    }
                }
                errorUncaught();
            } finally {
                this.pendingExits = listBuffer;
                this.caught = list;
                this.thrown = list2;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTopLevel(JCTree.JCCompilationUnit tree) {
        }

        public void analyzeTree(Env<AttrContext> env, TreeMaker make) {
            analyzeTree(env, env.tree, make);
        }

        /* JADX WARN: Multi-variable type inference failed */
        public void analyzeTree(Env<AttrContext> env, JCTree jCTree, TreeMaker treeMaker) {
            try {
                Flow.this.attrEnv = env;
                Flow.this.make = treeMaker;
                this.pendingExits = new ListBuffer<>();
                this.preciseRethrowTypes = new HashMap<>();
                this.caught = null;
                this.thrown = null;
                this.classDef = null;
                scan(jCTree);
            } finally {
                this.pendingExits = null;
                Flow.this.make = null;
                this.caught = null;
                this.thrown = null;
                this.classDef = null;
            }
        }
    }

    class LambdaFlowAnalyzer extends FlowAnalyzer {
        boolean inLambda;
        List<Type> inferredThrownTypes;

        LambdaFlowAnalyzer() {
            super();
        }

        @Override // com.sun.tools.javac.comp.Flow.FlowAnalyzer, com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            if ((tree.type != null && tree.type.isErroneous()) || this.inLambda) {
                return;
            }
            List<Type> prevCaught = this.caught;
            List<Type> prevThrown = this.thrown;
            ListBuffer<P> listBuffer = this.pendingExits;
            this.inLambda = true;
            try {
                this.pendingExits = new ListBuffer<>();
                this.caught = List.of(Flow.this.syms.throwableType);
                this.thrown = List.nil();
                scan(tree.body);
                this.inferredThrownTypes = this.thrown;
            } finally {
                this.pendingExits = listBuffer;
                this.caught = prevCaught;
                this.thrown = prevThrown;
                this.inLambda = false;
            }
        }

        @Override // com.sun.tools.javac.comp.Flow.FlowAnalyzer, com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
        }
    }

    public class AssignAnalyzer extends BaseAnalyzer<AssignPendingExit> {
        JCTree.JCClassDecl classDef;
        int firstadr;
        protected int nextadr;
        protected int returnadr;
        int startPos;
        Scope unrefdResources;
        protected JCTree.JCVariableDecl[] vardecls;
        FlowKind flowKind = FlowKind.NORMAL;
        private boolean isInitialConstructor = false;
        final Bits inits = new Bits();
        final Bits uninits = new Bits();
        final Bits uninitsTry = new Bits();
        final Bits initsWhenTrue = new Bits(true);
        final Bits initsWhenFalse = new Bits(true);
        final Bits uninitsWhenTrue = new Bits(true);
        final Bits uninitsWhenFalse = new Bits(true);

        @Override // com.sun.tools.javac.comp.Flow.BaseAnalyzer, com.sun.tools.javac.tree.TreeScanner
        public /* bridge */ /* synthetic */ void scan(JCTree jCTree) {
            super.scan(jCTree);
        }

        public class AssignPendingExit extends BaseAnalyzer.PendingExit {
            final Bits exit_inits;
            final Bits exit_uninits;
            final Bits inits;
            final Bits uninits;

            public AssignPendingExit(JCTree tree, Bits inits, Bits uninits) {
                super(tree);
                this.exit_inits = new Bits(true);
                this.exit_uninits = new Bits(true);
                this.inits = inits;
                this.uninits = uninits;
                this.exit_inits.assign(inits);
                this.exit_uninits.assign(uninits);
            }

            @Override // com.sun.tools.javac.comp.Flow.BaseAnalyzer.PendingExit
            void resolveJump() {
                this.inits.andSet(this.exit_inits);
                this.uninits.andSet(this.exit_uninits);
            }
        }

        public AssignAnalyzer() {
        }

        @Override // com.sun.tools.javac.comp.Flow.BaseAnalyzer
        void markDead() {
            if (!this.isInitialConstructor) {
                this.inits.inclRange(this.returnadr, this.nextadr);
            } else {
                for (int address = this.returnadr; address < this.nextadr; address++) {
                    if (!isFinalUninitializedStaticField(this.vardecls[address].sym)) {
                        this.inits.incl(address);
                    }
                }
            }
            this.uninits.inclRange(this.returnadr, this.nextadr);
        }

        protected boolean trackable(Symbol.VarSymbol sym) {
            return sym.pos >= this.startPos && (sym.owner.kind == 16 || isFinalUninitializedField(sym));
        }

        boolean isFinalUninitializedField(Symbol.VarSymbol sym) {
            return sym.owner.kind == 2 && (sym.flags() & 8590196752L) == 16 && this.classDef.sym.isEnclosedBy((Symbol.ClassSymbol) sym.owner);
        }

        boolean isFinalUninitializedStaticField(Symbol.VarSymbol sym) {
            return isFinalUninitializedField(sym) && sym.isStatic();
        }

        void newVar(JCTree.JCVariableDecl varDecl) {
            Symbol.VarSymbol sym = varDecl.sym;
            this.vardecls = (JCTree.JCVariableDecl[]) ArrayUtils.ensureCapacity(this.vardecls, this.nextadr);
            if ((sym.flags() & 16) == 0) {
                sym.flags_field |= Flags.EFFECTIVELY_FINAL;
            }
            sym.adr = this.nextadr;
            this.vardecls[this.nextadr] = varDecl;
            this.inits.excl(this.nextadr);
            this.uninits.incl(this.nextadr);
            this.nextadr++;
        }

        void letInit(JCDiagnostic.DiagnosticPosition pos, Symbol.VarSymbol sym) {
            if (sym.adr < this.firstadr || !trackable(sym)) {
                if ((sym.flags() & 16) != 0) {
                    Flow.this.log.error(pos, "var.might.already.be.assigned", sym);
                    return;
                }
                return;
            }
            if ((sym.flags() & Flags.EFFECTIVELY_FINAL) != 0) {
                if (!this.uninits.isMember(sym.adr)) {
                    sym.flags_field &= -2199023255553L;
                } else {
                    uninit(sym);
                }
            } else if ((sym.flags() & 16) != 0) {
                if ((sym.flags() & 8589934592L) != 0) {
                    if ((sym.flags() & Flags.UNION) != 0) {
                        Flow.this.log.error(pos, "multicatch.parameter.may.not.be.assigned", sym);
                    } else {
                        Flow.this.log.error(pos, "final.parameter.may.not.be.assigned", sym);
                    }
                } else if (!this.uninits.isMember(sym.adr)) {
                    Flow.this.log.error(pos, this.flowKind.errKey, sym);
                } else {
                    uninit(sym);
                }
            }
            this.inits.incl(sym.adr);
        }

        void uninit(Symbol.VarSymbol sym) {
            if (!this.inits.isMember(sym.adr)) {
                this.uninits.excl(sym.adr);
                this.uninitsTry.excl(sym.adr);
            } else {
                this.uninits.excl(sym.adr);
            }
        }

        void letInit(JCTree tree) {
            JCTree tree2 = TreeInfo.skipParens(tree);
            if (tree2.hasTag(JCTree.Tag.IDENT) || tree2.hasTag(JCTree.Tag.SELECT)) {
                Symbol sym = TreeInfo.symbol(tree2);
                if (sym.kind == 4) {
                    letInit(tree2.pos(), (Symbol.VarSymbol) sym);
                }
            }
        }

        void checkInit(JCDiagnostic.DiagnosticPosition pos, Symbol.VarSymbol sym) {
            checkInit(pos, sym, "var.might.not.have.been.initialized");
        }

        void checkInit(JCDiagnostic.DiagnosticPosition pos, Symbol.VarSymbol sym, String errkey) {
            if ((sym.adr >= this.firstadr || sym.owner.kind != 2) && trackable(sym) && !this.inits.isMember(sym.adr)) {
                Flow.this.log.error(pos, errkey, sym);
                this.inits.incl(sym.adr);
            }
        }

        private void resetBits(Bits... bits) {
            for (Bits b : bits) {
                b.reset();
            }
        }

        void split(boolean setToNull) {
            this.initsWhenFalse.assign(this.inits);
            this.uninitsWhenFalse.assign(this.uninits);
            this.initsWhenTrue.assign(this.inits);
            this.uninitsWhenTrue.assign(this.uninits);
            if (setToNull) {
                resetBits(this.inits, this.uninits);
            }
        }

        protected void merge() {
            this.inits.assign(this.initsWhenFalse.andSet(this.initsWhenTrue));
            this.uninits.assign(this.uninitsWhenFalse.andSet(this.uninitsWhenTrue));
        }

        void scanExpr(JCTree tree) {
            if (tree != null) {
                scan(tree);
                if (this.inits.isReset()) {
                    merge();
                }
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        void scanExprs(List<? extends JCTree.JCExpression> trees) {
            if (trees != null) {
                for (List list = trees; list.nonEmpty(); list = list.tail) {
                    scanExpr((JCTree) list.head);
                }
            }
        }

        void scanCond(JCTree tree) {
            if (tree.type.isFalse()) {
                if (this.inits.isReset()) {
                    merge();
                }
                this.initsWhenTrue.assign(this.inits);
                this.initsWhenTrue.inclRange(this.firstadr, this.nextadr);
                this.uninitsWhenTrue.assign(this.uninits);
                this.uninitsWhenTrue.inclRange(this.firstadr, this.nextadr);
                this.initsWhenFalse.assign(this.inits);
                this.uninitsWhenFalse.assign(this.uninits);
            } else if (tree.type.isTrue()) {
                if (this.inits.isReset()) {
                    merge();
                }
                this.initsWhenFalse.assign(this.inits);
                this.initsWhenFalse.inclRange(this.firstadr, this.nextadr);
                this.uninitsWhenFalse.assign(this.uninits);
                this.uninitsWhenFalse.inclRange(this.firstadr, this.nextadr);
                this.initsWhenTrue.assign(this.inits);
                this.uninitsWhenTrue.assign(this.uninits);
            } else {
                scan(tree);
                if (!this.inits.isReset()) {
                    split(tree.type != Flow.this.syms.unknownType);
                }
            }
            if (tree.type != Flow.this.syms.unknownType) {
                resetBits(this.inits, this.uninits);
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            if (tree.sym != null) {
                Lint lintPrev = Flow.this.lint;
                Flow.this.lint = Flow.this.lint.augment(tree.sym);
                try {
                    if (tree.sym == null) {
                        return;
                    }
                    JCTree.JCClassDecl classDefPrev = this.classDef;
                    int firstadrPrev = this.firstadr;
                    int nextadrPrev = this.nextadr;
                    ListBuffer<P> listBuffer = this.pendingExits;
                    this.pendingExits = new ListBuffer<>();
                    if (tree.name != Flow.this.names.empty) {
                        this.firstadr = this.nextadr;
                    }
                    this.classDef = tree;
                    try {
                        for (List list = tree.defs; list.nonEmpty(); list = list.tail) {
                            if (((JCTree) list.head).hasTag(JCTree.Tag.VARDEF)) {
                                JCTree.JCVariableDecl def = (JCTree.JCVariableDecl) list.head;
                                if ((8 & def.mods.flags) != 0) {
                                    Symbol.VarSymbol sym = def.sym;
                                    if (trackable(sym)) {
                                        newVar(def);
                                    }
                                }
                            }
                        }
                        for (List list2 = tree.defs; list2.nonEmpty(); list2 = list2.tail) {
                            if (!((JCTree) list2.head).hasTag(JCTree.Tag.METHODDEF) && (TreeInfo.flags((JCTree) list2.head) & 8) != 0) {
                                scan((JCTree) list2.head);
                            }
                        }
                        for (List list3 = tree.defs; list3.nonEmpty(); list3 = list3.tail) {
                            if (((JCTree) list3.head).hasTag(JCTree.Tag.VARDEF)) {
                                JCTree.JCVariableDecl def2 = (JCTree.JCVariableDecl) list3.head;
                                if ((def2.mods.flags & 8) == 0) {
                                    Symbol.VarSymbol sym2 = def2.sym;
                                    if (trackable(sym2)) {
                                        newVar(def2);
                                    }
                                }
                            }
                        }
                        for (List list4 = tree.defs; list4.nonEmpty(); list4 = list4.tail) {
                            if (!((JCTree) list4.head).hasTag(JCTree.Tag.METHODDEF) && (TreeInfo.flags((JCTree) list4.head) & 8) == 0) {
                                scan((JCTree) list4.head);
                            }
                        }
                        for (List list5 = tree.defs; list5.nonEmpty(); list5 = list5.tail) {
                            if (((JCTree) list5.head).hasTag(JCTree.Tag.METHODDEF)) {
                                scan((JCTree) list5.head);
                            }
                        }
                    } finally {
                        this.pendingExits = listBuffer;
                        this.nextadr = nextadrPrev;
                        this.firstadr = firstadrPrev;
                        this.classDef = classDefPrev;
                    }
                } finally {
                    Flow.this.lint = lintPrev;
                }
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl tree) {
            if (tree.body != null && (tree.sym.flags() & 4096) == 0) {
                Lint lintPrev = Flow.this.lint;
                Flow.this.lint = Flow.this.lint.augment(tree.sym);
                try {
                    if (tree.body == null) {
                        return;
                    }
                    if ((tree.sym.flags() & 562949953425408L) == 4096) {
                        return;
                    }
                    Bits initsPrev = new Bits(this.inits);
                    Bits uninitsPrev = new Bits(this.uninits);
                    int nextadrPrev = this.nextadr;
                    int firstadrPrev = this.firstadr;
                    int returnadrPrev = this.returnadr;
                    Assert.check(this.pendingExits.isEmpty());
                    boolean lastInitialConstructor = this.isInitialConstructor;
                    try {
                        this.isInitialConstructor = TreeInfo.isInitialConstructor(tree);
                        if (!this.isInitialConstructor) {
                            this.firstadr = this.nextadr;
                        }
                        List list = tree.params;
                        while (true) {
                            if (!list.nonEmpty()) {
                                break;
                            }
                            JCTree.JCVariableDecl def = (JCTree.JCVariableDecl) list.head;
                            scan(def);
                            if ((def.sym.flags() & 8589934592L) != 0) {
                                z = true;
                            }
                            Assert.check(z, "Method parameter without PARAMETER flag");
                            initParam(def);
                            list = list.tail;
                        }
                        scan(tree.body);
                        if (this.isInitialConstructor) {
                            boolean isSynthesized = (tree.sym.flags() & Flags.GENERATEDCONSTR) != 0;
                            for (int i = this.firstadr; i < this.nextadr; i++) {
                                JCTree.JCVariableDecl vardecl = this.vardecls[i];
                                Symbol.VarSymbol var = vardecl.sym;
                                if (var.owner == this.classDef.sym) {
                                    if (isSynthesized) {
                                        checkInit(TreeInfo.diagnosticPositionFor(var, vardecl), var, "var.not.initialized.in.default.constructor");
                                    } else {
                                        checkInit(TreeInfo.diagEndPos(tree.body), var);
                                    }
                                }
                            }
                        }
                        List list2 = this.pendingExits.toList();
                        this.pendingExits = new ListBuffer<>();
                        while (list2.nonEmpty()) {
                            AssignPendingExit exit = (AssignPendingExit) list2.head;
                            list2 = list2.tail;
                            Assert.check(exit.tree.hasTag(JCTree.Tag.RETURN), exit.tree);
                            if (this.isInitialConstructor) {
                                this.inits.assign(exit.exit_inits);
                                for (int i2 = this.firstadr; i2 < this.nextadr; i2++) {
                                    checkInit(exit.tree.pos(), this.vardecls[i2].sym);
                                }
                            }
                        }
                    } finally {
                        this.inits.assign(initsPrev);
                        this.uninits.assign(uninitsPrev);
                        this.nextadr = nextadrPrev;
                        this.firstadr = firstadrPrev;
                        this.returnadr = returnadrPrev;
                        this.isInitialConstructor = lastInitialConstructor;
                    }
                } finally {
                    Flow.this.lint = lintPrev;
                }
            }
        }

        protected void initParam(JCTree.JCVariableDecl def) {
            this.inits.incl(def.sym.adr);
            this.uninits.excl(def.sym.adr);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            Lint lintPrev = Flow.this.lint;
            Flow.this.lint = Flow.this.lint.augment(tree.sym);
            try {
                boolean track = trackable(tree.sym);
                if (track && tree.sym.owner.kind == 16) {
                    newVar(tree);
                }
                if (tree.init != null) {
                    scanExpr(tree.init);
                    if (track) {
                        letInit(tree.pos(), tree.sym);
                    }
                }
            } finally {
                Flow.this.lint = lintPrev;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBlock(JCTree.JCBlock tree) {
            int nextadrPrev = this.nextadr;
            scan(tree.stats);
            this.nextadr = nextadrPrev;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            FlowKind prevFlowKind = this.flowKind;
            this.flowKind = FlowKind.NORMAL;
            Bits initsSkip = new Bits(true);
            Bits uninitsSkip = new Bits(true);
            this.pendingExits = new ListBuffer<>();
            int prevErrors = Flow.this.log.nerrors;
            while (true) {
                Bits uninitsEntry = new Bits(this.uninits);
                uninitsEntry.excludeFrom(this.nextadr);
                scan(tree.body);
                resolveContinues(tree);
                scanCond(tree.cond);
                if (!this.flowKind.isFinal()) {
                    initsSkip.assign(this.initsWhenFalse);
                    uninitsSkip.assign(this.uninitsWhenFalse);
                }
                if (Flow.this.log.nerrors != prevErrors || this.flowKind.isFinal() || new Bits(uninitsEntry).diffSet(this.uninitsWhenTrue).nextBit(this.firstadr) == -1) {
                    break;
                }
                this.inits.assign(this.initsWhenTrue);
                this.uninits.assign(uninitsEntry.andSet(this.uninitsWhenTrue));
                this.flowKind = FlowKind.SPECULATIVE_LOOP;
            }
            this.flowKind = prevFlowKind;
            this.inits.assign(initsSkip);
            this.uninits.assign(uninitsSkip);
            resolveBreaks(tree, abstractCollection);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitWhileLoop(JCTree.JCWhileLoop tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            FlowKind prevFlowKind = this.flowKind;
            this.flowKind = FlowKind.NORMAL;
            Bits initsSkip = new Bits(true);
            Bits uninitsSkip = new Bits(true);
            this.pendingExits = new ListBuffer<>();
            int prevErrors = Flow.this.log.nerrors;
            Bits uninitsEntry = new Bits(this.uninits);
            uninitsEntry.excludeFrom(this.nextadr);
            while (true) {
                scanCond(tree.cond);
                if (!this.flowKind.isFinal()) {
                    initsSkip.assign(this.initsWhenFalse);
                    uninitsSkip.assign(this.uninitsWhenFalse);
                }
                this.inits.assign(this.initsWhenTrue);
                this.uninits.assign(this.uninitsWhenTrue);
                scan(tree.body);
                resolveContinues(tree);
                if (Flow.this.log.nerrors != prevErrors || this.flowKind.isFinal() || new Bits(uninitsEntry).diffSet(this.uninits).nextBit(this.firstadr) == -1) {
                    break;
                }
                this.uninits.assign(uninitsEntry.andSet(this.uninits));
                this.flowKind = FlowKind.SPECULATIVE_LOOP;
            }
            this.flowKind = prevFlowKind;
            this.inits.assign(initsSkip);
            this.uninits.assign(uninitsSkip);
            resolveBreaks(tree, abstractCollection);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitForLoop(JCTree.JCForLoop tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            FlowKind prevFlowKind = this.flowKind;
            this.flowKind = FlowKind.NORMAL;
            int nextadrPrev = this.nextadr;
            scan(tree.init);
            Bits initsSkip = new Bits(true);
            Bits uninitsSkip = new Bits(true);
            this.pendingExits = new ListBuffer<>();
            int prevErrors = Flow.this.log.nerrors;
            while (true) {
                Bits uninitsEntry = new Bits(this.uninits);
                uninitsEntry.excludeFrom(this.nextadr);
                if (tree.cond != null) {
                    scanCond(tree.cond);
                    if (!this.flowKind.isFinal()) {
                        initsSkip.assign(this.initsWhenFalse);
                        uninitsSkip.assign(this.uninitsWhenFalse);
                    }
                    this.inits.assign(this.initsWhenTrue);
                    this.uninits.assign(this.uninitsWhenTrue);
                } else if (!this.flowKind.isFinal()) {
                    initsSkip.assign(this.inits);
                    initsSkip.inclRange(this.firstadr, this.nextadr);
                    uninitsSkip.assign(this.uninits);
                    uninitsSkip.inclRange(this.firstadr, this.nextadr);
                }
                scan(tree.body);
                resolveContinues(tree);
                scan(tree.step);
                if (Flow.this.log.nerrors != prevErrors || this.flowKind.isFinal() || new Bits(uninitsEntry).diffSet(this.uninits).nextBit(this.firstadr) == -1) {
                    break;
                }
                this.uninits.assign(uninitsEntry.andSet(this.uninits));
                this.flowKind = FlowKind.SPECULATIVE_LOOP;
            }
            this.flowKind = prevFlowKind;
            this.inits.assign(initsSkip);
            this.uninits.assign(uninitsSkip);
            resolveBreaks(tree, abstractCollection);
            this.nextadr = nextadrPrev;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
            visitVarDef(tree.var);
            AbstractCollection abstractCollection = this.pendingExits;
            FlowKind prevFlowKind = this.flowKind;
            this.flowKind = FlowKind.NORMAL;
            int nextadrPrev = this.nextadr;
            scan(tree.expr);
            Bits initsStart = new Bits(this.inits);
            Bits uninitsStart = new Bits(this.uninits);
            letInit(tree.pos(), tree.var.sym);
            this.pendingExits = new ListBuffer<>();
            int prevErrors = Flow.this.log.nerrors;
            while (true) {
                Bits uninitsEntry = new Bits(this.uninits);
                uninitsEntry.excludeFrom(this.nextadr);
                scan(tree.body);
                resolveContinues(tree);
                if (Flow.this.log.nerrors != prevErrors || this.flowKind.isFinal() || new Bits(uninitsEntry).diffSet(this.uninits).nextBit(this.firstadr) == -1) {
                    break;
                }
                this.uninits.assign(uninitsEntry.andSet(this.uninits));
                this.flowKind = FlowKind.SPECULATIVE_LOOP;
            }
            this.flowKind = prevFlowKind;
            this.inits.assign(initsStart);
            this.uninits.assign(uninitsStart.andSet(this.uninits));
            resolveBreaks(tree, abstractCollection);
            this.nextadr = nextadrPrev;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLabelled(JCTree.JCLabeledStatement tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            scan(tree.body);
            resolveBreaks(tree, abstractCollection);
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSwitch(JCTree.JCSwitch tree) {
            AbstractCollection abstractCollection = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            int nextadrPrev = this.nextadr;
            scanExpr(tree.selector);
            Bits initsSwitch = new Bits(this.inits);
            Bits uninitsSwitch = new Bits(this.uninits);
            boolean hasDefault = false;
            for (List list = tree.cases; list.nonEmpty(); list = list.tail) {
                this.inits.assign(initsSwitch);
                this.uninits.assign(this.uninits.andSet(uninitsSwitch));
                JCTree.JCCase c = (JCTree.JCCase) list.head;
                if (c.pat == null) {
                    hasDefault = true;
                } else {
                    scanExpr(c.pat);
                }
                if (hasDefault) {
                    this.inits.assign(initsSwitch);
                    this.uninits.assign(this.uninits.andSet(uninitsSwitch));
                }
                scan(c.stats);
                addVars(c.stats, initsSwitch, uninitsSwitch);
                if (!hasDefault) {
                    this.inits.assign(initsSwitch);
                    this.uninits.assign(this.uninits.andSet(uninitsSwitch));
                }
            }
            if (!hasDefault) {
                this.inits.andSet(initsSwitch);
            }
            resolveBreaks(tree, abstractCollection);
            this.nextadr = nextadrPrev;
        }

        /* JADX WARN: Multi-variable type inference failed */
        private void addVars(List<JCTree.JCStatement> list, Bits bits, Bits bits2) {
            for (List list2 = list; list2.nonEmpty(); list2 = list2.tail) {
                JCTree jCTree = (JCTree) list2.head;
                if (jCTree.hasTag(JCTree.Tag.VARDEF)) {
                    int i = ((JCTree.JCVariableDecl) jCTree).sym.adr;
                    bits.excl(i);
                    bits2.incl(i);
                }
            }
        }

        /* JADX WARN: Type inference incomplete: some casts might be missing */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTry(JCTree.JCTry jCTry) {
            ListBuffer<JCTree.JCVariableDecl> listBuffer = new ListBuffer();
            Bits bits = new Bits(this.uninitsTry);
            ListBuffer<P> listBuffer2 = this.pendingExits;
            this.pendingExits = new ListBuffer<>();
            Bits bits2 = new Bits(this.inits);
            this.uninitsTry.assign(this.uninits);
            for (JCTree jCTree : jCTry.resources) {
                if (jCTree instanceof JCTree.JCVariableDecl) {
                    JCTree.JCVariableDecl jCVariableDecl = (JCTree.JCVariableDecl) jCTree;
                    visitVarDef(jCVariableDecl);
                    this.unrefdResources.enter(jCVariableDecl.sym);
                    listBuffer.append(jCVariableDecl);
                } else if (jCTree instanceof JCTree.JCExpression) {
                    scanExpr((JCTree.JCExpression) jCTree);
                } else {
                    throw new AssertionError(jCTry);
                }
            }
            scan(jCTry.body);
            this.uninitsTry.andSet(this.uninits);
            Bits bits3 = new Bits(this.inits);
            Bits bits4 = new Bits(this.uninits);
            int i = this.nextadr;
            if (!listBuffer.isEmpty() && Flow.this.lint.isEnabled(Lint.LintCategory.TRY)) {
                for (JCTree.JCVariableDecl jCVariableDecl2 : listBuffer) {
                    if (this.unrefdResources.includes(jCVariableDecl2.sym)) {
                        Flow.this.log.warning(Lint.LintCategory.TRY, jCVariableDecl2.pos(), "try.resource.not.referenced", jCVariableDecl2.sym);
                        this.unrefdResources.remove(jCVariableDecl2.sym);
                    }
                }
            }
            Bits bits5 = new Bits(bits2);
            Bits bits6 = new Bits(this.uninitsTry);
            for (List list = jCTry.catchers; list.nonEmpty(); list = list.tail) {
                JCTree.JCVariableDecl jCVariableDecl3 = ((JCTree.JCCatch) list.head).param;
                this.inits.assign(bits5);
                this.uninits.assign(bits6);
                scan(jCVariableDecl3);
                initParam(jCVariableDecl3);
                scan(((JCTree.JCCatch) list.head).body);
                bits3.andSet(this.inits);
                bits4.andSet(this.uninits);
                this.nextadr = i;
            }
            if (jCTry.finalizer != null) {
                this.inits.assign(bits2);
                this.uninits.assign(this.uninitsTry);
                ListBuffer<P> listBuffer3 = this.pendingExits;
                this.pendingExits = listBuffer2;
                scan(jCTry.finalizer);
                if (jCTry.finallyCanCompleteNormally) {
                    this.uninits.andSet(bits4);
                    while (listBuffer3.nonEmpty()) {
                        AssignPendingExit assignPendingExit = (AssignPendingExit) listBuffer3.next();
                        if (assignPendingExit.exit_inits != null) {
                            assignPendingExit.exit_inits.orSet(this.inits);
                            assignPendingExit.exit_uninits.andSet(this.uninits);
                        }
                        this.pendingExits.append(assignPendingExit);
                    }
                    this.inits.orSet(bits3);
                }
            } else {
                this.inits.assign(bits3);
                this.uninits.assign(bits4);
                ListBuffer<P> listBuffer4 = this.pendingExits;
                this.pendingExits = listBuffer2;
                while (listBuffer4.nonEmpty()) {
                    this.pendingExits.append((P) listBuffer4.next());
                }
            }
            this.uninitsTry.andSet(bits).andSet(this.uninits);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitConditional(JCTree.JCConditional tree) {
            scanCond(tree.cond);
            Bits initsBeforeElse = new Bits(this.initsWhenFalse);
            Bits uninitsBeforeElse = new Bits(this.uninitsWhenFalse);
            this.inits.assign(this.initsWhenTrue);
            this.uninits.assign(this.uninitsWhenTrue);
            if (tree.truepart.type.hasTag(TypeTag.BOOLEAN) && tree.falsepart.type.hasTag(TypeTag.BOOLEAN)) {
                scanCond(tree.truepart);
                Bits initsAfterThenWhenTrue = new Bits(this.initsWhenTrue);
                Bits initsAfterThenWhenFalse = new Bits(this.initsWhenFalse);
                Bits uninitsAfterThenWhenTrue = new Bits(this.uninitsWhenTrue);
                Bits uninitsAfterThenWhenFalse = new Bits(this.uninitsWhenFalse);
                this.inits.assign(initsBeforeElse);
                this.uninits.assign(uninitsBeforeElse);
                scanCond(tree.falsepart);
                this.initsWhenTrue.andSet(initsAfterThenWhenTrue);
                this.initsWhenFalse.andSet(initsAfterThenWhenFalse);
                this.uninitsWhenTrue.andSet(uninitsAfterThenWhenTrue);
                this.uninitsWhenFalse.andSet(uninitsAfterThenWhenFalse);
                return;
            }
            scanExpr(tree.truepart);
            Bits initsAfterThen = new Bits(this.inits);
            Bits uninitsAfterThen = new Bits(this.uninits);
            this.inits.assign(initsBeforeElse);
            this.uninits.assign(uninitsBeforeElse);
            scanExpr(tree.falsepart);
            this.inits.andSet(initsAfterThen);
            this.uninits.andSet(uninitsAfterThen);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIf(JCTree.JCIf tree) {
            scanCond(tree.cond);
            Bits initsBeforeElse = new Bits(this.initsWhenFalse);
            Bits uninitsBeforeElse = new Bits(this.uninitsWhenFalse);
            this.inits.assign(this.initsWhenTrue);
            this.uninits.assign(this.uninitsWhenTrue);
            scan(tree.thenpart);
            if (tree.elsepart != null) {
                Bits initsAfterThen = new Bits(this.inits);
                Bits uninitsAfterThen = new Bits(this.uninits);
                this.inits.assign(initsBeforeElse);
                this.uninits.assign(uninitsBeforeElse);
                scan(tree.elsepart);
                this.inits.andSet(initsAfterThen);
                this.uninits.andSet(uninitsAfterThen);
                return;
            }
            this.inits.andSet(initsBeforeElse);
            this.uninits.andSet(uninitsBeforeElse);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBreak(JCTree.JCBreak tree) {
            recordExit(new AssignPendingExit(tree, this.inits, this.uninits));
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitContinue(JCTree.JCContinue tree) {
            recordExit(new AssignPendingExit(tree, this.inits, this.uninits));
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReturn(JCTree.JCReturn tree) {
            scanExpr(tree.expr);
            recordExit(new AssignPendingExit(tree, this.inits, this.uninits));
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitThrow(JCTree.JCThrow tree) {
            scanExpr(tree.expr);
            markDead();
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation tree) {
            scanExpr(tree.meth);
            scanExprs(tree.args);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            scanExpr(tree.encl);
            scanExprs(tree.args);
            scan(tree.def);
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            Bits prevUninits = new Bits(this.uninits);
            Bits prevInits = new Bits(this.inits);
            int returnadrPrev = this.returnadr;
            ListBuffer<P> listBuffer = this.pendingExits;
            try {
                this.returnadr = this.nextadr;
                this.pendingExits = new ListBuffer<>();
                for (List list = tree.params; list.nonEmpty(); list = list.tail) {
                    JCTree.JCVariableDecl def = (JCTree.JCVariableDecl) list.head;
                    scan(def);
                    this.inits.incl(def.sym.adr);
                    this.uninits.excl(def.sym.adr);
                }
                if (tree.getBodyKind() == LambdaExpressionTree.BodyKind.EXPRESSION) {
                    scanExpr(tree.body);
                } else {
                    scan(tree.body);
                }
            } finally {
                this.returnadr = returnadrPrev;
                this.uninits.assign(prevUninits);
                this.inits.assign(prevInits);
                this.pendingExits = listBuffer;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewArray(JCTree.JCNewArray tree) {
            scanExprs(tree.dims);
            scanExprs(tree.elems);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssert(JCTree.JCAssert tree) {
            Bits initsExit = new Bits(this.inits);
            Bits uninitsExit = new Bits(this.uninits);
            scanCond(tree.cond);
            uninitsExit.andSet(this.uninitsWhenTrue);
            if (tree.detail != null) {
                this.inits.assign(this.initsWhenFalse);
                this.uninits.assign(this.uninitsWhenFalse);
                scanExpr(tree.detail);
            }
            this.inits.assign(initsExit);
            this.uninits.assign(uninitsExit);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssign(JCTree.JCAssign tree) {
            JCTree lhs = TreeInfo.skipParens(tree.lhs);
            if (!isIdentOrThisDotIdent(lhs)) {
                scanExpr(lhs);
            }
            scanExpr(tree.rhs);
            letInit(lhs);
        }

        private boolean isIdentOrThisDotIdent(JCTree lhs) {
            if (lhs.hasTag(JCTree.Tag.IDENT)) {
                return true;
            }
            if (!lhs.hasTag(JCTree.Tag.SELECT)) {
                return false;
            }
            JCTree.JCFieldAccess fa = (JCTree.JCFieldAccess) lhs;
            return fa.selected.hasTag(JCTree.Tag.IDENT) && ((JCTree.JCIdent) fa.selected).name == Flow.this.names._this;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            super.visitSelect(tree);
            if (Flow.this.enforceThisDotInit && tree.selected.hasTag(JCTree.Tag.IDENT) && ((JCTree.JCIdent) tree.selected).name == Flow.this.names._this && tree.sym.kind == 4) {
                checkInit(tree.pos(), (Symbol.VarSymbol) tree.sym);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssignop(JCTree.JCAssignOp tree) {
            scanExpr(tree.lhs);
            scanExpr(tree.rhs);
            letInit(tree.lhs);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitUnary(JCTree.JCUnary tree) {
            switch (tree.getTag()) {
                case NOT:
                    scanCond(tree.arg);
                    Bits t = new Bits(this.initsWhenFalse);
                    this.initsWhenFalse.assign(this.initsWhenTrue);
                    this.initsWhenTrue.assign(t);
                    t.assign(this.uninitsWhenFalse);
                    this.uninitsWhenFalse.assign(this.uninitsWhenTrue);
                    this.uninitsWhenTrue.assign(t);
                    break;
                case PREINC:
                case POSTINC:
                case PREDEC:
                case POSTDEC:
                    scanExpr(tree.arg);
                    letInit(tree.arg);
                    break;
                default:
                    scanExpr(tree.arg);
                    break;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBinary(JCTree.JCBinary tree) {
            switch (tree.getTag()) {
                case AND:
                    scanCond(tree.lhs);
                    Bits initsWhenFalseLeft = new Bits(this.initsWhenFalse);
                    Bits uninitsWhenFalseLeft = new Bits(this.uninitsWhenFalse);
                    this.inits.assign(this.initsWhenTrue);
                    this.uninits.assign(this.uninitsWhenTrue);
                    scanCond(tree.rhs);
                    this.initsWhenFalse.andSet(initsWhenFalseLeft);
                    this.uninitsWhenFalse.andSet(uninitsWhenFalseLeft);
                    break;
                case OR:
                    scanCond(tree.lhs);
                    Bits initsWhenTrueLeft = new Bits(this.initsWhenTrue);
                    Bits uninitsWhenTrueLeft = new Bits(this.uninitsWhenTrue);
                    this.inits.assign(this.initsWhenFalse);
                    this.uninits.assign(this.uninitsWhenFalse);
                    scanCond(tree.rhs);
                    this.initsWhenTrue.andSet(initsWhenTrueLeft);
                    this.uninitsWhenTrue.andSet(uninitsWhenTrueLeft);
                    break;
                default:
                    scanExpr(tree.lhs);
                    scanExpr(tree.rhs);
                    break;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            if (tree.sym.kind == 4) {
                checkInit(tree.pos(), (Symbol.VarSymbol) tree.sym);
                referenced(tree.sym);
            }
        }

        void referenced(Symbol sym) {
            this.unrefdResources.remove(sym);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
            tree.underlyingType.accept(this);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTopLevel(JCTree.JCCompilationUnit tree) {
        }

        public void analyzeTree(Env<?> env) {
            analyzeTree(env, env.tree);
        }

        public void analyzeTree(Env<?> env, JCTree tree) {
            try {
                this.startPos = tree.pos().getStartPosition();
                if (this.vardecls == null) {
                    this.vardecls = new JCTree.JCVariableDecl[32];
                } else {
                    for (int i = 0; i < this.vardecls.length; i++) {
                        this.vardecls[i] = null;
                    }
                }
                this.firstadr = 0;
                this.nextadr = 0;
                this.pendingExits = new ListBuffer<>();
                this.classDef = null;
                this.unrefdResources = new Scope(env.enclClass.sym);
                scan(tree);
                this.startPos = -1;
                resetBits(this.inits, this.uninits, this.uninitsTry, this.initsWhenTrue, this.initsWhenFalse, this.uninitsWhenTrue, this.uninitsWhenFalse);
                if (this.vardecls != null) {
                    for (int i2 = 0; i2 < this.vardecls.length; i2++) {
                        this.vardecls[i2] = null;
                    }
                }
                this.firstadr = 0;
                this.nextadr = 0;
                this.pendingExits = null;
                this.classDef = null;
                this.unrefdResources = null;
            } catch (Throwable th) {
                this.startPos = -1;
                resetBits(this.inits, this.uninits, this.uninitsTry, this.initsWhenTrue, this.initsWhenFalse, this.uninitsWhenTrue, this.uninitsWhenFalse);
                if (this.vardecls != null) {
                    for (int i3 = 0; i3 < this.vardecls.length; i3++) {
                        this.vardecls[i3] = null;
                    }
                }
                this.firstadr = 0;
                this.nextadr = 0;
                this.pendingExits = null;
                this.classDef = null;
                this.unrefdResources = null;
                throw th;
            }
        }
    }

    class CaptureAnalyzer extends BaseAnalyzer<BaseAnalyzer.PendingExit> {
        JCTree currentTree;

        CaptureAnalyzer() {
        }

        @Override // com.sun.tools.javac.comp.Flow.BaseAnalyzer
        void markDead() {
        }

        void checkEffectivelyFinal(JCDiagnostic.DiagnosticPosition pos, Symbol.VarSymbol sym) {
            if (this.currentTree != null && sym.owner.kind == 16 && sym.pos < this.currentTree.getStartPosition()) {
                switch (this.currentTree.getTag()) {
                    case CLASSDEF:
                        if (!Flow.this.allowEffectivelyFinalInInnerClasses) {
                            if ((sym.flags() & 16) == 0) {
                                reportInnerClsNeedsFinalError(pos, sym);
                                return;
                            }
                            return;
                        }
                        break;
                    case LAMBDA:
                        break;
                    default:
                        return;
                }
                if ((sym.flags() & 2199023255568L) == 0) {
                    reportEffectivelyFinalError(pos, sym);
                }
            }
        }

        void letInit(JCTree tree) {
            JCTree tree2 = TreeInfo.skipParens(tree);
            if (tree2.hasTag(JCTree.Tag.IDENT) || tree2.hasTag(JCTree.Tag.SELECT)) {
                Symbol sym = TreeInfo.symbol(tree2);
                if (this.currentTree != null && sym.kind == 4 && sym.owner.kind == 16 && ((Symbol.VarSymbol) sym).pos < this.currentTree.getStartPosition()) {
                    switch (this.currentTree.getTag()) {
                        case CLASSDEF:
                            if (!Flow.this.allowEffectivelyFinalInInnerClasses) {
                                reportInnerClsNeedsFinalError(tree2, sym);
                                return;
                            }
                            break;
                        case LAMBDA:
                            break;
                        default:
                            return;
                    }
                    reportEffectivelyFinalError(tree2, sym);
                }
            }
        }

        void reportEffectivelyFinalError(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
            String subKey = this.currentTree.hasTag(JCTree.Tag.LAMBDA) ? "lambda" : "inner.cls";
            Flow.this.log.error(pos, "cant.ref.non.effectively.final.var", sym, Flow.this.diags.fragment(subKey, new Object[0]));
        }

        void reportInnerClsNeedsFinalError(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
            Flow.this.log.error(pos, "local.var.accessed.from.icls.needs.final", sym);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            JCTree prevTree = this.currentTree;
            try {
                this.currentTree = tree.sym.isLocal() ? tree : null;
                super.visitClassDef(tree);
            } finally {
                this.currentTree = prevTree;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            JCTree prevTree = this.currentTree;
            try {
                this.currentTree = tree;
                super.visitLambda(tree);
            } finally {
                this.currentTree = prevTree;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            if (tree.sym.kind == 4) {
                checkEffectivelyFinal(tree, (Symbol.VarSymbol) tree.sym);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssign(JCTree.JCAssign tree) {
            JCTree lhs = TreeInfo.skipParens(tree.lhs);
            if (!(lhs instanceof JCTree.JCIdent)) {
                scan(lhs);
            }
            scan(tree.rhs);
            letInit(lhs);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssignop(JCTree.JCAssignOp tree) {
            scan(tree.lhs);
            scan(tree.rhs);
            letInit(tree.lhs);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitUnary(JCTree.JCUnary tree) {
            switch (tree.getTag()) {
                case PREINC:
                case POSTINC:
                case PREDEC:
                case POSTDEC:
                    scan(tree.arg);
                    letInit(tree.arg);
                    break;
                default:
                    scan(tree.arg);
                    break;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTopLevel(JCTree.JCCompilationUnit tree) {
        }

        public void analyzeTree(Env<AttrContext> env, TreeMaker make) {
            analyzeTree(env, env.tree, make);
        }

        /* JADX WARN: Multi-variable type inference failed */
        public void analyzeTree(Env<AttrContext> env, JCTree jCTree, TreeMaker treeMaker) {
            try {
                Flow.this.attrEnv = env;
                Flow.this.make = treeMaker;
                this.pendingExits = new ListBuffer<>();
                scan(jCTree);
            } finally {
                this.pendingExits = null;
                Flow.this.make = null;
            }
        }
    }
}
