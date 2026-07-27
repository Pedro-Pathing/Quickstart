package com.sun.tools.javac.comp;

import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.CompileStates;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeTranslator;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Names;
import java.util.HashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes.dex */
public class TransTypes extends TreeTranslator {
    private static final String statePreviousToFlowAssertMsg = "The current compile state [%s] of class %s is previous to FLOW";
    protected static final Context.Key<TransTypes> transTypesKey = new Context.Key<>();
    private final boolean addBridges;
    private boolean allowEnums;
    private boolean allowInterfaceBridges;
    private final CompileStates compileStates;
    private Enter enter;
    private Env<AttrContext> env;
    private Log log;
    private TreeMaker make;
    private Names names;
    Map<Symbol.MethodSymbol, Symbol.MethodSymbol> overridden;
    private Type pt;
    private final Resolve resolve;
    private Symtab syms;
    private Types types;
    private Filter<Symbol> overrideBridgeFilter = new Filter<Symbol>() { // from class: com.sun.tools.javac.comp.TransTypes.1
        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol s) {
            return (s.flags() & 1099511631872L) != 4096;
        }
    };
    JCTree currentMethod = null;

    public static TransTypes instance(Context context) {
        TransTypes instance = (TransTypes) context.get(transTypesKey);
        if (instance == null) {
            return new TransTypes(context);
        }
        return instance;
    }

    protected TransTypes(Context context) {
        context.put(transTypesKey, this);
        this.compileStates = CompileStates.instance(context);
        this.names = Names.instance(context);
        this.log = Log.instance(context);
        this.syms = Symtab.instance(context);
        this.enter = Enter.instance(context);
        this.overridden = new HashMap();
        Source source = Source.instance(context);
        this.allowEnums = source.allowEnums();
        this.addBridges = source.addBridges();
        this.allowInterfaceBridges = source.allowDefaultMethods();
        this.types = Types.instance(context);
        this.make = TreeMaker.instance(context);
        this.resolve = Resolve.instance(context);
    }

    JCTree.JCExpression cast(JCTree.JCExpression tree, Type target) {
        int oldpos = this.make.pos;
        this.make.at(tree.pos);
        if (!this.types.isSameType(tree.type, target)) {
            if (!this.resolve.isAccessible(this.env, target.tsym)) {
                this.resolve.logAccessErrorInternal(this.env, tree, target);
            }
            tree = this.make.TypeCast(this.make.Type(target), tree).setType(target);
        }
        this.make.pos = oldpos;
        return tree;
    }

    public JCTree.JCExpression coerce(Env<AttrContext> env, JCTree.JCExpression tree, Type target) {
        Env<AttrContext> prevEnv = this.env;
        try {
            this.env = env;
            return coerce(tree, target);
        } finally {
            this.env = prevEnv;
        }
    }

    JCTree.JCExpression coerce(JCTree.JCExpression tree, Type target) {
        Type btarget = target.baseType();
        if (tree.type.isPrimitive() == target.isPrimitive()) {
            return this.types.isAssignable(tree.type, btarget, this.types.noWarnings) ? tree : cast(tree, btarget);
        }
        return tree;
    }

    JCTree.JCExpression retype(JCTree.JCExpression tree, Type erasedType, Type target) {
        if (!erasedType.isPrimitive()) {
            if (target != null && target.isPrimitive()) {
                target = erasure(tree.type);
            }
            tree.type = erasedType;
            if (target != null) {
                return coerce(tree, target);
            }
        }
        return tree;
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v6, types: [A, com.sun.tools.javac.tree.JCTree] */
    /* JADX WARN: Type inference failed for: r2v4, types: [A, com.sun.tools.javac.tree.JCTree] */
    /* JADX WARN: Type inference failed for: r2v8, types: [A, com.sun.tools.javac.tree.JCTree] */
    <T extends JCTree> List<T> translateArgs(List<T> list, List<Type> list2, Type type) {
        if (list2.isEmpty()) {
            return list;
        }
        List list3 = list;
        List list4 = list2;
        while (list4.tail.nonEmpty()) {
            list3.head = translate((JCTree) list3.head, (Type) list4.head);
            list3 = list3.tail;
            list4 = list4.tail;
        }
        Type type2 = (Type) list4.head;
        boolean z = true;
        if (type == null && list3.length() != 1) {
            z = false;
        }
        Assert.check(z);
        if (type != null) {
            while (list3.nonEmpty()) {
                list3.head = translate((JCTree) list3.head, type);
                list3 = list3.tail;
            }
        } else {
            list3.head = translate((JCTree) list3.head, type2);
        }
        return list;
    }

    public <T extends JCTree> List<T> translateArgs(List<T> _args, List<Type> parameters, Type varargsElement, Env<AttrContext> localEnv) {
        Env<AttrContext> prevEnv = this.env;
        try {
            this.env = localEnv;
            return translateArgs(_args, parameters, varargsElement);
        } finally {
            this.env = prevEnv;
        }
    }

    void addBridge(JCDiagnostic.DiagnosticPosition pos, Symbol.MethodSymbol meth, Symbol.MethodSymbol impl, Symbol.ClassSymbol origin, boolean hypothetical, ListBuffer<JCTree> bridges) {
        JCTree.JCExpression receiver;
        JCTree.JCStatement stat;
        this.make.at(pos);
        Type origType = this.types.memberType(origin.type, meth);
        Type origErasure = erasure(origType);
        Type bridgeType = meth.erasure(this.types);
        long flags = (impl.flags() & 7) | 4096 | Flags.BRIDGE | (origin.isInterface() ? Flags.DEFAULT : 0L);
        if (hypothetical) {
            flags |= Flags.HYPOTHETICAL;
        }
        Symbol.MethodSymbol bridge = new Symbol.MethodSymbol(flags, meth.name, bridgeType, origin);
        bridge.params = createBridgeParams(impl, bridge, bridgeType);
        bridge.setAttributes(impl);
        if (!hypothetical) {
            JCTree.JCMethodDecl md = this.make.MethodDef(bridge, null);
            if (impl.owner != origin) {
                receiver = this.make.Super(this.types.supertype(origin.type).tsym.erasure(this.types), origin);
            } else {
                receiver = this.make.This(origin.erasure(this.types));
            }
            Type calltype = erasure(impl.type.mo178getReturnType());
            JCTree.JCExpression call = this.make.Apply(null, this.make.Select(receiver, impl).setType(calltype), translateArgs(this.make.Idents(md.params), origErasure.mo176getParameterTypes(), null)).setType(calltype);
            if (!origErasure.mo178getReturnType().hasTag(TypeTag.VOID)) {
                stat = this.make.Return(coerce(call, bridgeType.mo178getReturnType()));
            } else {
                stat = this.make.Exec(call);
            }
            md.body = this.make.Block(0L, List.of(stat));
            bridges.append(md);
        }
        origin.members().enter(bridge);
        this.overridden.put(bridge, meth);
    }

    /* JADX WARN: Multi-variable type inference failed */
    private List<Symbol.VarSymbol> createBridgeParams(Symbol.MethodSymbol impl, Symbol.MethodSymbol bridge, Type bridgeType) {
        List<Symbol.VarSymbol> bridgeParams = null;
        if (impl.params != null) {
            bridgeParams = List.nil();
            List list = impl.params;
            Type.MethodType mType = (Type.MethodType) bridgeType;
            for (List list2 = mType.argtypes; list.nonEmpty() && list2.nonEmpty(); list2 = list2.tail) {
                Symbol.VarSymbol param = new Symbol.VarSymbol(((Symbol.VarSymbol) list.head).flags() | 4096 | 8589934592L, ((Symbol.VarSymbol) list.head).name, (Type) list2.head, bridge);
                param.setAttributes((Symbol) list.head);
                bridgeParams = bridgeParams.append(param);
                list = list.tail;
            }
        }
        return bridgeParams;
    }

    void addBridgeIfNeeded(JCDiagnostic.DiagnosticPosition pos, Symbol sym, Symbol.ClassSymbol origin, ListBuffer<JCTree> bridges) {
        if (sym.kind == 16 && sym.name != this.names.init && (sym.flags() & 10) == 0 && (sym.flags() & 1099511631872L) != 4096 && sym.isMemberOf(origin, this.types)) {
            Symbol.MethodSymbol meth = (Symbol.MethodSymbol) sym;
            Symbol.MethodSymbol bridge = meth.binaryImplementation(origin, this.types);
            Symbol.MethodSymbol impl = meth.implementation(origin, this.types, true, this.overrideBridgeFilter);
            if (bridge == null || bridge == meth || !(impl == null || bridge.owner.isSubClass(impl.owner, this.types))) {
                if (impl != null && isBridgeNeeded(meth, impl, origin.type)) {
                    addBridge(pos, meth, impl, origin, bridge == impl, bridges);
                    return;
                }
                if (impl == meth && impl.owner != origin && (impl.flags() & 16) == 0 && (meth.flags() & Flags.AnnotationTypeElementMask) == 1 && (origin.flags() & 1) > (1 & impl.owner.flags())) {
                    addBridge(pos, meth, impl, origin, false, bridges);
                    return;
                }
                return;
            }
            if ((bridge.flags() & 1099511631872L) == 4096) {
                Symbol.MethodSymbol other = this.overridden.get(bridge);
                if (other != null && other != meth) {
                    if (impl == null || !impl.overrides(other, origin, this.types, true)) {
                        this.log.error(pos, "name.clash.same.erasure.no.override", other, other.location(origin.type, this.types), meth, meth.location(origin.type, this.types));
                        return;
                    }
                    return;
                }
                return;
            }
            if (!bridge.overrides(meth, origin, this.types, true)) {
                if (bridge.owner == origin || this.types.asSuper(bridge.owner.type, meth.owner) == null) {
                    this.log.error(pos, "name.clash.same.erasure.no.override", bridge, bridge.location(origin.type, this.types), meth, meth.location(origin.type, this.types));
                }
            }
        }
    }

    private boolean isBridgeNeeded(Symbol.MethodSymbol method, Symbol.MethodSymbol impl, Type dest) {
        if (impl == method) {
            if ((method.flags() & 1024) == 0) {
                return true ^ isSameMemberWhenErased(dest, method, method.erasure(this.types));
            }
            return false;
        }
        Type method_erasure = method.erasure(this.types);
        if (!isSameMemberWhenErased(dest, method, method_erasure)) {
            return true;
        }
        Type impl_erasure = impl.erasure(this.types);
        if (!isSameMemberWhenErased(dest, impl, impl_erasure)) {
            return true;
        }
        return true ^ this.types.isSameType(impl_erasure.mo178getReturnType(), method_erasure.mo178getReturnType());
    }

    private boolean isSameMemberWhenErased(Type type, Symbol.MethodSymbol method, Type erasure) {
        return this.types.isSameType(erasure(this.types.memberType(type, method)), erasure);
    }

    /* JADX WARN: Multi-variable type inference failed */
    void addBridges(JCDiagnostic.DiagnosticPosition pos, Symbol.TypeSymbol i, Symbol.ClassSymbol origin, ListBuffer<JCTree> bridges) {
        for (Scope.Entry e = i.members().elems; e != null; e = e.sibling) {
            addBridgeIfNeeded(pos, e.sym, origin, bridges);
        }
        for (List listInterfaces = this.types.interfaces(i.type); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
            addBridges(pos, ((Type) listInterfaces.head).tsym, origin, bridges);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void addBridges(JCDiagnostic.DiagnosticPosition pos, Symbol.ClassSymbol origin, ListBuffer<JCTree> bridges) {
        Type st = this.types.supertype(origin.type);
        while (st.hasTag(TypeTag.CLASS)) {
            addBridges(pos, st.tsym, origin, bridges);
            st = this.types.supertype(st);
        }
        for (List listInterfaces = this.types.interfaces(origin.type); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
            addBridges(pos, ((Type) listInterfaces.head).tsym, origin, bridges);
        }
    }

    public <T extends JCTree> T translate(T t, Type type) {
        Type type2 = this.pt;
        try {
            this.pt = type;
            return (T) translate(t);
        } finally {
            this.pt = type2;
        }
    }

    public <T extends JCTree> List<T> translate(List<T> trees, Type pt) {
        Type prevPt = this.pt;
        try {
            this.pt = pt;
            List<T> res = translate(trees);
            return res;
        } finally {
            this.pt = prevPt;
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitClassDef(JCTree.JCClassDecl tree) {
        translateClass(tree.sym);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitMethodDef(JCTree.JCMethodDecl tree) {
        JCTree previousMethod = this.currentMethod;
        try {
            this.currentMethod = tree;
            tree.restype = (JCTree.JCExpression) translate(tree.restype, (Type) null);
            tree.typarams = List.nil();
            tree.params = translateVarDefs(tree.params);
            tree.recvparam = (JCTree.JCVariableDecl) translate(tree.recvparam, (Type) null);
            tree.thrown = translate(tree.thrown, (Type) null);
            tree.body = (JCTree.JCBlock) translate(tree.body, tree.sym.erasure(this.types).mo178getReturnType());
            tree.type = erasure(tree.type);
            this.result = tree;
            this.currentMethod = previousMethod;
            for (Scope.Entry e = tree.sym.owner.members().lookup(tree.name); e.sym != null; e = e.next()) {
                if (e.sym != tree.sym && this.types.isSameType(erasure(e.sym.type), tree.type)) {
                    this.log.error(tree.pos(), "name.clash.same.erasure", tree.sym, e.sym);
                    return;
                }
            }
        } catch (Throwable th) {
            this.currentMethod = previousMethod;
            throw th;
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitVarDef(JCTree.JCVariableDecl tree) {
        tree.vartype = (JCTree.JCExpression) translate(tree.vartype, (Type) null);
        tree.init = (JCTree.JCExpression) translate(tree.init, tree.sym.erasure(this.types));
        tree.type = erasure(tree.type);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
        tree.body = (JCTree.JCStatement) translate(tree.body);
        tree.cond = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWhileLoop(JCTree.JCWhileLoop tree) {
        tree.cond = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        tree.body = (JCTree.JCStatement) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForLoop(JCTree.JCForLoop tree) {
        tree.init = translate(tree.init, (Type) null);
        if (tree.cond != null) {
            tree.cond = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        }
        tree.step = translate(tree.step, (Type) null);
        tree.body = (JCTree.JCStatement) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
        tree.var = (JCTree.JCVariableDecl) translate(tree.var, (Type) null);
        Type iterableType = tree.expr.type;
        tree.expr = (JCTree.JCExpression) translate(tree.expr, erasure(tree.expr.type));
        if (this.types.elemtype(tree.expr.type) == null) {
            tree.expr.type = iterableType;
        }
        tree.body = (JCTree.JCStatement) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLambda(JCTree.JCLambda tree) {
        JCTree prevMethod = this.currentMethod;
        try {
            this.currentMethod = null;
            tree.params = translate(tree.params);
            tree.body = translate(tree.body, tree.body.type != null ? erasure(tree.body.type) : null);
            tree.type = erasure(tree.type);
            this.result = tree;
        } finally {
            this.currentMethod = prevMethod;
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSwitch(JCTree.JCSwitch tree) {
        Type selsuper = this.types.supertype(tree.selector.type);
        boolean enumSwitch = selsuper != null && selsuper.tsym == this.syms.enumSym;
        Type target = enumSwitch ? erasure(tree.selector.type) : this.syms.intType;
        tree.selector = (JCTree.JCExpression) translate(tree.selector, target);
        tree.cases = translateCases(tree.cases);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitCase(JCTree.JCCase tree) {
        tree.pat = (JCTree.JCExpression) translate(tree.pat, (Type) null);
        tree.stats = translate(tree.stats);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSynchronized(JCTree.JCSynchronized tree) {
        tree.lock = (JCTree.JCExpression) translate(tree.lock, erasure(tree.lock.type));
        tree.body = (JCTree.JCBlock) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTry(JCTree.JCTry tree) {
        tree.resources = translate(tree.resources, this.syms.autoCloseableType);
        tree.body = (JCTree.JCBlock) translate(tree.body);
        tree.catchers = translateCatchers(tree.catchers);
        tree.finalizer = (JCTree.JCBlock) translate(tree.finalizer);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitConditional(JCTree.JCConditional tree) {
        tree.cond = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        tree.truepart = (JCTree.JCExpression) translate(tree.truepart, erasure(tree.type));
        tree.falsepart = (JCTree.JCExpression) translate(tree.falsepart, erasure(tree.type));
        tree.type = erasure(tree.type);
        this.result = retype(tree, tree.type, this.pt);
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIf(JCTree.JCIf tree) {
        tree.cond = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        tree.thenpart = (JCTree.JCStatement) translate(tree.thenpart);
        tree.elsepart = (JCTree.JCStatement) translate(tree.elsepart);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitExec(JCTree.JCExpressionStatement tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr, (Type) null);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReturn(JCTree.JCReturn tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr, this.currentMethod != null ? this.types.erasure(this.currentMethod.type).mo178getReturnType() : null);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitThrow(JCTree.JCThrow tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr, erasure(tree.expr.type));
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssert(JCTree.JCAssert tree) {
        tree.cond = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        if (tree.detail != null) {
            tree.detail = (JCTree.JCExpression) translate(tree.detail, erasure(tree.detail.type));
        }
        this.result = tree;
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitApply(JCTree.JCMethodInvocation tree) {
        tree.meth = (JCTree.JCExpression) translate(tree.meth, (Type) null);
        Symbol meth = TreeInfo.symbol(tree.meth);
        Type mt = meth.erasure(this.types);
        List listMo176getParameterTypes = mt.mo176getParameterTypes();
        if (this.allowEnums && meth.name == this.names.init && meth.owner == this.syms.enumSym) {
            listMo176getParameterTypes = listMo176getParameterTypes.tail.tail;
        }
        if (tree.varargsElement != null) {
            tree.varargsElement = this.types.erasure(tree.varargsElement);
        } else if (tree.args.length() != listMo176getParameterTypes.length()) {
            this.log.error(tree.pos(), "method.invoked.with.incorrect.number.arguments", Integer.valueOf(tree.args.length()), Integer.valueOf(listMo176getParameterTypes.length()));
        }
        tree.args = translateArgs(tree.args, listMo176getParameterTypes, tree.varargsElement);
        tree.type = this.types.erasure(tree.type);
        this.result = retype(tree, mt.mo178getReturnType(), this.pt);
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewClass(JCTree.JCNewClass tree) {
        if (tree.encl != null) {
            tree.encl = (JCTree.JCExpression) translate(tree.encl, erasure(tree.encl.type));
        }
        tree.clazz = (JCTree.JCExpression) translate(tree.clazz, (Type) null);
        if (tree.varargsElement != null) {
            tree.varargsElement = this.types.erasure(tree.varargsElement);
        }
        tree.args = translateArgs(tree.args, tree.constructor.erasure(this.types).mo176getParameterTypes(), tree.varargsElement);
        tree.def = (JCTree.JCClassDecl) translate(tree.def, (Type) null);
        if (tree.constructorType != null) {
            tree.constructorType = erasure(tree.constructorType);
        }
        tree.type = erasure(tree.type);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewArray(JCTree.JCNewArray tree) {
        tree.elemtype = (JCTree.JCExpression) translate(tree.elemtype, (Type) null);
        translate(tree.dims, this.syms.intType);
        if (tree.type != null) {
            tree.elems = translate(tree.elems, erasure(this.types.elemtype(tree.type)));
            tree.type = erasure(tree.type);
        } else {
            tree.elems = translate(tree.elems, (Type) null);
        }
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitParens(JCTree.JCParens tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr, this.pt);
        tree.type = erasure(tree.type);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssign(JCTree.JCAssign tree) {
        tree.lhs = (JCTree.JCExpression) translate(tree.lhs, (Type) null);
        tree.rhs = (JCTree.JCExpression) translate(tree.rhs, erasure(tree.lhs.type));
        tree.type = erasure(tree.lhs.type);
        this.result = retype(tree, tree.type, this.pt);
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssignop(JCTree.JCAssignOp tree) {
        tree.lhs = (JCTree.JCExpression) translate(tree.lhs, (Type) null);
        tree.rhs = (JCTree.JCExpression) translate(tree.rhs, tree.operator.type.mo176getParameterTypes().tail.head);
        tree.type = erasure(tree.type);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitUnary(JCTree.JCUnary tree) {
        tree.arg = (JCTree.JCExpression) translate(tree.arg, tree.operator.type.mo176getParameterTypes().head);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBinary(JCTree.JCBinary tree) {
        tree.lhs = (JCTree.JCExpression) translate(tree.lhs, tree.operator.type.mo176getParameterTypes().head);
        tree.rhs = (JCTree.JCExpression) translate(tree.rhs, tree.operator.type.mo176getParameterTypes().tail.head);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeCast(JCTree.JCTypeCast tree) {
        tree.clazz = translate(tree.clazz, (Type) null);
        Type originalTarget = tree.type;
        tree.type = erasure(tree.type);
        tree.expr = (JCTree.JCExpression) translate(tree.expr, tree.type);
        if (originalTarget.isIntersection()) {
            Type.IntersectionClassType ict = (Type.IntersectionClassType) originalTarget;
            for (Type c : ict.getExplicitComponents()) {
                Type ec = erasure(c);
                if (!this.types.isSameType(ec, tree.type)) {
                    tree.expr = coerce(tree.expr, ec);
                }
            }
        }
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeTest(JCTree.JCInstanceOf tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr, (Type) null);
        tree.clazz = translate(tree.clazz, (Type) null);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIndexed(JCTree.JCArrayAccess tree) {
        tree.indexed = (JCTree.JCExpression) translate(tree.indexed, erasure(tree.indexed.type));
        tree.index = (JCTree.JCExpression) translate(tree.index, this.syms.intType);
        this.result = retype(tree, this.types.elemtype(tree.indexed.type), this.pt);
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotation(JCTree.JCAnnotation tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIdent(JCTree.JCIdent tree) {
        Type et = tree.sym.erasure(this.types);
        if (tree.sym.kind == 2 && tree.sym.type.hasTag(TypeTag.TYPEVAR)) {
            this.result = this.make.at(tree.pos).Type(et);
            return;
        }
        if (tree.type.constValue() != null) {
            this.result = tree;
        } else if (tree.sym.kind == 4) {
            this.result = retype(tree, et, this.pt);
        } else {
            tree.type = erasure(tree.type);
            this.result = tree;
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSelect(JCTree.JCFieldAccess tree) {
        Type t = tree.selected.type;
        while (t.hasTag(TypeTag.TYPEVAR)) {
            t = t.getUpperBound();
        }
        if (t.isCompound()) {
            if ((tree.sym.flags() & 2097152) != 0) {
                tree.sym = ((Symbol.MethodSymbol) tree.sym).implemented((Symbol.TypeSymbol) tree.sym.owner, this.types);
            }
            tree.selected = coerce((JCTree.JCExpression) translate(tree.selected, erasure(tree.selected.type)), erasure(tree.sym.owner.type));
        } else {
            tree.selected = (JCTree.JCExpression) translate(tree.selected, erasure(t));
        }
        if (tree.type.constValue() != null) {
            this.result = tree;
        } else if (tree.sym.kind == 4) {
            this.result = retype(tree, tree.sym.erasure(this.types), this.pt);
        } else {
            tree.type = erasure(tree.type);
            this.result = tree;
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReference(JCTree.JCMemberReference tree) {
        tree.expr = (JCTree.JCExpression) translate(tree.expr, erasure(tree.expr.type));
        tree.type = erasure(tree.type);
        if (tree.varargsElement != null) {
            tree.varargsElement = erasure(tree.varargsElement);
        }
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeArray(JCTree.JCArrayTypeTree tree) {
        tree.elemtype = (JCTree.JCExpression) translate(tree.elemtype, (Type) null);
        tree.type = erasure(tree.type);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeApply(JCTree.JCTypeApply tree) {
        JCTree clazz = translate(tree.clazz, (Type) null);
        this.result = clazz;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeIntersection(JCTree.JCTypeIntersection tree) {
        tree.bounds = translate(tree.bounds, (Type) null);
        tree.type = erasure(tree.type);
        this.result = tree;
    }

    private Type erasure(Type t) {
        return this.types.erasure(t);
    }

    /* JADX WARN: Multi-variable type inference failed */
    private boolean boundsRestricted(Symbol.ClassSymbol c) {
        Type st = this.types.supertype(c.type);
        if (st.isParameterized()) {
            List listAllparams = st.allparams();
            for (List listAllparams2 = st.tsym.type.allparams(); !listAllparams.isEmpty() && !listAllparams2.isEmpty(); listAllparams2 = listAllparams2.tail) {
                Type actual = (Type) listAllparams.head;
                Type formal = (Type) listAllparams2.head;
                if (!this.types.isSameType(this.types.erasure(actual), this.types.erasure(formal))) {
                    return true;
                }
                listAllparams = listAllparams.tail;
            }
            return false;
        }
        return false;
    }

    private List<JCTree> addOverrideBridgesIfNeeded(JCDiagnostic.DiagnosticPosition pos, Symbol.ClassSymbol c) {
        ListBuffer<JCTree> buf = new ListBuffer<>();
        if (c.isInterface() || !boundsRestricted(c)) {
            return buf.toList();
        }
        Type t = this.types.supertype(c.type);
        Scope s = t.tsym.members();
        if (s.elems != null) {
            for (Symbol sym : s.getElements(new NeedsOverridBridgeFilter(c))) {
                Symbol.MethodSymbol m = (Symbol.MethodSymbol) sym;
                Symbol.MethodSymbol member = (Symbol.MethodSymbol) m.asMemberOf(c.type, this.types);
                Symbol.MethodSymbol impl = m.implementation(c, this.types, false);
                if (impl == null || impl.owner != c) {
                    if (!this.types.isSameType(member.erasure(this.types), m.erasure(this.types))) {
                        addOverrideBridges(pos, m, member, c, buf);
                    }
                }
            }
        }
        return buf.toList();
    }

    class NeedsOverridBridgeFilter implements Filter<Symbol> {
        Symbol.ClassSymbol c;

        NeedsOverridBridgeFilter(Symbol.ClassSymbol c) {
            this.c = c;
        }

        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol s) {
            return s.kind == 16 && !s.isConstructor() && s.isInheritedIn(this.c, TransTypes.this.types) && (s.flags() & 16) == 0 && (s.flags() & 1099511631872L) != 4096;
        }
    }

    private void addOverrideBridges(JCDiagnostic.DiagnosticPosition pos, Symbol.MethodSymbol impl, Symbol.MethodSymbol member, Symbol.ClassSymbol c, ListBuffer<JCTree> bridges) {
        JCTree.JCStatement stat;
        Type implErasure = impl.erasure(this.types);
        long flags = (impl.flags() & 7) | 4096 | Flags.BRIDGE | Flags.OVERRIDE_BRIDGE;
        Symbol.MethodSymbol member2 = new Symbol.MethodSymbol(flags, member.name, member.type, c);
        JCTree.JCMethodDecl md = this.make.MethodDef(member2, null);
        JCTree.JCExpression receiver = this.make.Super(this.types.supertype(c.type).tsym.erasure(this.types), c);
        Type calltype = erasure(impl.type.mo178getReturnType());
        JCTree.JCExpression call = this.make.Apply(null, this.make.Select(receiver, impl).setType(calltype), translateArgs(this.make.Idents(md.params), implErasure.mo176getParameterTypes(), null)).setType(calltype);
        if (!member2.getReturnType().hasTag(TypeTag.VOID)) {
            stat = this.make.Return(coerce(call, member2.erasure(this.types).mo178getReturnType()));
        } else {
            stat = this.make.Exec(call);
        }
        md.body = this.make.Block(0L, List.of(stat));
        c.members().enter(member2);
        bridges.append(md);
    }

    void translateClass(Symbol.ClassSymbol c) {
        Type st = this.types.supertype(c.type);
        if (st.hasTag(TypeTag.CLASS)) {
            translateClass((Symbol.ClassSymbol) st.tsym);
        }
        Env<AttrContext> myEnv = this.enter.getEnv(c);
        if (myEnv == null || (c.flags_field & Flags.TYPE_TRANSLATED) != 0) {
            return;
        }
        c.flags_field |= Flags.TYPE_TRANSLATED;
        boolean envHasCompState = this.compileStates.get(myEnv) != null;
        if (!envHasCompState && c.outermostClass() == c) {
            Assert.error("No info for outermost class: " + myEnv.enclClass.sym);
        }
        if (envHasCompState && CompileStates.CompileState.FLOW.isAfter(this.compileStates.get(myEnv))) {
            Assert.error(String.format(statePreviousToFlowAssertMsg, this.compileStates.get(myEnv), myEnv.enclClass.sym));
        }
        Env<AttrContext> oldEnv = this.env;
        try {
            this.env = myEnv;
            TreeMaker savedMake = this.make;
            Type savedPt = this.pt;
            this.make = this.make.forToplevel(this.env.toplevel);
            this.pt = null;
            try {
                JCTree.JCClassDecl tree = (JCTree.JCClassDecl) this.env.tree;
                tree.typarams = List.nil();
                super.visitClassDef(tree);
                this.make.at(tree.pos);
                if (this.addBridges) {
                    ListBuffer<JCTree> bridges = new ListBuffer<>();
                    if (this.allowInterfaceBridges || (tree.sym.flags() & 512) == 0) {
                        addBridges(tree.pos(), c, bridges);
                    }
                    tree.defs = bridges.toList().prependList(tree.defs);
                }
                tree.type = erasure(tree.type);
            } finally {
                this.make = savedMake;
                this.pt = savedPt;
            }
        } finally {
            this.env = oldEnv;
        }
    }

    public JCTree translateTopLevelClass(JCTree cdef, TreeMaker make) {
        this.make = make;
        this.pt = null;
        return translate(cdef, (Type) null);
    }
}
