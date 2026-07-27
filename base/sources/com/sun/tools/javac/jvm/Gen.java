package com.sun.tools.javac.jvm;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.TargetType;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Check;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.comp.Flow;
import com.sun.tools.javac.comp.Lower;
import com.sun.tools.javac.comp.Resolve;
import com.sun.tools.javac.jvm.Code;
import com.sun.tools.javac.jvm.Items;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.model.FilteredMemberList;
import com.sun.tools.javac.tree.EndPosTable;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import java.util.HashMap;
import java.util.Map;
import javax.lang.model.element.ElementKind;

/* JADX INFO: loaded from: classes.dex */
public class Gen extends JCTree.Visitor {
    protected static final Context.Key<Gen> genKey = new Context.Key<>();
    private Name accessDollar;
    private final boolean allowGenerics;
    private final boolean allowInvokedynamic;
    private Env<AttrContext> attrEnv;
    private final Check chk;
    private Code code;
    private final boolean debugCode;
    EndPosTable endPosTable;
    Env<GenContext> env;
    private final Flow flow;
    private final boolean genCrt;
    private final boolean generateIproxies;
    private Items items;
    private final int jsrlimit;
    private final boolean lineDebugInfo;
    private final Log log;
    private final Lower lower;
    private final TreeMaker make;
    private final Type methodType;
    private final Names names;
    private Pool pool;
    Type pt;
    Items.Item result;
    private final Resolve rs;
    private final Code.StackMapFormat stackMap;
    private final Map<Type, Symbol> stringBufferAppend;
    private final Type stringBufferType;
    private final Symtab syms;
    private final Target target;
    private JCTree.JCCompilationUnit toplevel;
    private final boolean typeAnnoAsserts;
    private final Types types;
    private boolean useJsrLocally;
    private final boolean varDebugInfo;
    private int nerrs = 0;
    private ClassReferenceVisitor classReferenceVisitor = new ClassReferenceVisitor();

    public static class CodeSizeOverflow extends RuntimeException {
        private static final long serialVersionUID = 0;
    }

    public static Gen instance(Context context) {
        Gen instance = (Gen) context.get(genKey);
        if (instance == null) {
            return new Gen(context);
        }
        return instance;
    }

    protected Gen(Context context) {
        boolean zIsSet;
        context.put(genKey, this);
        this.names = Names.instance(context);
        this.log = Log.instance(context);
        this.syms = Symtab.instance(context);
        this.chk = Check.instance(context);
        this.rs = Resolve.instance(context);
        this.make = TreeMaker.instance(context);
        this.target = Target.instance(context);
        this.types = Types.instance(context);
        this.methodType = new Type.MethodType(null, null, null, this.syms.methodClass);
        this.allowGenerics = Source.instance(context).allowGenerics();
        this.stringBufferType = this.target.useStringBuilder() ? this.syms.stringBuilderType : this.syms.stringBufferType;
        this.stringBufferAppend = new HashMap();
        this.accessDollar = this.names.fromString("access" + this.target.syntheticNameChar());
        this.flow = Flow.instance(context);
        this.lower = Lower.instance(context);
        Options options = Options.instance(context);
        boolean z = true;
        this.lineDebugInfo = options.isUnset(Option.G_CUSTOM) || options.isSet(Option.G_CUSTOM, "lines");
        if (options.isUnset(Option.G_CUSTOM)) {
            zIsSet = options.isSet(Option.G);
        } else {
            zIsSet = options.isSet(Option.G_CUSTOM, "vars");
        }
        this.varDebugInfo = zIsSet;
        this.genCrt = options.isSet(Option.XJCOV);
        this.debugCode = options.isSet("debugcode");
        this.allowInvokedynamic = this.target.hasInvokedynamic() || options.isSet("invokedynamic");
        this.pool = new Pool(this.types);
        this.typeAnnoAsserts = options.isSet("TypeAnnotationAsserts");
        if (!this.target.requiresIproxy() && !options.isSet("miranda")) {
            z = false;
        }
        this.generateIproxies = z;
        if (this.target.generateStackMapTable()) {
            this.stackMap = Code.StackMapFormat.JSR202;
        } else if (this.target.generateCLDCStackmap()) {
            this.stackMap = Code.StackMapFormat.CLDC;
        } else {
            this.stackMap = Code.StackMapFormat.NONE;
        }
        int setjsrlimit = 50;
        String jsrlimitString = options.get("jsrlimit");
        if (jsrlimitString != null) {
            try {
                setjsrlimit = Integer.parseInt(jsrlimitString);
            } catch (NumberFormatException e) {
            }
        }
        this.jsrlimit = setjsrlimit;
        this.useJsrLocally = false;
    }

    void loadIntConst(int n) {
        this.items.makeImmediateItem(this.syms.intType, Integer.valueOf(n)).load();
    }

    public static int zero(int tc) {
        switch (tc) {
            case 0:
            case 5:
            case 6:
            case 7:
                return 3;
            case 1:
                return 9;
            case 2:
                return 11;
            case 3:
                return 14;
            case 4:
            default:
                throw new AssertionError("zero");
        }
    }

    public static int one(int tc) {
        return zero(tc) + 1;
    }

    void emitMinusOne(int tc) {
        if (tc == 1) {
            this.items.makeImmediateItem(this.syms.longType, new Long(-1L)).load();
        } else {
            this.code.emitop0(2);
        }
    }

    Symbol binaryQualifier(Symbol sym, Type site) {
        if (site.hasTag(TypeTag.ARRAY)) {
            if (sym == this.syms.lengthVar || sym.owner != this.syms.arrayClass) {
                return sym;
            }
            Symbol qualifier = this.target.arrayBinaryCompatibility() ? new Symbol.ClassSymbol(1L, site.tsym.name, site, this.syms.noSymbol) : this.syms.objectType.tsym;
            return sym.clone(qualifier);
        }
        if (sym.owner == site.tsym || (sym.flags() & 4104) == 4104) {
            return sym;
        }
        if (!this.target.obeyBinaryCompatibility()) {
            return this.rs.isAccessible(this.attrEnv, (Symbol.TypeSymbol) sym.owner) ? sym : sym.clone(site.tsym);
        }
        if ((!this.target.interfaceFieldsBinaryCompatibility() && (sym.owner.flags() & 512) != 0 && sym.kind == 4) || sym.owner == this.syms.objectType.tsym) {
            return sym;
        }
        if (!this.target.interfaceObjectOverridesBinaryCompatibility() && (512 & sym.owner.flags()) != 0 && this.syms.objectType.tsym.members().lookup(sym.name).scope != null) {
            return sym;
        }
        return sym.clone(site.tsym);
    }

    int makeRef(JCDiagnostic.DiagnosticPosition pos, Type type) {
        checkDimension(pos, type);
        if (type.isAnnotated()) {
            return this.pool.put(type);
        }
        return this.pool.put(type.hasTag(TypeTag.CLASS) ? type.tsym : type);
    }

    /* JADX WARN: Multi-variable type inference failed */
    private void checkDimension(JCDiagnostic.DiagnosticPosition pos, Type t) {
        switch (t.getTag()) {
            case METHOD:
                checkDimension(pos, t.mo178getReturnType());
                for (List listMo176getParameterTypes = t.mo176getParameterTypes(); listMo176getParameterTypes.nonEmpty(); listMo176getParameterTypes = listMo176getParameterTypes.tail) {
                    checkDimension(pos, (Type) listMo176getParameterTypes.head);
                }
                break;
            case ARRAY:
                if (this.types.dimensions(t) > 255) {
                    this.log.error(pos, "limit.dimensions", new Object[0]);
                    this.nerrs++;
                }
                break;
        }
    }

    Items.LocalItem makeTemp(Type type) {
        Symbol.VarSymbol v = new Symbol.VarSymbol(4096L, this.names.empty, type, this.env.enclMethod.sym);
        this.code.newLocal(v);
        return this.items.makeLocalItem(v);
    }

    void callMethod(JCDiagnostic.DiagnosticPosition pos, Type site, Name name, List<Type> argtypes, boolean isStatic) {
        Symbol msym = this.rs.resolveInternalMethod(pos, this.attrEnv, site, name, argtypes, null);
        if (isStatic) {
            this.items.makeStaticItem(msym).invoke();
        } else {
            this.items.makeMemberItem(msym, name == this.names.init).invoke();
        }
    }

    private boolean isAccessSuper(JCTree.JCMethodDecl enclMethod) {
        return (enclMethod.mods.flags & 4096) != 0 && isOddAccessName(enclMethod.name);
    }

    private boolean isOddAccessName(Name name) {
        return name.startsWith(this.accessDollar) && (name.getByteAt(name.getByteLength() - 1) & 1) == 1;
    }

    void genFinalizer(Env<GenContext> env) {
        if (this.code.isAlive() && env.info.finalize != null) {
            env.info.finalize.gen();
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r2v0, types: [com.sun.tools.javac.jvm.Gen] */
    Env<GenContext> unwind(JCTree jCTree, Env<GenContext> env) {
        Env env2 = env;
        while (true) {
            genFinalizer(env2);
            if (env2.tree != jCTree) {
                env2 = env2.next;
            } else {
                return env2;
            }
        }
    }

    void endFinalizerGap(Env<GenContext> env) {
        if (env.info.gaps != null && env.info.gaps.length() % 2 == 1) {
            env.info.gaps.append(Integer.valueOf(this.code.curCP()));
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void endFinalizerGaps(Env<GenContext> env, Env<GenContext> env2) {
        Env env3 = null;
        Env env4 = env;
        while (env3 != env2) {
            endFinalizerGap(env4);
            env3 = env4;
            env4 = env4.next;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    boolean hasFinally(JCTree jCTree, Env<GenContext> env) {
        for (Env env2 = env; env2.tree != jCTree; env2 = env2.next) {
            if (env2.tree.hasTag(JCTree.Tag.TRY) && ((GenContext) env2.info).finalize.hasFinalizer()) {
                return true;
            }
        }
        return false;
    }

    /* JADX WARN: Multi-variable type inference failed */
    List<JCTree> normalizeDefs(List<JCTree> defs, Symbol.ClassSymbol c) {
        ListBuffer<JCTree.JCStatement> initCode = new ListBuffer<>();
        ListBuffer<Attribute.TypeCompound> initTAs = new ListBuffer<>();
        ListBuffer<JCTree.JCStatement> clinitCode = new ListBuffer<>();
        ListBuffer<Attribute.TypeCompound> clinitTAs = new ListBuffer<>();
        ListBuffer<JCTree> methodDefs = new ListBuffer<>();
        for (List list = defs; list.nonEmpty(); list = list.tail) {
            JCTree def = (JCTree) list.head;
            switch (def.getTag()) {
                case BLOCK:
                    JCTree.JCBlock block = (JCTree.JCBlock) def;
                    if ((8 & block.flags) != 0) {
                        clinitCode.append(block);
                    } else if ((block.flags & 4096) == 0) {
                        initCode.append(block);
                    }
                    break;
                case METHODDEF:
                    methodDefs.append(def);
                    break;
                case VARDEF:
                    JCTree.JCVariableDecl vdef = (JCTree.JCVariableDecl) def;
                    Symbol.VarSymbol sym = vdef.sym;
                    checkDimension(vdef.pos(), sym.type);
                    if (vdef.init != null) {
                        if ((8 & sym.flags()) == 0) {
                            JCTree.JCStatement init = this.make.at(vdef.pos()).Assignment(sym, vdef.init);
                            initCode.append(init);
                            this.endPosTable.replaceTree(vdef, init);
                            initTAs.addAll(getAndRemoveNonFieldTAs(sym));
                        } else if (sym.getConstValue() != null) {
                            checkStringConstant(vdef.init.pos(), sym.getConstValue());
                            vdef.init.accept(this.classReferenceVisitor);
                        } else {
                            JCTree.JCStatement init2 = this.make.at(vdef.pos).Assignment(sym, vdef.init);
                            clinitCode.append(init2);
                            this.endPosTable.replaceTree(vdef, init2);
                            clinitTAs.addAll(getAndRemoveNonFieldTAs(sym));
                        }
                    }
                    break;
                default:
                    Assert.error();
                    break;
            }
        }
        if (initCode.length() != 0) {
            List<JCTree.JCStatement> inits = initCode.toList();
            initTAs.addAll(c.getInitTypeAttributes());
            List<Attribute.TypeCompound> initTAlist = initTAs.toList();
            for (JCTree t : methodDefs) {
                normalizeMethod((JCTree.JCMethodDecl) t, inits, initTAlist);
            }
        }
        if (clinitCode.length() != 0) {
            Symbol.MethodSymbol clinit = new Symbol.MethodSymbol((c.flags() & 2048) | 8, this.names.clinit, new Type.MethodType(List.nil(), this.syms.voidType, List.nil(), this.syms.methodClass), c);
            c.members().enter(clinit);
            List<JCTree.JCStatement> clinitStats = clinitCode.toList();
            JCTree.JCBlock block2 = this.make.at(clinitStats.head.pos()).Block(0L, clinitStats);
            block2.endpos = TreeInfo.endPos(clinitStats.last());
            methodDefs.append(this.make.MethodDef(clinit, block2));
            if (!clinitTAs.isEmpty()) {
                clinit.appendUniqueTypeAttributes(clinitTAs.toList());
            }
            if (!c.getClassInitTypeAttributes().isEmpty()) {
                clinit.appendUniqueTypeAttributes(c.getClassInitTypeAttributes());
            }
        }
        return methodDefs.toList();
    }

    private List<Attribute.TypeCompound> getAndRemoveNonFieldTAs(Symbol.VarSymbol sym) {
        List<Attribute.TypeCompound> tas = sym.getRawTypeAttributes();
        ListBuffer<Attribute.TypeCompound> fieldTAs = new ListBuffer<>();
        ListBuffer<Attribute.TypeCompound> nonfieldTAs = new ListBuffer<>();
        for (Attribute.TypeCompound ta : tas) {
            if (ta.getPosition().type == TargetType.FIELD) {
                fieldTAs.add(ta);
            } else {
                if (this.typeAnnoAsserts) {
                    Assert.error("Type annotation does not have a valid positior");
                }
                nonfieldTAs.add(ta);
            }
        }
        sym.setTypeAttributes(fieldTAs.toList());
        return nonfieldTAs.toList();
    }

    private void checkStringConstant(JCDiagnostic.DiagnosticPosition pos, Object constValue) {
        if (this.nerrs != 0 || constValue == null || !(constValue instanceof String) || ((String) constValue).length() < 65535) {
            return;
        }
        this.log.error(pos, "limit.string", new Object[0]);
        this.nerrs++;
    }

    /* JADX WARN: Multi-variable type inference failed */
    void normalizeMethod(JCTree.JCMethodDecl md, List<JCTree.JCStatement> initCode, List<Attribute.TypeCompound> initTAs) {
        if (md.name == this.names.init && TreeInfo.isInitialConstructor(md)) {
            List list = md.body.stats;
            ListBuffer listBuffer = new ListBuffer();
            if (list.nonEmpty()) {
                while (TreeInfo.isSyntheticInit((JCTree) list.head)) {
                    listBuffer.append(list.head);
                    list = list.tail;
                }
                listBuffer.append(list.head);
                List list2 = list.tail;
                while (list2.nonEmpty() && TreeInfo.isSyntheticInit((JCTree) list2.head)) {
                    listBuffer.append(list2.head);
                    list2 = list2.tail;
                }
                listBuffer.appendList(initCode);
                while (list2.nonEmpty()) {
                    listBuffer.append(list2.head);
                    list2 = list2.tail;
                }
            }
            md.body.stats = listBuffer.toList();
            if (md.body.endpos == -1) {
                md.body.endpos = TreeInfo.endPos(md.body.stats.last());
            }
            md.sym.appendUniqueTypeAttributes(initTAs);
        }
    }

    void implementInterfaceMethods(Symbol.ClassSymbol c) {
        implementInterfaceMethods(c, c);
    }

    /* JADX WARN: Multi-variable type inference failed */
    void implementInterfaceMethods(Symbol.ClassSymbol c, Symbol.ClassSymbol site) {
        for (List listInterfaces = this.types.interfaces(c.type); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
            Symbol.ClassSymbol i = (Symbol.ClassSymbol) ((Type) listInterfaces.head).tsym;
            for (Scope.Entry e = i.members().elems; e != null; e = e.sibling) {
                if (e.sym.kind == 16 && (e.sym.flags() & 8) == 0) {
                    Symbol.MethodSymbol absMeth = (Symbol.MethodSymbol) e.sym;
                    Symbol.MethodSymbol implMeth = absMeth.binaryImplementation(site, this.types);
                    if (implMeth == null) {
                        addAbstractMethod(site, absMeth);
                    } else if ((implMeth.flags() & 2097152) != 0) {
                        adjustAbstractMethod(site, implMeth, absMeth);
                    }
                }
            }
            implementInterfaceMethods(i, site);
        }
    }

    private void addAbstractMethod(Symbol.ClassSymbol c, Symbol.MethodSymbol m) {
        Symbol.MethodSymbol absMeth = new Symbol.MethodSymbol(m.flags() | 2097152 | 4096, m.name, m.type, c);
        c.members().enter(absMeth);
    }

    private void adjustAbstractMethod(Symbol.ClassSymbol c, Symbol.MethodSymbol pm, Symbol.MethodSymbol im) {
        Type.MethodType pmt = (Type.MethodType) pm.type;
        Type imt = this.types.memberType(c.type, im);
        pmt.thrown = this.chk.intersect(pmt.mo179getThrownTypes(), imt.mo179getThrownTypes());
    }

    public void genDef(JCTree tree, Env<GenContext> env) {
        Env<GenContext> prevEnv = this.env;
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

    public void genStat(JCTree tree, Env<GenContext> env, int crtFlags) {
        if (!this.genCrt) {
            genStat(tree, env);
            return;
        }
        int startpc = this.code.curCP();
        genStat(tree, env);
        if (tree.hasTag(JCTree.Tag.BLOCK)) {
            crtFlags |= 2;
        }
        this.code.crt.put(tree, crtFlags, startpc, this.code.curCP());
    }

    public void genStat(JCTree tree, Env<GenContext> env) {
        if (this.code.isAlive()) {
            this.code.statBegin(tree.pos);
            genDef(tree, env);
        } else if (env.info.isSwitch && tree.hasTag(JCTree.Tag.VARDEF)) {
            this.code.newLocal(((JCTree.JCVariableDecl) tree).sym);
        }
    }

    public void genStats(List<JCTree.JCStatement> trees, Env<GenContext> env, int crtFlags) {
        if (!this.genCrt) {
            genStats(trees, env);
        } else {
            if (trees.length() == 1) {
                genStat(trees.head, env, crtFlags | 1);
                return;
            }
            int startpc = this.code.curCP();
            genStats(trees, env);
            this.code.crt.put(trees, crtFlags, startpc, this.code.curCP());
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public void genStats(List<? extends JCTree> trees, Env<GenContext> env) {
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            genStat((JCTree) list.head, env, 1);
        }
    }

    public Items.CondItem genCond(JCTree tree, int crtFlags) {
        if (!this.genCrt) {
            return genCond(tree, false);
        }
        int startpc = this.code.curCP();
        Items.CondItem item = genCond(tree, (crtFlags & 8) != 0);
        this.code.crt.put(tree, crtFlags, startpc, this.code.curCP());
        return item;
    }

    public Items.CondItem genCond(JCTree _tree, boolean markBranches) {
        JCTree inner_tree = TreeInfo.skipParens(_tree);
        if (inner_tree.hasTag(JCTree.Tag.CONDEXPR)) {
            JCTree.JCConditional tree = (JCTree.JCConditional) inner_tree;
            Items.CondItem cond = genCond(tree.cond, 8);
            if (cond.isTrue()) {
                this.code.resolve(cond.trueJumps);
                Items.CondItem result = genCond(tree.truepart, 16);
                if (markBranches) {
                    result.tree = tree.truepart;
                }
                return result;
            }
            if (cond.isFalse()) {
                this.code.resolve(cond.falseJumps);
                Items.CondItem result2 = genCond(tree.falsepart, 16);
                if (markBranches) {
                    result2.tree = tree.falsepart;
                }
                return result2;
            }
            Code.Chain secondJumps = cond.jumpFalse();
            this.code.resolve(cond.trueJumps);
            Items.CondItem first = genCond(tree.truepart, 16);
            if (markBranches) {
                first.tree = tree.truepart;
            }
            Code.Chain falseJumps = first.jumpFalse();
            this.code.resolve(first.trueJumps);
            Code.Chain trueJumps = this.code.branch(ByteCodes.goto_);
            this.code.resolve(secondJumps);
            Items.CondItem second = genCond(tree.falsepart, 16);
            Items.CondItem result3 = this.items.makeCondItem(second.opcode, Code.mergeChains(trueJumps, second.trueJumps), Code.mergeChains(falseJumps, second.falseJumps));
            if (markBranches) {
                result3.tree = tree.falsepart;
            }
            return result3;
        }
        Items.CondItem result4 = genExpr(_tree, this.syms.booleanType).mkCond();
        if (markBranches) {
            result4.tree = _tree;
        }
        return result4;
    }

    class ClassReferenceVisitor extends JCTree.Visitor {
        ClassReferenceVisitor() {
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTree(JCTree tree) {
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBinary(JCTree.JCBinary tree) {
            tree.lhs.accept(this);
            tree.rhs.accept(this);
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            if (tree.selected.type.hasTag(TypeTag.CLASS)) {
                Gen.this.makeRef(tree.selected.pos(), tree.selected.type);
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            if (tree.sym.owner instanceof Symbol.ClassSymbol) {
                Gen.this.pool.put(tree.sym.owner);
            }
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitConditional(JCTree.JCConditional tree) {
            tree.cond.accept(this);
            tree.truepart.accept(this);
            tree.falsepart.accept(this);
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitUnary(JCTree.JCUnary tree) {
            tree.arg.accept(this);
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

    public Items.Item genExpr(JCTree tree, Type pt) {
        Type prevPt = this.pt;
        try {
            if (tree.type.constValue() != null) {
                tree.accept(this.classReferenceVisitor);
                checkStringConstant(tree.pos(), tree.type.constValue());
                this.result = this.items.makeImmediateItem(tree.type, tree.type.constValue());
            } else {
                this.pt = pt;
                tree.accept(this);
            }
            return this.result.coerce(pt);
        } catch (Symbol.CompletionFailure ex) {
            this.chk.completionError(tree.pos(), ex);
            this.code.state.stacksize = 1;
            return this.items.makeStackItem(pt);
        } finally {
            this.pt = prevPt;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public void genArgs(List<JCTree.JCExpression> list, List<Type> list2) {
        List list3 = list;
        List list4 = list2;
        while (list3.nonEmpty()) {
            genExpr((JCTree) list3.head, (Type) list4.head).load();
            List list5 = list4.tail;
            list3 = list3.tail;
            list4 = list5;
        }
        Assert.check(list4.isEmpty());
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitMethodDef(JCTree.JCMethodDecl tree) {
        Env<GenContext> localEnv = this.env.dup(tree);
        localEnv.enclMethod = tree;
        this.pt = tree.sym.erasure(this.types).mo178getReturnType();
        checkDimension(tree.pos(), tree.sym.erasure(this.types));
        genMethod(tree, localEnv, false);
    }

    void genMethod(JCTree.JCMethodDecl tree, Env<GenContext> env, boolean fatcode) {
        Symbol.MethodSymbol meth = tree.sym;
        int extras = 0;
        if (meth.isConstructor()) {
            extras = 0 + 1;
            if (meth.enclClass().isInner() && !meth.enclClass().isStatic()) {
                extras++;
            }
        } else if ((tree.mods.flags & 8) == 0) {
            extras = 0 + 1;
        }
        if (Code.width(this.types.erasure(env.enclMethod.sym.type).mo176getParameterTypes()) + extras > 255) {
            this.log.error(tree.pos(), "limit.parameters", new Object[0]);
            this.nerrs++;
            return;
        }
        if (tree.body != null) {
            int startpcCrt = initCode(tree, env, fatcode);
            try {
                genStat(tree.body, env);
            } catch (CodeSizeOverflow e) {
                startpcCrt = initCode(tree, env, fatcode);
                genStat(tree.body, env);
            }
            if (this.code.state.stacksize != 0) {
                this.log.error(tree.body.pos(), "stack.sim.error", tree);
                throw new AssertionError();
            }
            if (this.code.isAlive()) {
                this.code.statBegin(TreeInfo.endPos(tree.body));
                if (env.enclMethod == null || env.enclMethod.sym.type.mo178getReturnType().hasTag(TypeTag.VOID)) {
                    this.code.emitop0(ByteCodes.return_);
                } else {
                    int startpc = this.code.entryPoint();
                    Items.CondItem c = this.items.makeCondItem(ByteCodes.goto_);
                    this.code.resolve(c.jumpTrue(), startpc);
                }
            }
            if (this.genCrt) {
                this.code.crt.put(tree.body, 2, startpcCrt, this.code.curCP());
            }
            this.code.endScopes(0);
            if (this.code.checkLimits(tree.pos(), this.log)) {
                this.nerrs++;
                return;
            }
            if (!fatcode && this.code.fatcode) {
                genMethod(tree, env, true);
            }
            if (this.stackMap == Code.StackMapFormat.JSR202) {
                this.code.lastFrame = null;
                this.code.frameBeforeLast = null;
            }
            this.code.compressCatchTable();
            this.code.fillExceptionParameterPositions();
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    private int initCode(JCTree.JCMethodDecl tree, Env<GenContext> env, boolean fatcode) {
        Symbol.MethodSymbol meth = tree.sym;
        Code code = new Code(meth, fatcode, this.lineDebugInfo ? this.toplevel.lineMap : null, this.varDebugInfo, this.stackMap, this.debugCode, this.genCrt ? new CRTable(tree, env.toplevel.endPositions) : null, this.syms, this.types, this.pool);
        this.code = code;
        meth.code = code;
        this.items = new Items(this.pool, this.code, this.syms, this.types);
        if (this.code.debugCode) {
            System.err.println(meth + " for body " + tree);
        }
        if ((tree.mods.flags & 8) == 0) {
            Type selfType = meth.owner.type;
            if (meth.isConstructor() && selfType != this.syms.objectType) {
                selfType = UninitializedType.uninitializedThis(selfType);
            }
            this.code.setDefined(this.code.newLocal(new Symbol.VarSymbol(16L, this.names._this, selfType, meth.owner)));
        }
        for (List list = tree.params; list.nonEmpty(); list = list.tail) {
            checkDimension(((JCTree.JCVariableDecl) list.head).pos(), ((JCTree.JCVariableDecl) list.head).sym.type);
            this.code.setDefined(this.code.newLocal(((JCTree.JCVariableDecl) list.head).sym));
        }
        int startpcCrt = this.genCrt ? this.code.curCP() : 0;
        this.code.entryPoint();
        this.code.pendingStackMap = false;
        return startpcCrt;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitVarDef(JCTree.JCVariableDecl tree) {
        Symbol.VarSymbol v = tree.sym;
        this.code.newLocal(v);
        if (tree.init != null) {
            checkStringConstant(tree.init.pos(), v.getConstValue());
            if (v.getConstValue() == null || this.varDebugInfo) {
                genExpr(tree.init, v.erasure(this.types)).load();
                this.items.makeLocalItem(v).store();
            }
        }
        checkDimension(tree.pos(), v.type);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSkip(JCTree.JCSkip tree) {
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBlock(JCTree.JCBlock tree) {
        int limit = this.code.nextreg;
        Env<GenContext> localEnv = this.env.dup(tree, new GenContext());
        genStats(tree.stats, localEnv);
        if (!this.env.tree.hasTag(JCTree.Tag.METHODDEF)) {
            this.code.statBegin(tree.endpos);
            this.code.endScopes(limit);
            this.code.pendingStatPos = -1;
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
        genLoop(tree, tree.body, tree.cond, List.nil(), false);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWhileLoop(JCTree.JCWhileLoop tree) {
        genLoop(tree, tree.body, tree.cond, List.nil(), true);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForLoop(JCTree.JCForLoop tree) {
        int limit = this.code.nextreg;
        genStats(tree.init, this.env);
        genLoop(tree, tree.body, tree.cond, tree.step, true);
        this.code.endScopes(limit);
    }

    private void genLoop(JCTree.JCStatement loop, JCTree.JCStatement body, JCTree.JCExpression cond, List<JCTree.JCExpressionStatement> step, boolean testFirst) {
        Items.CondItem c;
        Items.CondItem c2;
        Env<GenContext> loopEnv = this.env.dup(loop, new GenContext());
        int startpc = this.code.entryPoint();
        if (testFirst) {
            if (cond != null) {
                this.code.statBegin(cond.pos);
                c2 = genCond(TreeInfo.skipParens(cond), 8);
            } else {
                c2 = this.items.makeCondItem(ByteCodes.goto_);
            }
            Code.Chain loopDone = c2.jumpFalse();
            this.code.resolve(c2.trueJumps);
            genStat(body, loopEnv, 17);
            this.code.resolve(loopEnv.info.cont);
            genStats(step, loopEnv);
            this.code.resolve(this.code.branch(ByteCodes.goto_), startpc);
            this.code.resolve(loopDone);
        } else {
            genStat(body, loopEnv, 17);
            this.code.resolve(loopEnv.info.cont);
            genStats(step, loopEnv);
            if (cond != null) {
                this.code.statBegin(cond.pos);
                c = genCond(TreeInfo.skipParens(cond), 8);
            } else {
                c = this.items.makeCondItem(ByteCodes.goto_);
            }
            this.code.resolve(c.jumpTrue(), startpc);
            this.code.resolve(c.falseJumps);
        }
        this.code.resolve(loopEnv.info.exit);
        if (loopEnv.info.exit != null) {
            loopEnv.info.exit.state.defined.excludeFrom(this.code.nextreg);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
        throw new AssertionError();
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLabelled(JCTree.JCLabeledStatement tree) {
        Env<GenContext> localEnv = this.env.dup(tree, new GenContext());
        genStat(tree.body, localEnv, 1);
        this.code.resolve(localEnv.info.exit);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSwitch(JCTree.JCSwitch tree) {
        int limit;
        int lo;
        int[] offsets;
        int[] labels;
        List list;
        int nlabels;
        int[] labels2;
        int limit2 = this.code.nextreg;
        Assert.check(!tree.selector.type.hasTag(TypeTag.CLASS));
        int startpcCrt = this.genCrt ? this.code.curCP() : 0;
        Items.Item sel = genExpr(tree.selector, this.syms.intType);
        List<JCTree.JCCase> cases = tree.cases;
        if (cases.isEmpty()) {
            sel.load().drop();
            if (!this.genCrt) {
                limit = limit2;
            } else {
                this.code.crt.put(TreeInfo.skipParens(tree.selector), 8, startpcCrt, this.code.curCP());
                limit = limit2;
            }
        } else {
            sel.load();
            if (this.genCrt) {
                this.code.crt.put(TreeInfo.skipParens(tree.selector), 8, startpcCrt, this.code.curCP());
            }
            Env<GenContext> switchEnv = this.env.dup(tree, new GenContext());
            switchEnv.info.isSwitch = true;
            int lo2 = Integer.MAX_VALUE;
            int hi = Integer.MIN_VALUE;
            int nlabels2 = 0;
            int[] labels3 = new int[cases.length()];
            int defaultIndex = -1;
            List list2 = cases;
            for (int i = 0; i < labels3.length; i++) {
                if (((JCTree.JCCase) list2.head).pat != null) {
                    int val = ((Number) ((JCTree.JCCase) list2.head).pat.type.constValue()).intValue();
                    labels3[i] = val;
                    if (val < lo2) {
                        lo2 = val;
                    }
                    if (hi < val) {
                        hi = val;
                    }
                    nlabels2++;
                } else {
                    Assert.check(defaultIndex == -1);
                    defaultIndex = i;
                }
                list2 = list2.tail;
            }
            limit = limit2;
            long table_space_cost = (((long) hi) - ((long) lo2)) + 1 + 4;
            long lookup_space_cost = (((long) nlabels2) * 2) + 3;
            int lo3 = lo2;
            long lookup_time_cost = nlabels2;
            int opcode = (nlabels2 <= 0 || table_space_cost + (3 * 3) > lookup_space_cost + (lookup_time_cost * 3)) ? ByteCodes.lookupswitch : ByteCodes.tableswitch;
            int startpc = this.code.curCP();
            int opcode2 = opcode;
            this.code.emitop0(opcode2);
            this.code.align(4);
            int tableBase = this.code.curCP();
            this.code.emit4(-1);
            if (opcode2 != 170) {
                lo = lo3;
                this.code.emit4(nlabels2);
                for (int i2 = 0; i2 < nlabels2; i2++) {
                    this.code.emit4(-1);
                    this.code.emit4(-1);
                }
                int i3 = labels3.length;
                offsets = new int[i3];
            } else {
                lo = lo3;
                this.code.emit4(lo);
                this.code.emit4(hi);
                long i4 = lo;
                while (true) {
                    long lookup_time_cost2 = lookup_time_cost;
                    if (i4 > hi) {
                        break;
                    }
                    this.code.emit4(-1);
                    i4++;
                    lookup_time_cost = lookup_time_cost2;
                }
                offsets = null;
            }
            Code.State stateSwitch = this.code.state.dup();
            this.code.markDead();
            List list3 = cases;
            int i5 = 0;
            while (i5 < labels3.length) {
                JCTree.JCCase c = (JCTree.JCCase) list3.head;
                List list4 = list3.tail;
                List<JCTree.JCCase> cases2 = cases;
                int pc = this.code.entryPoint(stateSwitch);
                if (i5 != defaultIndex) {
                    list = list4;
                    if (opcode2 == 170) {
                        Code code = this.code;
                        nlabels = nlabels2;
                        int nlabels3 = tableBase + (((labels3[i5] - lo) + 3) * 4);
                        labels2 = labels3;
                        code.put4(nlabels3, pc - startpc);
                    } else {
                        nlabels = nlabels2;
                        labels2 = labels3;
                        offsets[i5] = pc - startpc;
                    }
                } else {
                    list = list4;
                    nlabels = nlabels2;
                    labels2 = labels3;
                    this.code.put4(tableBase, pc - startpc);
                }
                genStats(c.stats, switchEnv, 16);
                i5++;
                cases = cases2;
                list3 = list;
                labels3 = labels2;
                nlabels2 = nlabels;
            }
            int nlabels4 = nlabels2;
            int[] labels4 = labels3;
            this.code.resolve(switchEnv.info.exit);
            if (this.code.get4(tableBase) == -1) {
                this.code.put4(tableBase, this.code.entryPoint(stateSwitch) - startpc);
            }
            if (opcode2 == 170) {
                int defaultOffset = this.code.get4(tableBase);
                long i6 = lo;
                while (true) {
                    int defaultOffset2 = defaultOffset;
                    if (i6 > hi) {
                        break;
                    }
                    int opcode3 = opcode2;
                    int startpc2 = startpc;
                    int t = (int) (((long) tableBase) + (((i6 - ((long) lo)) + 3) * 4));
                    if (this.code.get4(t) != -1) {
                        defaultOffset = defaultOffset2;
                    } else {
                        defaultOffset = defaultOffset2;
                        this.code.put4(t, defaultOffset);
                    }
                    i6++;
                    startpc = startpc2;
                    opcode2 = opcode3;
                }
            } else {
                if (defaultIndex < 0) {
                    labels = labels4;
                } else {
                    int i7 = defaultIndex;
                    while (true) {
                        labels = labels4;
                        if (i7 >= labels.length - 1) {
                            break;
                        }
                        labels[i7] = labels[i7 + 1];
                        offsets[i7] = offsets[i7 + 1];
                        i7++;
                        labels4 = labels;
                    }
                }
                if (nlabels4 > 0) {
                    qsort2(labels, offsets, 0, nlabels4 - 1);
                }
                int i8 = 0;
                while (true) {
                    int nlabels5 = nlabels4;
                    if (i8 >= nlabels5) {
                        break;
                    }
                    int caseidx = ((i8 + 1) * 8) + tableBase;
                    this.code.put4(caseidx, labels[i8]);
                    this.code.put4(caseidx + 4, offsets[i8]);
                    i8++;
                    nlabels4 = nlabels5;
                }
            }
        }
        this.code.endScopes(limit);
    }

    static void qsort2(int[] keys, int[] values, int lo, int hi) {
        int i = lo;
        int j = hi;
        int pivot = keys[(i + j) / 2];
        while (true) {
            if (keys[i] < pivot) {
                i++;
            } else {
                while (pivot < keys[j]) {
                    j--;
                }
                if (i <= j) {
                    int temp1 = keys[i];
                    keys[i] = keys[j];
                    keys[j] = temp1;
                    int temp2 = values[i];
                    values[i] = values[j];
                    values[j] = temp2;
                    i++;
                    j--;
                }
                if (i > j) {
                    break;
                }
            }
        }
        if (lo < j) {
            qsort2(keys, values, lo, j);
        }
        if (i < hi) {
            qsort2(keys, values, i, hi);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSynchronized(JCTree.JCSynchronized tree) {
        int limit = this.code.nextreg;
        final Items.LocalItem lockVar = makeTemp(this.syms.objectType);
        genExpr(tree.lock, tree.lock.type).load().duplicate();
        lockVar.store();
        this.code.emitop0(ByteCodes.monitorenter);
        this.code.state.lock(lockVar.reg);
        final Env<GenContext> syncEnv = this.env.dup(tree, new GenContext());
        syncEnv.info.finalize = new GenFinalizer() { // from class: com.sun.tools.javac.jvm.Gen.1
            /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
            {
                super();
            }

            /* JADX WARN: Multi-variable type inference failed */
            @Override // com.sun.tools.javac.jvm.Gen.GenFinalizer
            void gen() {
                genLast();
                Assert.check(((GenContext) syncEnv.info).gaps.length() % 2 == 0);
                ((GenContext) syncEnv.info).gaps.append(Integer.valueOf(Gen.this.code.curCP()));
            }

            @Override // com.sun.tools.javac.jvm.Gen.GenFinalizer
            void genLast() {
                if (Gen.this.code.isAlive()) {
                    lockVar.load();
                    Gen.this.code.emitop0(ByteCodes.monitorexit);
                    Gen.this.code.state.unlock(lockVar.reg);
                }
            }
        };
        syncEnv.info.gaps = new ListBuffer<>();
        genTry(tree.body, List.nil(), syncEnv);
        this.code.endScopes(limit);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTry(final JCTree.JCTry tree) {
        final Env<GenContext> tryEnv = this.env.dup(tree, new GenContext());
        final Env<GenContext> oldEnv = this.env;
        if (!this.useJsrLocally) {
            this.useJsrLocally = this.stackMap == Code.StackMapFormat.NONE && (this.jsrlimit <= 0 || (this.jsrlimit < 100 && estimateCodeComplexity(tree.finalizer) > this.jsrlimit));
        }
        tryEnv.info.finalize = new GenFinalizer() { // from class: com.sun.tools.javac.jvm.Gen.2
            /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
            {
                super();
            }

            /* JADX WARN: Multi-variable type inference failed */
            @Override // com.sun.tools.javac.jvm.Gen.GenFinalizer
            void gen() {
                if (Gen.this.useJsrLocally) {
                    if (tree.finalizer != null) {
                        Code.State jsrState = Gen.this.code.state.dup();
                        jsrState.push(Code.jsrReturnValue);
                        ((GenContext) tryEnv.info).cont = new Code.Chain(Gen.this.code.emitJump(168), ((GenContext) tryEnv.info).cont, jsrState);
                    }
                    Assert.check(((GenContext) tryEnv.info).gaps.length() % 2 == 0);
                    ((GenContext) tryEnv.info).gaps.append(Integer.valueOf(Gen.this.code.curCP()));
                    return;
                }
                Assert.check(((GenContext) tryEnv.info).gaps.length() % 2 == 0);
                ((GenContext) tryEnv.info).gaps.append(Integer.valueOf(Gen.this.code.curCP()));
                genLast();
            }

            @Override // com.sun.tools.javac.jvm.Gen.GenFinalizer
            void genLast() {
                if (tree.finalizer != null) {
                    Gen.this.genStat(tree.finalizer, oldEnv, 2);
                }
            }

            @Override // com.sun.tools.javac.jvm.Gen.GenFinalizer
            boolean hasFinalizer() {
                return tree.finalizer != null;
            }
        };
        tryEnv.info.gaps = new ListBuffer<>();
        genTry(tree.body, tree.catchers, tryEnv);
    }

    /* JADX WARN: Multi-variable type inference failed */
    void genTry(JCTree body, List<JCTree.JCCatch> catchers, Env<GenContext> env) {
        Code.Chain exitChain;
        char c;
        int limit = this.code.nextreg;
        int startpc = this.code.curCP();
        Code.State stateTry = this.code.state.dup();
        genStat(body, env, 2);
        int endpc = this.code.curCP();
        boolean hasFinalizer = env.info.finalize != null && env.info.finalize.hasFinalizer();
        List<Integer> gaps = env.info.gaps.toList();
        this.code.statBegin(TreeInfo.endPos(body));
        genFinalizer(env);
        this.code.statBegin(TreeInfo.endPos(env.tree));
        Code.Chain exitChain2 = this.code.branch(ByteCodes.goto_);
        endFinalizerGap(env);
        if (startpc != endpc) {
            Code.Chain exitChain3 = exitChain2;
            List list = catchers;
            while (list.nonEmpty()) {
                this.code.entryPoint(stateTry, ((JCTree.JCCatch) list.head).param.sym.type);
                List list2 = list;
                Code.Chain exitChain4 = exitChain3;
                genCatch((JCTree.JCCatch) list.head, env, startpc, endpc, gaps);
                genFinalizer(env);
                if (hasFinalizer || list2.tail.nonEmpty()) {
                    this.code.statBegin(TreeInfo.endPos(env.tree));
                    c = 167;
                    exitChain3 = Code.mergeChains(exitChain4, this.code.branch(ByteCodes.goto_));
                } else {
                    exitChain3 = exitChain4;
                    c = 167;
                }
                endFinalizerGap(env);
                list = list2.tail;
            }
            exitChain = exitChain3;
        } else {
            exitChain = exitChain2;
        }
        if (hasFinalizer) {
            this.code.newRegSegment();
            int catchallpc = this.code.entryPoint(stateTry, this.syms.throwableType);
            int startseg = startpc;
            while (env.info.gaps.nonEmpty()) {
                int endseg = env.info.gaps.next().intValue();
                registerCatch(body.pos(), startseg, endseg, catchallpc, 0);
                startseg = env.info.gaps.next().intValue();
            }
            this.code.statBegin(TreeInfo.finalizerPos(env.tree));
            this.code.markStatBegin();
            Items.Item excVar = makeTemp(this.syms.throwableType);
            excVar.store();
            genFinalizer(env);
            excVar.load();
            registerCatch(body.pos(), startseg, env.info.gaps.next().intValue(), catchallpc, 0);
            this.code.emitop0(ByteCodes.athrow);
            this.code.markDead();
            if (env.info.cont != null) {
                this.code.resolve(env.info.cont);
                this.code.statBegin(TreeInfo.finalizerPos(env.tree));
                this.code.markStatBegin();
                Items.LocalItem retVar = makeTemp(this.syms.throwableType);
                retVar.store();
                env.info.finalize.genLast();
                this.code.emitop1w(ByteCodes.ret, retVar.reg);
                this.code.markDead();
            }
        }
        this.code.resolve(exitChain);
        this.code.endScopes(limit);
    }

    /* JADX WARN: Multi-variable type inference failed */
    void genCatch(JCTree.JCCatch tree, Env<GenContext> env, int startpc, int endpc, List<Integer> gaps) {
        if (startpc != endpc) {
            List<JCTree.JCExpression> subClauses = TreeInfo.isMultiCatch(tree) ? ((JCTree.JCTypeUnion) tree.param.vartype).alternatives : List.of(tree.param.vartype);
            List list = gaps;
            int startpc2 = startpc;
            while (list.nonEmpty()) {
                for (JCTree.JCExpression subCatch : subClauses) {
                    int catchType = makeRef(tree.pos(), subCatch.type);
                    int end = ((Integer) list.head).intValue();
                    registerCatch(tree.pos(), startpc2, end, this.code.curCP(), catchType);
                    if (subCatch.type.isAnnotated()) {
                        for (Attribute.TypeCompound tc : subCatch.type.getAnnotationMirrors()) {
                            tc.position.type_index = catchType;
                        }
                    }
                }
                List<A> list2 = list.tail;
                startpc2 = ((Integer) list2.head).intValue();
                list = list2.tail;
            }
            if (startpc2 < endpc) {
                for (JCTree.JCExpression subCatch2 : subClauses) {
                    int catchType2 = makeRef(tree.pos(), subCatch2.type);
                    registerCatch(tree.pos(), startpc2, endpc, this.code.curCP(), catchType2);
                    if (subCatch2.type.isAnnotated()) {
                        for (Attribute.TypeCompound tc2 : subCatch2.type.getAnnotationMirrors()) {
                            tc2.position.type_index = catchType2;
                        }
                    }
                }
            }
            Symbol.VarSymbol exparam = tree.param.sym;
            this.code.statBegin(tree.pos);
            this.code.markStatBegin();
            int limit = this.code.nextreg;
            this.code.newLocal(exparam);
            this.items.makeLocalItem(exparam).store();
            this.code.statBegin(TreeInfo.firstStatPos(tree.body));
            genStat(tree.body, env, 2);
            this.code.endScopes(limit);
            this.code.statBegin(TreeInfo.endPos(tree.body));
        }
    }

    void registerCatch(JCDiagnostic.DiagnosticPosition pos, int startpc, int endpc, int handler_pc, int catch_type) {
        char startpc1 = (char) startpc;
        char endpc1 = (char) endpc;
        char handler_pc1 = (char) handler_pc;
        if (startpc1 == startpc && endpc1 == endpc && handler_pc1 == handler_pc) {
            this.code.addCatch(startpc1, endpc1, handler_pc1, (char) catch_type);
        } else {
            if (!this.useJsrLocally && !this.target.generateStackMapTable()) {
                this.useJsrLocally = true;
                throw new CodeSizeOverflow();
            }
            this.log.error(pos, "limit.code.too.large.for.try.stmt", new Object[0]);
            this.nerrs++;
        }
    }

    /* JADX INFO: renamed from: com.sun.tools.javac.jvm.Gen$1ComplexityScanner, reason: invalid class name */
    class C1ComplexityScanner extends TreeScanner {
        int complexity = 0;

        C1ComplexityScanner() {
        }

        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree tree) {
            if (this.complexity > Gen.this.jsrlimit) {
                return;
            }
            super.scan(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
            super.visitDoLoop(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitWhileLoop(JCTree.JCWhileLoop tree) {
            super.visitWhileLoop(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitForLoop(JCTree.JCForLoop tree) {
            super.visitForLoop(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSwitch(JCTree.JCSwitch tree) {
            super.visitSwitch(tree);
            this.complexity += 5;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitCase(JCTree.JCCase tree) {
            super.visitCase(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSynchronized(JCTree.JCSynchronized tree) {
            super.visitSynchronized(tree);
            this.complexity += 6;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTry(JCTree.JCTry tree) {
            super.visitTry(tree);
            if (tree.finalizer != null) {
                this.complexity += 6;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitCatch(JCTree.JCCatch tree) {
            super.visitCatch(tree);
            this.complexity += 2;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitConditional(JCTree.JCConditional tree) {
            super.visitConditional(tree);
            this.complexity += 2;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIf(JCTree.JCIf tree) {
            super.visitIf(tree);
            this.complexity += 2;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBreak(JCTree.JCBreak tree) {
            super.visitBreak(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitContinue(JCTree.JCContinue tree) {
            super.visitContinue(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReturn(JCTree.JCReturn tree) {
            super.visitReturn(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitThrow(JCTree.JCThrow tree) {
            super.visitThrow(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssert(JCTree.JCAssert tree) {
            super.visitAssert(tree);
            this.complexity += 5;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation tree) {
            super.visitApply(tree);
            this.complexity += 2;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            scan(tree.encl);
            scan(tree.args);
            this.complexity += 2;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewArray(JCTree.JCNewArray tree) {
            super.visitNewArray(tree);
            this.complexity += 5;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssign(JCTree.JCAssign tree) {
            super.visitAssign(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssignop(JCTree.JCAssignOp tree) {
            super.visitAssignop(tree);
            this.complexity += 2;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitUnary(JCTree.JCUnary tree) {
            this.complexity++;
            if (tree.type.constValue() == null) {
                super.visitUnary(tree);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBinary(JCTree.JCBinary tree) {
            this.complexity++;
            if (tree.type.constValue() == null) {
                super.visitBinary(tree);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeTest(JCTree.JCInstanceOf tree) {
            super.visitTypeTest(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIndexed(JCTree.JCArrayAccess tree) {
            super.visitIndexed(tree);
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            super.visitSelect(tree);
            if (tree.sym.kind == 4) {
                this.complexity++;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            if (tree.sym.kind == 4) {
                this.complexity++;
                if (tree.type.constValue() == null && tree.sym.owner.kind == 2) {
                    this.complexity++;
                }
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLiteral(JCTree.JCLiteral tree) {
            this.complexity++;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTree(JCTree tree) {
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitWildcard(JCTree.JCWildcard tree) {
            throw new AssertionError(getClass().getName());
        }
    }

    int estimateCodeComplexity(JCTree tree) {
        if (tree == null) {
            return 0;
        }
        C1ComplexityScanner scanner = new C1ComplexityScanner();
        tree.accept(scanner);
        return scanner.complexity;
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIf(JCTree.JCIf tree) {
        int limit = this.code.nextreg;
        Code.Chain thenExit = null;
        Items.CondItem c = genCond(TreeInfo.skipParens(tree.cond), 8);
        Code.Chain elseChain = c.jumpFalse();
        if (!c.isFalse()) {
            this.code.resolve(c.trueJumps);
            genStat(tree.thenpart, this.env, 17);
            thenExit = this.code.branch(ByteCodes.goto_);
        }
        if (elseChain != null) {
            this.code.resolve(elseChain);
            if (tree.elsepart != null) {
                genStat(tree.elsepart, this.env, 17);
            }
        }
        this.code.resolve(thenExit);
        this.code.endScopes(limit);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitExec(JCTree.JCExpressionStatement tree) {
        JCTree.JCExpression e = tree.expr;
        switch (e.getTag()) {
            case POSTINC:
                ((JCTree.JCUnary) e).setTag(JCTree.Tag.PREINC);
                break;
            case POSTDEC:
                ((JCTree.JCUnary) e).setTag(JCTree.Tag.PREDEC);
                break;
        }
        genExpr(tree.expr, tree.expr.type).drop();
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBreak(JCTree.JCBreak tree) {
        Env<GenContext> targetEnv = unwind(tree.target, this.env);
        Assert.check(this.code.state.stacksize == 0);
        targetEnv.info.addExit(this.code.branch(ByteCodes.goto_));
        endFinalizerGaps(this.env, targetEnv);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitContinue(JCTree.JCContinue tree) {
        Env<GenContext> targetEnv = unwind(tree.target, this.env);
        Assert.check(this.code.state.stacksize == 0);
        targetEnv.info.addCont(this.code.branch(ByteCodes.goto_));
        endFinalizerGaps(this.env, targetEnv);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReturn(JCTree.JCReturn tree) {
        Env<GenContext> targetEnv;
        int limit = this.code.nextreg;
        int tmpPos = this.code.pendingStatPos;
        if (tree.expr != null) {
            Items.Item r = genExpr(tree.expr, this.pt).load();
            if (hasFinally(this.env.enclMethod, this.env)) {
                r = makeTemp(this.pt);
                r.store();
            }
            targetEnv = unwind(this.env.enclMethod, this.env);
            this.code.pendingStatPos = tmpPos;
            r.load();
            this.code.emitop0(Code.truncate(Code.typecode(this.pt)) + ByteCodes.ireturn);
        } else {
            targetEnv = unwind(this.env.enclMethod, this.env);
            this.code.pendingStatPos = tmpPos;
            this.code.emitop0(ByteCodes.return_);
        }
        endFinalizerGaps(this.env, targetEnv);
        this.code.endScopes(limit);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitThrow(JCTree.JCThrow tree) {
        genExpr(tree.expr, tree.expr.type).load();
        this.code.emitop0(ByteCodes.athrow);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitApply(JCTree.JCMethodInvocation tree) {
        setTypeAnnotationPositions(tree.pos);
        Items.Item m = genExpr(tree.meth, this.methodType);
        Symbol.MethodSymbol msym = (Symbol.MethodSymbol) TreeInfo.symbol(tree.meth);
        genArgs(tree.args, msym.externalType(this.types).mo176getParameterTypes());
        if (!msym.isDynamic()) {
            this.code.statBegin(tree.pos);
        }
        this.result = m.invoke();
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitConditional(JCTree.JCConditional tree) {
        Code.Chain thenExit = null;
        Items.CondItem c = genCond(tree.cond, 8);
        Code.Chain elseChain = c.jumpFalse();
        if (!c.isFalse()) {
            this.code.resolve(c.trueJumps);
            int startpc = this.genCrt ? this.code.curCP() : 0;
            genExpr(tree.truepart, this.pt).load();
            this.code.state.forceStackTop(tree.type);
            if (this.genCrt) {
                this.code.crt.put(tree.truepart, 16, startpc, this.code.curCP());
            }
            thenExit = this.code.branch(ByteCodes.goto_);
        }
        if (elseChain != null) {
            this.code.resolve(elseChain);
            int startpc2 = this.genCrt ? this.code.curCP() : 0;
            genExpr(tree.falsepart, this.pt).load();
            this.code.state.forceStackTop(tree.type);
            if (this.genCrt) {
                this.code.crt.put(tree.falsepart, 16, startpc2, this.code.curCP());
            }
        }
        this.code.resolve(thenExit);
        this.result = this.items.makeStackItem(this.pt);
    }

    private void setTypeAnnotationPositions(int treePos) {
        Symbol.MethodSymbol meth = this.code.meth;
        boolean initOrClinit = this.code.meth.getKind() == ElementKind.CONSTRUCTOR || this.code.meth.getKind() == ElementKind.STATIC_INIT;
        for (Attribute.TypeCompound ta : meth.getRawTypeAttributes()) {
            if (ta.hasUnknownPosition()) {
                ta.tryFixPosition();
            }
            if (ta.position.matchesPos(treePos)) {
                ta.position.updatePosOffset(this.code.cp);
            }
        }
        if (!initOrClinit) {
            return;
        }
        for (Attribute.TypeCompound ta2 : meth.owner.getRawTypeAttributes()) {
            if (ta2.hasUnknownPosition()) {
                ta2.tryFixPosition();
            }
            if (ta2.position.matchesPos(treePos)) {
                ta2.position.updatePosOffset(this.code.cp);
            }
        }
        Symbol.ClassSymbol clazz = meth.enclClass();
        for (Symbol s : new FilteredMemberList(clazz.members())) {
            if (s.getKind().isField()) {
                for (Attribute.TypeCompound ta3 : s.getRawTypeAttributes()) {
                    if (ta3.hasUnknownPosition()) {
                        ta3.tryFixPosition();
                    }
                    if (ta3.position.matchesPos(treePos)) {
                        ta3.position.updatePosOffset(this.code.cp);
                    }
                }
            }
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewClass(JCTree.JCNewClass tree) {
        Assert.check(tree.encl == null && tree.def == null);
        setTypeAnnotationPositions(tree.pos);
        this.code.emitop2(ByteCodes.new_, makeRef(tree.pos(), tree.type));
        this.code.emitop0(89);
        genArgs(tree.args, tree.constructor.externalType(this.types).mo176getParameterTypes());
        this.items.makeMemberItem(tree.constructor, true).invoke();
        this.result = this.items.makeStackItem(tree.type);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewArray(JCTree.JCNewArray tree) {
        setTypeAnnotationPositions(tree.pos);
        if (tree.elems != null) {
            Type elemtype = this.types.elemtype(tree.type);
            loadIntConst(tree.elems.length());
            Items.Item arr = makeNewArray(tree.pos(), tree.type, 1);
            int i = 0;
            for (List list = tree.elems; list.nonEmpty(); list = list.tail) {
                arr.duplicate();
                loadIntConst(i);
                i++;
                genExpr((JCTree) list.head, elemtype).load();
                this.items.makeIndexedItem(elemtype).store();
            }
            this.result = arr;
            return;
        }
        for (List list2 = tree.dims; list2.nonEmpty(); list2 = list2.tail) {
            genExpr((JCTree) list2.head, this.syms.intType).load();
        }
        this.result = makeNewArray(tree.pos(), tree.type, tree.dims.length());
    }

    Items.Item makeNewArray(JCDiagnostic.DiagnosticPosition pos, Type type, int ndims) {
        Type elemtype = this.types.elemtype(type);
        if (this.types.dimensions(type) > 255) {
            this.log.error(pos, "limit.dimensions", new Object[0]);
            this.nerrs++;
        }
        int elemcode = Code.arraycode(elemtype);
        if (elemcode == 0 || (elemcode == 1 && ndims == 1)) {
            this.code.emitAnewarray(makeRef(pos, elemtype), type);
        } else if (elemcode == 1) {
            this.code.emitMultianewarray(ndims, makeRef(pos, type), type);
        } else {
            this.code.emitNewarray(elemcode, type);
        }
        return this.items.makeStackItem(type);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitParens(JCTree.JCParens tree) {
        this.result = genExpr(tree.expr, tree.expr.type);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssign(JCTree.JCAssign tree) {
        Items.Item l = genExpr(tree.lhs, tree.lhs.type);
        genExpr(tree.rhs, tree.lhs.type).load();
        this.result = this.items.makeAssignItem(l);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssignop(JCTree.JCAssignOp tree) {
        Items.Item l;
        Symbol.OperatorSymbol operator = (Symbol.OperatorSymbol) tree.operator;
        if (operator.opcode == 256) {
            makeStringBuffer(tree.pos());
            l = genExpr(tree.lhs, tree.lhs.type);
            if (l.width() > 0) {
                this.code.emitop0(((l.width() - 1) * 3) + 90);
            }
            l.load();
            appendString(tree.lhs);
            appendStrings(tree.rhs);
            bufferToString(tree.pos());
        } else {
            l = genExpr(tree.lhs, tree.lhs.type);
            if ((tree.hasTag(JCTree.Tag.PLUS_ASG) || tree.hasTag(JCTree.Tag.MINUS_ASG)) && (l instanceof Items.LocalItem) && tree.lhs.type.getTag().isSubRangeOf(TypeTag.INT) && tree.rhs.type.getTag().isSubRangeOf(TypeTag.INT) && tree.rhs.type.constValue() != null) {
                int ival = ((Number) tree.rhs.type.constValue()).intValue();
                if (tree.hasTag(JCTree.Tag.MINUS_ASG)) {
                    ival = -ival;
                }
                ((Items.LocalItem) l).incr(ival);
                this.result = l;
                return;
            }
            l.duplicate();
            l.coerce(operator.type.mo176getParameterTypes().head).load();
            completeBinop(tree.lhs, tree.rhs, operator).coerce(tree.lhs.type);
        }
        this.result = this.items.makeAssignItem(l);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitUnary(JCTree.JCUnary tree) {
        Symbol.OperatorSymbol operator = (Symbol.OperatorSymbol) tree.operator;
        if (tree.hasTag(JCTree.Tag.NOT)) {
            this.result = genCond((JCTree) tree.arg, false).negate();
            return;
        }
        Items.Item od = genExpr(tree.arg, operator.type.mo176getParameterTypes().head);
        switch (tree.getTag()) {
            case POSTINC:
            case POSTDEC:
                od.duplicate();
                if ((od instanceof Items.LocalItem) && (operator.opcode == 96 || operator.opcode == 100)) {
                    Items.Item res = od.load();
                    ((Items.LocalItem) od).incr(tree.hasTag(JCTree.Tag.POSTINC) ? 1 : -1);
                    this.result = res;
                } else {
                    Items.Item res2 = od.load();
                    od.stash(od.typecode);
                    this.code.emitop0(one(od.typecode));
                    this.code.emitop0(operator.opcode);
                    if (od.typecode != 0 && Code.truncate(od.typecode) == 0) {
                        this.code.emitop0((od.typecode + 145) - 5);
                    }
                    od.store();
                    this.result = res2;
                }
                break;
            case POS:
                this.result = od.load();
                break;
            case NEG:
                this.result = od.load();
                this.code.emitop0(operator.opcode);
                break;
            case COMPL:
                this.result = od.load();
                emitMinusOne(od.typecode);
                this.code.emitop0(operator.opcode);
                break;
            case PREINC:
            case PREDEC:
                od.duplicate();
                if ((od instanceof Items.LocalItem) && (operator.opcode == 96 || operator.opcode == 100)) {
                    ((Items.LocalItem) od).incr(tree.hasTag(JCTree.Tag.PREINC) ? 1 : -1);
                    this.result = od;
                } else {
                    od.load();
                    this.code.emitop0(one(od.typecode));
                    this.code.emitop0(operator.opcode);
                    if (od.typecode != 0 && Code.truncate(od.typecode) == 0) {
                        this.code.emitop0((od.typecode + 145) - 5);
                    }
                    this.result = this.items.makeAssignItem(od);
                }
                break;
            case NULLCHK:
                this.result = od.load();
                this.code.emitop0(89);
                genNullCheck(tree.pos());
                break;
            default:
                Assert.error();
                break;
        }
    }

    private void genNullCheck(JCDiagnostic.DiagnosticPosition pos) {
        callMethod(pos, this.syms.objectType, this.names.getClass, List.nil(), false);
        this.code.emitop0(87);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBinary(JCTree.JCBinary tree) {
        Symbol.OperatorSymbol operator = (Symbol.OperatorSymbol) tree.operator;
        if (operator.opcode == 256) {
            makeStringBuffer(tree.pos());
            appendStrings(tree);
            bufferToString(tree.pos());
            this.result = this.items.makeStackItem(this.syms.stringType);
            return;
        }
        if (tree.hasTag(JCTree.Tag.AND)) {
            Items.CondItem lcond = genCond(tree.lhs, 8);
            if (!lcond.isFalse()) {
                Code.Chain falseJumps = lcond.jumpFalse();
                this.code.resolve(lcond.trueJumps);
                Items.CondItem rcond = genCond(tree.rhs, 16);
                this.result = this.items.makeCondItem(rcond.opcode, rcond.trueJumps, Code.mergeChains(falseJumps, rcond.falseJumps));
                return;
            }
            this.result = lcond;
            return;
        }
        if (tree.hasTag(JCTree.Tag.OR)) {
            Items.CondItem lcond2 = genCond(tree.lhs, 8);
            if (!lcond2.isTrue()) {
                Code.Chain trueJumps = lcond2.jumpTrue();
                this.code.resolve(lcond2.falseJumps);
                Items.CondItem rcond2 = genCond(tree.rhs, 16);
                this.result = this.items.makeCondItem(rcond2.opcode, Code.mergeChains(trueJumps, rcond2.trueJumps), rcond2.falseJumps);
                return;
            }
            this.result = lcond2;
            return;
        }
        Items.Item od = genExpr(tree.lhs, operator.type.mo176getParameterTypes().head);
        od.load();
        this.result = completeBinop(tree.lhs, tree.rhs, operator);
    }

    void makeStringBuffer(JCDiagnostic.DiagnosticPosition pos) {
        this.code.emitop2(ByteCodes.new_, makeRef(pos, this.stringBufferType));
        this.code.emitop0(89);
        callMethod(pos, this.stringBufferType, this.names.init, List.nil(), false);
    }

    void appendString(JCTree tree) {
        Type t = tree.type.baseType();
        if (!t.isPrimitive() && t.tsym != this.syms.stringType.tsym) {
            t = this.syms.objectType;
        }
        this.items.makeMemberItem(getStringBufferAppend(tree, t), false).invoke();
    }

    Symbol getStringBufferAppend(JCTree tree, Type t) {
        Assert.checkNull(t.constValue());
        Symbol method = this.stringBufferAppend.get(t);
        if (method == null) {
            Symbol method2 = this.rs.resolveInternalMethod(tree.pos(), this.attrEnv, this.stringBufferType, this.names.append, List.of(t), null);
            this.stringBufferAppend.put(t, method2);
            return method2;
        }
        return method;
    }

    void appendStrings(JCTree tree) {
        JCTree tree2 = TreeInfo.skipParens(tree);
        if (tree2.hasTag(JCTree.Tag.PLUS) && tree2.type.constValue() == null) {
            JCTree.JCBinary op = (JCTree.JCBinary) tree2;
            if (op.operator.kind == 16 && ((Symbol.OperatorSymbol) op.operator).opcode == 256) {
                appendStrings(op.lhs);
                appendStrings(op.rhs);
                return;
            }
        }
        genExpr(tree2, tree2.type).load();
        appendString(tree2);
    }

    void bufferToString(JCDiagnostic.DiagnosticPosition pos) {
        callMethod(pos, this.stringBufferType, this.names.toString, List.nil(), false);
    }

    Items.Item completeBinop(JCTree lhs, JCTree rhs, Symbol.OperatorSymbol operator) {
        Type.MethodType optype = (Type.MethodType) operator.type;
        int opcode = operator.opcode;
        if (opcode >= 159 && opcode <= 164 && (rhs.type.constValue() instanceof Number) && ((Number) rhs.type.constValue()).intValue() == 0) {
            opcode -= 6;
        } else if (opcode >= 165 && opcode <= 166 && TreeInfo.isNull(rhs)) {
            opcode += 33;
        } else {
            Type rtype = operator.erasure(this.types).mo176getParameterTypes().tail.head;
            if (opcode >= 270 && opcode <= 275) {
                opcode -= 150;
                rtype = this.syms.intType;
            }
            genExpr(rhs, rtype).load();
            if (opcode >= 512) {
                this.code.emitop0(opcode >> 9);
                opcode &= 255;
            }
        }
        if ((opcode >= 153 && opcode <= 166) || opcode == 198 || opcode == 199) {
            return this.items.makeCondItem(opcode);
        }
        this.code.emitop0(opcode);
        return this.items.makeStackItem(optype.restype);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeCast(JCTree.JCTypeCast tree) {
        setTypeAnnotationPositions(tree.pos);
        this.result = genExpr(tree.expr, tree.clazz.type).load();
        if (!tree.clazz.type.isPrimitive() && this.types.asSuper(tree.expr.type, tree.clazz.type.tsym) == null) {
            this.code.emitop2(ByteCodes.checkcast, makeRef(tree.pos(), tree.clazz.type));
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitWildcard(JCTree.JCWildcard tree) {
        throw new AssertionError(getClass().getName());
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeTest(JCTree.JCInstanceOf tree) {
        setTypeAnnotationPositions(tree.pos);
        genExpr(tree.expr, tree.expr.type).load();
        this.code.emitop2(ByteCodes.instanceof_, makeRef(tree.pos(), tree.clazz.type));
        this.result = this.items.makeStackItem(this.syms.booleanType);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIndexed(JCTree.JCArrayAccess tree) {
        genExpr(tree.indexed, tree.indexed.type).load();
        genExpr(tree.index, this.syms.intType).load();
        this.result = this.items.makeIndexedItem(tree.type);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIdent(JCTree.JCIdent tree) {
        Items.Item res;
        Symbol sym = tree.sym;
        if (tree.name == this.names._this || tree.name == this.names._super) {
            if (tree.name == this.names._this) {
                res = this.items.makeThisItem();
            } else {
                res = this.items.makeSuperItem();
            }
            if (sym.kind == 16) {
                res.load();
                res = this.items.makeMemberItem(sym, true);
            }
            this.result = res;
            return;
        }
        if (sym.kind == 4 && sym.owner.kind == 16) {
            this.result = this.items.makeLocalItem((Symbol.VarSymbol) sym);
            return;
        }
        if (isInvokeDynamic(sym)) {
            this.result = this.items.makeDynamicItem(sym);
            return;
        }
        if ((sym.flags() & 8) != 0) {
            if (!isAccessSuper(this.env.enclMethod)) {
                sym = binaryQualifier(sym, this.env.enclClass.type);
            }
            this.result = this.items.makeStaticItem(sym);
        } else {
            this.items.makeThisItem().load();
            Symbol sym2 = binaryQualifier(sym, this.env.enclClass.type);
            this.result = this.items.makeMemberItem(sym2, (sym2.flags() & 2) != 0);
        }
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSelect(JCTree.JCFieldAccess tree) {
        Items.Item base;
        Symbol sym = tree.sym;
        if (tree.name == this.names._class) {
            Assert.check(this.target.hasClassLiterals());
            this.code.emitLdc(makeRef(tree.pos(), tree.selected.type));
            this.result = this.items.makeStackItem(this.pt);
            return;
        }
        Symbol ssym = TreeInfo.symbol(tree.selected);
        boolean z = true;
        boolean selectSuper = ssym != null && (ssym.kind == 2 || ssym.name == this.names._super);
        boolean accessSuper = isAccessSuper(this.env.enclMethod);
        if (selectSuper) {
            base = this.items.makeSuperItem();
        } else {
            base = genExpr(tree.selected, tree.selected.type);
        }
        if (sym.kind == 4 && ((Symbol.VarSymbol) sym).getConstValue() != null) {
            if ((sym.flags() & 8) != 0) {
                if (!selectSuper && (ssym == null || ssym.kind != 2)) {
                    base = base.load();
                }
                base.drop();
            } else {
                base.load();
                genNullCheck(tree.selected.pos());
            }
            this.result = this.items.makeImmediateItem(sym.type, ((Symbol.VarSymbol) sym).getConstValue());
            return;
        }
        if (isInvokeDynamic(sym)) {
            this.result = this.items.makeDynamicItem(sym);
            return;
        }
        Symbol sym2 = binaryQualifier(sym, tree.selected.type);
        if ((sym2.flags() & 8) != 0) {
            if (!selectSuper && (ssym == null || ssym.kind != 2)) {
                base = base.load();
            }
            base.drop();
            this.result = this.items.makeStaticItem(sym2);
            return;
        }
        base.load();
        if (sym2 == this.syms.lengthVar) {
            this.code.emitop0(ByteCodes.arraylength);
            this.result = this.items.makeStackItem(this.syms.intType);
            return;
        }
        Items items = this.items;
        if ((sym2.flags() & 2) == 0 && !selectSuper && !accessSuper) {
            z = false;
        }
        this.result = items.makeMemberItem(sym2, z);
    }

    public boolean isInvokeDynamic(Symbol sym) {
        return sym.kind == 16 && ((Symbol.MethodSymbol) sym).isDynamic();
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLiteral(JCTree.JCLiteral tree) {
        if (tree.type.hasTag(TypeTag.BOT)) {
            this.code.emitop0(1);
            if (this.types.dimensions(this.pt) > 1) {
                this.code.emitop2(ByteCodes.checkcast, makeRef(tree.pos(), this.pt));
                this.result = this.items.makeStackItem(this.pt);
                return;
            } else {
                this.result = this.items.makeStackItem(tree.type);
                return;
            }
        }
        this.result = this.items.makeImmediateItem(tree.type, tree.value);
    }

    @Override // com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLetExpr(JCTree.LetExpr tree) {
        int limit = this.code.nextreg;
        genStats(tree.defs, this.env);
        this.result = genExpr(tree.expr, tree.expr.type).load();
        this.code.endScopes(limit);
    }

    private void generateReferencesToPrunedTree(Symbol.ClassSymbol classSymbol, Pool pool) {
        List<JCTree> prunedInfo = this.lower.prunedTree.get(classSymbol);
        if (prunedInfo != null) {
            for (JCTree prunedTree : prunedInfo) {
                prunedTree.accept(this.classReferenceVisitor);
            }
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public boolean genClass(Env<AttrContext> env, JCTree.JCClassDecl jCClassDecl) {
        try {
            this.attrEnv = env;
            Symbol.ClassSymbol classSymbol = jCClassDecl.sym;
            this.toplevel = env.toplevel;
            this.endPosTable = this.toplevel.endPositions;
            if (this.generateIproxies && (classSymbol.flags() & 1536) == 1024 && !this.allowGenerics) {
                implementInterfaceMethods(classSymbol);
            }
            classSymbol.pool = this.pool;
            this.pool.reset();
            jCClassDecl.defs = normalizeDefs(jCClassDecl.defs, classSymbol);
            generateReferencesToPrunedTree(classSymbol, this.pool);
            Env<GenContext> env2 = new Env<>(jCClassDecl, new GenContext());
            env2.toplevel = env.toplevel;
            env2.enclClass = jCClassDecl;
            for (List list = jCClassDecl.defs; list.nonEmpty(); list = list.tail) {
                genDef((JCTree) list.head, env2);
            }
            if (this.pool.numEntries() > 65535) {
                this.log.error(jCClassDecl.pos(), "limit.pool", new Object[0]);
                this.nerrs++;
            }
            if (this.nerrs != 0) {
                for (List list2 = jCClassDecl.defs; list2.nonEmpty(); list2 = list2.tail) {
                    if (((JCTree) list2.head).hasTag(JCTree.Tag.METHODDEF)) {
                        ((JCTree.JCMethodDecl) list2.head).sym.code = null;
                    }
                }
            }
            jCClassDecl.defs = List.nil();
            return this.nerrs == 0;
        } finally {
            this.attrEnv = null;
            this.env = null;
            this.toplevel = null;
            this.endPosTable = null;
            this.nerrs = 0;
        }
    }

    abstract class GenFinalizer {
        abstract void gen();

        abstract void genLast();

        GenFinalizer() {
        }

        boolean hasFinalizer() {
            return true;
        }
    }

    static class GenContext {
        Code.Chain exit = null;
        Code.Chain cont = null;
        GenFinalizer finalize = null;
        boolean isSwitch = false;
        ListBuffer<Integer> gaps = null;

        GenContext() {
        }

        void addExit(Code.Chain c) {
            this.exit = Code.mergeChains(c, this.exit);
        }

        void addCont(Code.Chain c) {
            this.cont = Code.mergeChains(c, this.cont);
        }
    }
}
