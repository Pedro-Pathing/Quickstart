package com.sun.tools.javac.comp;

import com.android.tools.r8.DataResource;
import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.jvm.ByteCodes;
import com.sun.tools.javac.jvm.ClassReader;
import com.sun.tools.javac.jvm.ClassWriter;
import com.sun.tools.javac.jvm.Target;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.tree.EndPosTable;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.tree.TreeTranslator;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Convert;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import dk.sgjesse.r8api.DescriptorUtils;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;

/* JADX INFO: loaded from: classes.dex */
public class Lower extends TreeTranslator {
    private static final int ASSIGNcode = 2;
    private static final int DEREFcode = 0;
    private static final int FIRSTASGOPcode = 12;
    private static final int POSTDECcode = 10;
    private static final int POSTINCcode = 8;
    private static final int PREDECcode = 6;
    private static final int PREINCcode = 4;
    private List<Symbol.ClassSymbol> accessConstrTags;
    private Map<Symbol, Symbol.MethodSymbol> accessConstrs;
    private Map<Symbol, Integer> accessNums;
    private Map<Symbol, Symbol.MethodSymbol[]> accessSyms;
    private ListBuffer<Symbol> accessed;
    Map<Symbol, Symbol> actualSymbols;
    private boolean allowEnums;
    private Symbol.ClassSymbol assertionsDisabledClassCache;
    private Attr attr;
    Env<AttrContext> attrEnv;
    private ConstFold cfolder;
    private Check chk;
    private final Name classDollar;
    Map<Symbol.ClassSymbol, JCTree.JCClassDecl> classdefs;
    Symbol.ClassSymbol currentClass;
    JCTree.JCMethodDecl currentMethodDef;
    Symbol.MethodSymbol currentMethodSym;
    private boolean debugLower;
    private final Name dollarAssertionsDisabled;
    private JCTree.JCExpression enclOp;
    EndPosTable endPosTable;
    Map<Symbol.ClassSymbol, List<Symbol.VarSymbol>> freevarCache;
    private Log log;
    private TreeMaker make;
    private JCDiagnostic.DiagnosticPosition make_pos;
    private Names names;
    List<Symbol.VarSymbol> outerThisStack;
    JCTree.JCClassDecl outermostClassDef;
    JCTree outermostMemberDef;
    private Option.PkgInfo pkginfoOpt;
    Scope proxies;
    private ClassReader reader;
    private Resolve rs;
    private Source source;
    private Symtab syms;
    private Symbol.MethodSymbol systemArraycopyMethod;
    private Target target;
    ListBuffer<JCTree> translated;
    Scope twrVars;
    private final TypeEnvs typeEnvs;
    private Types types;
    private ClassWriter writer;
    protected static final Context.Key<Lower> lowerKey = new Context.Key<>();
    private static final int NCODES = accessCode(ByteCodes.lushrl) + 2;
    public Map<Symbol.ClassSymbol, List<JCTree>> prunedTree = new WeakHashMap();
    Map<Symbol, Symbol> lambdaTranslationMap = null;
    ClassMap classMap = new ClassMap();
    Map<Symbol.TypeSymbol, EnumMapping> enumSwitchMap = new LinkedHashMap();
    JCTree.Visitor conflictsChecker = new TreeScanner() { // from class: com.sun.tools.javac.comp.Lower.1
        Symbol.TypeSymbol currentClass;

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl that) {
            Lower.this.chk.checkConflicts(that.pos(), that.sym, this.currentClass);
            super.visitMethodDef(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl that) {
            if (that.sym.owner.kind == 2) {
                Lower.this.chk.checkConflicts(that.pos(), that.sym, this.currentClass);
            }
            super.visitVarDef(that);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl that) {
            Symbol.TypeSymbol prevCurrentClass = this.currentClass;
            this.currentClass = that.sym;
            try {
                super.visitClassDef(that);
            } finally {
                this.currentClass = prevCurrentClass;
            }
        }
    };

    interface TreeBuilder {
        JCTree build(JCTree jCTree);
    }

    public static Lower instance(Context context) {
        Lower instance = (Lower) context.get(lowerKey);
        if (instance == null) {
            return new Lower(context);
        }
        return instance;
    }

    protected Lower(Context context) {
        context.put(lowerKey, this);
        this.names = Names.instance(context);
        this.log = Log.instance(context);
        this.syms = Symtab.instance(context);
        this.rs = Resolve.instance(context);
        this.chk = Check.instance(context);
        this.attr = Attr.instance(context);
        this.make = TreeMaker.instance(context);
        this.writer = ClassWriter.instance(context);
        this.reader = ClassReader.instance(context);
        this.cfolder = ConstFold.instance(context);
        this.target = Target.instance(context);
        this.source = Source.instance(context);
        this.typeEnvs = TypeEnvs.instance(context);
        this.allowEnums = this.source.allowEnums();
        this.dollarAssertionsDisabled = this.names.fromString(this.target.syntheticNameChar() + "assertionsDisabled");
        this.classDollar = this.names.fromString("class" + this.target.syntheticNameChar());
        this.types = Types.instance(context);
        Options options = Options.instance(context);
        this.debugLower = options.isSet("debuglower");
        this.pkginfoOpt = Option.PkgInfo.get(options);
    }

    class ClassMap extends TreeScanner {
        ClassMap() {
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            Lower.this.classdefs.put(tree.sym, tree);
            super.visitClassDef(tree);
        }
    }

    JCTree.JCClassDecl classDef(Symbol.ClassSymbol c) {
        JCTree.JCClassDecl def = this.classdefs.get(c);
        if (def == null && this.outermostMemberDef != null) {
            this.classMap.scan(this.outermostMemberDef);
            def = this.classdefs.get(c);
        }
        if (def == null) {
            this.classMap.scan(this.outermostClassDef);
            return this.classdefs.get(c);
        }
        return def;
    }

    abstract class BasicFreeVarCollector extends TreeScanner {
        abstract void addFreeVars(Symbol.ClassSymbol classSymbol);

        abstract void visitSymbol(Symbol symbol);

        BasicFreeVarCollector() {
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            visitSymbol(tree.sym);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            Symbol.ClassSymbol c = (Symbol.ClassSymbol) tree.constructor.owner;
            addFreeVars(c);
            super.visitNewClass(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation tree) {
            if (TreeInfo.name(tree.meth) == Lower.this.names._super) {
                addFreeVars((Symbol.ClassSymbol) TreeInfo.symbol(tree.meth).owner);
            }
            super.visitApply(tree);
        }
    }

    class FreeVarCollector extends BasicFreeVarCollector {
        Symbol.ClassSymbol clazz;
        List<Symbol.VarSymbol> fvs;
        Symbol owner;

        FreeVarCollector(Symbol.ClassSymbol clazz) {
            super();
            this.clazz = clazz;
            this.owner = clazz.owner;
            this.fvs = List.nil();
        }

        private void addFreeVar(Symbol.VarSymbol v) {
            for (List list = this.fvs; list.nonEmpty(); list = list.tail) {
                if (list.head == v) {
                    return;
                }
            }
            List<Symbol.VarSymbol> l = this.fvs;
            this.fvs = l.prepend(v);
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.comp.Lower.BasicFreeVarCollector
        void addFreeVars(Symbol.ClassSymbol c) {
            List<Symbol.VarSymbol> fvs = Lower.this.freevarCache.get(c);
            if (fvs != null) {
                for (List list = fvs; list.nonEmpty(); list = list.tail) {
                    addFreeVar((Symbol.VarSymbol) list.head);
                }
            }
        }

        @Override // com.sun.tools.javac.comp.Lower.BasicFreeVarCollector
        void visitSymbol(Symbol _sym) {
            Symbol sym = _sym;
            if (sym.kind == 4 || sym.kind == 16) {
                while (sym != null && sym.owner != this.owner) {
                    sym = Lower.this.proxies.lookup(Lower.this.proxyName(sym.name)).sym;
                }
                if (sym != null && sym.owner == this.owner) {
                    Symbol.VarSymbol v = (Symbol.VarSymbol) sym;
                    if (v.getConstValue() == null) {
                        addFreeVar(v);
                        return;
                    }
                    return;
                }
                if (Lower.this.outerThisStack.head != null && Lower.this.outerThisStack.head != _sym) {
                    visitSymbol(Lower.this.outerThisStack.head);
                }
            }
        }

        @Override // com.sun.tools.javac.comp.Lower.BasicFreeVarCollector, com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            Symbol.ClassSymbol c = (Symbol.ClassSymbol) tree.constructor.owner;
            if (tree.encl == null && c.hasOuterInstance() && Lower.this.outerThisStack.head != null) {
                visitSymbol(Lower.this.outerThisStack.head);
            }
            super.visitNewClass(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            if ((tree.name == Lower.this.names._this || tree.name == Lower.this.names._super) && tree.selected.type.tsym != this.clazz && Lower.this.outerThisStack.head != null) {
                visitSymbol(Lower.this.outerThisStack.head);
            }
            super.visitSelect(tree);
        }

        @Override // com.sun.tools.javac.comp.Lower.BasicFreeVarCollector, com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation tree) {
            if (TreeInfo.name(tree.meth) == Lower.this.names._super) {
                Symbol constructor = TreeInfo.symbol(tree.meth);
                Symbol.ClassSymbol c = (Symbol.ClassSymbol) constructor.owner;
                if (c.hasOuterInstance() && !tree.meth.hasTag(JCTree.Tag.SELECT) && Lower.this.outerThisStack.head != null) {
                    visitSymbol(Lower.this.outerThisStack.head);
                }
            }
            super.visitApply(tree);
        }
    }

    Symbol.ClassSymbol ownerToCopyFreeVarsFrom(Symbol.ClassSymbol c) {
        if (!c.isLocal()) {
            return null;
        }
        Symbol currentOwner = c.owner;
        while ((currentOwner.owner.kind & 2) != 0 && currentOwner.isLocal()) {
            currentOwner = currentOwner.owner;
        }
        if ((currentOwner.owner.kind & 20) == 0 || !c.isSubClass(currentOwner, this.types)) {
            return null;
        }
        return (Symbol.ClassSymbol) currentOwner;
    }

    List<Symbol.VarSymbol> freevars(Symbol.ClassSymbol c) {
        List<Symbol.VarSymbol> fvs = this.freevarCache.get(c);
        if (fvs != null) {
            return fvs;
        }
        if ((c.owner.kind & 20) != 0) {
            FreeVarCollector collector = new FreeVarCollector(c);
            collector.scan(classDef(c));
            List<Symbol.VarSymbol> fvs2 = collector.fvs;
            this.freevarCache.put(c, fvs2);
            return fvs2;
        }
        Symbol.ClassSymbol owner = ownerToCopyFreeVarsFrom(c);
        if (owner != null) {
            List<Symbol.VarSymbol> fvs3 = this.freevarCache.get(owner);
            this.freevarCache.put(c, fvs3);
            return fvs3;
        }
        return List.nil();
    }

    EnumMapping mapForEnum(JCDiagnostic.DiagnosticPosition pos, Symbol.TypeSymbol enumClass) {
        EnumMapping map = this.enumSwitchMap.get(enumClass);
        if (map == null) {
            Map<Symbol.TypeSymbol, EnumMapping> map2 = this.enumSwitchMap;
            EnumMapping map3 = new EnumMapping(pos, enumClass);
            map2.put(enumClass, map3);
            return map3;
        }
        return map;
    }

    class EnumMapping {
        final Symbol.TypeSymbol forEnum;
        final Symbol.VarSymbol mapVar;
        JCDiagnostic.DiagnosticPosition pos;
        int next = 1;
        final Map<Symbol.VarSymbol, Integer> values = new LinkedHashMap();

        EnumMapping(JCDiagnostic.DiagnosticPosition pos, Symbol.TypeSymbol forEnum) {
            this.pos = null;
            this.forEnum = forEnum;
            this.pos = pos;
            Name varName = Lower.this.names.fromString(Lower.this.target.syntheticNameChar() + "SwitchMap" + Lower.this.target.syntheticNameChar() + Lower.this.writer.xClassName(forEnum.type).toString().replace(DataResource.SEPARATOR, DescriptorUtils.JAVA_PACKAGE_SEPARATOR).replace(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, Lower.this.target.syntheticNameChar()));
            Symbol.ClassSymbol outerCacheClass = Lower.this.outerCacheClass();
            this.mapVar = new Symbol.VarSymbol(4120L, varName, new Type.ArrayType(Lower.this.syms.intType, Lower.this.syms.arrayClass), outerCacheClass);
            Lower.this.enterSynthetic(pos, this.mapVar, outerCacheClass.members());
        }

        JCTree.JCLiteral forConstant(Symbol.VarSymbol v) {
            Integer result = this.values.get(v);
            if (result == null) {
                Map<Symbol.VarSymbol, Integer> map = this.values;
                int i = this.next;
                this.next = i + 1;
                Integer numValueOf = Integer.valueOf(i);
                result = numValueOf;
                map.put(v, numValueOf);
            }
            return Lower.this.make.Literal(result);
        }

        void translate() {
            Lower.this.make.at(this.pos.getStartPosition());
            JCTree.JCClassDecl owner = Lower.this.classDef((Symbol.ClassSymbol) this.mapVar.owner);
            Symbol.MethodSymbol valuesMethod = Lower.this.lookupMethod(this.pos, Lower.this.names.values, this.forEnum.type, List.nil());
            JCTree.JCExpression size = Lower.this.make.Select(Lower.this.make.App(Lower.this.make.QualIdent(valuesMethod)), Lower.this.syms.lengthVar);
            JCTree.JCExpression mapVarInit = Lower.this.make.NewArray(Lower.this.make.Type(Lower.this.syms.intType), List.of(size), null).setType((Type) new Type.ArrayType(Lower.this.syms.intType, Lower.this.syms.arrayClass));
            ListBuffer<JCTree.JCStatement> stmts = new ListBuffer<>();
            Symbol ordinalMethod = Lower.this.lookupMethod(this.pos, Lower.this.names.ordinal, this.forEnum.type, List.nil());
            List<JCTree.JCCatch> catcher = List.nil().prepend(Lower.this.make.Catch(Lower.this.make.VarDef(new Symbol.VarSymbol(8589934592L, Lower.this.names.ex, Lower.this.syms.noSuchFieldErrorType, Lower.this.syms.noSymbol), null), Lower.this.make.Block(0L, List.nil())));
            for (Map.Entry<Symbol.VarSymbol, Integer> e : this.values.entrySet()) {
                Symbol.VarSymbol enumerator = e.getKey();
                Integer mappedValue = e.getValue();
                Symbol.MethodSymbol valuesMethod2 = valuesMethod;
                JCTree.JCExpression size2 = size;
                JCTree.JCExpression assign = Lower.this.make.Assign(Lower.this.make.Indexed(this.mapVar, Lower.this.make.App(Lower.this.make.Select(Lower.this.make.QualIdent(enumerator), ordinalMethod))), Lower.this.make.Literal(mappedValue)).setType((Type) Lower.this.syms.intType);
                JCTree.JCStatement exec = Lower.this.make.Exec(assign);
                JCTree.JCStatement _try = Lower.this.make.Try(Lower.this.make.Block(0L, List.of(exec)), catcher, null);
                stmts.append(_try);
                valuesMethod = valuesMethod2;
                size = size2;
            }
            owner.defs = owner.defs.prepend(Lower.this.make.Block(8L, stmts.toList())).prepend(Lower.this.make.VarDef(this.mapVar, mapVarInit));
        }
    }

    TreeMaker make_at(JCDiagnostic.DiagnosticPosition pos) {
        this.make_pos = pos;
        return this.make.at(pos);
    }

    JCTree.JCExpression makeLit(Type type, Object value) {
        return this.make.Literal(type.getTag(), value).setType(type.constType(value));
    }

    JCTree.JCExpression makeNull() {
        return makeLit(this.syms.botType, null);
    }

    JCTree.JCNewClass makeNewClass(Type ctype, List<JCTree.JCExpression> args) {
        JCTree.JCNewClass tree = this.make.NewClass(null, null, this.make.QualIdent(ctype.tsym), args, null);
        tree.constructor = this.rs.resolveConstructor(this.make_pos, this.attrEnv, ctype, TreeInfo.types(args), List.nil());
        tree.type = ctype;
        return tree;
    }

    JCTree.JCUnary makeUnary(JCTree.Tag optag, JCTree.JCExpression arg) {
        JCTree.JCUnary tree = this.make.Unary(optag, arg);
        tree.operator = this.rs.resolveUnaryOperator(this.make_pos, optag, this.attrEnv, arg.type);
        tree.type = tree.operator.type.mo178getReturnType();
        return tree;
    }

    JCTree.JCBinary makeBinary(JCTree.Tag optag, JCTree.JCExpression lhs, JCTree.JCExpression rhs) {
        JCTree.JCBinary tree = this.make.Binary(optag, lhs, rhs);
        tree.operator = this.rs.resolveBinaryOperator(this.make_pos, optag, this.attrEnv, lhs.type, rhs.type);
        tree.type = tree.operator.type.mo178getReturnType();
        return tree;
    }

    JCTree.JCAssignOp makeAssignop(JCTree.Tag optag, JCTree lhs, JCTree rhs) {
        JCTree.JCAssignOp tree = this.make.Assignop(optag, lhs, rhs);
        tree.operator = this.rs.resolveBinaryOperator(this.make_pos, tree.getTag().noAssignOp(), this.attrEnv, lhs.type, rhs.type);
        tree.type = lhs.type;
        return tree;
    }

    JCTree.JCExpression makeString(JCTree.JCExpression tree) {
        if (!tree.type.isPrimitiveOrVoid()) {
            return tree;
        }
        Symbol valueOfSym = lookupMethod(tree.pos(), this.names.valueOf, this.syms.stringType, List.of(tree.type));
        return this.make.App(this.make.QualIdent(valueOfSym), List.of(tree));
    }

    JCTree.JCClassDecl makeEmptyClass(long flags, Symbol.ClassSymbol owner) {
        return makeEmptyClass(flags, owner, null, true);
    }

    JCTree.JCClassDecl makeEmptyClass(long flags, Symbol.ClassSymbol owner, Name flatname, boolean addToDefs) {
        Symbol.ClassSymbol c = this.reader.defineClass(this.names.empty, owner);
        if (flatname != null) {
            c.flatname = flatname;
        } else {
            c.flatname = this.chk.localClassName(c);
        }
        c.sourcefile = owner.sourcefile;
        c.completer = null;
        c.members_field = new Scope(c);
        c.flags_field = flags;
        Type.ClassType ctype = (Type.ClassType) c.type;
        ctype.supertype_field = this.syms.objectType;
        ctype.interfaces_field = List.nil();
        JCTree.JCClassDecl odef = classDef(owner);
        enterSynthetic(odef.pos(), c, owner.members());
        this.chk.compiled.put(c.flatname, c);
        JCTree.JCClassDecl cdef = this.make.ClassDef(this.make.Modifiers(flags), this.names.empty, List.nil(), null, List.nil(), List.nil());
        cdef.sym = c;
        cdef.type = c.type;
        if (addToDefs) {
            odef.defs = odef.defs.prepend(cdef);
        }
        return cdef;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void enterSynthetic(JCDiagnostic.DiagnosticPosition pos, Symbol sym, Scope s) {
        s.enter(sym);
    }

    private Name makeSyntheticName(Name name, Scope s) {
        do {
            name = name.append(this.target.syntheticNameChar(), this.names.empty);
        } while (lookupSynthetic(name, s) != null);
        return name;
    }

    void checkConflicts(List<JCTree> translatedTrees) {
        for (JCTree t : translatedTrees) {
            t.accept(this.conflictsChecker);
        }
    }

    private Symbol lookupSynthetic(Name name, Scope s) {
        Symbol sym = s.lookup(name).sym;
        if (sym == null || (sym.flags() & 4096) == 0) {
            return null;
        }
        return sym;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Symbol.MethodSymbol lookupMethod(JCDiagnostic.DiagnosticPosition pos, Name name, Type qual, List<Type> args) {
        return this.rs.resolveInternalMethod(pos, this.attrEnv, qual, name, args, List.nil());
    }

    private Symbol.MethodSymbol lookupConstructor(JCDiagnostic.DiagnosticPosition pos, Type qual, List<Type> args) {
        return this.rs.resolveInternalConstructor(pos, this.attrEnv, qual, args, null);
    }

    private Symbol.VarSymbol lookupField(JCDiagnostic.DiagnosticPosition pos, Type qual, Name name) {
        return this.rs.resolveInternalField(pos, this.attrEnv, qual, name);
    }

    /* JADX WARN: Multi-variable type inference failed */
    private void checkAccessConstructorTags() {
        for (List list = this.accessConstrTags; list.nonEmpty(); list = list.tail) {
            Symbol.ClassSymbol c = (Symbol.ClassSymbol) list.head;
            if (!isTranslatedClassAvailable(c)) {
                JCTree.JCClassDecl cdec = makeEmptyClass(4104L, c.outermostClass(), c.flatname, false);
                swapAccessConstructorTag(c, cdec.sym);
                this.translated.append(cdec);
            }
        }
    }

    private boolean isTranslatedClassAvailable(Symbol.ClassSymbol c) {
        for (JCTree tree : this.translated) {
            if (tree.hasTag(JCTree.Tag.CLASSDEF) && ((JCTree.JCClassDecl) tree).sym == c) {
                return true;
            }
        }
        return false;
    }

    void swapAccessConstructorTag(Symbol.ClassSymbol oldCTag, Symbol.ClassSymbol newCTag) {
        for (Symbol.MethodSymbol methodSymbol : this.accessConstrs.values()) {
            Assert.check(methodSymbol.type.hasTag(TypeTag.METHOD));
            Type.MethodType oldMethodType = (Type.MethodType) methodSymbol.type;
            if (oldMethodType.argtypes.head.tsym == oldCTag) {
                methodSymbol.type = this.types.createMethodTypeWithParameters(oldMethodType, oldMethodType.mo176getParameterTypes().tail.prepend(newCTag.erasure(this.types)));
            }
        }
    }

    private static int accessCode(int bytecode) {
        if (96 <= bytecode && bytecode <= 131) {
            return ((bytecode - 96) * 2) + 12;
        }
        if (bytecode == 256) {
            return 84;
        }
        if (270 <= bytecode && bytecode <= 275) {
            return (((((bytecode - 270) + 131) + 2) - 96) * 2) + 12;
        }
        return -1;
    }

    private static int accessCode(JCTree tree, JCTree enclOp) {
        if (enclOp == null) {
            return 0;
        }
        if (enclOp.hasTag(JCTree.Tag.ASSIGN) && tree == TreeInfo.skipParens(((JCTree.JCAssign) enclOp).lhs)) {
            return 2;
        }
        if (enclOp.getTag().isIncOrDecUnaryOp() && tree == TreeInfo.skipParens(((JCTree.JCUnary) enclOp).arg)) {
            return mapTagToUnaryOpCode(enclOp.getTag());
        }
        if (!enclOp.getTag().isAssignop() || tree != TreeInfo.skipParens(((JCTree.JCAssignOp) enclOp).lhs)) {
            return 0;
        }
        return accessCode(((Symbol.OperatorSymbol) ((JCTree.JCAssignOp) enclOp).operator).opcode);
    }

    private Symbol.OperatorSymbol binaryAccessOperator(int acode) {
        for (Scope.Entry e = this.syms.predefClass.members().elems; e != null; e = e.sibling) {
            if (e.sym instanceof Symbol.OperatorSymbol) {
                Symbol.OperatorSymbol op = (Symbol.OperatorSymbol) e.sym;
                if (accessCode(op.opcode) == acode) {
                    return op;
                }
            }
        }
        return null;
    }

    private static JCTree.Tag treeTag(Symbol.OperatorSymbol operator) {
        switch (operator.opcode) {
            case 96:
            case 97:
            case 98:
            case 99:
            case 256:
                return JCTree.Tag.PLUS_ASG;
            case 100:
            case 101:
            case 102:
            case 103:
                return JCTree.Tag.MINUS_ASG;
            case 104:
            case 105:
            case 106:
            case 107:
                return JCTree.Tag.MUL_ASG;
            case 108:
            case 109:
            case 110:
            case 111:
                return JCTree.Tag.DIV_ASG;
            case 112:
            case ByteCodes.lmod /* 113 */:
            case ByteCodes.fmod /* 114 */:
            case 115:
                return JCTree.Tag.MOD_ASG;
            case 120:
            case 121:
            case ByteCodes.ishll /* 270 */:
            case ByteCodes.lshll /* 271 */:
                return JCTree.Tag.SL_ASG;
            case 122:
            case 123:
            case 272:
            case ByteCodes.lshrl /* 273 */:
                return JCTree.Tag.SR_ASG;
            case 124:
            case 125:
            case ByteCodes.iushrl /* 274 */:
            case ByteCodes.lushrl /* 275 */:
                return JCTree.Tag.USR_ASG;
            case 126:
            case 127:
                return JCTree.Tag.BITAND_ASG;
            case 128:
            case 129:
                return JCTree.Tag.BITOR_ASG;
            case 130:
            case 131:
                return JCTree.Tag.BITXOR_ASG;
            default:
                throw new AssertionError();
        }
    }

    Name accessName(int anum, int acode) {
        return this.names.fromString("access" + this.target.syntheticNameChar() + anum + (acode / 10) + (acode % 10));
    }

    Symbol.MethodSymbol accessSymbol(Symbol sym, JCTree tree, JCTree enclOp, boolean protAccess, boolean refSuper) {
        Symbol vsym;
        Integer anum;
        int acode;
        List<Type> argtypes;
        Type restype;
        List<Type> thrown;
        List<Type> argtypes2;
        List<Type> argtypes3;
        Symbol.ClassSymbol accOwner = (refSuper && protAccess) ? (Symbol.ClassSymbol) ((JCTree.JCFieldAccess) tree).selected.type.tsym : accessClass(sym, protAccess, tree);
        if (sym.owner == accOwner) {
            vsym = sym;
        } else {
            Symbol vsym2 = sym.clone(accOwner);
            this.actualSymbols.put(vsym2, sym);
            vsym = vsym2;
        }
        Integer anum2 = this.accessNums.get(vsym);
        if (anum2 != null) {
            anum = anum2;
        } else {
            Integer anum3 = Integer.valueOf(this.accessed.length());
            this.accessNums.put(vsym, anum3);
            this.accessSyms.put(vsym, new Symbol.MethodSymbol[NCODES]);
            this.accessed.append(vsym);
            anum = anum3;
        }
        switch (vsym.kind) {
            case 4:
                acode = accessCode(tree, enclOp);
                if (acode >= 12) {
                    Symbol.OperatorSymbol operator = binaryAccessOperator(acode);
                    if (operator.opcode == 256) {
                        argtypes2 = List.of(this.syms.objectType);
                    } else {
                        argtypes2 = operator.type.mo176getParameterTypes().tail;
                    }
                    argtypes = argtypes2;
                } else if (acode == 2) {
                    argtypes = List.of(vsym.erasure(this.types));
                } else {
                    argtypes = List.nil();
                }
                Type restype2 = vsym.erasure(this.types);
                List<Type> thrown2 = List.nil();
                restype = restype2;
                thrown = thrown2;
                break;
            case 16:
                acode = 0;
                argtypes = vsym.erasure(this.types).mo176getParameterTypes();
                Type restype3 = vsym.erasure(this.types).mo178getReturnType();
                List<Type> thrown3 = vsym.type.mo179getThrownTypes();
                restype = restype3;
                thrown = thrown3;
                break;
            default:
                throw new AssertionError();
        }
        int acode2 = (protAccess && refSuper) ? acode + 1 : acode;
        if ((vsym.flags() & 8) == 0) {
            argtypes3 = argtypes.prepend(vsym.owner.erasure(this.types));
        } else {
            argtypes3 = argtypes;
        }
        Symbol.MethodSymbol[] accessors = this.accessSyms.get(vsym);
        Symbol.MethodSymbol accessor = accessors[acode2];
        if (accessor == null) {
            Symbol.MethodSymbol accessor2 = new Symbol.MethodSymbol(4104L, accessName(anum.intValue(), acode2), new Type.MethodType(argtypes3, restype, thrown, this.syms.methodClass), accOwner);
            enterSynthetic(tree.pos(), accessor2, accOwner.members());
            accessors[acode2] = accessor2;
            return accessor2;
        }
        return accessor;
    }

    JCTree.JCExpression accessBase(JCDiagnostic.DiagnosticPosition pos, Symbol sym) {
        if ((sym.flags() & 8) != 0) {
            return access(this.make.at(pos.getStartPosition()).QualIdent(sym.owner));
        }
        return makeOwnerThis(pos, sym, true);
    }

    boolean needsPrivateAccess(Symbol sym) {
        if ((sym.flags() & 2) == 0 || sym.owner == this.currentClass) {
            return false;
        }
        if (sym.name == this.names.init && sym.owner.isLocal()) {
            sym.flags_field &= -3;
            return false;
        }
        return true;
    }

    boolean needsProtectedAccess(Symbol sym, JCTree tree) {
        if ((sym.flags() & 4) == 0 || sym.owner.owner == this.currentClass.owner || sym.packge() == this.currentClass.packge()) {
            return false;
        }
        if (!this.currentClass.isSubClass(sym.owner, this.types)) {
            return true;
        }
        if ((sym.flags() & 8) == 0 && tree.hasTag(JCTree.Tag.SELECT) && TreeInfo.name(((JCTree.JCFieldAccess) tree).selected) != this.names._super) {
            return !((JCTree.JCFieldAccess) tree).selected.type.tsym.isSubClass(this.currentClass, this.types);
        }
        return false;
    }

    Symbol.ClassSymbol accessClass(Symbol sym, boolean protAccess, JCTree tree) {
        if (protAccess) {
            Symbol.ClassSymbol c = this.currentClass;
            if (tree.hasTag(JCTree.Tag.SELECT) && (sym.flags() & 8) == 0) {
                Symbol qualifier = ((JCTree.JCFieldAccess) tree).selected.type.tsym;
                while (!qualifier.isSubClass(c, this.types)) {
                    c = c.owner.enclClass();
                }
                return c;
            }
            while (!c.isSubClass(sym.owner, this.types)) {
                c = c.owner.enclClass();
            }
            return c;
        }
        Symbol qualifier2 = sym.owner;
        return qualifier2.enclClass();
    }

    private void addPrunedInfo(JCTree tree) {
        List<JCTree> infoList = this.prunedTree.get(this.currentClass);
        this.prunedTree.put(this.currentClass, infoList == null ? List.of(tree) : infoList.prepend(tree));
    }

    /* JADX WARN: Code restructure failed: missing block: B:100:0x01a0, code lost:
    
        r8 = r15.make.at(r8.pos).Ident(r7);
        ((com.sun.tools.javac.tree.JCTree.JCIdent) r8).name = r1;
     */
    /* JADX WARN: Code restructure failed: missing block: B:101:0x01b2, code lost:
    
        ((com.sun.tools.javac.tree.JCTree.JCFieldAccess) r8).selected = r0;
        ((com.sun.tools.javac.tree.JCTree.JCFieldAccess) r8).name = r1;
     */
    /* JADX WARN: Code restructure failed: missing block: B:103:0x01bd, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:116:?, code lost:
    
        return r15.make.at(r8.pos).Ident(r1);
     */
    /* JADX WARN: Code restructure failed: missing block: B:117:?, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:118:?, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:119:?, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:120:?, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:121:?, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:122:?, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:123:?, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:124:?, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:125:?, code lost:
    
        return r8;
     */
    /* JADX WARN: Code restructure failed: missing block: B:20:0x0068, code lost:
    
        if (r8.hasTag(com.sun.tools.javac.tree.JCTree.Tag.SELECT) == false) goto L22;
     */
    /* JADX WARN: Code restructure failed: missing block: B:21:0x006a, code lost:
    
        r0 = ((com.sun.tools.javac.tree.JCTree.JCFieldAccess) r8).selected;
     */
    /* JADX WARN: Code restructure failed: missing block: B:22:0x0070, code lost:
    
        r0 = null;
     */
    /* JADX WARN: Code restructure failed: missing block: B:24:0x0073, code lost:
    
        switch(r7.kind) {
            case 2: goto L83;
            case 4: goto L26;
            case 16: goto L26;
            default: goto L118;
        };
     */
    /* JADX WARN: Code restructure failed: missing block: B:27:0x007d, code lost:
    
        if (r7.owner.kind != 2) goto L75;
     */
    /* JADX WARN: Code restructure failed: missing block: B:28:0x007f, code lost:
    
        if (r19 == false) goto L31;
     */
    /* JADX WARN: Code restructure failed: missing block: B:30:0x0085, code lost:
    
        if (needsPrivateAccess(r7) == false) goto L33;
     */
    /* JADX WARN: Code restructure failed: missing block: B:32:0x008b, code lost:
    
        if (needsProtectedAccess(r7, r8) == false) goto L34;
     */
    /* JADX WARN: Code restructure failed: missing block: B:33:0x008d, code lost:
    
        r3 = true;
     */
    /* JADX WARN: Code restructure failed: missing block: B:34:0x008f, code lost:
    
        r3 = false;
     */
    /* JADX WARN: Code restructure failed: missing block: B:35:0x0090, code lost:
    
        r10 = r3;
     */
    /* JADX WARN: Code restructure failed: missing block: B:36:0x0091, code lost:
    
        if (r10 != false) goto L41;
     */
    /* JADX WARN: Code restructure failed: missing block: B:38:0x0097, code lost:
    
        if (needsPrivateAccess(r7) == false) goto L40;
     */
    /* JADX WARN: Code restructure failed: missing block: B:40:0x009a, code lost:
    
        r3 = false;
     */
    /* JADX WARN: Code restructure failed: missing block: B:41:0x009c, code lost:
    
        r3 = true;
     */
    /* JADX WARN: Code restructure failed: missing block: B:42:0x009d, code lost:
    
        r11 = r3;
     */
    /* JADX WARN: Code restructure failed: missing block: B:43:0x009e, code lost:
    
        if (r0 != null) goto L50;
     */
    /* JADX WARN: Code restructure failed: missing block: B:45:0x00a6, code lost:
    
        if (r7.owner == r15.syms.predefClass) goto L50;
     */
    /* JADX WARN: Code restructure failed: missing block: B:47:0x00b0, code lost:
    
        if (r7.isMemberOf(r15.currentClass, r15.types) != false) goto L50;
     */
    /* JADX WARN: Code restructure failed: missing block: B:48:0x00b2, code lost:
    
        r5 = true;
     */
    /* JADX WARN: Code restructure failed: missing block: B:50:0x00b5, code lost:
    
        r12 = r5;
     */
    /* JADX WARN: Code restructure failed: missing block: B:51:0x00b6, code lost:
    
        if (r11 != false) goto L53;
     */
    /* JADX WARN: Code restructure failed: missing block: B:52:0x00b8, code lost:
    
        if (r12 == false) goto L119;
     */
    /* JADX WARN: Code restructure failed: missing block: B:53:0x00ba, code lost:
    
        r15.make.at(r8.pos);
     */
    /* JADX WARN: Code restructure failed: missing block: B:54:0x00c3, code lost:
    
        if (r7.kind != 4) goto L59;
     */
    /* JADX WARN: Code restructure failed: missing block: B:55:0x00c5, code lost:
    
        r3 = ((com.sun.tools.javac.code.Symbol.VarSymbol) r7).getConstValue();
     */
    /* JADX WARN: Code restructure failed: missing block: B:56:0x00cc, code lost:
    
        if (r3 == null) goto L59;
     */
    /* JADX WARN: Code restructure failed: missing block: B:57:0x00ce, code lost:
    
        addPrunedInfo(r8);
     */
    /* JADX WARN: Code restructure failed: missing block: B:58:0x00d7, code lost:
    
        return makeLit(r7.type, r3);
     */
    /* JADX WARN: Code restructure failed: missing block: B:59:0x00d8, code lost:
    
        if (r11 == false) goto L72;
     */
    /* JADX WARN: Code restructure failed: missing block: B:60:0x00da, code lost:
    
        r3 = com.sun.tools.javac.util.List.nil();
     */
    /* JADX WARN: Code restructure failed: missing block: B:61:0x00e7, code lost:
    
        if ((r7.flags() & 8) != 0) goto L65;
     */
    /* JADX WARN: Code restructure failed: missing block: B:62:0x00e9, code lost:
    
        if (r0 != null) goto L64;
     */
    /* JADX WARN: Code restructure failed: missing block: B:63:0x00eb, code lost:
    
        r0 = makeOwnerThis(r8.pos(), r7, true);
     */
    /* JADX WARN: Code restructure failed: missing block: B:64:0x00f3, code lost:
    
        r9 = null;
        r13 = r3.prepend(r0);
     */
    /* JADX WARN: Code restructure failed: missing block: B:65:0x00fb, code lost:
    
        r9 = r0;
        r13 = r3;
     */
    /* JADX WARN: Code restructure failed: missing block: B:66:0x00fd, code lost:
    
        r0 = accessSymbol(r7, r8, r18, r10, r19);
        r1 = r15.make;
     */
    /* JADX WARN: Code restructure failed: missing block: B:67:0x010b, code lost:
    
        if (r9 == null) goto L69;
     */
    /* JADX WARN: Code restructure failed: missing block: B:68:0x010d, code lost:
    
        r2 = r9;
     */
    /* JADX WARN: Code restructure failed: missing block: B:69:0x010f, code lost:
    
        r2 = r15.make.QualIdent(r0.owner);
     */
    /* JADX WARN: Code restructure failed: missing block: B:70:0x0117, code lost:
    
        r1 = r1.Select(r2, r0);
     */
    /* JADX WARN: Code restructure failed: missing block: B:71:0x0121, code lost:
    
        return r15.make.App(r1, r13);
     */
    /* JADX WARN: Code restructure failed: missing block: B:72:0x0122, code lost:
    
        if (r12 == false) goto L120;
     */
    /* JADX WARN: Code restructure failed: missing block: B:74:0x013e, code lost:
    
        return r15.make.at(r8.pos).Select(accessBase(r8.pos(), r7), r7).setType(r8.type);
     */
    /* JADX WARN: Code restructure failed: missing block: B:76:0x0143, code lost:
    
        if (r7.owner.kind != 16) goto L121;
     */
    /* JADX WARN: Code restructure failed: missing block: B:78:0x0147, code lost:
    
        if (r15.lambdaTranslationMap == null) goto L122;
     */
    /* JADX WARN: Code restructure failed: missing block: B:79:0x0149, code lost:
    
        r1 = r15.lambdaTranslationMap.get(r7);
     */
    /* JADX WARN: Code restructure failed: missing block: B:80:0x0151, code lost:
    
        if (r1 == null) goto L103;
     */
    /* JADX WARN: Code restructure failed: missing block: B:84:0x0165, code lost:
    
        if (r7.owner.kind == 1) goto L117;
     */
    /* JADX WARN: Code restructure failed: missing block: B:85:0x0167, code lost:
    
        r1 = com.sun.tools.javac.util.Convert.shortName(r7.flatName());
     */
    /* JADX WARN: Code restructure failed: missing block: B:86:0x016f, code lost:
    
        if (r0 == null) goto L111;
     */
    /* JADX WARN: Code restructure failed: missing block: B:88:0x0175, code lost:
    
        if (com.sun.tools.javac.tree.TreeInfo.symbol(r0) == null) goto L112;
     */
    /* JADX WARN: Code restructure failed: missing block: B:90:0x017d, code lost:
    
        if (com.sun.tools.javac.tree.TreeInfo.symbol(r0).kind == 1) goto L113;
     */
    /* JADX WARN: Code restructure failed: missing block: B:92:0x0185, code lost:
    
        if (r0.hasTag(com.sun.tools.javac.tree.JCTree.Tag.SELECT) == false) goto L94;
     */
    /* JADX WARN: Code restructure failed: missing block: B:93:0x0187, code lost:
    
        r2 = ((com.sun.tools.javac.tree.JCTree.JCFieldAccess) r0).selected;
     */
    /* JADX WARN: Code restructure failed: missing block: B:94:0x018d, code lost:
    
        r2 = null;
     */
    /* JADX WARN: Code restructure failed: missing block: B:95:0x018e, code lost:
    
        r0 = r2;
     */
    /* JADX WARN: Code restructure failed: missing block: B:97:0x0196, code lost:
    
        if (r8.hasTag(com.sun.tools.javac.tree.JCTree.Tag.IDENT) == false) goto L99;
     */
    /* JADX WARN: Code restructure failed: missing block: B:98:0x0198, code lost:
    
        ((com.sun.tools.javac.tree.JCTree.JCIdent) r8).name = r1;
     */
    /* JADX WARN: Code restructure failed: missing block: B:99:0x019e, code lost:
    
        if (r0 != null) goto L101;
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    com.sun.tools.javac.tree.JCTree.JCExpression access(com.sun.tools.javac.code.Symbol r16, com.sun.tools.javac.tree.JCTree.JCExpression r17, com.sun.tools.javac.tree.JCTree.JCExpression r18, boolean r19) {
        /*
            Method dump skipped, instruction units count: 460
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Lower.access(com.sun.tools.javac.code.Symbol, com.sun.tools.javac.tree.JCTree$JCExpression, com.sun.tools.javac.tree.JCTree$JCExpression, boolean):com.sun.tools.javac.tree.JCTree$JCExpression");
    }

    JCTree.JCExpression access(JCTree.JCExpression tree) {
        Symbol sym = TreeInfo.symbol(tree);
        return sym == null ? tree : access(sym, tree, null, false);
    }

    Symbol accessConstructor(JCDiagnostic.DiagnosticPosition pos, Symbol constr) {
        List<Type> argtypes;
        if (needsPrivateAccess(constr)) {
            Symbol.ClassSymbol accOwner = constr.owner.enclClass();
            Symbol.MethodSymbol aconstr = this.accessConstrs.get(constr);
            if (aconstr == null) {
                List<Type> argtypes2 = constr.type.mo176getParameterTypes();
                if ((accOwner.flags_field & 16384) == 0) {
                    argtypes = argtypes2;
                } else {
                    argtypes = argtypes2.prepend(this.syms.intType).prepend(this.syms.stringType);
                }
                Symbol.MethodSymbol aconstr2 = new Symbol.MethodSymbol(4096L, this.names.init, new Type.MethodType(argtypes.append(accessConstructorTag().erasure(this.types)), constr.type.mo178getReturnType(), constr.type.mo179getThrownTypes(), this.syms.methodClass), accOwner);
                enterSynthetic(pos, aconstr2, accOwner.members());
                this.accessConstrs.put(constr, aconstr2);
                this.accessed.append(constr);
                return aconstr2;
            }
            return aconstr;
        }
        return constr;
    }

    Symbol.ClassSymbol accessConstructorTag() {
        Symbol.ClassSymbol topClass = this.currentClass.outermostClass();
        Name flatname = this.names.fromString("" + ((Object) topClass.getQualifiedName()) + this.target.syntheticNameChar() + "1");
        Symbol.ClassSymbol ctag = this.chk.compiled.get(flatname);
        if (ctag == null) {
            ctag = makeEmptyClass(4104L, topClass).sym;
        }
        this.accessConstrTags = this.accessConstrTags.prepend(ctag);
        return ctag;
    }

    void makeAccessible(Symbol sym) {
        JCTree.JCClassDecl cdef = classDef(sym.owner.enclClass());
        if (cdef == null) {
            Assert.error("class def not found: " + sym + " in " + sym.owner);
        }
        if (sym.name == this.names.init) {
            cdef.defs = cdef.defs.prepend(accessConstructorDef(cdef.pos, sym, this.accessConstrs.get(sym)));
            return;
        }
        Symbol.MethodSymbol[] accessors = this.accessSyms.get(sym);
        for (int i = 0; i < NCODES; i++) {
            if (accessors[i] != null) {
                cdef.defs = cdef.defs.prepend(accessDef(cdef.pos, sym, accessors[i], i));
            }
        }
    }

    private static JCTree.Tag mapUnaryOpCodeToTag(int unaryOpCode) {
        switch (unaryOpCode) {
            case 4:
                return JCTree.Tag.PREINC;
            case 5:
            case 7:
            case 9:
            default:
                return JCTree.Tag.NO_TAG;
            case 6:
                return JCTree.Tag.PREDEC;
            case 8:
                return JCTree.Tag.POSTINC;
            case 10:
                return JCTree.Tag.POSTDEC;
        }
    }

    private static int mapTagToUnaryOpCode(JCTree.Tag tag) {
        switch (tag) {
            case PREINC:
                return 4;
            case PREDEC:
                return 6;
            case POSTINC:
                return 8;
            case POSTDEC:
                return 10;
            default:
                return -1;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r5v4, types: [A, com.sun.tools.javac.tree.JCTree$JCExpression] */
    JCTree accessDef(int pos, Symbol vsym, Symbol.MethodSymbol accessor, int acode) {
        JCTree.JCExpression site;
        List<JCTree.JCExpression> args;
        JCTree.JCStatement stat;
        JCTree.JCExpression expr;
        this.currentClass = vsym.owner.enclClass();
        this.make.at(pos);
        JCTree.JCMethodDecl md = this.make.MethodDef(accessor, null);
        Symbol sym = this.actualSymbols.get(vsym);
        if (sym == null) {
            sym = vsym;
        }
        if ((sym.flags() & 8) != 0) {
            site = this.make.Ident(sym);
            args = this.make.Idents(md.params);
        } else {
            JCTree.JCExpression site2 = this.make.Ident(md.params.head);
            if (acode % 2 != 0) {
                site2.setType(this.types.erasure(this.types.supertype(vsym.owner.enclClass().type)));
            }
            JCTree.JCExpression ref = this.make.Select(site2, sym);
            site = ref;
            args = this.make.Idents(md.params.tail);
        }
        if (sym.kind == 4) {
            int acode1 = acode - (acode & 1);
            switch (acode1) {
                case 0:
                    expr = site;
                    break;
                case 1:
                case 3:
                case 5:
                case 7:
                case 9:
                default:
                    expr = this.make.Assignop(treeTag(binaryAccessOperator(acode1)), site, args.head);
                    ((JCTree.JCAssignOp) expr).operator = binaryAccessOperator(acode1);
                    break;
                case 2:
                    expr = this.make.Assign(site, args.head);
                    break;
                case 4:
                case 6:
                case 8:
                case 10:
                    expr = makeUnary(mapUnaryOpCodeToTag(acode1), site);
                    break;
            }
            stat = this.make.Return(expr.setType(sym.type));
        } else {
            stat = this.make.Call(this.make.App(site, args));
        }
        md.body = this.make.Block(0L, List.of(stat));
        for (List list = md.params; list.nonEmpty(); list = list.tail) {
            ((JCTree.JCVariableDecl) list.head).vartype = access(((JCTree.JCVariableDecl) list.head).vartype);
        }
        md.restype = access(md.restype);
        for (List list2 = md.thrown; list2.nonEmpty(); list2 = list2.tail) {
            list2.head = access((JCTree.JCExpression) list2.head);
        }
        return md;
    }

    JCTree accessConstructorDef(int pos, Symbol constr, Symbol.MethodSymbol accessor) {
        this.make.at(pos);
        JCTree.JCMethodDecl md = this.make.MethodDef(accessor, accessor.externalType(this.types), null);
        JCTree.JCIdent callee = this.make.Ident(this.names._this);
        callee.sym = constr;
        callee.type = constr.type;
        md.body = this.make.Block(0L, List.of(this.make.Call(this.make.App(callee, this.make.Idents(md.params.reverse().tail.reverse())))));
        return md;
    }

    Name proxyName(Name name) {
        return this.names.fromString("val" + this.target.syntheticNameChar() + ((Object) name));
    }

    List<JCTree.JCVariableDecl> freevarDefs(int pos, List<Symbol.VarSymbol> freevars, Symbol owner) {
        return freevarDefs(pos, freevars, owner, 0L);
    }

    /* JADX WARN: Multi-variable type inference failed */
    List<JCTree.JCVariableDecl> freevarDefs(int pos, List<Symbol.VarSymbol> freevars, Symbol owner, long additionalFlags) {
        long flags = additionalFlags | 4112;
        if (owner.kind == 2 && this.target.usePrivateSyntheticFields()) {
            flags |= 2;
        }
        List<JCTree.JCVariableDecl> defs = List.nil();
        List<JCTree.JCVariableDecl> defs2 = defs;
        for (List list = freevars; list.nonEmpty(); list = list.tail) {
            Symbol.VarSymbol v = (Symbol.VarSymbol) list.head;
            Symbol.VarSymbol proxy = new Symbol.VarSymbol(flags, proxyName(v.name), v.erasure(this.types), owner);
            this.proxies.enter(proxy);
            JCTree.JCVariableDecl vd = this.make.at(pos).VarDef(proxy, null);
            vd.vartype = access(vd.vartype);
            defs2 = defs2.prepend(vd);
        }
        return defs2;
    }

    Name outerThisName(Type type, Symbol owner) {
        Type t = type.getEnclosingType();
        int nestingLevel = 0;
        while (t.hasTag(TypeTag.CLASS)) {
            t = t.getEnclosingType();
            nestingLevel++;
        }
        Name result = this.names.fromString("this" + this.target.syntheticNameChar() + nestingLevel);
        while (owner.kind == 2 && ((Symbol.ClassSymbol) owner).members().lookup(result).scope != null) {
            result = this.names.fromString(result.toString() + this.target.syntheticNameChar());
        }
        return result;
    }

    private Symbol.VarSymbol makeOuterThisVarSymbol(Symbol owner, long flags) {
        if (owner.kind == 2 && this.target.usePrivateSyntheticFields()) {
            flags |= 2;
        }
        Type target = this.types.erasure(owner.enclClass().type.getEnclosingType());
        Symbol.VarSymbol outerThis = new Symbol.VarSymbol(flags, outerThisName(target, owner), target, owner);
        this.outerThisStack = this.outerThisStack.prepend(outerThis);
        return outerThis;
    }

    private JCTree.JCVariableDecl makeOuterThisVarDecl(int pos, Symbol.VarSymbol sym) {
        JCTree.JCVariableDecl vd = this.make.at(pos).VarDef(sym, null);
        vd.vartype = access(vd.vartype);
        return vd;
    }

    JCTree.JCVariableDecl outerThisDef(int pos, Symbol.MethodSymbol owner) {
        Symbol.ClassSymbol c = owner.enclClass();
        boolean isMandated = (owner.isConstructor() && owner.isAnonymous()) || (owner.isConstructor() && c.isInner() && !c.isPrivate() && !c.isStatic());
        long flags = ((long) ((isMandated ? 32768 : 4096) | 16)) | 8589934592L;
        Symbol.VarSymbol outerThis = makeOuterThisVarSymbol(owner, flags);
        owner.extraParams = owner.extraParams.prepend(outerThis);
        return makeOuterThisVarDecl(pos, outerThis);
    }

    JCTree.JCVariableDecl outerThisDef(int pos, Symbol.ClassSymbol owner) {
        Symbol.VarSymbol outerThis = makeOuterThisVarSymbol(owner, 4112L);
        return makeOuterThisVarDecl(pos, outerThis);
    }

    /* JADX WARN: Multi-variable type inference failed */
    List<JCTree.JCExpression> loadFreevars(JCDiagnostic.DiagnosticPosition pos, List<Symbol.VarSymbol> freevars) {
        List<JCTree.JCExpression> args = List.nil();
        for (List list = freevars; list.nonEmpty(); list = list.tail) {
            args = args.prepend(loadFreevar(pos, (Symbol.VarSymbol) list.head));
        }
        return args;
    }

    JCTree.JCExpression loadFreevar(JCDiagnostic.DiagnosticPosition pos, Symbol.VarSymbol v) {
        return access(v, this.make.at(pos).Ident(v), null, false);
    }

    JCTree.JCExpression makeThis(JCDiagnostic.DiagnosticPosition pos, Symbol.TypeSymbol c) {
        if (this.currentClass == c) {
            return this.make.at(pos).This(c.erasure(this.types));
        }
        return makeOuterThis(pos, c);
    }

    JCTree makeTwrTry(JCTree.JCTry tree) {
        make_at(tree.pos());
        this.twrVars = this.twrVars.dup();
        JCTree.JCBlock twrBlock = makeTwrBlock(tree.resources, tree.body, tree.finallyCanCompleteNormally, 0);
        if (tree.catchers.isEmpty() && tree.finalizer == null) {
            this.result = translate(twrBlock);
        } else {
            this.result = translate(this.make.Try(twrBlock, tree.catchers, tree.finalizer));
        }
        this.twrVars = this.twrVars.leave();
        return this.result;
    }

    private JCTree.JCBlock makeTwrBlock(List<JCTree> resources, JCTree.JCBlock block, boolean finallyCanCompleteNormally, int depth) {
        JCTree.JCExpression expr;
        if (resources.isEmpty()) {
            return block;
        }
        ListBuffer<JCTree.JCStatement> stats = new ListBuffer<>();
        JCTree resource = resources.head;
        if (resource instanceof JCTree.JCVariableDecl) {
            JCTree.JCVariableDecl var = (JCTree.JCVariableDecl) resource;
            expr = this.make.Ident(var.sym).setType(resource.type);
            stats.add(var);
        } else {
            Assert.check(resource instanceof JCTree.JCExpression);
            Symbol.VarSymbol syntheticTwrVar = new Symbol.VarSymbol(4112L, makeSyntheticName(this.names.fromString("twrVar" + depth), this.twrVars), resource.type.hasTag(TypeTag.BOT) ? this.syms.autoCloseableType : resource.type, this.currentMethodSym);
            this.twrVars.enter(syntheticTwrVar);
            JCTree.JCVariableDecl syntheticTwrVarDecl = this.make.VarDef(syntheticTwrVar, (JCTree.JCExpression) resource);
            expr = this.make.Ident(syntheticTwrVar);
            stats.add(syntheticTwrVarDecl);
        }
        Symbol.VarSymbol primaryException = new Symbol.VarSymbol(4096L, makeSyntheticName(this.names.fromString("primaryException" + depth), this.twrVars), this.syms.throwableType, this.currentMethodSym);
        this.twrVars.enter(primaryException);
        JCTree.JCVariableDecl primaryExceptionTreeDecl = this.make.VarDef(primaryException, makeNull());
        stats.add(primaryExceptionTreeDecl);
        Symbol.VarSymbol param = new Symbol.VarSymbol(4112L, this.names.fromString("t" + this.target.syntheticNameChar()), this.syms.throwableType, this.currentMethodSym);
        JCTree.JCVariableDecl paramTree = this.make.VarDef(param, null);
        JCTree.JCStatement assign = this.make.Assignment(primaryException, this.make.Ident(param));
        JCTree.JCStatement rethrowStat = this.make.Throw(this.make.Ident(param));
        JCTree.JCBlock catchBlock = this.make.Block(0L, List.of(assign, rethrowStat));
        JCTree.JCCatch catchClause = this.make.Catch(paramTree, catchBlock);
        int oldPos = this.make.pos;
        this.make.at(TreeInfo.endPos(block));
        JCTree.JCBlock finallyClause = makeTwrFinallyClause(primaryException, expr);
        this.make.at(oldPos);
        JCTree.JCTry outerTry = this.make.Try(makeTwrBlock(resources.tail, block, finallyCanCompleteNormally, depth + 1), List.of(catchClause), finallyClause);
        outerTry.finallyCanCompleteNormally = finallyCanCompleteNormally;
        stats.add(outerTry);
        JCTree.JCBlock newBlock = this.make.Block(0L, stats.toList());
        return newBlock;
    }

    private JCTree.JCBlock makeTwrFinallyClause(Symbol primaryException, JCTree.JCExpression resource) {
        Symbol.VarSymbol catchException = new Symbol.VarSymbol(4096L, this.make.paramName(2), this.syms.throwableType, this.currentMethodSym);
        JCTree.JCStatement addSuppressionStatement = this.make.Exec(makeCall(this.make.Ident(primaryException), this.names.addSuppressed, List.of(this.make.Ident(catchException))));
        JCTree.JCBlock tryBlock = this.make.Block(0L, List.of(makeResourceCloseInvocation(resource)));
        JCTree.JCVariableDecl catchExceptionDecl = this.make.VarDef(catchException, null);
        JCTree.JCBlock catchBlock = this.make.Block(0L, List.of(addSuppressionStatement));
        List<JCTree.JCCatch> catchClauses = List.of(this.make.Catch(catchExceptionDecl, catchBlock));
        JCTree.JCTry tryTree = this.make.Try(tryBlock, catchClauses, null);
        tryTree.finallyCanCompleteNormally = true;
        JCTree.JCIf closeIfStatement = this.make.If(makeNonNullCheck(this.make.Ident(primaryException)), tryTree, makeResourceCloseInvocation(resource));
        return this.make.Block(0L, List.of(this.make.If(makeNonNullCheck(resource), closeIfStatement, null)));
    }

    private JCTree.JCStatement makeResourceCloseInvocation(JCTree.JCExpression resource) {
        if (this.types.asSuper(resource.type, this.syms.autoCloseableType.tsym) == null) {
            resource = (JCTree.JCExpression) convert(resource, this.syms.autoCloseableType);
        }
        JCTree.JCExpression resourceClose = makeCall(resource, this.names.close, List.nil());
        return this.make.Exec(resourceClose);
    }

    private JCTree.JCExpression makeNonNullCheck(JCTree.JCExpression expression) {
        return makeBinary(JCTree.Tag.NE, expression, makeNull());
    }

    /* JADX WARN: Multi-variable type inference failed */
    JCTree.JCExpression makeOuterThis(JCDiagnostic.DiagnosticPosition pos, Symbol.TypeSymbol c) {
        Symbol.VarSymbol ot;
        List list = this.outerThisStack;
        if (list.isEmpty()) {
            this.log.error(pos, "no.encl.instance.of.type.in.scope", c);
            Assert.error();
            return makeNull();
        }
        Symbol.VarSymbol ot2 = (Symbol.VarSymbol) list.head;
        JCTree.JCExpression tree = access(this.make.at(pos).Ident(ot2));
        Symbol.TypeSymbol otc = ot2.type.tsym;
        while (otc != c) {
            do {
                list = list.tail;
                if (list.isEmpty()) {
                    this.log.error(pos, "no.encl.instance.of.type.in.scope", c);
                    Assert.error();
                    return tree;
                }
                ot = (Symbol.VarSymbol) list.head;
            } while (ot.owner != otc);
            if (otc.owner.kind != 1 && !otc.hasOuterInstance()) {
                this.chk.earlyRefError(pos, c);
                Assert.error();
                return makeNull();
            }
            tree = access(this.make.at(pos).Select(tree, ot));
            otc = ot.type.tsym;
        }
        return tree;
    }

    JCTree.JCExpression makeOwnerThis(JCDiagnostic.DiagnosticPosition pos, Symbol sym, boolean preciseMatch) {
        Symbol c = sym.owner;
        Symbol.ClassSymbol classSymbol = this.currentClass;
        if (!preciseMatch ? classSymbol.isSubClass(sym.owner, this.types) : sym.isMemberOf(classSymbol, this.types)) {
            return this.make.at(pos).This(c.erasure(this.types));
        }
        return makeOwnerThisN(pos, sym, preciseMatch);
    }

    /* JADX WARN: Code restructure failed: missing block: B:20:0x0079, code lost:
    
        return r4;
     */
    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Removed duplicated region for block: B:17:0x005d  */
    /* JADX WARN: Removed duplicated region for block: B:23:0x0050 A[SYNTHETIC] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    com.sun.tools.javac.tree.JCTree.JCExpression makeOwnerThisN(com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition r9, com.sun.tools.javac.code.Symbol r10, boolean r11) {
        /*
            r8 = this;
            com.sun.tools.javac.code.Symbol r0 = r10.owner
            com.sun.tools.javac.util.List<com.sun.tools.javac.code.Symbol$VarSymbol> r1 = r8.outerThisStack
            boolean r2 = r1.isEmpty()
            java.lang.String r3 = "no.encl.instance.of.type.in.scope"
            if (r2 == 0) goto L1d
            com.sun.tools.javac.util.Log r2 = r8.log
            java.lang.Object[] r4 = new java.lang.Object[]{r0}
            r2.error(r9, r3, r4)
            com.sun.tools.javac.util.Assert.error()
            com.sun.tools.javac.tree.JCTree$JCExpression r2 = r8.makeNull()
            return r2
        L1d:
            A r2 = r1.head
            com.sun.tools.javac.code.Symbol$VarSymbol r2 = (com.sun.tools.javac.code.Symbol.VarSymbol) r2
            com.sun.tools.javac.tree.TreeMaker r4 = r8.make
            com.sun.tools.javac.tree.TreeMaker r4 = r4.at(r9)
            com.sun.tools.javac.tree.JCTree$JCIdent r4 = r4.Ident(r2)
            com.sun.tools.javac.tree.JCTree$JCExpression r4 = r8.access(r4)
            com.sun.tools.javac.code.Type r5 = r2.type
            com.sun.tools.javac.code.Symbol$TypeSymbol r5 = r5.tsym
        L33:
            if (r11 == 0) goto L3e
            com.sun.tools.javac.code.Types r6 = r8.types
            boolean r6 = r10.isMemberOf(r5, r6)
            if (r6 == 0) goto L48
            goto L79
        L3e:
            com.sun.tools.javac.code.Symbol r6 = r10.owner
            com.sun.tools.javac.code.Types r7 = r8.types
            boolean r6 = r5.isSubClass(r6, r7)
            if (r6 != 0) goto L79
        L48:
            com.sun.tools.javac.util.List<A> r1 = r1.tail
            boolean r6 = r1.isEmpty()
            if (r6 == 0) goto L5d
            com.sun.tools.javac.util.Log r6 = r8.log
            java.lang.Object[] r7 = new java.lang.Object[]{r0}
            r6.error(r9, r3, r7)
            com.sun.tools.javac.util.Assert.error()
            return r4
        L5d:
            A r6 = r1.head
            r2 = r6
            com.sun.tools.javac.code.Symbol$VarSymbol r2 = (com.sun.tools.javac.code.Symbol.VarSymbol) r2
            com.sun.tools.javac.code.Symbol r6 = r2.owner
            if (r6 != r5) goto L48
            com.sun.tools.javac.tree.TreeMaker r6 = r8.make
            com.sun.tools.javac.tree.TreeMaker r6 = r6.at(r9)
            com.sun.tools.javac.tree.JCTree$JCExpression r6 = r6.Select(r4, r2)
            com.sun.tools.javac.tree.JCTree$JCExpression r4 = r8.access(r6)
            com.sun.tools.javac.code.Type r6 = r2.type
            com.sun.tools.javac.code.Symbol$TypeSymbol r5 = r6.tsym
            goto L33
        L79:
            return r4
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.comp.Lower.makeOwnerThisN(com.sun.tools.javac.util.JCDiagnostic$DiagnosticPosition, com.sun.tools.javac.code.Symbol, boolean):com.sun.tools.javac.tree.JCTree$JCExpression");
    }

    JCTree.JCStatement initField(int pos, Name name) {
        Scope.Entry e = this.proxies.lookup(name);
        Symbol rhs = e.sym;
        Assert.check(rhs.owner.kind == 16);
        Symbol lhs = e.next().sym;
        Assert.check(rhs.owner.owner == lhs.owner);
        this.make.at(pos);
        return this.make.Exec(this.make.Assign(this.make.Select(this.make.This(lhs.owner.erasure(this.types)), lhs), this.make.Ident(rhs)).setType(lhs.erasure(this.types)));
    }

    JCTree.JCStatement initOuterThis(int pos) {
        Symbol.VarSymbol rhs = this.outerThisStack.head;
        Assert.check(rhs.owner.kind == 16);
        Symbol.VarSymbol lhs = this.outerThisStack.tail.head;
        Assert.check(rhs.owner.owner == lhs.owner);
        this.make.at(pos);
        return this.make.Exec(this.make.Assign(this.make.Select(this.make.This(lhs.owner.erasure(this.types)), lhs), this.make.Ident(rhs)).setType(lhs.erasure(this.types)));
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Symbol.ClassSymbol outerCacheClass() {
        Symbol.ClassSymbol clazz = this.outermostClassDef.sym;
        if ((clazz.flags() & 512) == 0 && !this.target.useInnerCacheClass()) {
            return clazz;
        }
        Scope s = clazz.members();
        for (Scope.Entry e = s.elems; e != null; e = e.sibling) {
            if (e.sym.kind == 2 && e.sym.name == this.names.empty && (e.sym.flags() & 512) == 0) {
                return (Symbol.ClassSymbol) e.sym;
            }
        }
        return makeEmptyClass(4104L, clazz).sym;
    }

    private Symbol.MethodSymbol classDollarSym(JCDiagnostic.DiagnosticPosition pos) {
        Symbol.ClassSymbol outerCacheClass = outerCacheClass();
        Symbol.MethodSymbol classDollarSym = (Symbol.MethodSymbol) lookupSynthetic(this.classDollar, outerCacheClass.members());
        if (classDollarSym == null) {
            classDollarSym = new Symbol.MethodSymbol(4104L, this.classDollar, new Type.MethodType(List.of(this.syms.stringType), this.types.erasure(this.syms.classType), List.nil(), this.syms.methodClass), outerCacheClass);
            enterSynthetic(pos, classDollarSym, outerCacheClass.members());
            JCTree.JCMethodDecl md = this.make.MethodDef(classDollarSym, null);
            try {
                md.body = classDollarSymBody(pos, md);
            } catch (Symbol.CompletionFailure ex) {
                md.body = this.make.Block(0L, List.nil());
                this.chk.completionError(pos, ex);
            }
            JCTree.JCClassDecl outerCacheClassDef = classDef(outerCacheClass);
            outerCacheClassDef.defs = outerCacheClassDef.defs.prepend(md);
        }
        return classDollarSym;
    }

    JCTree.JCBlock classDollarSymBody(JCDiagnostic.DiagnosticPosition pos, JCTree.JCMethodDecl md) {
        long j;
        JCTree.JCBlock returnResult;
        JCTree.JCStatement rethrow;
        Symbol.MethodSymbol classDollarSym = md.sym;
        Symbol.ClassSymbol outerCacheClass = (Symbol.ClassSymbol) classDollarSym.owner;
        if (this.target.classLiteralsNoInit()) {
            Symbol.VarSymbol clsym = new Symbol.VarSymbol(4104L, this.names.fromString("cl" + this.target.syntheticNameChar()), this.syms.classLoaderType, outerCacheClass);
            enterSynthetic(pos, clsym, outerCacheClass.members());
            JCTree.JCVariableDecl cldef = this.make.VarDef(clsym, null);
            JCTree.JCClassDecl outerCacheClassDef = classDef(outerCacheClass);
            outerCacheClassDef.defs = outerCacheClassDef.defs.prepend(cldef);
            JCTree.JCNewArray newcache = this.make.NewArray(this.make.Type(outerCacheClass.type), List.of(this.make.Literal(TypeTag.INT, 0).setType((Type) this.syms.intType)), null);
            newcache.type = new Type.ArrayType(this.types.erasure(outerCacheClass.type), this.syms.arrayClass);
            Symbol.MethodSymbol forNameSym = lookupMethod(this.make_pos, this.names.forName, this.types.erasure(this.syms.classType), List.of(this.syms.stringType, (Type) this.syms.booleanType, this.syms.classLoaderType));
            JCTree.JCExpression clvalue = this.make.Conditional(makeBinary(JCTree.Tag.EQ, this.make.Ident(clsym), makeNull()), this.make.Assign(this.make.Ident(clsym), makeCall(makeCall(makeCall(newcache, this.names.getClass, List.nil()), this.names.getComponentType, List.nil()), this.names.getClassLoader, List.nil())).setType(this.syms.classLoaderType), this.make.Ident(clsym)).setType(this.syms.classLoaderType);
            List<JCTree.JCExpression> args = List.of((JCTree.JCExpression) this.make.Ident(md.params.head.sym), makeLit(this.syms.booleanType, 0), clvalue);
            JCTree.JCBlock returnResult2 = this.make.Block(0L, List.of(this.make.Call(this.make.App(this.make.Ident(forNameSym), args))));
            returnResult = returnResult2;
            j = 0;
        } else {
            Symbol.MethodSymbol forNameSym2 = lookupMethod(this.make_pos, this.names.forName, this.types.erasure(this.syms.classType), List.of(this.syms.stringType));
            j = 0;
            returnResult = this.make.Block(0L, List.of(this.make.Call(this.make.App(this.make.QualIdent(forNameSym2), List.of(this.make.Ident(md.params.head.sym))))));
        }
        long j2 = j;
        Symbol.VarSymbol catchParam = new Symbol.VarSymbol(4096L, this.make.paramName(1), this.syms.classNotFoundExceptionType, classDollarSym);
        if (this.target.hasInitCause()) {
            JCTree.JCMethodInvocation throwExpr = makeCall(makeNewClass(this.syms.noClassDefFoundErrorType, List.nil()), this.names.initCause, List.of(this.make.Ident(catchParam)));
            rethrow = this.make.Throw(throwExpr);
        } else {
            Symbol.MethodSymbol getMessageSym = lookupMethod(this.make_pos, this.names.getMessage, this.syms.classNotFoundExceptionType, List.nil());
            rethrow = this.make.Throw(makeNewClass(this.syms.noClassDefFoundErrorType, List.of(this.make.App(this.make.Select(this.make.Ident(catchParam), getMessageSym), List.nil()))));
        }
        JCTree.JCBlock rethrowStmt = this.make.Block(j2, List.of(rethrow));
        JCTree.JCCatch catchBlock = this.make.Catch(this.make.VarDef(catchParam, null), rethrowStmt);
        JCTree.JCStatement tryCatch = this.make.Try(returnResult, List.of(catchBlock), null);
        return this.make.Block(j2, List.of(tryCatch));
    }

    private JCTree.JCMethodInvocation makeCall(JCTree.JCExpression left, Name name, List<JCTree.JCExpression> args) {
        Assert.checkNonNull(left.type);
        Symbol funcsym = lookupMethod(this.make_pos, name, left.type, TreeInfo.types(args));
        return this.make.App(this.make.Select(left, funcsym), args);
    }

    private Name cacheName(String sig) {
        StringBuilder buf;
        StringBuilder buf2 = new StringBuilder();
        if (sig.startsWith("[")) {
            buf = buf2.append("array");
            while (sig.startsWith("[")) {
                buf = buf.append(this.target.syntheticNameChar());
                sig = sig.substring(1);
            }
            if (sig.startsWith("L")) {
                sig = sig.substring(0, sig.length() - 1);
            }
        } else {
            buf = buf2.append("class" + this.target.syntheticNameChar());
        }
        return this.names.fromString(buf.append(sig.replace(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, this.target.syntheticNameChar())).toString());
    }

    private Symbol.VarSymbol cacheSym(JCDiagnostic.DiagnosticPosition pos, String sig) {
        Symbol.ClassSymbol outerCacheClass = outerCacheClass();
        Name cname = cacheName(sig);
        Symbol.VarSymbol cacheSym = (Symbol.VarSymbol) lookupSynthetic(cname, outerCacheClass.members());
        if (cacheSym == null) {
            Symbol.VarSymbol cacheSym2 = new Symbol.VarSymbol(4104L, cname, this.types.erasure(this.syms.classType), outerCacheClass);
            enterSynthetic(pos, cacheSym2, outerCacheClass.members());
            JCTree.JCVariableDecl cacheDef = this.make.VarDef(cacheSym2, null);
            JCTree.JCClassDecl outerCacheClassDef = classDef(outerCacheClass);
            outerCacheClassDef.defs = outerCacheClassDef.defs.prepend(cacheDef);
            return cacheSym2;
        }
        return cacheSym;
    }

    private JCTree.JCExpression classOf(JCTree clazz) {
        return classOfType(clazz.type, clazz.pos());
    }

    private JCTree.JCExpression classOfType(Type type, JCDiagnostic.DiagnosticPosition pos) {
        switch (type.getTag()) {
            case BYTE:
            case SHORT:
            case CHAR:
            case INT:
            case LONG:
            case FLOAT:
            case DOUBLE:
            case BOOLEAN:
            case VOID:
                Symbol.ClassSymbol c = this.types.boxedClass(type);
                Symbol typeSym = this.rs.accessBase(this.rs.findIdentInType(this.attrEnv, c.type, this.names.TYPE, 4), pos, c.type, this.names.TYPE, true);
                if (typeSym.kind == 4) {
                    ((Symbol.VarSymbol) typeSym).getConstValue();
                }
                return this.make.QualIdent(typeSym);
            case CLASS:
            case ARRAY:
                if (this.target.hasClassLiterals()) {
                    Symbol.VarSymbol sym = new Symbol.VarSymbol(25L, this.names._class, this.syms.classType, type.tsym);
                    return make_at(pos).Select(this.make.Type(type), sym);
                }
                String sig = this.writer.xClassName(type).toString().replace(DataResource.SEPARATOR, DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
                Symbol cs = cacheSym(pos, sig);
                return make_at(pos).Conditional(makeBinary(JCTree.Tag.EQ, this.make.Ident(cs), makeNull()), this.make.Assign(this.make.Ident(cs), this.make.App(this.make.Ident(classDollarSym(pos)), List.of(this.make.Literal(TypeTag.CLASS, sig).setType(this.syms.stringType)))).setType(this.types.erasure(this.syms.classType)), this.make.Ident(cs)).setType(this.types.erasure(this.syms.classType));
            default:
                throw new AssertionError();
        }
    }

    private Symbol.ClassSymbol assertionsDisabledClass() {
        if (this.assertionsDisabledClassCache != null) {
            return this.assertionsDisabledClassCache;
        }
        this.assertionsDisabledClassCache = makeEmptyClass(4104L, this.outermostClassDef.sym).sym;
        return this.assertionsDisabledClassCache;
    }

    private JCTree.JCExpression assertFlagTest(JCDiagnostic.DiagnosticPosition pos) {
        Symbol.ClassSymbol outermostClass = this.outermostClassDef.sym;
        Symbol.ClassSymbol container = !this.currentClass.isInterface() ? this.currentClass : assertionsDisabledClass();
        Symbol.VarSymbol assertDisabledSym = (Symbol.VarSymbol) lookupSynthetic(this.dollarAssertionsDisabled, container.members());
        if (assertDisabledSym == null) {
            assertDisabledSym = new Symbol.VarSymbol(4120L, this.dollarAssertionsDisabled, this.syms.booleanType, container);
            enterSynthetic(pos, assertDisabledSym, container.members());
            Symbol.MethodSymbol desiredAssertionStatusSym = lookupMethod(pos, this.names.desiredAssertionStatus, this.types.erasure(this.syms.classType), List.nil());
            JCTree.JCClassDecl containerDef = classDef(container);
            make_at(containerDef.pos());
            JCTree.JCUnary notStatus = makeUnary(JCTree.Tag.NOT, this.make.App(this.make.Select(classOfType(this.types.erasure(outermostClass.type), containerDef.pos()), desiredAssertionStatusSym)));
            JCTree.JCVariableDecl assertDisabledDef = this.make.VarDef(assertDisabledSym, notStatus);
            containerDef.defs = containerDef.defs.prepend(assertDisabledDef);
            if (this.currentClass.isInterface()) {
                JCTree.JCClassDecl currentClassDef = classDef(this.currentClass);
                make_at(currentClassDef.pos());
                JCTree.JCStatement dummy = this.make.If(this.make.QualIdent(assertDisabledSym), this.make.Skip(), null);
                JCTree.JCBlock clinit = this.make.Block(8L, List.of(dummy));
                currentClassDef.defs = currentClassDef.defs.prepend(clinit);
            }
        }
        make_at(pos);
        return makeUnary(JCTree.Tag.NOT, this.make.Ident(assertDisabledSym));
    }

    JCTree abstractRval(JCTree rval, Type type, TreeBuilder builder) {
        JCTree rval2 = TreeInfo.skipParens(rval);
        switch (rval2.getTag()) {
            case LITERAL:
                return builder.build(rval2);
            case IDENT:
                JCTree.JCIdent id = (JCTree.JCIdent) rval2;
                if ((id.sym.flags() & 16) != 0 && id.sym.owner.kind == 16) {
                    return builder.build(rval2);
                }
                break;
        }
        Symbol.VarSymbol var = new Symbol.VarSymbol(4112L, this.names.fromString(this.target.syntheticNameChar() + "" + rval2.hashCode()), type, this.currentMethodSym);
        JCTree.JCVariableDecl def = this.make.VarDef(var, (JCTree.JCExpression) convert(rval2, type));
        JCTree built = builder.build(this.make.Ident(var));
        JCTree res = this.make.LetExpr(def, built);
        res.type = built.type;
        return res;
    }

    JCTree abstractRval(JCTree rval, TreeBuilder builder) {
        return abstractRval(rval, rval.type, builder);
    }

    JCTree abstractLval(JCTree lval, final TreeBuilder builder) {
        JCTree lval2 = TreeInfo.skipParens(lval);
        switch (lval2.getTag()) {
            case IDENT:
                return builder.build(lval2);
            case SELECT:
                final JCTree.JCFieldAccess s = (JCTree.JCFieldAccess) lval2;
                TreeInfo.skipParens(s.selected);
                Symbol lid = TreeInfo.symbol(s.selected);
                return (lid == null || lid.kind != 2) ? abstractRval(s.selected, new TreeBuilder() { // from class: com.sun.tools.javac.comp.Lower.2
                    @Override // com.sun.tools.javac.comp.Lower.TreeBuilder
                    public JCTree build(JCTree selected) {
                        return builder.build(Lower.this.make.Select((JCTree.JCExpression) selected, s.sym));
                    }
                }) : builder.build(lval2);
            case INDEXED:
                final JCTree.JCArrayAccess i = (JCTree.JCArrayAccess) lval2;
                return abstractRval(i.indexed, new TreeBuilder() { // from class: com.sun.tools.javac.comp.Lower.3
                    @Override // com.sun.tools.javac.comp.Lower.TreeBuilder
                    public JCTree build(final JCTree indexed) {
                        return Lower.this.abstractRval(i.index, Lower.this.syms.intType, new TreeBuilder() { // from class: com.sun.tools.javac.comp.Lower.3.1
                            @Override // com.sun.tools.javac.comp.Lower.TreeBuilder
                            public JCTree build(JCTree index) {
                                JCTree newLval = Lower.this.make.Indexed((JCTree.JCExpression) indexed, (JCTree.JCExpression) index);
                                newLval.setType(i.type);
                                return builder.build(newLval);
                            }
                        });
                    }
                });
            case TYPECAST:
                return abstractLval(((JCTree.JCTypeCast) lval2).expr, builder);
            default:
                throw new AssertionError(lval2);
        }
    }

    JCTree makeComma(JCTree expr1, final JCTree expr2) {
        return abstractRval(expr1, new TreeBuilder() { // from class: com.sun.tools.javac.comp.Lower.4
            @Override // com.sun.tools.javac.comp.Lower.TreeBuilder
            public JCTree build(JCTree discarded) {
                return expr2;
            }
        });
    }

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
    @Override // com.sun.tools.javac.tree.TreeTranslator
    public <T extends JCTree> T translate(T t) {
        if (t == null) {
            return null;
        }
        make_at(t.pos());
        T t2 = (T) super.translate(t);
        if (this.endPosTable != null && t2 != t) {
            this.endPosTable.replaceTree(t, t2);
        }
        return t2;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public <T extends JCTree> T translate(T t, Type type) {
        if (t == null) {
            return null;
        }
        return (T) boxIfNeeded(translate(t), type);
    }

    public <T extends JCTree> T translate(T t, JCTree.JCExpression jCExpression) {
        JCTree.JCExpression jCExpression2 = this.enclOp;
        this.enclOp = jCExpression;
        T t2 = (T) translate(t);
        this.enclOp = jCExpression2;
        return t2;
    }

    public <T extends JCTree> List<T> translate(List<T> trees, JCTree.JCExpression enclOp) {
        JCTree.JCExpression prevEnclOp = this.enclOp;
        this.enclOp = enclOp;
        List<T> res = translate(trees);
        this.enclOp = prevEnclOp;
        return res;
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v3, types: [A, com.sun.tools.javac.tree.JCTree] */
    public <T extends JCTree> List<T> translate(List<T> trees, Type type) {
        if (trees == null) {
            return null;
        }
        for (List list = trees; list.nonEmpty(); list = list.tail) {
            list.head = translate((JCTree) list.head, type);
        }
        return trees;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTopLevel(JCTree.JCCompilationUnit tree) {
        long flags;
        if (needPackageInfoClass(tree)) {
            Name name = this.names.package_info;
            if (this.target.isPackageInfoSynthetic()) {
                long flags2 = 1536 | 4096;
                flags = flags2;
            } else {
                flags = 1536;
            }
            JCTree.JCClassDecl packageAnnotationsClass = this.make.ClassDef(this.make.Modifiers(flags, tree.packageAnnotations), name, List.nil(), null, List.nil(), List.nil());
            Symbol.ClassSymbol c = tree.packge.package_info;
            c.flags_field |= flags;
            c.setAttributes(tree.packge);
            Type.ClassType ctype = (Type.ClassType) c.type;
            ctype.supertype_field = this.syms.objectType;
            ctype.interfaces_field = List.nil();
            packageAnnotationsClass.sym = c;
            this.translated.append(packageAnnotationsClass);
        }
    }

    private boolean needPackageInfoClass(JCTree.JCCompilationUnit tree) {
        switch (this.pkginfoOpt) {
            case ALWAYS:
                return true;
            case LEGACY:
                return tree.packageAnnotations.nonEmpty();
            case NONEMPTY:
                for (Attribute.Compound a : tree.packge.getDeclarationAttributes()) {
                    Attribute.RetentionPolicy p = this.types.getRetention(a);
                    if (p != Attribute.RetentionPolicy.SOURCE) {
                        return true;
                    }
                }
                return false;
            default:
                throw new AssertionError();
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r12v3, types: [A, com.sun.tools.javac.tree.JCTree] */
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
    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitClassDef(JCTree.JCClassDecl tree) {
        Env<AttrContext> prevEnv = this.attrEnv;
        Symbol.ClassSymbol currentClassPrev = this.currentClass;
        Symbol.MethodSymbol currentMethodSymPrev = this.currentMethodSym;
        this.currentClass = tree.sym;
        this.currentMethodSym = null;
        this.attrEnv = this.typeEnvs.remove(this.currentClass);
        if (this.attrEnv == null) {
            this.attrEnv = prevEnv;
        }
        this.classdefs.put(this.currentClass, tree);
        this.proxies = this.proxies.dup(this.currentClass);
        List<Symbol.VarSymbol> prevOuterThisStack = this.outerThisStack;
        if ((tree.mods.flags & 16384) != 0 && (this.types.supertype(this.currentClass.type).tsym.flags() & 16384) == 0) {
            visitEnumDef(tree);
        }
        JCTree.JCVariableDecl otdef = null;
        if (this.currentClass.hasOuterInstance()) {
            otdef = outerThisDef(tree.pos, this.currentClass);
        }
        List<JCTree.JCVariableDecl> fvdefs = freevarDefs(tree.pos, freevars(this.currentClass), this.currentClass);
        tree.extending = (JCTree.JCExpression) translate(tree.extending);
        tree.implementing = translate(tree.implementing);
        if (this.currentClass.isLocal()) {
            Symbol.ClassSymbol encl = this.currentClass.owner.enclClass();
            if (encl.trans_local == null) {
                encl.trans_local = List.nil();
            }
            encl.trans_local = encl.trans_local.prepend(this.currentClass);
        }
        List<JCTree> seen = List.nil();
        while (tree.defs != seen) {
            List<JCTree> unseen = tree.defs;
            for (List list = unseen; list.nonEmpty() && list != seen; list = list.tail) {
                JCTree outermostMemberDefPrev = this.outermostMemberDef;
                if (outermostMemberDefPrev == null) {
                    this.outermostMemberDef = (JCTree) list.head;
                }
                list.head = translate((JCTree) list.head);
                this.outermostMemberDef = outermostMemberDefPrev;
            }
            seen = unseen;
        }
        if ((tree.mods.flags & 4) != 0) {
            tree.mods.flags |= 1;
        }
        tree.mods.flags &= 32273;
        tree.name = Convert.shortName(this.currentClass.flatName());
        for (List list2 = fvdefs; list2.nonEmpty(); list2 = list2.tail) {
            tree.defs = tree.defs.prepend(list2.head);
            enterSynthetic(tree.pos(), ((JCTree.JCVariableDecl) list2.head).sym, this.currentClass.members());
        }
        if (this.currentClass.hasOuterInstance()) {
            tree.defs = tree.defs.prepend(otdef);
            enterSynthetic(tree.pos(), otdef.sym, this.currentClass.members());
        }
        this.proxies = this.proxies.leave();
        this.outerThisStack = prevOuterThisStack;
        this.translated.append(tree);
        this.attrEnv = prevEnv;
        this.currentClass = currentClassPrev;
        this.currentMethodSym = currentMethodSymPrev;
        this.result = make_at(tree.pos()).Block(4096L, List.nil());
    }

    /* JADX WARN: Multi-variable type inference failed */
    private void visitEnumDef(JCTree.JCClassDecl tree) {
        ListBuffer listBuffer;
        JCTree.JCExpression e_class;
        boolean z;
        boolean z2;
        List<JCTree.JCStatement> valuesBody;
        make_at(tree.pos());
        if (tree.extending == null) {
            tree.extending = this.make.Type(this.types.supertype(tree.type));
        }
        JCTree.JCExpression e_class2 = classOfType(tree.sym.type, tree.pos()).setType(this.types.erasure(this.syms.classType));
        int nextOrdinal = 0;
        ListBuffer<JCTree.JCExpression> values = new ListBuffer<>();
        ListBuffer<JCTree> enumDefs = new ListBuffer<>();
        ListBuffer listBuffer2 = new ListBuffer();
        for (List list = tree.defs; list.nonEmpty(); list = list.tail) {
            if (((JCTree) list.head).hasTag(JCTree.Tag.VARDEF) && (((JCTree.JCVariableDecl) list.head).mods.flags & 16384) != 0) {
                JCTree.JCVariableDecl var = (JCTree.JCVariableDecl) list.head;
                visitEnumConstantDef(var, nextOrdinal);
                values.append(this.make.QualIdent(var.sym));
                enumDefs.append(var);
                nextOrdinal++;
            } else {
                listBuffer2.append(list.head);
            }
        }
        Name valuesName = this.names.fromString(this.target.syntheticNameChar() + "VALUES");
        while (tree.sym.members().lookup(valuesName).scope != null) {
            valuesName = this.names.fromString(((Object) valuesName) + "" + this.target.syntheticNameChar());
        }
        Type arrayType = new Type.ArrayType(this.types.erasure(tree.type), this.syms.arrayClass);
        Symbol.VarSymbol valuesVar = new Symbol.VarSymbol(4122L, valuesName, arrayType, tree.type.tsym);
        JCTree.JCNewArray newArray = this.make.NewArray(this.make.Type(this.types.erasure(tree.type)), List.nil(), values.toList());
        newArray.type = arrayType;
        enumDefs.append(this.make.VarDef(valuesVar, newArray));
        tree.sym.members().enter(valuesVar);
        Symbol valuesSym = lookupMethod(tree.pos(), this.names.values, tree.type, List.nil());
        if (!useClone()) {
            Name resultName = this.names.fromString(this.target.syntheticNameChar() + "result");
            while (tree.sym.members().lookup(resultName).scope != null) {
                resultName = this.names.fromString(((Object) resultName) + "" + this.target.syntheticNameChar());
            }
            Symbol.VarSymbol resultVar = new Symbol.VarSymbol(4112L, resultName, arrayType, valuesSym);
            JCTree.JCNewArray resultArray = this.make.NewArray(this.make.Type(this.types.erasure(tree.type)), List.of(this.make.Select(this.make.Ident(valuesVar), this.syms.lengthVar)), null);
            resultArray.type = arrayType;
            JCTree.JCVariableDecl decl = this.make.VarDef(resultVar, resultArray);
            if (this.systemArraycopyMethod == null) {
                this.systemArraycopyMethod = new Symbol.MethodSymbol(9L, this.names.fromString("arraycopy"), new Type.MethodType(List.of(this.syms.objectType, (Type) this.syms.intType, this.syms.objectType, this.syms.intType, this.syms.intType), this.syms.voidType, List.nil(), this.syms.methodClass), this.syms.systemType.tsym);
            }
            listBuffer = listBuffer2;
            e_class = e_class2;
            z = false;
            z2 = true;
            JCTree.JCStatement copy = this.make.Exec(this.make.App(this.make.Select(this.make.Ident(this.syms.systemType.tsym), this.systemArraycopyMethod), List.of(this.make.Ident(valuesVar), (JCTree.JCIdent) this.make.Literal(0), this.make.Ident(resultVar), (JCTree.JCIdent[]) new JCTree.JCExpression[]{this.make.Literal(0), this.make.Select(this.make.Ident(valuesVar), this.syms.lengthVar)})));
            JCTree.JCStatement ret = this.make.Return(this.make.Ident(resultVar));
            valuesBody = List.of((JCTree.JCStatement) decl, copy, ret);
        } else {
            JCTree.JCTypeCast valuesResult = this.make.TypeCast(valuesSym.type.mo178getReturnType(), this.make.App(this.make.Select(this.make.Ident(valuesVar), this.syms.arrayCloneMethod)));
            valuesBody = List.of(this.make.Return(valuesResult));
            e_class = e_class2;
            listBuffer = listBuffer2;
            z2 = true;
            z = false;
        }
        JCTree.JCMethodDecl valuesDef = this.make.MethodDef((Symbol.MethodSymbol) valuesSym, this.make.Block(0L, valuesBody));
        enumDefs.append(valuesDef);
        if (this.debugLower) {
            System.err.println(tree.sym + ".valuesDef = " + valuesDef);
        }
        Symbol.MethodSymbol valueOfSym = lookupMethod(tree.pos(), this.names.valueOf, tree.sym.type, List.of(this.syms.stringType));
        Assert.check((valueOfSym.flags() & 8) != 0 ? z2 : z);
        Symbol.VarSymbol nameArgSym = valueOfSym.params.head;
        JCTree.JCIdent nameVal = this.make.Ident(nameArgSym);
        JCTree.JCStatement enum_ValueOf = this.make.Return(this.make.TypeCast(tree.sym.type, makeCall(this.make.Ident(this.syms.enumSym), this.names.valueOf, List.of((JCTree.JCIdent) e_class, nameVal))));
        JCTree.JCMethodDecl valueOf = this.make.MethodDef(valueOfSym, this.make.Block(0L, List.of(enum_ValueOf)));
        nameVal.sym = valueOf.params.head.sym;
        if (this.debugLower) {
            System.err.println(tree.sym + ".valueOf = " + valueOf);
        }
        enumDefs.append(valueOf);
        enumDefs.appendList(listBuffer.toList());
        tree.defs = enumDefs.toList();
    }

    private boolean useClone() {
        try {
            Scope.Entry e = this.syms.objectType.tsym.members().lookup(this.names.clone);
            return e.sym != null;
        } catch (Symbol.CompletionFailure e2) {
            return false;
        }
    }

    private void visitEnumConstantDef(JCTree.JCVariableDecl var, int ordinal) {
        JCTree.JCNewClass varDef = (JCTree.JCNewClass) var.init;
        varDef.args = varDef.args.prepend(makeLit(this.syms.intType, Integer.valueOf(ordinal))).prepend(makeLit(this.syms.stringType, var.name.toString()));
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitMethodDef(JCTree.JCMethodDecl tree) {
        if (tree.name == this.names.init && (this.currentClass.flags_field & 16384) != 0) {
            JCTree.JCVariableDecl nameParam = make_at(tree.pos()).Param(this.names.fromString(this.target.syntheticNameChar() + "enum" + this.target.syntheticNameChar() + "name"), this.syms.stringType, tree.sym);
            nameParam.mods.flags |= 4096;
            nameParam.sym.flags_field |= 4096;
            JCTree.JCVariableDecl ordParam = this.make.Param(this.names.fromString(this.target.syntheticNameChar() + "enum" + this.target.syntheticNameChar() + "ordinal"), this.syms.intType, tree.sym);
            ordParam.mods.flags |= 4096;
            ordParam.sym.flags_field |= 4096;
            tree.params = tree.params.prepend(ordParam).prepend(nameParam);
            Symbol.MethodSymbol m = tree.sym;
            m.extraParams = m.extraParams.prepend(ordParam.sym);
            m.extraParams = m.extraParams.prepend(nameParam.sym);
            Type olderasure = m.erasure(this.types);
            m.erasure_field = new Type.MethodType(olderasure.mo176getParameterTypes().prepend(this.syms.intType).prepend(this.syms.stringType), olderasure.mo178getReturnType(), olderasure.mo179getThrownTypes(), this.syms.methodClass);
        }
        JCTree.JCMethodDecl prevMethodDef = this.currentMethodDef;
        Symbol.MethodSymbol prevMethodSym = this.currentMethodSym;
        try {
            this.currentMethodDef = tree;
            this.currentMethodSym = tree.sym;
            visitMethodDefInternal(tree);
        } finally {
            this.currentMethodDef = prevMethodDef;
            this.currentMethodSym = prevMethodSym;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    private void visitMethodDefInternal(JCTree.JCMethodDecl tree) {
        JCTree.JCVariableDecl otdef;
        if (tree.name == this.names.init && (this.currentClass.isInner() || this.currentClass.isLocal())) {
            Symbol.MethodSymbol m = tree.sym;
            this.proxies = this.proxies.dup(m);
            List<Symbol.VarSymbol> prevOuterThisStack = this.outerThisStack;
            List<Symbol.VarSymbol> fvs = freevars(this.currentClass);
            if (!this.currentClass.hasOuterInstance()) {
                otdef = null;
            } else {
                JCTree.JCVariableDecl otdef2 = outerThisDef(tree.pos, m);
                otdef = otdef2;
            }
            List<JCTree.JCVariableDecl> fvdefs = freevarDefs(tree.pos, fvs, m, 8589934592L);
            tree.restype = (JCTree.JCExpression) translate(tree.restype);
            tree.params = translateVarDefs(tree.params);
            tree.thrown = translate(tree.thrown);
            if (tree.body == null) {
                this.result = tree;
                return;
            }
            tree.params = tree.params.appendList(fvdefs);
            if (this.currentClass.hasOuterInstance()) {
                tree.params = tree.params.prepend(otdef);
            }
            JCTree.JCStatement selfCall = (JCTree.JCStatement) translate(tree.body.stats.head);
            List<JCTree.JCStatement> added = List.nil();
            if (fvs.nonEmpty()) {
                List<Type> addedargtypes = List.nil();
                for (List list = fvs; list.nonEmpty(); list = list.tail) {
                    if (TreeInfo.isInitialConstructor(tree)) {
                        Name pName = proxyName(((Symbol.VarSymbol) list.head).name);
                        m.capturedLocals = m.capturedLocals.append((Symbol.VarSymbol) this.proxies.lookup(pName).sym);
                        added = added.prepend(initField(tree.body.pos, pName));
                    }
                    addedargtypes = addedargtypes.prepend(((Symbol.VarSymbol) list.head).erasure(this.types));
                }
                Type olderasure = m.erasure(this.types);
                m.erasure_field = new Type.MethodType(olderasure.mo176getParameterTypes().appendList(addedargtypes), olderasure.mo178getReturnType(), olderasure.mo179getThrownTypes(), this.syms.methodClass);
            }
            if (this.currentClass.hasOuterInstance() && TreeInfo.isInitialConstructor(tree)) {
                added = added.prepend(initOuterThis(tree.body.pos));
            }
            this.proxies = this.proxies.leave();
            List<JCTree.JCStatement> stats = translate(tree.body.stats.tail);
            if (this.target.initializeFieldsBeforeSuper()) {
                tree.body.stats = stats.prepend(selfCall).prependList(added);
            } else {
                tree.body.stats = stats.prependList(added).prepend(selfCall);
            }
            this.outerThisStack = prevOuterThisStack;
        } else {
            Map<Symbol, Symbol> prevLambdaTranslationMap = this.lambdaTranslationMap;
            try {
                this.lambdaTranslationMap = ((tree.sym.flags() & 4096) == 0 || !tree.sym.name.startsWith(this.names.lambda)) ? null : makeTranslationMap(tree);
                super.visitMethodDef(tree);
            } finally {
                this.lambdaTranslationMap = prevLambdaTranslationMap;
            }
        }
        this.result = tree;
    }

    private Map<Symbol, Symbol> makeTranslationMap(JCTree.JCMethodDecl tree) {
        Map<Symbol, Symbol> translationMap = new HashMap<>();
        for (JCTree.JCVariableDecl vd : tree.params) {
            Symbol p = vd.sym;
            if (p != p.baseSymbol()) {
                translationMap.put(p.baseSymbol(), p);
            }
        }
        return translationMap;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
        tree.annotations = List.nil();
        tree.underlyingType = (JCTree.JCExpression) translate(tree.underlyingType);
        if (tree.type.isAnnotated()) {
            tree.type = tree.underlyingType.type.unannotatedType().annotatedType(tree.type.getAnnotationMirrors());
        } else if (tree.underlyingType.type.isAnnotated()) {
            tree.type = tree.underlyingType.type;
        }
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTypeCast(JCTree.JCTypeCast tree) {
        tree.clazz = translate(tree.clazz);
        if (tree.type.isPrimitive() != tree.expr.type.isPrimitive()) {
            tree.expr = (JCTree.JCExpression) translate(tree.expr, tree.type);
        } else {
            tree.expr = (JCTree.JCExpression) translate(tree.expr);
        }
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewClass(JCTree.JCNewClass tree) {
        JCTree.JCExpression thisArg;
        Symbol.ClassSymbol c = (Symbol.ClassSymbol) tree.constructor.owner;
        boolean isEnum = (tree.constructor.owner.flags() & 16384) != 0;
        List<Type> argTypes = tree.constructor.type.mo176getParameterTypes();
        if (isEnum) {
            argTypes = argTypes.prepend(this.syms.intType).prepend(this.syms.stringType);
        }
        tree.args = boxArgs(argTypes, tree.args, tree.varargsElement);
        tree.varargsElement = null;
        if (c.isLocal()) {
            tree.args = tree.args.appendList(loadFreevars(tree.pos(), freevars(c)));
        }
        Symbol constructor = accessConstructor(tree.pos(), tree.constructor);
        if (constructor != tree.constructor) {
            tree.args = tree.args.append(makeNull());
            tree.constructor = constructor;
        }
        if (c.hasOuterInstance()) {
            if (tree.encl != null) {
                thisArg = this.attr.makeNullCheck((JCTree.JCExpression) translate(tree.encl));
                thisArg.type = tree.encl.type;
            } else if (c.isLocal()) {
                thisArg = makeThis(tree.pos(), c.type.getEnclosingType().tsym);
            } else {
                thisArg = makeOwnerThis(tree.pos(), c, false);
            }
            tree.args = tree.args.prepend(thisArg);
        }
        tree.encl = null;
        if (tree.def != null) {
            translate(tree.def);
            tree.clazz = access(make_at(tree.clazz.pos()).Ident(tree.def.sym));
            tree.def = null;
        } else {
            tree.clazz = access(c, tree.clazz, this.enclOp, false);
        }
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitConditional(JCTree.JCConditional tree) {
        JCTree.JCExpression jCExpression = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        tree.cond = jCExpression;
        if (jCExpression.type.isTrue()) {
            this.result = convert(translate(tree.truepart, tree.type), tree.type);
            addPrunedInfo(jCExpression);
        } else if (jCExpression.type.isFalse()) {
            this.result = convert(translate(tree.falsepart, tree.type), tree.type);
            addPrunedInfo(jCExpression);
        } else {
            tree.truepart = (JCTree.JCExpression) translate(tree.truepart, tree.type);
            tree.falsepart = (JCTree.JCExpression) translate(tree.falsepart, tree.type);
            this.result = tree;
        }
    }

    private JCTree convert(JCTree tree, Type pt) {
        if (tree.type == pt || tree.type.hasTag(TypeTag.BOT)) {
            return tree;
        }
        JCTree result = make_at(tree.pos()).TypeCast(this.make.Type(pt), (JCTree.JCExpression) tree);
        result.type = tree.type.constValue() != null ? this.cfolder.coerce(tree.type, pt) : pt;
        return result;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIf(JCTree.JCIf tree) {
        JCTree.JCExpression jCExpression = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        tree.cond = jCExpression;
        if (jCExpression.type.isTrue()) {
            this.result = translate(tree.thenpart);
            addPrunedInfo(jCExpression);
        } else {
            if (jCExpression.type.isFalse()) {
                if (tree.elsepart != null) {
                    this.result = translate(tree.elsepart);
                } else {
                    this.result = this.make.Skip();
                }
                addPrunedInfo(jCExpression);
                return;
            }
            tree.thenpart = (JCTree.JCStatement) translate(tree.thenpart);
            tree.elsepart = (JCTree.JCStatement) translate(tree.elsepart);
            this.result = tree;
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssert(JCTree.JCAssert tree) {
        if (tree.detail == null) {
            tree.pos();
        } else {
            tree.detail.pos();
        }
        tree.cond = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        if (!tree.cond.type.isTrue()) {
            JCTree.JCExpression cond = assertFlagTest(tree.pos());
            List<JCTree.JCExpression> exnArgs = tree.detail == null ? List.nil() : List.of(translate(tree.detail));
            if (!tree.cond.type.isFalse()) {
                cond = makeBinary(JCTree.Tag.AND, cond, makeUnary(JCTree.Tag.NOT, tree.cond));
            }
            this.result = this.make.If(cond, make_at(tree).Throw(makeNewClass(this.syms.assertionErrorType, exnArgs)), null);
            return;
        }
        this.result = this.make.Skip();
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitApply(JCTree.JCMethodInvocation tree) {
        JCTree.JCExpression thisArg;
        Symbol meth = TreeInfo.symbol(tree.meth);
        List listMo176getParameterTypes = meth.type.mo176getParameterTypes();
        if (this.allowEnums && meth.name == this.names.init && meth.owner == this.syms.enumSym) {
            listMo176getParameterTypes = listMo176getParameterTypes.tail.tail;
        }
        tree.args = boxArgs(listMo176getParameterTypes, tree.args, tree.varargsElement);
        tree.varargsElement = null;
        Name methName = TreeInfo.name(tree.meth);
        if (meth.name == this.names.init) {
            Symbol constructor = accessConstructor(tree.pos(), meth);
            if (constructor != meth) {
                tree.args = tree.args.append(makeNull());
                TreeInfo.setSymbol(tree.meth, constructor);
            }
            Symbol.ClassSymbol c = (Symbol.ClassSymbol) constructor.owner;
            if (c.isLocal()) {
                tree.args = tree.args.appendList(loadFreevars(tree.pos(), freevars(c)));
            }
            if ((c.flags_field & 16384) != 0 || c.getQualifiedName() == this.names.java_lang_Enum) {
                List list = this.currentMethodDef.params;
                if (this.currentMethodSym.owner.hasOuterInstance()) {
                    list = list.tail;
                }
                tree.args = tree.args.prepend(make_at(tree.pos()).Ident(((JCTree.JCVariableDecl) list.tail.head).sym)).prepend(this.make.Ident(((JCTree.JCVariableDecl) list.head).sym));
            }
            if (c.hasOuterInstance()) {
                if (tree.meth.hasTag(JCTree.Tag.SELECT)) {
                    thisArg = this.attr.makeNullCheck((JCTree.JCExpression) translate(((JCTree.JCFieldAccess) tree.meth).selected));
                    tree.meth = this.make.Ident(constructor);
                    ((JCTree.JCIdent) tree.meth).name = methName;
                } else if (c.isLocal() || methName == this.names._this) {
                    JCTree.JCExpression thisArg2 = tree.meth;
                    thisArg = makeThis(thisArg2.pos(), c.type.getEnclosingType().tsym);
                } else {
                    thisArg = makeOwnerThisN(tree.meth.pos(), c, false);
                }
                tree.args = tree.args.prepend(thisArg);
            }
        } else {
            tree.meth = (JCTree.JCExpression) translate(tree.meth);
            if (tree.meth.hasTag(JCTree.Tag.APPLY)) {
                JCTree.JCMethodInvocation app = (JCTree.JCMethodInvocation) tree.meth;
                app.args = tree.args.prependList(app.args);
                this.result = app;
                return;
            }
        }
        this.result = tree;
    }

    /* JADX WARN: Multi-variable type inference failed */
    List<JCTree.JCExpression> boxArgs(List<Type> list, List<JCTree.JCExpression> list2, Type type) {
        List list3 = list2;
        if (list.isEmpty()) {
            return list3;
        }
        boolean z = false;
        ListBuffer listBuffer = new ListBuffer();
        List list4 = list;
        while (true) {
            if (!list4.tail.nonEmpty()) {
                break;
            }
            JCTree.JCExpression jCExpression = (JCTree.JCExpression) translate((JCTree) list3.head, (Type) list4.head);
            if (jCExpression != list3.head) {
                z = true;
            }
            z |= z;
            listBuffer.append(jCExpression);
            list3 = list3.tail;
            list4 = list4.tail;
        }
        Type type2 = (Type) list4.head;
        if (type != null) {
            ListBuffer listBuffer2 = new ListBuffer();
            while (list3.nonEmpty()) {
                listBuffer2.append((JCTree.JCExpression) translate((JCTree) list3.head, type));
                list3 = list3.tail;
            }
            JCTree.JCNewArray jCNewArrayNewArray = this.make.NewArray(this.make.Type(type), List.nil(), listBuffer2.toList());
            jCNewArrayNewArray.type = new Type.ArrayType(type, this.syms.arrayClass);
            listBuffer.append(jCNewArrayNewArray);
        } else {
            if (list3.length() != 1) {
                throw new AssertionError(list3);
            }
            JCTree.JCExpression jCExpression2 = (JCTree.JCExpression) translate((JCTree) list3.head, type2);
            boolean z2 = z | (jCExpression2 != list3.head);
            listBuffer.append(jCExpression2);
            if (!z2) {
                return list2;
            }
        }
        return listBuffer.toList();
    }

    <T extends JCTree> T boxIfNeeded(T tree, Type type) {
        boolean havePrimitive = tree.type.isPrimitive();
        if (havePrimitive == type.isPrimitive()) {
            return tree;
        }
        if (havePrimitive) {
            Type unboxedTarget = this.types.unboxedType(type);
            if (!unboxedTarget.hasTag(TypeTag.NONE)) {
                if (!this.types.isSubtype(tree.type, unboxedTarget)) {
                    tree.type = unboxedTarget.constType(tree.type.constValue());
                }
                return boxPrimitive((JCTree.JCExpression) tree, type);
            }
            return boxPrimitive((JCTree.JCExpression) tree);
        }
        return unbox((JCTree.JCExpression) tree, type);
    }

    JCTree.JCExpression boxPrimitive(JCTree.JCExpression tree) {
        return boxPrimitive(tree, this.types.boxedClass(tree.type).type);
    }

    JCTree.JCExpression boxPrimitive(JCTree.JCExpression tree, Type box) {
        make_at(tree.pos());
        if (this.target.boxWithConstructors()) {
            Symbol ctor = lookupConstructor(tree.pos(), box, List.nil().prepend(tree.type));
            return this.make.Create(ctor, List.of(tree));
        }
        Symbol valueOfSym = lookupMethod(tree.pos(), this.names.valueOf, box, List.nil().prepend(tree.type));
        return this.make.App(this.make.QualIdent(valueOfSym), List.of(tree));
    }

    JCTree.JCExpression unbox(JCTree.JCExpression tree, Type primitive) {
        Type unboxedType = this.types.unboxedType(tree.type);
        if (unboxedType.hasTag(TypeTag.NONE)) {
            unboxedType = primitive;
            if (!unboxedType.isPrimitive()) {
                throw new AssertionError(unboxedType);
            }
            make_at(tree.pos());
            tree = this.make.TypeCast(this.types.boxedClass(unboxedType).type, tree);
        } else if (!this.types.isSubtype(unboxedType, primitive)) {
            throw new AssertionError(tree);
        }
        make_at(tree.pos());
        Symbol valueSym = lookupMethod(tree.pos(), unboxedType.tsym.name.append(this.names.Value), tree.type, List.nil());
        return this.make.App(this.make.Select(tree, valueSym));
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitParens(JCTree.JCParens tree) {
        JCTree expr = translate(tree.expr);
        this.result = expr == tree.expr ? tree : expr;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIndexed(JCTree.JCArrayAccess tree) {
        tree.indexed = (JCTree.JCExpression) translate(tree.indexed);
        tree.index = (JCTree.JCExpression) translate(tree.index, this.syms.intType);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssign(JCTree.JCAssign tree) {
        tree.lhs = (JCTree.JCExpression) translate(tree.lhs, tree);
        tree.rhs = (JCTree.JCExpression) translate(tree.rhs, tree.lhs.type);
        if (tree.lhs.hasTag(JCTree.Tag.APPLY)) {
            JCTree.JCMethodInvocation app = (JCTree.JCMethodInvocation) tree.lhs;
            app.args = List.of(tree.rhs).prependList(app.args);
            this.result = app;
            return;
        }
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAssignop(final JCTree.JCAssignOp tree) {
        JCTree lhsAccess = access(TreeInfo.skipParens(tree.lhs));
        final boolean boxingReq = !tree.lhs.type.isPrimitive() && tree.operator.type.mo178getReturnType().isPrimitive();
        if (boxingReq || lhsAccess.hasTag(JCTree.Tag.APPLY)) {
            JCTree newTree = abstractLval(tree.lhs, new TreeBuilder() { // from class: com.sun.tools.javac.comp.Lower.5
                @Override // com.sun.tools.javac.comp.Lower.TreeBuilder
                public JCTree build(JCTree lhs) {
                    JCTree.Tag newTag = tree.getTag().noAssignOp();
                    Symbol newOperator = Lower.this.rs.resolveBinaryOperator(tree.pos(), newTag, Lower.this.attrEnv, tree.type, tree.rhs.type);
                    JCTree.JCExpression expr = (JCTree.JCExpression) lhs;
                    if (expr.type != tree.type) {
                        expr = Lower.this.make.TypeCast(tree.type, expr);
                    }
                    JCTree.JCBinary opResult = Lower.this.make.Binary(newTag, expr, tree.rhs);
                    opResult.operator = newOperator;
                    opResult.type = newOperator.type.mo178getReturnType();
                    JCTree.JCExpression newRhs = boxingReq ? Lower.this.make.TypeCast(Lower.this.types.unboxedType(tree.type), opResult) : opResult;
                    return Lower.this.make.Assign((JCTree.JCExpression) lhs, newRhs).setType(tree.type);
                }
            });
            this.result = translate(newTree);
            return;
        }
        tree.lhs = (JCTree.JCExpression) translate(tree.lhs, tree);
        tree.rhs = (JCTree.JCExpression) translate(tree.rhs, tree.operator.type.mo176getParameterTypes().tail.head);
        if (tree.lhs.hasTag(JCTree.Tag.APPLY)) {
            JCTree.JCMethodInvocation app = (JCTree.JCMethodInvocation) tree.lhs;
            JCTree.JCExpression rhs = ((Symbol.OperatorSymbol) tree.operator).opcode == 256 ? makeString(tree.rhs) : tree.rhs;
            app.args = List.of(rhs).prependList(app.args);
            this.result = app;
            return;
        }
        this.result = tree;
    }

    JCTree lowerBoxedPostop(final JCTree.JCUnary tree) {
        final boolean cast = TreeInfo.skipParens(tree.arg).hasTag(JCTree.Tag.TYPECAST);
        return abstractLval(tree.arg, new TreeBuilder() { // from class: com.sun.tools.javac.comp.Lower.6
            @Override // com.sun.tools.javac.comp.Lower.TreeBuilder
            public JCTree build(final JCTree tmp1) {
                return Lower.this.abstractRval(tmp1, tree.arg.type, new TreeBuilder() { // from class: com.sun.tools.javac.comp.Lower.6.1
                    @Override // com.sun.tools.javac.comp.Lower.TreeBuilder
                    public JCTree build(JCTree tmp2) {
                        JCTree.Tag opcode = tree.hasTag(JCTree.Tag.POSTINC) ? JCTree.Tag.PLUS_ASG : JCTree.Tag.MINUS_ASG;
                        JCTree lhs = cast ? Lower.this.make.TypeCast(tree.arg.type, (JCTree.JCExpression) tmp1) : tmp1;
                        JCTree update = Lower.this.makeAssignop(opcode, lhs, Lower.this.make.Literal(1));
                        return Lower.this.makeComma(update, tmp2);
                    }
                });
            }
        });
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitUnary(JCTree.JCUnary tree) {
        boolean isUpdateOperator = tree.getTag().isIncOrDecUnaryOp();
        if (isUpdateOperator && !tree.arg.type.isPrimitive()) {
            switch (tree.getTag()) {
                case PREINC:
                case PREDEC:
                    JCTree.Tag opcode = tree.hasTag(JCTree.Tag.PREINC) ? JCTree.Tag.PLUS_ASG : JCTree.Tag.MINUS_ASG;
                    JCTree.JCAssignOp newTree = makeAssignop(opcode, tree.arg, this.make.Literal(1));
                    this.result = translate(newTree, tree.type);
                    return;
                case POSTINC:
                case POSTDEC:
                    this.result = translate(lowerBoxedPostop(tree), tree.type);
                    return;
                default:
                    throw new AssertionError(tree);
            }
        }
        tree.arg = (JCTree.JCExpression) boxIfNeeded(translate(tree.arg, tree), tree.type);
        if (tree.hasTag(JCTree.Tag.NOT) && tree.arg.type.constValue() != null) {
            tree.type = this.cfolder.fold1(257, tree.arg.type);
        }
        if (isUpdateOperator && tree.arg.hasTag(JCTree.Tag.APPLY)) {
            this.result = tree.arg;
        } else {
            this.result = tree;
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBinary(JCTree.JCBinary tree) {
        List<Type> formals = tree.operator.type.mo176getParameterTypes();
        JCTree.JCExpression jCExpression = (JCTree.JCExpression) translate(tree.lhs, formals.head);
        tree.lhs = jCExpression;
        switch (tree.getTag()) {
            case OR:
                if (jCExpression.type.isTrue()) {
                    this.result = jCExpression;
                    return;
                } else if (jCExpression.type.isFalse()) {
                    this.result = translate(tree.rhs, formals.tail.head);
                    return;
                }
            case AND:
                if (jCExpression.type.isFalse()) {
                    this.result = jCExpression;
                    return;
                } else if (jCExpression.type.isTrue()) {
                    this.result = translate(tree.rhs, formals.tail.head);
                    return;
                }
        }
        tree.rhs = (JCTree.JCExpression) translate(tree.rhs, formals.tail.head);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIdent(JCTree.JCIdent tree) {
        this.result = access(tree.sym, tree, this.enclOp, false);
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
        if (this.types.elemtype(tree.expr.type) == null) {
            visitIterableForeachLoop(tree);
        } else {
            visitArrayForeachLoop(tree);
        }
    }

    private void visitArrayForeachLoop(JCTree.JCEnhancedForLoop tree) {
        make_at(tree.expr.pos());
        Symbol.VarSymbol arraycache = new Symbol.VarSymbol(4096L, this.names.fromString("arr" + this.target.syntheticNameChar()), tree.expr.type, this.currentMethodSym);
        JCTree.JCStatement arraycachedef = this.make.VarDef(arraycache, tree.expr);
        Symbol.VarSymbol lencache = new Symbol.VarSymbol(4096L, this.names.fromString("len" + this.target.syntheticNameChar()), this.syms.intType, this.currentMethodSym);
        JCTree.JCStatement lencachedef = this.make.VarDef(lencache, this.make.Select(this.make.Ident(arraycache), this.syms.lengthVar));
        Symbol.VarSymbol index = new Symbol.VarSymbol(4096L, this.names.fromString("i" + this.target.syntheticNameChar()), this.syms.intType, this.currentMethodSym);
        JCTree.JCVariableDecl indexdef = this.make.VarDef(index, this.make.Literal(TypeTag.INT, 0));
        JCTree.JCExpression jCExpression = indexdef.init;
        Type typeConstType = this.syms.intType.constType(0);
        indexdef.type = typeConstType;
        jCExpression.type = typeConstType;
        List<JCTree.JCStatement> loopinit = List.of((JCTree.JCVariableDecl) arraycachedef, (JCTree.JCVariableDecl) lencachedef, indexdef);
        JCTree.JCBinary cond = makeBinary(JCTree.Tag.LT, this.make.Ident(index), this.make.Ident(lencache));
        JCTree.JCExpressionStatement step = this.make.Exec(makeUnary(JCTree.Tag.PREINC, this.make.Ident(index)));
        Type elemtype = this.types.elemtype(tree.expr.type);
        JCTree.JCExpression loopvarinit = this.make.Indexed(this.make.Ident(arraycache), this.make.Ident(index)).setType(elemtype);
        JCTree.JCVariableDecl loopvardef = (JCTree.JCVariableDecl) this.make.VarDef(tree.var.mods, tree.var.name, tree.var.vartype, loopvarinit).setType(tree.var.type);
        loopvardef.sym = tree.var.sym;
        JCTree.JCBlock body = this.make.Block(0L, List.of((JCTree.JCStatement) loopvardef, tree.body));
        this.result = translate(this.make.ForLoop(loopinit, cond, List.of(step), body));
        patchTargets(body, tree, this.result);
    }

    private void patchTargets(JCTree body, final JCTree src, final JCTree dest) {
        new TreeScanner() { // from class: com.sun.tools.javac.comp.Lower.1Patcher
            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitBreak(JCTree.JCBreak tree) {
                if (tree.target == src) {
                    tree.target = dest;
                }
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitContinue(JCTree.JCContinue tree) {
                if (tree.target == src) {
                    tree.target = dest;
                }
            }

            @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitClassDef(JCTree.JCClassDecl tree) {
            }
        }.scan(body);
    }

    private void visitIterableForeachLoop(JCTree.JCEnhancedForLoop tree) {
        JCTree.JCExpression vardefinit;
        make_at(tree.expr.pos());
        Type iteratorTarget = this.syms.objectType;
        Type iterableType = this.types.asSuper(this.types.cvarUpperBound(tree.expr.type), this.syms.iterableType.tsym);
        if (iterableType.getTypeArguments().nonEmpty()) {
            iteratorTarget = this.types.erasure(iterableType.getTypeArguments().head);
        }
        Type eType = tree.expr.type;
        while (eType.hasTag(TypeTag.TYPEVAR)) {
            eType = eType.getUpperBound();
        }
        tree.expr.type = this.types.erasure(eType);
        if (eType.isCompound()) {
            tree.expr = this.make.TypeCast(this.types.erasure(iterableType), tree.expr);
        }
        Symbol.MethodSymbol iterator = lookupMethod(tree.expr.pos(), this.names.iterator, eType, List.nil());
        Symbol.VarSymbol itvar = new Symbol.VarSymbol(4096L, this.names.fromString("i" + this.target.syntheticNameChar()), this.types.erasure(this.types.asSuper(iterator.type.mo178getReturnType(), this.syms.iteratorType.tsym)), this.currentMethodSym);
        JCTree.JCStatement init = this.make.VarDef(itvar, this.make.App(this.make.Select(tree.expr, iterator).setType(this.types.erasure(iterator.type))));
        Symbol.MethodSymbol hasNext = lookupMethod(tree.expr.pos(), this.names.hasNext, itvar.type, List.nil());
        JCTree.JCMethodInvocation cond = this.make.App(this.make.Select(this.make.Ident(itvar), hasNext));
        Symbol.MethodSymbol next = lookupMethod(tree.expr.pos(), this.names.next, itvar.type, List.nil());
        JCTree.JCExpression vardefinit2 = this.make.App(this.make.Select(this.make.Ident(itvar), next));
        if (tree.var.type.isPrimitive()) {
            vardefinit = this.make.TypeCast(this.types.cvarUpperBound(iteratorTarget), vardefinit2);
        } else {
            vardefinit = this.make.TypeCast(tree.var.type, vardefinit2);
        }
        JCTree.JCVariableDecl indexDef = (JCTree.JCVariableDecl) this.make.VarDef(tree.var.mods, tree.var.name, tree.var.vartype, vardefinit).setType(tree.var.type);
        indexDef.sym = tree.var.sym;
        JCTree.JCBlock body = this.make.Block(0L, List.of((JCTree.JCStatement) indexDef, tree.body));
        body.endpos = TreeInfo.endPos(tree.body);
        this.result = translate(this.make.ForLoop(List.of(init), cond, List.nil(), body));
        patchTargets(body, tree, this.result);
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitVarDef(JCTree.JCVariableDecl tree) {
        Symbol.MethodSymbol oldMethodSym = this.currentMethodSym;
        tree.mods = (JCTree.JCModifiers) translate(tree.mods);
        tree.vartype = (JCTree.JCExpression) translate(tree.vartype);
        if (this.currentMethodSym == null) {
            this.currentMethodSym = new Symbol.MethodSymbol((tree.mods.flags & 8) | 1048576, this.names.empty, null, this.currentClass);
        }
        if (tree.init != null) {
            tree.init = (JCTree.JCExpression) translate(tree.init, tree.type);
        }
        this.result = tree;
        this.currentMethodSym = oldMethodSym;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitBlock(JCTree.JCBlock tree) {
        Symbol.MethodSymbol oldMethodSym = this.currentMethodSym;
        if (this.currentMethodSym == null) {
            this.currentMethodSym = new Symbol.MethodSymbol(tree.flags | 1048576, this.names.empty, null, this.currentClass);
        }
        super.visitBlock(tree);
        this.currentMethodSym = oldMethodSym;
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
        tree.init = translate(tree.init);
        if (tree.cond != null) {
            tree.cond = (JCTree.JCExpression) translate(tree.cond, this.syms.booleanType);
        }
        tree.step = translate(tree.step);
        tree.body = (JCTree.JCStatement) translate(tree.body);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReturn(JCTree.JCReturn tree) {
        if (tree.expr != null) {
            tree.expr = (JCTree.JCExpression) translate(tree.expr, this.types.erasure(this.currentMethodDef.restype.type));
        }
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSwitch(JCTree.JCSwitch tree) {
        Type target;
        Type selsuper = this.types.supertype(tree.selector.type);
        boolean enumSwitch = (selsuper == null || (tree.selector.type.tsym.flags() & 16384) == 0) ? false : true;
        boolean stringSwitch = selsuper != null && this.types.isSameType(tree.selector.type, this.syms.stringType);
        if (enumSwitch) {
            target = tree.selector.type;
        } else {
            Symtab symtab = this.syms;
            target = stringSwitch ? symtab.stringType : symtab.intType;
        }
        tree.selector = (JCTree.JCExpression) translate(tree.selector, target);
        tree.cases = translateCases(tree.cases);
        if (enumSwitch) {
            this.result = visitEnumSwitch(tree);
        } else if (stringSwitch) {
            this.result = visitStringSwitch(tree);
        } else {
            this.result = tree;
        }
    }

    public JCTree visitEnumSwitch(JCTree.JCSwitch tree) {
        Symbol.TypeSymbol enumSym = tree.selector.type.tsym;
        EnumMapping map = mapForEnum(tree.pos(), enumSym);
        make_at(tree.pos());
        Symbol ordinalMethod = lookupMethod(tree.pos(), this.names.ordinal, tree.selector.type, List.nil());
        JCTree.JCArrayAccess selector = this.make.Indexed(map.mapVar, this.make.App(this.make.Select(tree.selector, ordinalMethod)));
        ListBuffer<JCTree.JCCase> cases = new ListBuffer<>();
        for (JCTree.JCCase c : tree.cases) {
            if (c.pat != null) {
                Symbol.VarSymbol label = (Symbol.VarSymbol) TreeInfo.symbol(c.pat);
                JCTree.JCLiteral pat = map.forConstant(label);
                cases.append(this.make.Case(pat, c.stats));
            } else {
                cases.append(c);
            }
        }
        JCTree.JCSwitch enumSwitch = this.make.Switch(selector, cases.toList());
        patchTargets(enumSwitch, tree, enumSwitch);
        return enumSwitch;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public JCTree visitStringSwitch(JCTree.JCSwitch tree) {
        int i;
        JCTree.JCExpression caseExpr;
        int alternatives;
        Iterator<JCTree.JCCase> it;
        char c;
        List<JCTree.JCCase> caseList = tree.getCases();
        int alternatives2 = caseList.size();
        if (alternatives2 == 0) {
            return this.make.at(tree.pos()).Exec(this.attr.makeNullCheck(tree.getExpression()));
        }
        ListBuffer<JCTree.JCStatement> stmtList = new ListBuffer<>();
        char c2 = 0;
        Map<String, Integer> caseLabelToPosition = new LinkedHashMap<>(alternatives2 + 1, 1.0f);
        Map<Integer, Set<String>> hashToString = new LinkedHashMap<>(alternatives2 + 1, 1.0f);
        int casePosition = 0;
        Iterator<JCTree.JCCase> it2 = caseList.iterator();
        while (true) {
            i = 1;
            if (!it2.hasNext()) {
                break;
            }
            JCTree.JCExpression expression = it2.next().getExpression();
            if (expression == null) {
                alternatives = alternatives2;
                it = it2;
                c = c2;
            } else {
                String labelExpr = (String) expression.type.constValue();
                Integer mapping = caseLabelToPosition.put(labelExpr, Integer.valueOf(casePosition));
                Assert.checkNull(mapping);
                int hashCode = labelExpr.hashCode();
                Set<String> stringSet = hashToString.get(Integer.valueOf(hashCode));
                if (stringSet == null) {
                    alternatives = alternatives2;
                    it = it2;
                    c = 0;
                    Set<String> stringSet2 = new LinkedHashSet<>(1, 1.0f);
                    stringSet2.add(labelExpr);
                    hashToString.put(Integer.valueOf(hashCode), stringSet2);
                } else {
                    alternatives = alternatives2;
                    it = it2;
                    c = 0;
                    boolean added = stringSet.add(labelExpr);
                    Assert.check(added);
                }
            }
            casePosition++;
            c2 = c;
            alternatives2 = alternatives;
            it2 = it;
        }
        Symbol.VarSymbol dollar_s = new Symbol.VarSymbol(4112L, this.names.fromString("s" + tree.pos + this.target.syntheticNameChar()), this.syms.stringType, this.currentMethodSym);
        stmtList.append(this.make.at(tree.pos()).VarDef(dollar_s, tree.getExpression()).setType(dollar_s.type));
        Symbol.VarSymbol dollar_tmp = new Symbol.VarSymbol(4096L, this.names.fromString("tmp" + tree.pos + this.target.syntheticNameChar()), this.syms.intType, this.currentMethodSym);
        JCTree.JCVariableDecl dollar_tmp_def = (JCTree.JCVariableDecl) this.make.VarDef(dollar_tmp, this.make.Literal(TypeTag.INT, -1)).setType(dollar_tmp.type);
        JCTree.JCExpression jCExpression = dollar_tmp_def.init;
        Type.JCPrimitiveType jCPrimitiveType = this.syms.intType;
        dollar_tmp.type = jCPrimitiveType;
        jCExpression.type = jCPrimitiveType;
        stmtList.append(dollar_tmp_def);
        ListBuffer<JCTree.JCCase> caseBuffer = new ListBuffer<>();
        JCTree.JCMethodInvocation hashCodeCall = makeCall(this.make.Ident(dollar_s), this.names.hashCode, List.nil()).setType(this.syms.intType);
        JCTree.JCSwitch switch1 = this.make.Switch(hashCodeCall, caseBuffer.toList());
        Iterator<Map.Entry<Integer, Set<String>>> it3 = hashToString.entrySet().iterator();
        while (true) {
            if (!it3.hasNext()) {
                break;
            }
            Map.Entry<Integer, Set<String>> entry = it3.next();
            int hashCode2 = entry.getKey().intValue();
            Set<String> stringsWithHashCode = entry.getValue();
            Map<Integer, Set<String>> hashToString2 = hashToString;
            Assert.check(stringsWithHashCode.size() >= i ? i : 0);
            JCTree.JCStatement elsepart = null;
            for (String caseLabel : stringsWithHashCode) {
                Symbol.VarSymbol dollar_s2 = dollar_s;
                JCTree.JCMethodInvocation stringEqualsCall = makeCall(this.make.Ident(dollar_s), this.names.equals, List.of(this.make.Literal(caseLabel)));
                elsepart = this.make.If(stringEqualsCall, this.make.Exec(this.make.Assign(this.make.Ident(dollar_tmp), this.make.Literal(caseLabelToPosition.get(caseLabel))).setType(dollar_tmp.type)), elsepart);
                casePosition = casePosition;
                dollar_s = dollar_s2;
                dollar_tmp_def = dollar_tmp_def;
                hashCodeCall = hashCodeCall;
                it3 = it3;
                entry = entry;
            }
            Symbol.VarSymbol dollar_s3 = dollar_s;
            ListBuffer<JCTree.JCStatement> lb = new ListBuffer<>();
            JCTree.JCBreak breakStmt = this.make.Break(null);
            breakStmt.target = switch1;
            lb.append(elsepart).append(breakStmt);
            caseBuffer.append(this.make.Case(this.make.Literal(Integer.valueOf(hashCode2)), lb.toList()));
            hashToString = hashToString2;
            casePosition = casePosition;
            dollar_s = dollar_s3;
            dollar_tmp_def = dollar_tmp_def;
            i = 1;
        }
        switch1.cases = caseBuffer.toList();
        stmtList.append(switch1);
        ListBuffer<JCTree.JCCase> lb2 = new ListBuffer<>();
        JCTree.JCSwitch switch2 = this.make.Switch(this.make.Ident(dollar_tmp), lb2.toList());
        for (JCTree.JCCase oneCase : caseList) {
            patchTargets(oneCase, tree, switch2);
            boolean isDefault = oneCase.getExpression() == null;
            if (isDefault) {
                caseExpr = null;
            } else {
                caseExpr = this.make.Literal(caseLabelToPosition.get((String) TreeInfo.skipParens(oneCase.getExpression()).type.constValue()));
            }
            lb2.append(this.make.Case(caseExpr, oneCase.getStatements()));
        }
        switch2.cases = lb2.toList();
        stmtList.append(switch2);
        return this.make.Block(0L, stmtList.toList());
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r1v6, types: [A, com.sun.tools.javac.tree.JCTree] */
    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitNewArray(JCTree.JCNewArray tree) {
        tree.elemtype = (JCTree.JCExpression) translate(tree.elemtype);
        for (List list = tree.dims; list.tail != null; list = list.tail) {
            if (list.head != 0) {
                list.head = translate((JCTree) list.head, this.syms.intType);
            }
        }
        List<JCTree.JCExpression> t = tree.elems;
        tree.elems = translate(t, this.types.elemtype(tree.type));
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSelect(JCTree.JCFieldAccess tree) {
        boolean qualifiedSuperAccess = tree.selected.hasTag(JCTree.Tag.SELECT) && TreeInfo.name(tree.selected) == this.names._super && !this.types.isDirectSuperInterface(((JCTree.JCFieldAccess) tree.selected).selected.type.tsym, this.currentClass);
        tree.selected = (JCTree.JCExpression) translate(tree.selected);
        if (tree.name == this.names._class) {
            this.result = classOf(tree.selected);
            return;
        }
        if (tree.name == this.names._super && this.types.isDirectSuperInterface(tree.selected.type.tsym, this.currentClass)) {
            Symbol.TypeSymbol supSym = tree.selected.type.tsym;
            Assert.checkNonNull(this.types.asSuper(this.currentClass.type, supSym));
            this.result = tree;
        } else if (tree.name == this.names._this || tree.name == this.names._super) {
            this.result = makeThis(tree.pos(), tree.selected.type.tsym);
        } else {
            this.result = access(tree.sym, tree, this.enclOp, qualifiedSuperAccess);
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLetExpr(JCTree.LetExpr tree) {
        tree.defs = translateVarDefs(tree.defs);
        tree.expr = translate(tree.expr, tree.type);
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitAnnotation(JCTree.JCAnnotation tree) {
        this.result = tree;
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitTry(JCTree.JCTry tree) {
        if (tree.resources.nonEmpty()) {
            this.result = makeTwrTry(tree);
            return;
        }
        boolean hasBody = tree.body.getStatements().nonEmpty();
        boolean hasCatchers = tree.catchers.nonEmpty();
        boolean hasFinally = tree.finalizer != null && tree.finalizer.getStatements().nonEmpty();
        if (!hasCatchers && !hasFinally) {
            this.result = translate(tree.body);
            return;
        }
        if (!hasBody) {
            if (hasFinally) {
                this.result = translate(tree.finalizer);
                return;
            } else {
                this.result = translate(tree.body);
                return;
            }
        }
        super.visitTry(tree);
    }

    /* JADX WARN: Multi-variable type inference failed */
    public List<JCTree> translateTopLevelClass(Env<AttrContext> env, JCTree cdef, TreeMaker make) {
        try {
            this.attrEnv = env;
            this.make = make;
            this.endPosTable = env.toplevel.endPositions;
            this.currentClass = null;
            this.currentMethodDef = null;
            this.outermostClassDef = cdef.hasTag(JCTree.Tag.CLASSDEF) ? (JCTree.JCClassDecl) cdef : null;
            this.outermostMemberDef = null;
            this.translated = new ListBuffer<>();
            this.classdefs = new HashMap();
            this.actualSymbols = new HashMap();
            this.freevarCache = new HashMap();
            this.proxies = new Scope(this.syms.noSymbol);
            this.twrVars = new Scope(this.syms.noSymbol);
            this.outerThisStack = List.nil();
            this.accessNums = new HashMap();
            this.accessSyms = new HashMap();
            this.accessConstrs = new HashMap();
            this.accessConstrTags = List.nil();
            this.accessed = new ListBuffer<>();
            translate(cdef, (JCTree.JCExpression) null);
            for (List list = this.accessed.toList(); list.nonEmpty(); list = list.tail) {
                makeAccessible((Symbol) list.head);
            }
            for (EnumMapping map : this.enumSwitchMap.values()) {
                map.translate();
            }
            checkConflicts(this.translated.toList());
            checkAccessConstructorTags();
            ListBuffer<JCTree> translated = this.translated;
            this.attrEnv = null;
            this.make = null;
            this.endPosTable = null;
            this.currentClass = null;
            this.currentMethodDef = null;
            this.outermostClassDef = null;
            this.outermostMemberDef = null;
            this.translated = null;
            this.classdefs = null;
            this.actualSymbols = null;
            this.freevarCache = null;
            this.proxies = null;
            this.outerThisStack = null;
            this.accessNums = null;
            this.accessSyms = null;
            this.accessConstrs = null;
            this.accessConstrTags = null;
            this.accessed = null;
            this.enumSwitchMap.clear();
            this.assertionsDisabledClassCache = null;
            return translated.toList();
        } catch (Throwable th) {
            this.attrEnv = null;
            this.make = null;
            this.endPosTable = null;
            this.currentClass = null;
            this.currentMethodDef = null;
            this.outermostClassDef = null;
            this.outermostMemberDef = null;
            this.translated = null;
            this.classdefs = null;
            this.actualSymbols = null;
            this.freevarCache = null;
            this.proxies = null;
            this.outerThisStack = null;
            this.accessNums = null;
            this.accessSyms = null;
            this.accessConstrs = null;
            this.accessConstrTags = null;
            this.accessed = null;
            this.enumSwitchMap.clear();
            this.assertionsDisabledClassCache = null;
            throw th;
        }
    }
}
