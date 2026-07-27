package com.sun.tools.javac.comp;

import com.sun.source.tree.LambdaExpressionTree;
import com.sun.source.tree.MemberReferenceTree;
import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.Lower;
import com.sun.tools.javac.jvm.Pool;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeTranslator;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.DiagnosticSource;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import javax.lang.model.type.TypeKind;
import org.firstinspires.ftc.onbotjava.RequestConditions;

/* JADX INFO: loaded from: classes.dex */
public class LambdaToMethod extends TreeTranslator {
    public static final int FLAG_BRIDGES = 4;
    public static final int FLAG_MARKERS = 2;
    public static final int FLAG_SERIALIZABLE = 1;
    protected static final Context.Key<LambdaToMethod> unlambdaKey = new Context.Key<>();
    private LambdaAnalyzerPreprocessor analyzer;
    private Attr attr;
    private Env<AttrContext> attrEnv;
    private LambdaAnalyzerPreprocessor.TranslationContext<?> context;
    private Map<JCTree, LambdaAnalyzerPreprocessor.TranslationContext<?>> contextMap;
    private JCDiagnostic.Factory diags;
    private boolean dumpLambdaToMethodStats;
    private final boolean forceSerializable;
    private KlassInfo kInfo;
    private Log log;
    private Lower lower;
    private TreeMaker make;
    private Names names;
    private Resolve rs;
    private Symtab syms;
    private TransTypes transTypes;
    private Types types;

    enum LambdaSymbolKind {
        PARAM,
        LOCAL_VAR,
        CAPTURED_VAR,
        CAPTURED_THIS,
        CAPTURED_OUTER_THIS,
        TYPE_VAR
    }

    public static LambdaToMethod instance(Context context) {
        LambdaToMethod instance = (LambdaToMethod) context.get(unlambdaKey);
        if (instance == null) {
            return new LambdaToMethod(context);
        }
        return instance;
    }

    private LambdaToMethod(Context context) {
        context.put(unlambdaKey, this);
        this.diags = JCDiagnostic.Factory.instance(context);
        this.log = Log.instance(context);
        this.lower = Lower.instance(context);
        this.names = Names.instance(context);
        this.syms = Symtab.instance(context);
        this.rs = Resolve.instance(context);
        this.make = TreeMaker.instance(context);
        this.types = Types.instance(context);
        this.transTypes = TransTypes.instance(context);
        this.analyzer = new LambdaAnalyzerPreprocessor();
        Options options = Options.instance(context);
        this.dumpLambdaToMethodStats = options.isSet("dumpLambdaToMethodStats");
        this.attr = Attr.instance(context);
        this.forceSerializable = options.isSet("forceSerializable");
    }

    private class KlassInfo {
        private ListBuffer<JCTree> appendedMethodList;
        private final JCTree.JCClassDecl clazz;
        private final Symbol.MethodSymbol deserMethodSym;
        private final Symbol.VarSymbol deserParamSym;
        private final Map<String, ListBuffer<JCTree.JCStatement>> deserializeCases;

        private KlassInfo(JCTree.JCClassDecl clazz) {
            this.clazz = clazz;
            this.appendedMethodList = new ListBuffer<>();
            this.deserializeCases = new HashMap();
            Type.MethodType type = new Type.MethodType(List.of(LambdaToMethod.this.syms.serializedLambdaType), LambdaToMethod.this.syms.objectType, List.nil(), LambdaToMethod.this.syms.methodClass);
            this.deserMethodSym = LambdaToMethod.this.makePrivateSyntheticMethod(8L, LambdaToMethod.this.names.deserializeLambda, type, clazz.sym);
            this.deserParamSym = new Symbol.VarSymbol(16L, LambdaToMethod.this.names.fromString("lambda"), LambdaToMethod.this.syms.serializedLambdaType, this.deserMethodSym);
        }

        /* JADX INFO: Access modifiers changed from: private */
        public void addMethod(JCTree decl) {
            this.appendedMethodList = this.appendedMethodList.prepend(decl);
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator
    public <T extends JCTree> T translate(T t) {
        LambdaAnalyzerPreprocessor.TranslationContext<?> translationContext = this.contextMap.get(t);
        return (T) translate(t, translationContext != null ? translationContext : this.context);
    }

    <T extends JCTree> T translate(T t, LambdaAnalyzerPreprocessor.TranslationContext<?> translationContext) {
        LambdaAnalyzerPreprocessor.TranslationContext<?> translationContext2 = this.context;
        try {
            this.context = translationContext;
            return (T) super.translate(t);
        } finally {
            this.context = translationContext2;
        }
    }

    <T extends JCTree> List<T> translate(List<T> trees, LambdaAnalyzerPreprocessor.TranslationContext<?> newContext) {
        ListBuffer listBuffer = new ListBuffer();
        for (T tree : trees) {
            listBuffer.append(translate(tree, newContext));
        }
        return listBuffer.toList();
    }

    public JCTree translateTopLevelClass(Env<AttrContext> env, JCTree cdef, TreeMaker make) {
        this.make = make;
        this.attrEnv = env;
        this.context = null;
        this.contextMap = new HashMap();
        return translate(cdef);
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitClassDef(JCTree.JCClassDecl tree) {
        if (tree.sym.owner.kind == 1) {
            tree = this.analyzer.analyzeAndPreprocessClass(tree);
        }
        KlassInfo prevKlassInfo = this.kInfo;
        try {
            this.kInfo = new KlassInfo(tree);
            super.visitClassDef(tree);
            if (!this.kInfo.deserializeCases.isEmpty()) {
                int prevPos = this.make.pos;
                try {
                    this.make.at(tree);
                    this.kInfo.addMethod(makeDeserializeMethod(tree.sym));
                    this.make.at(prevPos);
                } catch (Throwable th) {
                    this.make.at(prevPos);
                    throw th;
                }
            }
            List<JCTree> newMethods = this.kInfo.appendedMethodList.toList();
            tree.defs = tree.defs.appendList(newMethods);
            for (JCTree lambda : newMethods) {
                tree.sym.members().enter(((JCTree.JCMethodDecl) lambda).sym);
            }
            this.result = tree;
        } finally {
            this.kInfo = prevKlassInfo;
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitLambda(JCTree.JCLambda tree) {
        List<JCTree.JCExpression> listTypes;
        LambdaAnalyzerPreprocessor.LambdaTranslationContext localContext = (LambdaAnalyzerPreprocessor.LambdaTranslationContext) this.context;
        Symbol.MethodSymbol sym = localContext.translatedSym;
        Type.MethodType lambdaType = (Type.MethodType) sym.type;
        Symbol owner = localContext.owner;
        ListBuffer<Attribute.TypeCompound> ownerTypeAnnos = new ListBuffer<>();
        ListBuffer<Attribute.TypeCompound> lambdaTypeAnnos = new ListBuffer<>();
        for (Attribute.TypeCompound tc : owner.getRawTypeAttributes()) {
            if (tc.position.onLambda == tree) {
                lambdaTypeAnnos.append(tc);
            } else {
                ownerTypeAnnos.append(tc);
            }
        }
        if (lambdaTypeAnnos.nonEmpty()) {
            owner.setTypeAttributes(ownerTypeAnnos.toList());
            sym.setTypeAttributes(lambdaTypeAnnos.toList());
        }
        TreeMaker treeMaker = this.make;
        JCTree.JCModifiers jCModifiersModifiers = this.make.Modifiers(sym.flags_field);
        Name name = sym.name;
        JCTree.JCExpression jCExpressionQualIdent = this.make.QualIdent(lambdaType.mo178getReturnType().tsym);
        List<JCTree.JCTypeParameter> listNil = List.nil();
        List<JCTree.JCVariableDecl> list = localContext.syntheticParams;
        if (lambdaType.mo179getThrownTypes() == null) {
            listTypes = List.nil();
        } else {
            listTypes = this.make.Types(lambdaType.mo179getThrownTypes());
        }
        JCTree.JCMethodDecl lambdaDecl = treeMaker.MethodDef(jCModifiersModifiers, name, jCExpressionQualIdent, listNil, list, listTypes, null, null);
        lambdaDecl.sym = sym;
        lambdaDecl.type = lambdaType;
        lambdaDecl.body = (JCTree.JCBlock) translate(makeLambdaBody(tree, lambdaDecl));
        this.kInfo.addMethod(lambdaDecl);
        ListBuffer<JCTree.JCExpression> syntheticInits = new ListBuffer<>();
        if (localContext.methodReferenceReceiver != null) {
            syntheticInits.append(localContext.methodReferenceReceiver);
        } else if (!sym.isStatic()) {
            syntheticInits.append(makeThis(sym.owner.enclClass().asType(), localContext.owner.enclClass()));
        }
        for (Symbol fv : localContext.getSymbolMap(LambdaSymbolKind.CAPTURED_VAR).keySet()) {
            if (fv != localContext.self) {
                JCTree captured_local = this.make.Ident(fv).setType(fv.type);
                syntheticInits.append((JCTree.JCExpression) captured_local);
            }
        }
        Iterator<Symbol> it = localContext.getSymbolMap(LambdaSymbolKind.CAPTURED_OUTER_THIS).keySet().iterator();
        while (it.hasNext()) {
            JCTree captured_local2 = this.make.QualThis(it.next().type);
            syntheticInits.append((JCTree.JCExpression) captured_local2);
        }
        List<JCTree.JCExpression> indy_args = translate(syntheticInits.toList(), localContext.prev);
        int refKind = referenceKind(sym);
        this.result = makeMetafactoryIndyCall(this.context, refKind, sym, indy_args);
    }

    private JCTree.JCIdent makeThis(Type type, Symbol owner) {
        Symbol.VarSymbol _this = new Symbol.VarSymbol(8589938704L, this.names._this, type, owner);
        return this.make.Ident(_this);
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitReference(JCTree.JCMemberReference tree) {
        JCTree.JCExpression init;
        LambdaAnalyzerPreprocessor.ReferenceTranslationContext localContext = (LambdaAnalyzerPreprocessor.ReferenceTranslationContext) this.context;
        Symbol refSym = localContext.isSignaturePolymorphic() ? localContext.sigPolySym : tree.sym;
        switch (tree.kind) {
            case IMPLICIT_INNER:
            case SUPER:
                init = makeThis(localContext.owner.enclClass().asType(), localContext.owner.enclClass());
                break;
            case BOUND:
                JCTree.JCExpression init2 = tree.getQualifierExpression();
                init = this.attr.makeNullCheck(init2);
                break;
            case UNBOUND:
            case STATIC:
            case TOPLEVEL:
            case ARRAY_CTOR:
                init = null;
                break;
            default:
                throw new InternalError("Should not have an invalid kind");
        }
        List<JCTree.JCExpression> indy_args = init == null ? List.nil() : translate(List.of(init), localContext.prev);
        this.result = makeMetafactoryIndyCall(localContext, localContext.referenceKind(), refSym, indy_args);
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitIdent(JCTree.JCIdent tree) {
        if (this.context == null || !this.analyzer.lambdaIdentSymbolFilter(tree.sym)) {
            super.visitIdent(tree);
            return;
        }
        int prevPos = this.make.pos;
        try {
            this.make.at(tree);
            LambdaAnalyzerPreprocessor.LambdaTranslationContext lambdaContext = (LambdaAnalyzerPreprocessor.LambdaTranslationContext) this.context;
            JCTree ltree = lambdaContext.translate(tree);
            if (ltree != null) {
                this.result = ltree;
            } else {
                super.visitIdent(tree);
            }
        } finally {
            this.make.at(prevPos);
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitSelect(JCTree.JCFieldAccess tree) {
        if (this.context == null || !this.analyzer.lambdaFieldAccessFilter(tree)) {
            super.visitSelect(tree);
            return;
        }
        int prevPos = this.make.pos;
        try {
            this.make.at(tree);
            LambdaAnalyzerPreprocessor.LambdaTranslationContext lambdaContext = (LambdaAnalyzerPreprocessor.LambdaTranslationContext) this.context;
            JCTree ltree = lambdaContext.translate(tree);
            if (ltree != null) {
                this.result = ltree;
            } else {
                super.visitSelect(tree);
            }
        } finally {
            this.make.at(prevPos);
        }
    }

    @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
    public void visitVarDef(JCTree.JCVariableDecl tree) {
        LambdaAnalyzerPreprocessor.LambdaTranslationContext lambdaContext = (LambdaAnalyzerPreprocessor.LambdaTranslationContext) this.context;
        if (this.context != null && lambdaContext.getSymbolMap(LambdaSymbolKind.LOCAL_VAR).containsKey(tree.sym)) {
            tree.init = (JCTree.JCExpression) translate(tree.init);
            tree.sym = (Symbol.VarSymbol) lambdaContext.getSymbolMap(LambdaSymbolKind.LOCAL_VAR).get(tree.sym);
            this.result = tree;
            return;
        }
        if (this.context != null && lambdaContext.getSymbolMap(LambdaSymbolKind.TYPE_VAR).containsKey(tree.sym)) {
            JCTree.JCExpression init = (JCTree.JCExpression) translate(tree.init);
            Symbol.VarSymbol xsym = (Symbol.VarSymbol) lambdaContext.getSymbolMap(LambdaSymbolKind.TYPE_VAR).get(tree.sym);
            int prevPos = this.make.pos;
            try {
                this.result = this.make.at(tree).VarDef(xsym, init);
                this.make.at(prevPos);
                Scope sc = tree.sym.owner.members();
                if (sc != null) {
                    sc.remove(tree.sym);
                    sc.enter(xsym);
                    return;
                }
                return;
            } catch (Throwable th) {
                this.make.at(prevPos);
                throw th;
            }
        }
        super.visitVarDef(tree);
    }

    private JCTree.JCBlock makeLambdaBody(JCTree.JCLambda tree, JCTree.JCMethodDecl lambdaMethodDecl) {
        if (tree.getBodyKind() == LambdaExpressionTree.BodyKind.EXPRESSION) {
            return makeLambdaExpressionBody((JCTree.JCExpression) tree.body, lambdaMethodDecl);
        }
        return makeLambdaStatementBody((JCTree.JCBlock) tree.body, lambdaMethodDecl, tree.canCompleteNormally);
    }

    private JCTree.JCBlock makeLambdaExpressionBody(JCTree.JCExpression expr, JCTree.JCMethodDecl lambdaMethodDecl) {
        Type restype = lambdaMethodDecl.type.mo178getReturnType();
        boolean isLambda_void = expr.type.hasTag(TypeTag.VOID);
        boolean isTarget_void = restype.hasTag(TypeTag.VOID);
        boolean isTarget_Void = this.types.isSameType(restype, this.types.boxedClass(this.syms.voidType).type);
        int prevPos = this.make.pos;
        try {
            if (isTarget_void) {
                JCTree.JCStatement stat = this.make.at(expr).Exec(expr);
                return this.make.Block(0L, List.of(stat));
            }
            if (!isLambda_void || !isTarget_Void) {
                JCTree.JCExpression retExpr = this.transTypes.coerce(this.attrEnv, expr, restype);
                return this.make.at(retExpr).Block(0L, List.of(this.make.Return(retExpr)));
            }
            ListBuffer<JCTree.JCStatement> stats = new ListBuffer<>();
            stats.append(this.make.at(expr).Exec(expr));
            stats.append(this.make.Return(this.make.Literal(TypeTag.BOT, null).setType(this.syms.botType)));
            return this.make.Block(0L, stats.toList());
        } finally {
            this.make.at(prevPos);
        }
    }

    private JCTree.JCBlock makeLambdaStatementBody(JCTree.JCBlock block, final JCTree.JCMethodDecl lambdaMethodDecl, boolean completeNormally) {
        final Type restype = lambdaMethodDecl.type.mo178getReturnType();
        final boolean isTarget_void = restype.hasTag(TypeTag.VOID);
        boolean isTarget_Void = this.types.isSameType(restype, this.types.boxedClass(this.syms.voidType).type);
        JCTree.JCBlock trans_block = (JCTree.JCBlock) new TreeTranslator() { // from class: com.sun.tools.javac.comp.LambdaToMethod.1LambdaBodyTranslator
            @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitClassDef(JCTree.JCClassDecl tree) {
                this.result = tree;
            }

            @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitLambda(JCTree.JCLambda tree) {
                this.result = tree;
            }

            @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
            public void visitReturn(JCTree.JCReturn tree) {
                boolean isLambda_void = tree.expr == null;
                if (isTarget_void && !isLambda_void) {
                    Symbol.VarSymbol loc = LambdaToMethod.this.makeSyntheticVar(0L, LambdaToMethod.this.names.fromString("$loc"), tree.expr.type, lambdaMethodDecl.sym);
                    JCTree.JCVariableDecl varDef = LambdaToMethod.this.make.VarDef(loc, tree.expr);
                    this.result = LambdaToMethod.this.make.Block(0L, List.of((JCTree.JCReturn) varDef, LambdaToMethod.this.make.Return(null)));
                } else if (!isTarget_void || !isLambda_void) {
                    tree.expr = LambdaToMethod.this.transTypes.coerce(LambdaToMethod.this.attrEnv, tree.expr, restype);
                    this.result = tree;
                } else {
                    this.result = tree;
                }
            }
        }.translate(block);
        if (completeNormally && isTarget_Void) {
            trans_block.stats = trans_block.stats.append(this.make.Return(this.make.Literal(TypeTag.BOT, null).setType(this.syms.botType)));
        }
        return trans_block;
    }

    private JCTree.JCMethodDecl makeDeserializeMethod(Symbol kSym) {
        ListBuffer<JCTree.JCCase> cases = new ListBuffer<>();
        ListBuffer<JCTree.JCBreak> breaks = new ListBuffer<>();
        for (Map.Entry<String, ListBuffer<JCTree.JCStatement>> entry : this.kInfo.deserializeCases.entrySet()) {
            JCTree.JCBreak br = this.make.Break(null);
            breaks.add(br);
            List<JCTree.JCStatement> stmts = entry.getValue().append(br).toList();
            cases.add(this.make.Case(this.make.Literal(entry.getKey()), stmts));
        }
        JCTree.JCSwitch sw = this.make.Switch(deserGetter("getImplMethodName", this.syms.stringType), cases.toList());
        Iterator<JCTree.JCBreak> it = breaks.iterator();
        while (it.hasNext()) {
            it.next().target = sw;
        }
        JCTree.JCBlock body = this.make.Block(0L, List.of((JCTree.JCThrow) sw, this.make.Throw(makeNewClass(this.syms.illegalArgumentExceptionType, List.of(this.make.Literal("Invalid lambda deserialization"))))));
        JCTree.JCMethodDecl deser = this.make.MethodDef(this.make.Modifiers(this.kInfo.deserMethodSym.flags()), this.names.deserializeLambda, this.make.QualIdent(this.kInfo.deserMethodSym.getReturnType().tsym), List.nil(), List.of(this.make.VarDef(this.kInfo.deserParamSym, null)), List.nil(), body, null);
        deser.sym = this.kInfo.deserMethodSym;
        deser.type = this.kInfo.deserMethodSym.type;
        return deser;
    }

    JCTree.JCNewClass makeNewClass(Type ctype, List<JCTree.JCExpression> args, Symbol cons) {
        JCTree.JCNewClass tree = this.make.NewClass(null, null, this.make.QualIdent(ctype.tsym), args, null);
        tree.constructor = cons;
        tree.type = ctype;
        return tree;
    }

    JCTree.JCNewClass makeNewClass(Type ctype, List<JCTree.JCExpression> args) {
        return makeNewClass(ctype, args, this.rs.resolveConstructor(null, this.attrEnv, ctype, TreeInfo.types(args), List.nil()));
    }

    private void addDeserializationCase(int implMethodKind, Symbol refSym, Type targetType, Symbol.MethodSymbol samSym, JCDiagnostic.DiagnosticPosition pos, List<Object> staticArgs, Type.MethodType indyType) {
        String functionalInterfaceClass = classSig(targetType);
        String functionalInterfaceMethodName = samSym.getSimpleName().toString();
        String functionalInterfaceMethodSignature = typeSig(this.types.erasure(samSym.type));
        String implClass = classSig(this.types.erasure(refSym.owner.type));
        String implMethodName = refSym.getQualifiedName().toString();
        String implMethodSignature = typeSig(this.types.erasure(refSym.type));
        JCTree.JCExpression kindTest = eqTest(this.syms.intType, deserGetter("getImplMethodKind", this.syms.intType), this.make.Literal(Integer.valueOf(implMethodKind)));
        ListBuffer<JCTree.JCExpression> serArgs = new ListBuffer<>();
        Iterator<Type> it = indyType.mo176getParameterTypes().iterator();
        int i = 0;
        while (it.hasNext()) {
            Type t = it.next();
            Iterator<Type> it2 = it;
            List<JCTree.JCExpression> indexAsArg = new ListBuffer().append(this.make.Literal(Integer.valueOf(i))).toList();
            List<Type> argTypes = new ListBuffer().append(this.syms.intType).toList();
            serArgs.add(this.make.TypeCast(this.types.erasure(t), deserGetter("getCapturedArg", this.syms.objectType, argTypes, indexAsArg)));
            i++;
            it = it2;
            implMethodName = implMethodName;
        }
        String implMethodName2 = implMethodName;
        JCTree.JCStatement jCStatementIf = this.make.If(deserTest(deserTest(deserTest(deserTest(deserTest(kindTest, "getFunctionalInterfaceClass", functionalInterfaceClass), "getFunctionalInterfaceMethodName", functionalInterfaceMethodName), "getFunctionalInterfaceMethodSignature", functionalInterfaceMethodSignature), "getImplClass", implClass), "getImplMethodSignature", implMethodSignature), this.make.Return(makeIndyCall(pos, this.syms.lambdaMetafactory, this.names.altMetafactory, staticArgs, indyType, serArgs.toList(), samSym.name)), null);
        ListBuffer<JCTree.JCStatement> stmts = (ListBuffer) this.kInfo.deserializeCases.get(implMethodName2);
        if (stmts == null) {
            stmts = new ListBuffer<>();
            this.kInfo.deserializeCases.put(implMethodName2, stmts);
        }
        stmts.append(jCStatementIf);
    }

    private JCTree.JCExpression eqTest(Type argType, JCTree.JCExpression arg1, JCTree.JCExpression arg2) {
        JCTree.JCBinary testExpr = this.make.Binary(JCTree.Tag.EQ, arg1, arg2);
        testExpr.operator = this.rs.resolveBinaryOperator(null, JCTree.Tag.EQ, this.attrEnv, argType, argType);
        testExpr.setType((Type) this.syms.booleanType);
        return testExpr;
    }

    private JCTree.JCExpression deserTest(JCTree.JCExpression prev, String func, String lit) {
        Type.MethodType eqmt = new Type.MethodType(List.of(this.syms.objectType), this.syms.booleanType, List.nil(), this.syms.methodClass);
        Symbol eqsym = this.rs.resolveQualifiedMethod(null, this.attrEnv, this.syms.objectType, this.names.equals, List.of(this.syms.objectType), List.nil());
        JCTree.JCMethodInvocation eqtest = this.make.Apply(List.nil(), this.make.Select(deserGetter(func, this.syms.stringType), eqsym).setType((Type) eqmt), List.of(this.make.Literal(lit)));
        eqtest.setType((Type) this.syms.booleanType);
        JCTree.JCBinary compound = this.make.Binary(JCTree.Tag.AND, prev, eqtest);
        compound.operator = this.rs.resolveBinaryOperator(null, JCTree.Tag.AND, this.attrEnv, this.syms.booleanType, this.syms.booleanType);
        compound.setType((Type) this.syms.booleanType);
        return compound;
    }

    private JCTree.JCExpression deserGetter(String func, Type type) {
        return deserGetter(func, type, List.nil(), List.nil());
    }

    private JCTree.JCExpression deserGetter(String func, Type type, List<Type> argTypes, List<JCTree.JCExpression> args) {
        Type.MethodType getmt = new Type.MethodType(argTypes, type, List.nil(), this.syms.methodClass);
        Symbol getsym = this.rs.resolveQualifiedMethod(null, this.attrEnv, this.syms.serializedLambdaType, this.names.fromString(func), argTypes, List.nil());
        return this.make.Apply(List.nil(), this.make.Select(this.make.Ident(this.kInfo.deserParamSym).setType(this.syms.serializedLambdaType), getsym).setType((Type) getmt), args).setType(type);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Symbol.MethodSymbol makePrivateSyntheticMethod(long flags, Name name, Type type, Symbol owner) {
        return new Symbol.MethodSymbol(4096 | flags | 2, name, type, owner);
    }

    private Symbol.VarSymbol makeSyntheticVar(long flags, String name, Type type, Symbol owner) {
        return makeSyntheticVar(flags, this.names.fromString(name), type, owner);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Symbol.VarSymbol makeSyntheticVar(long flags, Name name, Type type, Symbol owner) {
        return new Symbol.VarSymbol(flags | 4096, name, type, owner);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void setVarargsIfNeeded(JCTree tree, Type varargsElement) {
        if (varargsElement != null) {
            switch (tree.getTag()) {
                case APPLY:
                    ((JCTree.JCMethodInvocation) tree).varargsElement = varargsElement;
                    return;
                case NEWCLASS:
                    ((JCTree.JCNewClass) tree).varargsElement = varargsElement;
                    return;
                default:
                    throw new AssertionError();
            }
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public List<JCTree.JCExpression> convertArgs(Symbol meth, List<JCTree.JCExpression> args, Type varargsElement) {
        Assert.check(meth.kind == 16);
        List<Type> formals = this.types.erasure(meth.type).mo176getParameterTypes();
        if (varargsElement != null) {
            Assert.check((meth.flags() & Flags.VARARGS) != 0);
        }
        return this.transTypes.translateArgs(args, formals, varargsElement, this.attrEnv);
    }

    private class MemberReferenceToLambda {
        private final LambdaAnalyzerPreprocessor.ReferenceTranslationContext localContext;
        private final Symbol owner;
        private final JCTree.JCMemberReference tree;
        private final ListBuffer<JCTree.JCExpression> args = new ListBuffer<>();
        private final ListBuffer<JCTree.JCVariableDecl> params = new ListBuffer<>();
        private JCTree.JCExpression receiverExpression = null;

        MemberReferenceToLambda(JCTree.JCMemberReference tree, LambdaAnalyzerPreprocessor.ReferenceTranslationContext localContext, Symbol owner) {
            this.tree = tree;
            this.localContext = localContext;
            this.owner = owner;
        }

        JCTree.JCLambda lambda() {
            JCTree.JCExpression expr;
            int prevPos = LambdaToMethod.this.make.pos;
            try {
                LambdaToMethod.this.make.at(this.tree);
                Symbol.VarSymbol rcvr = addParametersReturnReceiver();
                if (this.tree.getMode() == MemberReferenceTree.ReferenceMode.INVOKE) {
                    expr = expressionInvoke(rcvr);
                } else {
                    expr = expressionNew();
                }
                JCTree.JCLambda slam = LambdaToMethod.this.make.Lambda(this.params.toList(), expr);
                slam.targets = this.tree.targets;
                slam.type = this.tree.type;
                slam.pos = this.tree.pos;
                return slam;
            } finally {
                LambdaToMethod.this.make.at(prevPos);
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        Symbol.VarSymbol addParametersReturnReceiver() {
            Symbol.VarSymbol rcvr;
            Type samDesc = this.localContext.bridgedRefSig();
            List listMo176getParameterTypes = samDesc.mo176getParameterTypes();
            List listMo176getParameterTypes2 = this.tree.getDescriptorType(LambdaToMethod.this.types).mo176getParameterTypes();
            switch (this.tree.kind) {
                case BOUND:
                    rcvr = addParameter("rec$", this.tree.getQualifierExpression().type, false);
                    this.receiverExpression = LambdaToMethod.this.attr.makeNullCheck(this.tree.getQualifierExpression());
                    break;
                case UNBOUND:
                    rcvr = addParameter("rec$", samDesc.mo176getParameterTypes().head, false);
                    listMo176getParameterTypes = listMo176getParameterTypes.tail;
                    listMo176getParameterTypes2 = listMo176getParameterTypes2.tail;
                    break;
                default:
                    rcvr = null;
                    break;
            }
            List listMo176getParameterTypes3 = this.tree.sym.type.mo176getParameterTypes();
            int implSize = listMo176getParameterTypes3.size();
            int samSize = listMo176getParameterTypes.size();
            int last = this.localContext.needsVarArgsConversion() ? implSize - 1 : implSize;
            boolean checkForIntersection = this.tree.varargsElement != null || implSize == listMo176getParameterTypes2.size();
            for (int i = 0; listMo176getParameterTypes3.nonEmpty() && i < last; i++) {
                Type parmType = (Type) listMo176getParameterTypes3.head;
                if (checkForIntersection && ((Type) listMo176getParameterTypes2.head).getKind() == TypeKind.TYPEVAR) {
                    Type.TypeVar tv = (Type.TypeVar) listMo176getParameterTypes2.head;
                    if (tv.bound.getKind() == TypeKind.INTERSECTION) {
                        parmType = (Type) listMo176getParameterTypes.head;
                    }
                }
                addParameter("x$" + i, parmType, true);
                listMo176getParameterTypes3 = listMo176getParameterTypes3.tail;
                listMo176getParameterTypes = listMo176getParameterTypes.tail;
                listMo176getParameterTypes2 = listMo176getParameterTypes2.tail;
            }
            for (int i2 = last; i2 < samSize; i2++) {
                addParameter("xva$" + i2, this.tree.varargsElement, true);
            }
            return rcvr;
        }

        JCTree.JCExpression getReceiverExpression() {
            return this.receiverExpression;
        }

        private JCTree.JCExpression makeReceiver(Symbol.VarSymbol rcvr) {
            if (rcvr == null) {
                return null;
            }
            JCTree.JCExpression rcvrExpr = LambdaToMethod.this.make.Ident(rcvr);
            Type rcvrType = this.tree.ownerAccessible ? this.tree.sym.enclClass().type : this.tree.expr.type;
            if (rcvrType == LambdaToMethod.this.syms.arrayClass.type) {
                rcvrType = this.tree.getQualifierExpression().type;
            }
            if (!rcvr.type.tsym.isSubClass(rcvrType.tsym, LambdaToMethod.this.types)) {
                return LambdaToMethod.this.make.TypeCast(LambdaToMethod.this.make.Type(rcvrType), rcvrExpr).setType(rcvrType);
            }
            return rcvrExpr;
        }

        private JCTree.JCExpression expressionInvoke(Symbol.VarSymbol rcvr) {
            JCTree.JCExpression qualifier;
            if (rcvr != null) {
                qualifier = makeReceiver(rcvr);
            } else {
                qualifier = this.tree.getQualifierExpression();
            }
            JCTree.JCFieldAccess select = LambdaToMethod.this.make.Select(qualifier, this.tree.sym.name);
            select.sym = this.tree.sym;
            select.type = this.tree.sym.erasure(LambdaToMethod.this.types);
            JCTree.JCExpression apply = LambdaToMethod.this.transTypes.coerce(LambdaToMethod.this.make.Apply(List.nil(), select, LambdaToMethod.this.convertArgs(this.tree.sym, this.args.toList(), this.tree.varargsElement)).setType(this.tree.sym.erasure(LambdaToMethod.this.types).mo178getReturnType()), this.localContext.generatedRefSig().mo178getReturnType());
            LambdaToMethod.this.setVarargsIfNeeded(apply, this.tree.varargsElement);
            return apply;
        }

        private JCTree.JCExpression expressionNew() {
            if (this.tree.kind == JCTree.JCMemberReference.ReferenceKind.ARRAY_CTOR) {
                JCTree.JCNewArray newArr = LambdaToMethod.this.make.NewArray(LambdaToMethod.this.make.Type(LambdaToMethod.this.types.elemtype(this.tree.getQualifierExpression().type)), List.of(LambdaToMethod.this.make.Ident(this.params.first())), null);
                newArr.type = this.tree.getQualifierExpression().type;
                return newArr;
            }
            JCTree.JCNewClass newClass = LambdaToMethod.this.make.NewClass(null, List.nil(), LambdaToMethod.this.make.Type(this.tree.getQualifierExpression().type), LambdaToMethod.this.convertArgs(this.tree.sym, this.args.toList(), this.tree.varargsElement), null);
            newClass.constructor = this.tree.sym;
            newClass.constructorType = this.tree.sym.erasure(LambdaToMethod.this.types);
            newClass.type = this.tree.getQualifierExpression().type;
            LambdaToMethod.this.setVarargsIfNeeded(newClass, this.tree.varargsElement);
            return newClass;
        }

        private Symbol.VarSymbol addParameter(String name, Type p, boolean genArg) {
            Symbol.VarSymbol vsym = new Symbol.VarSymbol(8589938688L, LambdaToMethod.this.names.fromString(name), p, this.owner);
            vsym.pos = this.tree.pos;
            this.params.append(LambdaToMethod.this.make.VarDef(vsym, null));
            if (genArg) {
                this.args.append(LambdaToMethod.this.make.Ident(vsym));
            }
            return vsym;
        }
    }

    private Type.MethodType typeToMethodType(Type mt) {
        Type type = this.types.erasure(mt);
        return new Type.MethodType(type.mo176getParameterTypes(), type.mo178getReturnType(), type.mo179getThrownTypes(), this.syms.methodClass);
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r13v0, types: [T extends com.sun.tools.javac.tree.JCTree$JCFunctionalExpression, com.sun.tools.javac.tree.JCTree$JCFunctionalExpression, com.sun.tools.javac.util.JCDiagnostic$DiagnosticPosition] */
    /* JADX WARN: Type inference failed for: r18v0 */
    /* JADX WARN: Type inference failed for: r18v1, types: [int] */
    /* JADX WARN: Type inference failed for: r18v2 */
    /* JADX WARN: Type inference failed for: r1v19 */
    /* JADX WARN: Type inference failed for: r1v39 */
    /* JADX WARN: Type inference failed for: r1v40 */
    /* JADX WARN: Type inference failed for: r22v0, types: [com.sun.tools.javac.comp.LambdaToMethod] */
    private JCTree.JCExpression makeMetafactoryIndyCall(LambdaAnalyzerPreprocessor.TranslationContext<?> translationContext, int i, Symbol symbol, List<JCTree.JCExpression> list) throws Throwable {
        ?? r18;
        List list2;
        int i2;
        ?? r13 = translationContext.tree;
        Symbol.MethodSymbol methodSymbol = (Symbol.MethodSymbol) this.types.findDescriptorSymbol(r13.type.tsym);
        List listOf = List.of(typeToMethodType(methodSymbol.type), (Type.MethodType) new Pool.MethodHandle(i, symbol, this.types), typeToMethodType(r13.getDescriptorType(this.types)));
        ListBuffer listBuffer = new ListBuffer();
        Iterator<JCTree.JCExpression> it = list.iterator();
        while (it.hasNext()) {
            listBuffer.append(it.next().type);
        }
        Type.MethodType methodType = new Type.MethodType(listBuffer.toList(), r13.type, List.nil(), this.syms.methodClass);
        Name name = translationContext.needsAltMetafactory() ? this.names.altMetafactory : this.names.metafactory;
        if (translationContext.needsAltMetafactory()) {
            ListBuffer listBuffer2 = new ListBuffer();
            for (Type type : r13.targets.tail) {
                if (type.tsym != this.syms.serializableType.tsym) {
                    listBuffer2.append(type.tsym);
                }
            }
            boolean zIsSerializable = translationContext.isSerializable();
            boolean zNonEmpty = listBuffer2.nonEmpty();
            boolean zNonEmpty2 = translationContext.bridges.nonEmpty();
            ?? r1 = zIsSerializable;
            if (zNonEmpty) {
                r1 = (zIsSerializable ? 1 : 0) | 2;
            }
            if (!zNonEmpty2) {
                r18 = r1;
            } else {
                r18 = r1 | 4;
            }
            List listAppend = listOf.append(Integer.valueOf((int) r18));
            if (zNonEmpty) {
                listAppend = listAppend.append(Integer.valueOf(listBuffer2.length())).appendList(listBuffer2.toList());
            }
            if (!zNonEmpty2) {
                list2 = listAppend;
            } else {
                List listAppend2 = listAppend.append(Integer.valueOf(translationContext.bridges.length() - 1));
                for (Symbol symbol2 : translationContext.bridges) {
                    if (!this.types.isSameType(symbol2.erasure(this.types), methodSymbol.erasure(this.types))) {
                        listAppend2 = listAppend2.append(symbol2.erasure(this.types));
                    }
                }
                list2 = listAppend2;
            }
            if (translationContext.isSerializable()) {
                int i3 = this.make.pos;
                try {
                    this.make.at(this.kInfo.clazz);
                    i2 = i3;
                    try {
                        addDeserializationCase(i, symbol, r13.type, methodSymbol, r13, list2, methodType);
                        this.make.at(i2);
                    } catch (Throwable th) {
                        th = th;
                        this.make.at(i2);
                        throw th;
                    }
                } catch (Throwable th2) {
                    th = th2;
                    i2 = i3;
                }
            }
            listOf = list2;
        }
        return makeIndyCall(r13, this.syms.lambdaMetafactory, name, listOf, methodType, list, methodSymbol.name);
    }

    private JCTree.JCExpression makeIndyCall(JCDiagnostic.DiagnosticPosition pos, Type site, Name bsmName, List<Object> staticArgs, Type.MethodType indyType, List<JCTree.JCExpression> indyArgs, Name methName) throws Throwable {
        Symbol.DynamicMethodSymbol dynSym;
        int prevPos = this.make.pos;
        try {
            try {
                this.make.at(pos);
                try {
                    List<Type> bsm_staticArgs = List.of(this.syms.methodHandleLookupType, this.syms.stringType, this.syms.methodTypeType).appendList(bsmStaticArgToTypes(staticArgs));
                    Symbol bsm = this.rs.resolveInternalMethod(pos, this.attrEnv, site, bsmName, bsm_staticArgs, List.nil());
                    dynSym = new Symbol.DynamicMethodSymbol(methName, this.syms.noSymbol, bsm.isStatic() ? 6 : 5, (Symbol.MethodSymbol) bsm, indyType, staticArgs.toArray());
                    try {
                    } catch (Throwable th) {
                        th = th;
                        this.make.at(prevPos);
                        throw th;
                    }
                } catch (Throwable th2) {
                    th = th2;
                }
            } catch (Throwable th3) {
                th = th3;
                this.make.at(prevPos);
                throw th;
            }
        } catch (Throwable th4) {
            th = th4;
        }
        try {
            JCTree.JCFieldAccess qualifier = this.make.Select(this.make.QualIdent(site.tsym), bsmName);
            qualifier.sym = dynSym;
            qualifier.type = indyType.mo178getReturnType();
            try {
                JCTree.JCMethodInvocation proxyCall = this.make.Apply(List.nil(), qualifier, indyArgs);
                proxyCall.type = indyType.mo178getReturnType();
                this.make.at(prevPos);
                return proxyCall;
            } catch (Throwable th5) {
                th = th5;
                this.make.at(prevPos);
                throw th;
            }
        } catch (Throwable th6) {
            th = th6;
            this.make.at(prevPos);
            throw th;
        }
    }

    private List<Type> bsmStaticArgToTypes(List<Object> args) {
        ListBuffer<Type> argtypes = new ListBuffer<>();
        for (Object arg : args) {
            argtypes.append(bsmStaticArgToType(arg));
        }
        return argtypes.toList();
    }

    private Type bsmStaticArgToType(Object arg) {
        Assert.checkNonNull(arg);
        if (arg instanceof Symbol.ClassSymbol) {
            return this.syms.classType;
        }
        if (arg instanceof Integer) {
            return this.syms.intType;
        }
        if (arg instanceof Long) {
            return this.syms.longType;
        }
        if (arg instanceof Float) {
            return this.syms.floatType;
        }
        if (arg instanceof Double) {
            return this.syms.doubleType;
        }
        if (arg instanceof String) {
            return this.syms.stringType;
        }
        if (arg instanceof Pool.MethodHandle) {
            return this.syms.methodHandleType;
        }
        if (arg instanceof Type.MethodType) {
            return this.syms.methodTypeType;
        }
        Assert.error("bad static arg " + arg.getClass());
        return null;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public int referenceKind(Symbol refSym) {
        if (refSym.isConstructor()) {
            return 8;
        }
        if (refSym.isStatic()) {
            return 6;
        }
        if ((refSym.flags() & 2) != 0) {
            return 7;
        }
        if (refSym.enclClass().isInterface()) {
            return 9;
        }
        return 5;
    }

    class LambdaAnalyzerPreprocessor extends TreeTranslator {
        private List<Frame> frameStack;
        private Map<Symbol, JCTree.JCClassDecl> localClassDefs;
        private List<Symbol.ClassSymbol> typesUnderConstruction;
        private int lambdaCount = 0;
        private SyntheticMethodNameCounter syntheticMethodNameCounts = new SyntheticMethodNameCounter();
        private Map<Symbol.ClassSymbol, Symbol> clinits = new HashMap();

        LambdaAnalyzerPreprocessor() {
        }

        static /* synthetic */ int access$3408(LambdaAnalyzerPreprocessor x0) {
            int i = x0.lambdaCount;
            x0.lambdaCount = i + 1;
            return i;
        }

        private class SyntheticMethodNameCounter {
            private Map<String, Integer> map;

            private SyntheticMethodNameCounter() {
                this.map = new HashMap();
            }

            int getIndex(StringBuilder buf) {
                String temp = buf.toString();
                Integer count = this.map.get(temp);
                if (count == null) {
                    count = 0;
                }
                Integer count2 = Integer.valueOf(count.intValue() + 1);
                this.map.put(temp, count2);
                return count2.intValue();
            }
        }

        /* JADX INFO: Access modifiers changed from: private */
        public JCTree.JCClassDecl analyzeAndPreprocessClass(JCTree.JCClassDecl tree) {
            this.frameStack = List.nil();
            this.typesUnderConstruction = List.nil();
            this.localClassDefs = new HashMap();
            return (JCTree.JCClassDecl) translate(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation tree) {
            List<Symbol.ClassSymbol> previousNascentTypes = this.typesUnderConstruction;
            try {
                Name methName = TreeInfo.name(tree.meth);
                if (methName == LambdaToMethod.this.names._this || methName == LambdaToMethod.this.names._super) {
                    this.typesUnderConstruction = this.typesUnderConstruction.prepend(currentClass());
                }
                super.visitApply(tree);
            } finally {
                this.typesUnderConstruction = previousNascentTypes;
            }
        }

        /* JADX INFO: Access modifiers changed from: private */
        public Symbol.ClassSymbol currentClass() {
            for (Frame frame : this.frameStack) {
                if (frame.tree.hasTag(JCTree.Tag.CLASSDEF)) {
                    JCTree.JCClassDecl cdef = (JCTree.JCClassDecl) frame.tree;
                    return cdef.sym;
                }
            }
            return null;
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBlock(JCTree.JCBlock tree) {
            List<Frame> prevStack = this.frameStack;
            try {
                if (this.frameStack.nonEmpty() && this.frameStack.head.tree.hasTag(JCTree.Tag.CLASSDEF)) {
                    this.frameStack = this.frameStack.prepend(new Frame(tree));
                }
                super.visitBlock(tree);
            } finally {
                this.frameStack = prevStack;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            List<Frame> prevStack = this.frameStack;
            int prevLambdaCount = this.lambdaCount;
            SyntheticMethodNameCounter prevSyntheticMethodNameCounts = this.syntheticMethodNameCounts;
            Map<Symbol.ClassSymbol, Symbol> prevClinits = this.clinits;
            DiagnosticSource prevSource = LambdaToMethod.this.log.currentSource();
            try {
                LambdaToMethod.this.log.useSource(tree.sym.sourcefile);
                this.lambdaCount = 0;
                this.syntheticMethodNameCounts = new SyntheticMethodNameCounter();
                prevClinits = new HashMap();
                if (tree.sym.owner.kind == 16) {
                    this.localClassDefs.put(tree.sym, tree);
                }
                if (directlyEnclosingLambda() != null) {
                    tree.sym.owner = owner();
                    if (tree.sym.hasOuterInstance()) {
                        for (TranslationContext<?> localContext = context(); localContext != null; localContext = localContext.prev) {
                            if (localContext.tree.getTag() == JCTree.Tag.LAMBDA) {
                                ((LambdaTranslationContext) localContext).addSymbol(tree.sym.type.getEnclosingType().tsym, LambdaSymbolKind.CAPTURED_THIS);
                            }
                        }
                    }
                }
                this.frameStack = this.frameStack.prepend(new Frame(tree));
                super.visitClassDef(tree);
            } finally {
                LambdaToMethod.this.log.useSource(prevSource.getFile());
                this.frameStack = prevStack;
                this.lambdaCount = prevLambdaCount;
                this.syntheticMethodNameCounts = prevSyntheticMethodNameCounts;
                this.clinits = prevClinits;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            if (context() != null && lambdaIdentSymbolFilter(tree.sym)) {
                if (tree.sym.kind == 4 && tree.sym.owner.kind == 16 && tree.type.constValue() == null) {
                    for (TranslationContext<?> localContext = context(); localContext != null; localContext = localContext.prev) {
                        if (localContext.tree.getTag() == JCTree.Tag.LAMBDA) {
                            if (capturedDecl(localContext.depth, tree.sym) == null) {
                                break;
                            } else {
                                ((LambdaTranslationContext) localContext).addSymbol(tree.sym, LambdaSymbolKind.CAPTURED_VAR);
                            }
                        }
                    }
                } else if (tree.sym.owner.kind == 2) {
                    for (TranslationContext<?> localContext2 = context(); localContext2 != null; localContext2 = localContext2.prev) {
                        if (localContext2.tree.hasTag(JCTree.Tag.LAMBDA)) {
                            JCTree block = capturedDecl(localContext2.depth, tree.sym);
                            if (block != null) {
                                switch (block.getTag()) {
                                    case CLASSDEF:
                                        JCTree.JCClassDecl cdecl = (JCTree.JCClassDecl) block;
                                        ((LambdaTranslationContext) localContext2).addSymbol(cdecl.sym, LambdaSymbolKind.CAPTURED_THIS);
                                        break;
                                    default:
                                        Assert.error("bad block kind");
                                        break;
                                }
                            }
                        }
                    }
                }
            }
            super.visitIdent(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            analyzeLambda(tree, "lambda.stat");
        }

        private void analyzeLambda(JCTree.JCLambda tree, JCTree.JCExpression methodReferenceReceiver) {
            JCTree.JCExpression rcvr = (JCTree.JCExpression) translate(methodReferenceReceiver);
            LambdaTranslationContext context = analyzeLambda(tree, "mref.stat.1");
            if (rcvr != null) {
                context.methodReferenceReceiver = rcvr;
            }
        }

        private LambdaTranslationContext analyzeLambda(JCTree.JCLambda tree, String statKey) {
            List<Frame> prevStack = this.frameStack;
            try {
                LambdaTranslationContext context = new LambdaTranslationContext(tree);
                if (LambdaToMethod.this.dumpLambdaToMethodStats) {
                    LambdaToMethod.this.log.note(tree, statKey, Boolean.valueOf(context.needsAltMetafactory()), context.translatedSym);
                }
                this.frameStack = this.frameStack.prepend(new Frame(tree));
                for (JCTree.JCVariableDecl param : tree.params) {
                    context.addSymbol(param.sym, LambdaSymbolKind.PARAM);
                    this.frameStack.head.addLocal(param.sym);
                }
                LambdaToMethod.this.contextMap.put(tree, context);
                super.visitLambda(tree);
                context.complete();
                return context;
            } finally {
                this.frameStack = prevStack;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl tree) {
            List<Frame> prevStack = this.frameStack;
            try {
                this.frameStack = this.frameStack.prepend(new Frame(tree));
                super.visitMethodDef(tree);
            } finally {
                this.frameStack = prevStack;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            Symbol.TypeSymbol def = tree.type.tsym;
            boolean inReferencedClass = currentlyInClass(def);
            boolean isLocal = def.isLocal();
            if ((inReferencedClass && isLocal) || lambdaNewClassFilter(context(), tree)) {
                for (TranslationContext<?> localContext = context(); localContext != null; localContext = localContext.prev) {
                    if (localContext.tree.getTag() == JCTree.Tag.LAMBDA) {
                        ((LambdaTranslationContext) localContext).addSymbol(tree.type.getEnclosingType().tsym, LambdaSymbolKind.CAPTURED_THIS);
                    }
                }
            }
            TranslationContext<?> localContext2 = context();
            if (localContext2 != null && !inReferencedClass && isLocal) {
                LambdaTranslationContext lambdaContext = (LambdaTranslationContext) context();
                captureLocalClassDefs(def, lambdaContext);
            }
            super.visitNewClass(tree);
        }

        void captureLocalClassDefs(Symbol csym, LambdaTranslationContext lambdaContext) {
            JCTree.JCClassDecl localCDef = this.localClassDefs.get(csym);
            if (localCDef != null && lambdaContext.freeVarProcessedLocalClasses.add(csym)) {
                Lower lower = LambdaToMethod.this.lower;
                lower.getClass();
                Lower.BasicFreeVarCollector fvc = new Lower.BasicFreeVarCollector(lower, lambdaContext) { // from class: com.sun.tools.javac.comp.LambdaToMethod.LambdaAnalyzerPreprocessor.1
                    final /* synthetic */ LambdaTranslationContext val$lambdaContext;

                    /* JADX WARN: 'super' call moved to the top of the method (can break code semantics) */
                    {
                        super();
                        this.val$lambdaContext = lambdaContext;
                        lower.getClass();
                    }

                    @Override // com.sun.tools.javac.comp.Lower.BasicFreeVarCollector
                    void addFreeVars(Symbol.ClassSymbol c) {
                        LambdaAnalyzerPreprocessor.this.captureLocalClassDefs(c, this.val$lambdaContext);
                    }

                    @Override // com.sun.tools.javac.comp.Lower.BasicFreeVarCollector
                    void visitSymbol(Symbol sym) {
                        if (sym.kind == 4 && sym.owner.kind == 16 && ((Symbol.VarSymbol) sym).getConstValue() == null) {
                            for (TranslationContext<?> localContext = LambdaAnalyzerPreprocessor.this.context(); localContext != null; localContext = localContext.prev) {
                                if (localContext.tree.getTag() == JCTree.Tag.LAMBDA) {
                                    JCTree block = LambdaAnalyzerPreprocessor.this.capturedDecl(localContext.depth, sym);
                                    if (block != null) {
                                        ((LambdaTranslationContext) localContext).addSymbol(sym, LambdaSymbolKind.CAPTURED_VAR);
                                    } else {
                                        return;
                                    }
                                }
                            }
                        }
                    }
                };
                fvc.scan(localCDef);
            }
        }

        boolean currentlyInClass(Symbol csym) {
            for (Frame frame : this.frameStack) {
                if (frame.tree.hasTag(JCTree.Tag.CLASSDEF)) {
                    JCTree.JCClassDecl cdef = (JCTree.JCClassDecl) frame.tree;
                    if (cdef.sym == csym) {
                        return true;
                    }
                }
            }
            return false;
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReference(JCTree.JCMemberReference tree) {
            ReferenceTranslationContext rcontext = new ReferenceTranslationContext(tree);
            LambdaToMethod.this.contextMap.put(tree, rcontext);
            if (rcontext.needsConversionToLambda()) {
                MemberReferenceToLambda conv = LambdaToMethod.this.new MemberReferenceToLambda(tree, rcontext, owner());
                analyzeLambda(conv.lambda(), conv.getReceiverExpression());
            } else {
                super.visitReference(tree);
                if (LambdaToMethod.this.dumpLambdaToMethodStats) {
                    LambdaToMethod.this.log.note(tree, "mref.stat", Boolean.valueOf(rcontext.needsAltMetafactory()), null);
                }
            }
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            if (context() != null && tree.sym.kind == 4 && (tree.sym.name == LambdaToMethod.this.names._this || tree.sym.name == LambdaToMethod.this.names._super)) {
                for (TranslationContext<?> localContext = context(); localContext != null; localContext = localContext.prev) {
                    if (localContext.tree.hasTag(JCTree.Tag.LAMBDA)) {
                        JCTree.JCClassDecl clazz = (JCTree.JCClassDecl) capturedDecl(localContext.depth, tree.sym);
                        if (clazz == null) {
                            break;
                        } else {
                            ((LambdaTranslationContext) localContext).addSymbol(clazz.sym, LambdaSymbolKind.CAPTURED_THIS);
                        }
                    }
                }
            }
            super.visitSelect(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeTranslator, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            TranslationContext<?> context = context();
            LambdaTranslationContext ltc = (context == null || !(context instanceof LambdaTranslationContext)) ? null : (LambdaTranslationContext) context;
            if (ltc != null) {
                if (this.frameStack.head.tree.hasTag(JCTree.Tag.LAMBDA)) {
                    ltc.addSymbol(tree.sym, LambdaSymbolKind.LOCAL_VAR);
                }
                Type type = tree.sym.asType();
                if (inClassWithinLambda() && !LambdaToMethod.this.types.isSameType(LambdaToMethod.this.types.erasure(type), type)) {
                    ltc.addSymbol(tree.sym, LambdaSymbolKind.TYPE_VAR);
                }
            }
            List<Frame> prevStack = this.frameStack;
            try {
                if (tree.sym.owner.kind == 16) {
                    this.frameStack.head.addLocal(tree.sym);
                }
                this.frameStack = this.frameStack.prepend(new Frame(tree));
                super.visitVarDef(tree);
            } finally {
                this.frameStack = prevStack;
            }
        }

        /* JADX INFO: Access modifiers changed from: private */
        public Symbol owner() {
            return owner(false);
        }

        /* JADX WARN: Multi-variable type inference failed */
        private Symbol owner(boolean skipLambda) {
            List list = this.frameStack;
            while (list.nonEmpty()) {
                switch (((Frame) list.head).tree.getTag()) {
                    case CLASSDEF:
                        return ((JCTree.JCClassDecl) ((Frame) list.head).tree).sym;
                    case VARDEF:
                        if (((JCTree.JCVariableDecl) ((Frame) list.head).tree).sym.isLocal()) {
                            list = list.tail;
                            continue;
                        } else {
                            JCTree.JCClassDecl cdecl = (JCTree.JCClassDecl) ((Frame) list.tail.head).tree;
                            return initSym(cdecl.sym, 8 & ((JCTree.JCVariableDecl) ((Frame) list.head).tree).sym.flags());
                        }
                        break;
                    case BLOCK:
                        JCTree.JCClassDecl cdecl2 = (JCTree.JCClassDecl) ((Frame) list.tail.head).tree;
                        return initSym(cdecl2.sym, 8 & ((JCTree.JCBlock) ((Frame) list.head).tree).flags);
                    case METHODDEF:
                        return ((JCTree.JCMethodDecl) ((Frame) list.head).tree).sym;
                    case LAMBDA:
                        if (!skipLambda) {
                            return ((LambdaTranslationContext) LambdaToMethod.this.contextMap.get(((Frame) list.head).tree)).translatedSym;
                        }
                        break;
                }
                list = list.tail;
            }
            Assert.error();
            return null;
        }

        private Symbol initSym(Symbol.ClassSymbol csym, long flags) {
            boolean isStatic = (8 & flags) != 0;
            if (isStatic) {
                Symbol.MethodSymbol clinit = LambdaToMethod.this.attr.removeClinit(csym);
                if (clinit != null) {
                    this.clinits.put(csym, clinit);
                    return clinit;
                }
                Symbol.MethodSymbol clinit2 = (Symbol.MethodSymbol) this.clinits.get(csym);
                if (clinit2 == null) {
                    Symbol.MethodSymbol clinit3 = LambdaToMethod.this.makePrivateSyntheticMethod(8L, LambdaToMethod.this.names.clinit, new Type.MethodType(List.nil(), LambdaToMethod.this.syms.voidType, List.nil(), LambdaToMethod.this.syms.methodClass), csym);
                    this.clinits.put(csym, clinit3);
                    return clinit3;
                }
                return clinit2;
            }
            Iterator<Symbol> it = csym.members_field.getElementsByName(LambdaToMethod.this.names.init).iterator();
            if (it.hasNext()) {
                Symbol s = it.next();
                return s;
            }
            Assert.error("init not found");
            return null;
        }

        /* JADX INFO: Access modifiers changed from: private */
        /* JADX WARN: Multi-variable type inference failed */
        public JCTree directlyEnclosingLambda() {
            if (this.frameStack.isEmpty()) {
                return null;
            }
            for (List list = this.frameStack; list.nonEmpty(); list = list.tail) {
                switch (((Frame) list.head).tree.getTag()) {
                    case CLASSDEF:
                    case METHODDEF:
                        return null;
                    case VARDEF:
                    case BLOCK:
                    default:
                        break;
                    case LAMBDA:
                        return ((Frame) list.head).tree;
                }
            }
            Assert.error();
            return null;
        }

        /* JADX WARN: Multi-variable type inference failed */
        private boolean inClassWithinLambda() {
            if (this.frameStack.isEmpty()) {
                return false;
            }
            List list = this.frameStack;
            boolean classFound = false;
            while (list.nonEmpty()) {
                switch (((Frame) list.head).tree.getTag()) {
                    case CLASSDEF:
                        classFound = true;
                        list = list.tail;
                        break;
                    case LAMBDA:
                        return classFound;
                    default:
                        list = list.tail;
                        break;
                }
            }
            return false;
        }

        /* JADX INFO: Access modifiers changed from: private */
        public JCTree capturedDecl(int depth, Symbol sym) {
            int currentDepth = this.frameStack.size() - 1;
            for (Frame block : this.frameStack) {
                switch (block.tree.getTag()) {
                    case CLASSDEF:
                        Symbol.ClassSymbol clazz = ((JCTree.JCClassDecl) block.tree).sym;
                        if (sym.isMemberOf(clazz, LambdaToMethod.this.types)) {
                            if (currentDepth > depth) {
                                return null;
                            }
                            return block.tree;
                        }
                        break;
                        break;
                    case VARDEF:
                        if (((JCTree.JCVariableDecl) block.tree).sym == sym && sym.owner.kind == 16) {
                            if (currentDepth > depth) {
                                return null;
                            }
                            return block.tree;
                        }
                        break;
                    case BLOCK:
                    case METHODDEF:
                    case LAMBDA:
                        if (block.locals != null && block.locals.contains(sym)) {
                            if (currentDepth > depth) {
                                return null;
                            }
                            return block.tree;
                        }
                        break;
                        break;
                    default:
                        Assert.error("bad decl kind " + block.tree.getTag());
                        break;
                }
                currentDepth--;
            }
            return null;
        }

        /* JADX INFO: Access modifiers changed from: private */
        public TranslationContext<?> context() {
            for (Frame frame : this.frameStack) {
                TranslationContext<?> context = (TranslationContext) LambdaToMethod.this.contextMap.get(frame.tree);
                if (context != null) {
                    return context;
                }
            }
            return null;
        }

        /* JADX INFO: Access modifiers changed from: private */
        public boolean lambdaIdentSymbolFilter(Symbol sym) {
            return ((sym.kind != 4 && sym.kind != 16) || sym.isStatic() || sym.name == LambdaToMethod.this.names.init) ? false : true;
        }

        /* JADX INFO: Access modifiers changed from: private */
        public boolean lambdaFieldAccessFilter(JCTree.JCFieldAccess fAccess) {
            LambdaTranslationContext lambdaContext = LambdaToMethod.this.context instanceof LambdaTranslationContext ? (LambdaTranslationContext) LambdaToMethod.this.context : null;
            return (lambdaContext == null || fAccess.sym.isStatic() || fAccess.name != LambdaToMethod.this.names._this || fAccess.sym.owner.kind != 2 || lambdaContext.translatedSymbols.get(LambdaSymbolKind.CAPTURED_OUTER_THIS).isEmpty()) ? false : true;
        }

        private boolean lambdaNewClassFilter(TranslationContext<?> context, JCTree.JCNewClass tree) {
            if (context == null || tree.encl != null || tree.def != null || tree.type.getEnclosingType().hasTag(TypeTag.NONE)) {
                return false;
            }
            Type encl = tree.type.getEnclosingType();
            for (Type current = context.owner.enclClass().type; !current.hasTag(TypeTag.NONE); current = current.getEnclosingType()) {
                if (current.tsym.isSubClass(encl.tsym, LambdaToMethod.this.types)) {
                    return true;
                }
            }
            return false;
        }

        private class Frame {
            List<Symbol> locals;
            final JCTree tree;

            public Frame(JCTree tree) {
                this.tree = tree;
            }

            void addLocal(Symbol sym) {
                if (this.locals == null) {
                    this.locals = List.nil();
                }
                this.locals = this.locals.prepend(sym);
            }
        }

        private abstract class TranslationContext<T extends JCTree.JCFunctionalExpression> {
            final List<Symbol> bridges;
            final int depth;
            final Symbol owner;
            final TranslationContext<?> prev;
            final T tree;

            TranslationContext(T tree) {
                this.tree = tree;
                this.owner = LambdaAnalyzerPreprocessor.this.owner();
                this.depth = LambdaAnalyzerPreprocessor.this.frameStack.size() - 1;
                this.prev = LambdaAnalyzerPreprocessor.this.context();
                Symbol.ClassSymbol csym = LambdaToMethod.this.types.makeFunctionalInterfaceClass(LambdaToMethod.this.attrEnv, LambdaToMethod.this.names.empty, tree.targets, 1536L);
                this.bridges = LambdaToMethod.this.types.functionalInterfaceBridges(csym);
            }

            boolean needsAltMetafactory() {
                return this.tree.targets.length() > 1 || isSerializable() || this.bridges.length() > 1;
            }

            boolean isSerializable() {
                if (LambdaToMethod.this.forceSerializable) {
                    return true;
                }
                for (Type target : this.tree.targets) {
                    if (LambdaToMethod.this.types.asSuper(target, LambdaToMethod.this.syms.serializableType.tsym) != null) {
                        return true;
                    }
                }
                return false;
            }

            String enclosingMethodName() {
                return syntheticMethodNameComponent(this.owner.name);
            }

            String syntheticMethodNameComponent(Name name) {
                if (name == null) {
                    return "null";
                }
                String methodName = name.toString();
                if (methodName.equals("<clinit>")) {
                    return "static";
                }
                if (methodName.equals("<init>")) {
                    return RequestConditions.REQUEST_KEY_NEW;
                }
                return methodName;
            }
        }

        private class LambdaTranslationContext extends TranslationContext<JCTree.JCLambda> {
            final Symbol assignedTo;
            final Set<Symbol> freeVarProcessedLocalClasses;
            JCTree.JCExpression methodReferenceReceiver;
            final Symbol self;
            List<JCTree.JCVariableDecl> syntheticParams;
            Symbol.MethodSymbol translatedSym;
            Map<LambdaSymbolKind, Map<Symbol, Symbol>> translatedSymbols;

            /* JADX WARN: Multi-variable type inference failed */
            LambdaTranslationContext(JCTree.JCLambda tree) {
                super(tree);
                Frame frame = (Frame) LambdaAnalyzerPreprocessor.this.frameStack.head;
                switch (frame.tree.getTag()) {
                    case VARDEF:
                        Symbol.VarSymbol varSymbol = ((JCTree.JCVariableDecl) frame.tree).sym;
                        this.self = varSymbol;
                        this.assignedTo = varSymbol;
                        break;
                    case ASSIGN:
                        this.self = null;
                        this.assignedTo = TreeInfo.symbol(((JCTree.JCAssign) frame.tree).getVariable());
                        break;
                    default:
                        this.self = null;
                        this.assignedTo = null;
                        break;
                }
                this.translatedSym = LambdaToMethod.this.makePrivateSyntheticMethod(0L, null, null, this.owner.enclClass());
                this.translatedSymbols = new EnumMap(LambdaSymbolKind.class);
                this.translatedSymbols.put(LambdaSymbolKind.PARAM, new LinkedHashMap());
                this.translatedSymbols.put(LambdaSymbolKind.LOCAL_VAR, new LinkedHashMap());
                this.translatedSymbols.put(LambdaSymbolKind.CAPTURED_VAR, new LinkedHashMap());
                this.translatedSymbols.put(LambdaSymbolKind.CAPTURED_THIS, new LinkedHashMap());
                this.translatedSymbols.put(LambdaSymbolKind.CAPTURED_OUTER_THIS, new LinkedHashMap());
                this.translatedSymbols.put(LambdaSymbolKind.TYPE_VAR, new LinkedHashMap());
                this.freeVarProcessedLocalClasses = new HashSet();
            }

            private String serializedLambdaDisambiguation() {
                StringBuilder buf = new StringBuilder();
                Assert.check((this.owner.type == null && LambdaAnalyzerPreprocessor.this.directlyEnclosingLambda() == null) ? false : true);
                if (this.owner.type != null) {
                    buf.append(LambdaToMethod.this.typeSig(this.owner.type));
                    buf.append(":");
                }
                buf.append((CharSequence) LambdaToMethod.this.types.findDescriptorSymbol(((JCTree.JCLambda) this.tree).type.tsym).owner.flatName());
                buf.append(" ");
                if (this.assignedTo != null) {
                    buf.append((CharSequence) this.assignedTo.flatName());
                    buf.append("=");
                }
                for (Symbol fv : getSymbolMap(LambdaSymbolKind.CAPTURED_VAR).keySet()) {
                    if (fv != this.self) {
                        buf.append(LambdaToMethod.this.typeSig(fv.type));
                        buf.append(" ");
                        buf.append((CharSequence) fv.flatName());
                        buf.append(DocLint.TAGS_SEPARATOR);
                    }
                }
                return buf.toString();
            }

            private Name lambdaName() {
                return LambdaToMethod.this.names.lambda.append(LambdaToMethod.this.names.fromString(enclosingMethodName() + "$" + LambdaAnalyzerPreprocessor.access$3408(LambdaAnalyzerPreprocessor.this)));
            }

            private Name serializedLambdaName() {
                StringBuilder buf = new StringBuilder();
                buf.append((CharSequence) LambdaToMethod.this.names.lambda);
                buf.append(enclosingMethodName());
                buf.append('$');
                String disam = serializedLambdaDisambiguation();
                buf.append(Integer.toHexString(disam.hashCode()));
                buf.append('$');
                buf.append(LambdaAnalyzerPreprocessor.this.syntheticMethodNameCounts.getIndex(buf));
                String result = buf.toString();
                return LambdaToMethod.this.names.fromString(result);
            }

            Symbol translate(final Symbol sym, LambdaSymbolKind skind) {
                Symbol ret;
                switch (skind) {
                    case CAPTURED_THIS:
                        ret = sym;
                        break;
                    case TYPE_VAR:
                        ret = new Symbol.VarSymbol(sym.flags(), sym.name, LambdaToMethod.this.types.erasure(sym.type), sym.owner);
                        ((Symbol.VarSymbol) ret).pos = ((Symbol.VarSymbol) sym).pos;
                        break;
                    case CAPTURED_VAR:
                        ret = new Symbol.VarSymbol(8589938704L, sym.name, LambdaToMethod.this.types.erasure(sym.type), this.translatedSym) { // from class: com.sun.tools.javac.comp.LambdaToMethod.LambdaAnalyzerPreprocessor.LambdaTranslationContext.1
                            @Override // com.sun.tools.javac.code.Symbol
                            public Symbol baseSymbol() {
                                return sym;
                            }
                        };
                        break;
                    case CAPTURED_OUTER_THIS:
                        Name name = LambdaToMethod.this.names.fromString(new String(sym.flatName().toString() + ((Object) LambdaToMethod.this.names.dollarThis)));
                        Symbol ret2 = new Symbol.VarSymbol(8589938704L, name, LambdaToMethod.this.types.erasure(sym.type), this.translatedSym) { // from class: com.sun.tools.javac.comp.LambdaToMethod.LambdaAnalyzerPreprocessor.LambdaTranslationContext.2
                            @Override // com.sun.tools.javac.code.Symbol
                            public Symbol baseSymbol() {
                                return sym;
                            }
                        };
                        ret = ret2;
                        break;
                    case LOCAL_VAR:
                        ret = new Symbol.VarSymbol(sym.flags() & 16, sym.name, sym.type, this.translatedSym);
                        ((Symbol.VarSymbol) ret).pos = ((Symbol.VarSymbol) sym).pos;
                        break;
                    case PARAM:
                        ret = new Symbol.VarSymbol((16 & sym.flags()) | 8589934592L, sym.name, LambdaToMethod.this.types.erasure(sym.type), this.translatedSym);
                        ((Symbol.VarSymbol) ret).pos = ((Symbol.VarSymbol) sym).pos;
                        break;
                    default:
                        Assert.error(skind.name());
                        throw new AssertionError();
                }
                if (ret != sym) {
                    ret.setDeclarationAttributes(sym.getRawAttributes());
                    ret.setTypeAttributes(sym.getRawTypeAttributes());
                }
                return ret;
            }

            void addSymbol(Symbol sym, LambdaSymbolKind skind) {
                Symbol.ClassSymbol currentClass;
                if (skind == LambdaSymbolKind.CAPTURED_THIS && sym != null && sym.kind == 2 && !LambdaAnalyzerPreprocessor.this.typesUnderConstruction.isEmpty() && (currentClass = LambdaAnalyzerPreprocessor.this.currentClass()) != null && LambdaAnalyzerPreprocessor.this.typesUnderConstruction.contains(currentClass)) {
                    Assert.check(sym != currentClass);
                    skind = LambdaSymbolKind.CAPTURED_OUTER_THIS;
                }
                Map<Symbol, Symbol> transMap = getSymbolMap(skind);
                if (!transMap.containsKey(sym)) {
                    transMap.put(sym, translate(sym, skind));
                }
            }

            Map<Symbol, Symbol> getSymbolMap(LambdaSymbolKind skind) {
                Map<Symbol, Symbol> m = this.translatedSymbols.get(skind);
                Assert.checkNonNull(m);
                return m;
            }

            JCTree translate(JCTree.JCIdent lambdaIdent) {
                for (LambdaSymbolKind kind : LambdaSymbolKind.values()) {
                    Map<Symbol, Symbol> m = getSymbolMap(kind);
                    switch (kind) {
                        case CAPTURED_OUTER_THIS:
                            if (lambdaIdent.sym.owner.kind == 2 && m.containsKey(lambdaIdent.sym.owner)) {
                                Symbol tSym = m.get(lambdaIdent.sym.owner);
                                JCTree.JCExpression t = LambdaToMethod.this.make.Ident(tSym).setType(lambdaIdent.sym.owner.type);
                                tSym.setTypeAttributes(lambdaIdent.sym.owner.getRawTypeAttributes());
                                JCTree.JCExpression t2 = LambdaToMethod.this.make.Select(t, lambdaIdent.name);
                                t2.setType(lambdaIdent.type);
                                TreeInfo.setSymbol(t2, lambdaIdent.sym);
                                return t2;
                            }
                            break;
                        default:
                            if (m.containsKey(lambdaIdent.sym)) {
                                Symbol tSym2 = m.get(lambdaIdent.sym);
                                JCTree t3 = LambdaToMethod.this.make.Ident(tSym2).setType(lambdaIdent.type);
                                tSym2.setTypeAttributes(lambdaIdent.sym.getRawTypeAttributes());
                                return t3;
                            }
                            break;
                            break;
                    }
                }
                return null;
            }

            public JCTree translate(JCTree.JCFieldAccess fieldAccess) {
                Assert.check(fieldAccess.name == LambdaToMethod.this.names._this);
                Map<Symbol, Symbol> m = this.translatedSymbols.get(LambdaSymbolKind.CAPTURED_OUTER_THIS);
                if (m.containsKey(fieldAccess.sym.owner)) {
                    Symbol tSym = m.get(fieldAccess.sym.owner);
                    JCTree.JCExpression t = LambdaToMethod.this.make.Ident(tSym).setType(fieldAccess.sym.owner.type);
                    tSym.setTypeAttributes(fieldAccess.sym.owner.getRawTypeAttributes());
                    return t;
                }
                return null;
            }

            void complete() {
                Name nameLambdaName;
                if (this.syntheticParams != null) {
                    return;
                }
                boolean inInterface = this.translatedSym.owner.isInterface();
                boolean thisReferenced = !getSymbolMap(LambdaSymbolKind.CAPTURED_THIS).isEmpty();
                this.translatedSym.flags_field = (this.owner.flags_field & 2048) | 562949953425408L | (2048 & this.owner.owner.flags_field) | 2 | (thisReferenced ? inInterface ? Flags.DEFAULT : 0L : 8L);
                ListBuffer<JCTree.JCVariableDecl> params = new ListBuffer<>();
                ListBuffer<Symbol.VarSymbol> parameterSymbols = new ListBuffer<>();
                for (Symbol thisSym : getSymbolMap(LambdaSymbolKind.CAPTURED_VAR).values()) {
                    params.append(LambdaToMethod.this.make.VarDef((Symbol.VarSymbol) thisSym, null));
                    parameterSymbols.append((Symbol.VarSymbol) thisSym);
                }
                for (Symbol thisSym2 : getSymbolMap(LambdaSymbolKind.CAPTURED_OUTER_THIS).values()) {
                    params.append(LambdaToMethod.this.make.VarDef((Symbol.VarSymbol) thisSym2, null));
                    parameterSymbols.append((Symbol.VarSymbol) thisSym2);
                }
                for (Symbol thisSym3 : getSymbolMap(LambdaSymbolKind.PARAM).values()) {
                    params.append(LambdaToMethod.this.make.VarDef((Symbol.VarSymbol) thisSym3, null));
                    parameterSymbols.append((Symbol.VarSymbol) thisSym3);
                }
                this.syntheticParams = params.toList();
                this.translatedSym.params = parameterSymbols.toList();
                Symbol.MethodSymbol methodSymbol = this.translatedSym;
                if (isSerializable()) {
                    nameLambdaName = serializedLambdaName();
                } else {
                    nameLambdaName = lambdaName();
                }
                methodSymbol.name = nameLambdaName;
                this.translatedSym.type = LambdaToMethod.this.types.createMethodTypeWithParameters(generatedLambdaSig(), TreeInfo.types(this.syntheticParams));
            }

            Type generatedLambdaSig() {
                return LambdaToMethod.this.types.erasure(((JCTree.JCLambda) this.tree).getDescriptorType(LambdaToMethod.this.types));
            }
        }

        private final class ReferenceTranslationContext extends TranslationContext<JCTree.JCMemberReference> {
            final boolean isSuper;
            final Symbol sigPolySym;

            ReferenceTranslationContext(JCTree.JCMemberReference tree) {
                super(tree);
                this.isSuper = tree.hasKind(JCTree.JCMemberReference.ReferenceKind.SUPER);
                this.sigPolySym = isSignaturePolymorphic() ? LambdaToMethod.this.makePrivateSyntheticMethod(tree.sym.flags(), tree.sym.name, bridgedRefSig(), tree.sym.enclClass()) : null;
            }

            int referenceKind() {
                return LambdaToMethod.this.referenceKind(((JCTree.JCMemberReference) this.tree).sym);
            }

            boolean needsVarArgsConversion() {
                return ((JCTree.JCMemberReference) this.tree).varargsElement != null;
            }

            boolean isArrayOp() {
                return ((JCTree.JCMemberReference) this.tree).sym.owner == LambdaToMethod.this.syms.arrayClass;
            }

            boolean receiverAccessible() {
                return ((JCTree.JCMemberReference) this.tree).ownerAccessible;
            }

            /* JADX WARN: Multi-variable type inference failed */
            boolean isPrivateInOtherClass() {
                return ((((JCTree.JCMemberReference) this.tree).sym.flags() & 2) == 0 || LambdaToMethod.this.types.isSameType(LambdaToMethod.this.types.erasure(((JCTree.JCMemberReference) this.tree).sym.enclClass().asType()), LambdaToMethod.this.types.erasure(this.owner.enclClass().asType()))) ? false : true;
            }

            final boolean isSignaturePolymorphic() {
                return ((JCTree.JCMemberReference) this.tree).sym.kind == 16 && LambdaToMethod.this.types.isSignaturePolymorphic((Symbol.MethodSymbol) ((JCTree.JCMemberReference) this.tree).sym);
            }

            /* JADX WARN: Multi-variable type inference failed */
            boolean interfaceParameterIsIntersectionType() {
                List listMo176getParameterTypes = ((JCTree.JCMemberReference) this.tree).getDescriptorType(LambdaToMethod.this.types).mo176getParameterTypes();
                if (((JCTree.JCMemberReference) this.tree).kind == JCTree.JCMemberReference.ReferenceKind.UNBOUND) {
                    listMo176getParameterTypes = listMo176getParameterTypes.tail;
                }
                while (listMo176getParameterTypes.nonEmpty()) {
                    Type pt = (Type) listMo176getParameterTypes.head;
                    if (pt.getKind() == TypeKind.TYPEVAR) {
                        Type.TypeVar tv = (Type.TypeVar) pt;
                        if (tv.bound.getKind() == TypeKind.INTERSECTION) {
                            return true;
                        }
                    }
                    listMo176getParameterTypes = listMo176getParameterTypes.tail;
                }
                return false;
            }

            final boolean needsConversionToLambda() {
                return interfaceParameterIsIntersectionType() || this.isSuper || needsVarArgsConversion() || isArrayOp() || isPrivateInOtherClass() || !receiverAccessible() || (((JCTree.JCMemberReference) this.tree).getMode() == MemberReferenceTree.ReferenceMode.NEW && ((JCTree.JCMemberReference) this.tree).kind != JCTree.JCMemberReference.ReferenceKind.ARRAY_CTOR && (((JCTree.JCMemberReference) this.tree).sym.owner.isLocal() || ((JCTree.JCMemberReference) this.tree).sym.owner.isInner()));
            }

            Type generatedRefSig() {
                return LambdaToMethod.this.types.erasure(((JCTree.JCMemberReference) this.tree).sym.type);
            }

            Type bridgedRefSig() {
                return LambdaToMethod.this.types.erasure(LambdaToMethod.this.types.findDescriptorSymbol(((JCTree.JCMemberReference) this.tree).targets.head.tsym).type);
            }
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public String typeSig(Type type) {
        L2MSignatureGenerator sg = new L2MSignatureGenerator();
        sg.assembleSig(type);
        return sg.toString();
    }

    private String classSig(Type type) {
        L2MSignatureGenerator sg = new L2MSignatureGenerator();
        sg.assembleClassSig(type);
        return sg.toString();
    }

    private class L2MSignatureGenerator extends Types.SignatureGenerator {
        StringBuilder sb;

        L2MSignatureGenerator() {
            super(LambdaToMethod.this.types);
            this.sb = new StringBuilder();
        }

        @Override // com.sun.tools.javac.code.Types.SignatureGenerator
        protected void append(char ch) {
            this.sb.append(ch);
        }

        @Override // com.sun.tools.javac.code.Types.SignatureGenerator
        protected void append(byte[] ba) {
            this.sb.append(new String(ba));
        }

        @Override // com.sun.tools.javac.code.Types.SignatureGenerator
        protected void append(Name name) {
            this.sb.append(name.toString());
        }

        public String toString() {
            return this.sb.toString();
        }
    }
}
