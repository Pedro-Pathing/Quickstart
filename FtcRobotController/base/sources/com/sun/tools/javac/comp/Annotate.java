package com.sun.tools.javac.comp;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Kinds;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeAnnotationPosition;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Pair;
import java.util.Map;

/* JADX INFO: loaded from: classes.dex */
public class Annotate {
    protected static final Context.Key<Annotate> annotateKey = new Context.Key<>();
    final Attr attr;
    final ConstFold cfolder;
    final Check chk;
    final Log log;
    final TreeMaker make;
    final Names names;
    final Resolve rs;
    final Symtab syms;
    final Types types;
    private int enterCount = 0;
    ListBuffer<Worker> q = new ListBuffer<>();
    ListBuffer<Worker> typesQ = new ListBuffer<>();
    ListBuffer<Worker> repeatedQ = new ListBuffer<>();
    ListBuffer<Worker> afterRepeatedQ = new ListBuffer<>();
    ListBuffer<Worker> validateQ = new ListBuffer<>();

    public interface Worker {
        void run();

        String toString();
    }

    public static Annotate instance(Context context) {
        Annotate instance = (Annotate) context.get(annotateKey);
        if (instance == null) {
            return new Annotate(context);
        }
        return instance;
    }

    protected Annotate(Context context) {
        context.put(annotateKey, this);
        this.attr = Attr.instance(context);
        this.make = TreeMaker.instance(context);
        this.log = Log.instance(context);
        this.syms = Symtab.instance(context);
        this.names = Names.instance(context);
        this.rs = Resolve.instance(context);
        this.types = Types.instance(context);
        this.cfolder = ConstFold.instance(context);
        this.chk = Check.instance(context);
    }

    public void earlier(Worker a) {
        this.q.prepend(a);
    }

    public void normal(Worker a) {
        this.q.append(a);
    }

    public void typeAnnotation(Worker a) {
        this.typesQ.append(a);
    }

    public void repeated(Worker a) {
        this.repeatedQ.append(a);
    }

    public void afterRepeated(Worker a) {
        this.afterRepeatedQ.append(a);
    }

    public void validate(Worker a) {
        this.validateQ.append(a);
    }

    public void enterStart() {
        this.enterCount++;
    }

    public void enterDone() {
        this.enterCount--;
        flush();
    }

    public void enterDoneWithoutFlush() {
        this.enterCount--;
    }

    public void flush() {
        if (this.enterCount != 0) {
            return;
        }
        this.enterCount++;
        while (this.q.nonEmpty()) {
            try {
                this.q.next().run();
            } finally {
                this.enterCount--;
            }
        }
        while (this.typesQ.nonEmpty()) {
            this.typesQ.next().run();
        }
        while (this.repeatedQ.nonEmpty()) {
            this.repeatedQ.next().run();
        }
        while (this.afterRepeatedQ.nonEmpty()) {
            this.afterRepeatedQ.next().run();
        }
        while (this.validateQ.nonEmpty()) {
            this.validateQ.next().run();
        }
    }

    public class AnnotateRepeatedContext<T extends Attribute.Compound> {
        public final Map<Symbol.TypeSymbol, ListBuffer<T>> annotated;
        public final Env<AttrContext> env;
        public final boolean isTypeCompound;
        public final Log log;
        public final Map<T, JCDiagnostic.DiagnosticPosition> pos;

        public AnnotateRepeatedContext(Env<AttrContext> env, Map<Symbol.TypeSymbol, ListBuffer<T>> annotated, Map<T, JCDiagnostic.DiagnosticPosition> pos, Log log, boolean isTypeCompound) {
            Assert.checkNonNull(env);
            Assert.checkNonNull(annotated);
            Assert.checkNonNull(pos);
            Assert.checkNonNull(log);
            this.env = env;
            this.annotated = annotated;
            this.pos = pos;
            this.log = log;
            this.isTypeCompound = isTypeCompound;
        }

        public T processRepeatedAnnotations(List<T> list, Symbol symbol) {
            return (T) Annotate.this.processRepeatedAnnotations(list, this, symbol);
        }

        public void annotateRepeated(Worker a) {
            Annotate.this.repeated(a);
        }
    }

    Attribute.Compound enterAnnotation(JCTree.JCAnnotation a, Type expected, Env<AttrContext> env) {
        return enterAnnotation(a, expected, env, false);
    }

    Attribute.TypeCompound enterTypeAnnotation(JCTree.JCAnnotation a, Type expected, Env<AttrContext> env) {
        return (Attribute.TypeCompound) enterAnnotation(a, expected, env, true);
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r2v39, types: [A, com.sun.tools.javac.tree.JCTree$JCAssign] */
    Attribute.Compound enterAnnotation(JCTree.JCAnnotation a, Type expected, Env<AttrContext> env, boolean typeAnnotation) {
        Type at;
        Type at2 = a.annotationType.type != null ? a.annotationType.type : this.attr.attribType(a.annotationType, env);
        a.type = this.chk.checkType(a.annotationType.pos(), at2, expected);
        if (a.type.isErroneous()) {
            this.attr.postAttr(a);
            if (typeAnnotation) {
                return new Attribute.TypeCompound(a.type, List.nil(), new TypeAnnotationPosition());
            }
            return new Attribute.Compound(a.type, List.nil());
        }
        if ((a.type.tsym.flags() & 8192) == 0) {
            this.log.error(a.annotationType.pos(), "not.annotation.type", a.type.toString());
            this.attr.postAttr(a);
            if (typeAnnotation) {
                return new Attribute.TypeCompound(a.type, List.nil(), null);
            }
            return new Attribute.Compound(a.type, List.nil());
        }
        List<JCTree.JCExpression> args = a.args;
        if (args.length() == 1 && !args.head.hasTag(JCTree.Tag.ASSIGN)) {
            args.head = this.make.at(args.head.pos).Assign(this.make.Ident(this.names.value), args.head);
        }
        ListBuffer<Pair<Symbol.MethodSymbol, Attribute>> buf = new ListBuffer<>();
        List list = args;
        while (list.nonEmpty()) {
            JCTree.JCExpression t = (JCTree.JCExpression) list.head;
            if (!t.hasTag(JCTree.Tag.ASSIGN)) {
                this.log.error(t.pos(), "annotation.value.must.be.name.value", new Object[0]);
                at = at2;
            } else {
                JCTree.JCAssign assign = (JCTree.JCAssign) t;
                if (!assign.lhs.hasTag(JCTree.Tag.IDENT)) {
                    this.log.error(t.pos(), "annotation.value.must.be.name.value", new Object[0]);
                    at = at2;
                } else {
                    JCTree.JCIdent left = (JCTree.JCIdent) assign.lhs;
                    at = at2;
                    Symbol method = this.rs.resolveQualifiedMethod(assign.rhs.pos(), env, a.type, left.name, List.nil(), null);
                    left.sym = method;
                    left.type = method.type;
                    if (method.owner != a.type.tsym) {
                        this.log.error(left.pos(), "no.annotation.member", left.name, a.type);
                    }
                    Type result = method.type.mo178getReturnType();
                    Attribute value = enterAttributeValue(result, assign.rhs, env);
                    if (!method.type.isErroneous()) {
                        buf.append(new Pair<>((Symbol.MethodSymbol) method, value));
                    }
                    t.type = result;
                }
            }
            list = list.tail;
            at2 = at;
        }
        if (typeAnnotation) {
            if (a.attribute == null || !(a.attribute instanceof Attribute.TypeCompound)) {
                Attribute.TypeCompound tc = new Attribute.TypeCompound(a.type, buf.toList(), new TypeAnnotationPosition());
                a.attribute = tc;
                return tc;
            }
            return a.attribute;
        }
        Attribute.Compound ac = new Attribute.Compound(a.type, buf.toList());
        a.attribute = ac;
        return ac;
    }

    /* JADX WARN: Multi-variable type inference failed */
    Attribute enterAttributeValue(Type expected, JCTree.JCExpression tree, Env<AttrContext> env) {
        try {
            expected.tsym.complete();
        } catch (Symbol.CompletionFailure e) {
            this.log.error(tree.pos(), "cant.resolve", Kinds.kindName(e.sym), e.sym);
            expected = this.syms.errType;
        }
        if (expected.hasTag(TypeTag.ARRAY)) {
            if (!tree.hasTag(JCTree.Tag.NEWARRAY)) {
                tree = this.make.at(tree.pos).NewArray(null, List.nil(), List.of(tree));
            }
            JCTree.JCNewArray na = (JCTree.JCNewArray) tree;
            if (na.elemtype != null) {
                this.log.error(na.elemtype.pos(), "new.not.allowed.in.annotation", new Object[0]);
            }
            ListBuffer<Attribute> buf = new ListBuffer<>();
            for (List list = na.elems; list.nonEmpty(); list = list.tail) {
                buf.append(enterAttributeValue(this.types.elemtype(expected), (JCTree.JCExpression) list.head, env));
            }
            na.type = expected;
            return new Attribute.Array(expected, (Attribute[]) buf.toArray(new Attribute[buf.length()]));
        }
        if (tree.hasTag(JCTree.Tag.NEWARRAY)) {
            if (!expected.isErroneous()) {
                this.log.error(tree.pos(), "annotation.value.not.allowable.type", new Object[0]);
            }
            JCTree.JCNewArray na2 = (JCTree.JCNewArray) tree;
            if (na2.elemtype != null) {
                this.log.error(na2.elemtype.pos(), "new.not.allowed.in.annotation", new Object[0]);
            }
            for (List list2 = na2.elems; list2.nonEmpty(); list2 = list2.tail) {
                enterAttributeValue(this.syms.errType, (JCTree.JCExpression) list2.head, env);
            }
            return new Attribute.Error(this.syms.errType);
        }
        if ((expected.tsym.flags() & 8192) != 0) {
            if (tree.hasTag(JCTree.Tag.ANNOTATION)) {
                return enterAnnotation((JCTree.JCAnnotation) tree, expected, env);
            }
            this.log.error(tree.pos(), "annotation.value.must.be.annotation", new Object[0]);
            expected = this.syms.errType;
        }
        if (tree.hasTag(JCTree.Tag.ANNOTATION)) {
            if (!expected.isErroneous()) {
                this.log.error(tree.pos(), "annotation.not.valid.for.type", expected);
            }
            enterAnnotation((JCTree.JCAnnotation) tree, this.syms.errType, env);
            return new Attribute.Error(((JCTree.JCAnnotation) tree).annotationType.type);
        }
        if (expected.isPrimitive() || this.types.isSameType(expected, this.syms.stringType)) {
            Type result = this.attr.attribExpr(tree, env, expected);
            if (result.isErroneous()) {
                return new Attribute.Error(result.getOriginalType());
            }
            if (result.constValue() == null) {
                this.log.error(tree.pos(), "attribute.value.must.be.constant", new Object[0]);
                return new Attribute.Error(expected);
            }
            return new Attribute.Constant(expected, this.cfolder.coerce(result, expected).constValue());
        }
        if (expected.tsym == this.syms.classType.tsym) {
            Type result2 = this.attr.attribExpr(tree, env, expected);
            if (result2.isErroneous()) {
                if (TreeInfo.name(tree) == this.names._class && ((JCTree.JCFieldAccess) tree).selected.type.isErroneous()) {
                    Name n = ((JCTree.JCFieldAccess) tree).selected.type.tsym.flatName();
                    return new Attribute.UnresolvedClass(expected, this.types.createErrorType(n, this.syms.unknownSymbol, this.syms.classType));
                }
                return new Attribute.Error(result2.getOriginalType());
            }
            if (TreeInfo.name(tree) != this.names._class) {
                this.log.error(tree.pos(), "annotation.value.must.be.class.literal", new Object[0]);
                return new Attribute.Error(this.syms.errType);
            }
            return new Attribute.Class(this.types, ((JCTree.JCFieldAccess) tree).selected.type);
        }
        if (expected.hasTag(TypeTag.CLASS) && (expected.tsym.flags() & 16384) != 0) {
            Type result3 = this.attr.attribExpr(tree, env, expected);
            Symbol sym = TreeInfo.symbol(tree);
            if (sym == null || TreeInfo.nonstaticSelect(tree) || sym.kind != 4 || (16384 & sym.flags()) == 0) {
                this.log.error(tree.pos(), "enum.annotation.must.be.enum.constant", new Object[0]);
                return new Attribute.Error(result3.getOriginalType());
            }
            Symbol.VarSymbol enumerator = (Symbol.VarSymbol) sym;
            return new Attribute.Enum(expected, enumerator);
        }
        if (!expected.isErroneous()) {
            this.log.error(tree.pos(), "annotation.value.not.allowable.type", new Object[0]);
        }
        return new Attribute.Error(this.attr.attribExpr(tree, env, expected));
    }

    /* JADX INFO: Access modifiers changed from: private */
    /* JADX WARN: Multi-variable type inference failed */
    public <T extends Attribute.Compound> T processRepeatedAnnotations(List<T> list, AnnotateRepeatedContext<T> annotateRepeatedContext, Symbol symbol) {
        T t = list.head;
        List listNil = List.nil();
        Type type = null;
        Type typeMakeArrayType = null;
        Type type2 = null;
        Symbol.MethodSymbol methodSymbolValidateContainer = null;
        Assert.check((list.isEmpty() || list.tail.isEmpty()) ? false : true);
        int i = 0;
        for (List list2 = list; !list2.isEmpty(); list2 = list2.tail) {
            i++;
            Assert.check(i > 1 || !list2.tail.isEmpty());
            Attribute.Compound compound = (Attribute.Compound) list2.head;
            type = compound.type;
            if (typeMakeArrayType == null) {
                typeMakeArrayType = this.types.makeArrayType(type);
            }
            Type containingType = getContainingType(compound, annotateRepeatedContext.pos.get(compound), i > 1);
            if (containingType != null) {
                Assert.check(type2 == null || containingType == type2);
                type2 = containingType;
                methodSymbolValidateContainer = validateContainer(type2, type, annotateRepeatedContext.pos.get(compound));
                if (methodSymbolValidateContainer != null) {
                    listNil = listNil.prepend(compound);
                }
            }
        }
        if (!listNil.isEmpty()) {
            List listReverse = listNil.reverse();
            TreeMaker treeMakerAt = this.make.at(annotateRepeatedContext.pos.get(t));
            Pair pair = new Pair(methodSymbolValidateContainer, new Attribute.Array(typeMakeArrayType, (List<Attribute>) listReverse));
            if (annotateRepeatedContext.isTypeCompound) {
                Attribute.TypeCompound typeCompound = new Attribute.TypeCompound(type2, List.of(pair), ((Attribute.TypeCompound) list.head).position);
                typeCompound.setSynthesized(true);
                return typeCompound;
            }
            JCTree.JCAnnotation jCAnnotationAnnotation = treeMakerAt.Annotation(new Attribute.Compound(type2, List.of(pair)));
            if (!this.chk.annotationApplicable(jCAnnotationAnnotation, symbol)) {
                this.log.error(jCAnnotationAnnotation.pos(), "invalid.repeatable.annotation.incompatible.target", type2, type);
            }
            if (!this.chk.validateAnnotationDeferErrors(jCAnnotationAnnotation)) {
                this.log.error(jCAnnotationAnnotation.pos(), "duplicate.annotation.invalid.repeated", type);
            }
            T t2 = (T) enterAnnotation(jCAnnotationAnnotation, type2, annotateRepeatedContext.env);
            t2.setSynthesized(true);
            return t2;
        }
        return null;
    }

    private Type getContainingType(Attribute.Compound currentAnno, JCDiagnostic.DiagnosticPosition pos, boolean reportError) {
        Type origAnnoType = currentAnno.type;
        Symbol.TypeSymbol origAnnoDecl = origAnnoType.tsym;
        Attribute.Compound ca = origAnnoDecl.attribute(this.syms.repeatableType.tsym);
        if (ca == null) {
            if (reportError) {
                this.log.error(pos, "duplicate.annotation.missing.container", origAnnoType, this.syms.repeatableType);
                return null;
            }
            return null;
        }
        return filterSame(extractContainingType(ca, pos, origAnnoDecl), origAnnoType);
    }

    private Type filterSame(Type t, Type s) {
        if (t == null || s == null) {
            return t;
        }
        if (this.types.isSameType(t, s)) {
            return null;
        }
        return t;
    }

    private Type extractContainingType(Attribute.Compound ca, JCDiagnostic.DiagnosticPosition pos, Symbol.TypeSymbol annoDecl) {
        if (ca.values.isEmpty()) {
            this.log.error(pos, "invalid.repeatable.annotation", annoDecl);
            return null;
        }
        Pair<Symbol.MethodSymbol, Attribute> p = ca.values.head;
        Name name = p.fst.name;
        if (name != this.names.value) {
            this.log.error(pos, "invalid.repeatable.annotation", annoDecl);
            return null;
        }
        if (!(p.snd instanceof Attribute.Class)) {
            this.log.error(pos, "invalid.repeatable.annotation", annoDecl);
            return null;
        }
        return ((Attribute.Class) p.snd).getValue();
    }

    private Symbol.MethodSymbol validateContainer(Type targetContainerType, Type originalAnnoType, JCDiagnostic.DiagnosticPosition pos) {
        Symbol.MethodSymbol containerValueSymbol = null;
        boolean fatalError = false;
        Scope scope = targetContainerType.tsym.members();
        int nr_value_elems = 0;
        boolean error = false;
        for (Symbol elm : scope.getElementsByName(this.names.value)) {
            nr_value_elems++;
            if (nr_value_elems == 1 && elm.kind == 16) {
                containerValueSymbol = (Symbol.MethodSymbol) elm;
            } else {
                error = true;
            }
        }
        if (error) {
            this.log.error(pos, "invalid.repeatable.annotation.multiple.values", targetContainerType, Integer.valueOf(nr_value_elems));
            return null;
        }
        if (nr_value_elems == 0) {
            this.log.error(pos, "invalid.repeatable.annotation.no.value", targetContainerType);
            return null;
        }
        if (containerValueSymbol.kind != 16) {
            this.log.error(pos, "invalid.repeatable.annotation.invalid.value", targetContainerType);
            fatalError = true;
        }
        Type valueRetType = containerValueSymbol.type.mo178getReturnType();
        Type expectedType = this.types.makeArrayType(originalAnnoType);
        if (!this.types.isArray(valueRetType) || !this.types.isSameType(expectedType, valueRetType)) {
            this.log.error(pos, "invalid.repeatable.annotation.value.return", targetContainerType, valueRetType, expectedType);
            fatalError = true;
        }
        if (error) {
            fatalError = true;
        }
        if (fatalError) {
            return null;
        }
        return containerValueSymbol;
    }
}
