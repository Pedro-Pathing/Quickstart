package com.sun.tools.javac.code;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Check;
import com.sun.tools.javac.comp.Enter;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.jvm.ClassFile;
import com.sun.tools.javac.jvm.ClassReader;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.JavacMessages;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Warner;
import dk.sgjesse.r8api.DescriptorUtils;
import java.lang.ref.SoftReference;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class Types {
    final boolean allowBoxing;
    final boolean allowCovariantReturns;
    final boolean allowDefaultMethods;
    final boolean allowObjectToPrimitiveCast;
    final Name capturedName;
    final Check chk;
    JCDiagnostic.Factory diags;
    final Enter enter;
    private final FunctionDescriptorLookupError functionDescriptorLookupError;
    final JavacMessages messages;
    final Names names;
    public final Warner noWarnings;
    final ClassReader reader;
    final Symtab syms;
    protected static final Context.Key<Types> typesKey = new Context.Key<>();
    private static final Type.Mapping newInstanceFun = new Type.Mapping("newInstanceFun") { // from class: com.sun.tools.javac.code.Types.22
        @Override // com.sun.tools.javac.code.Type.Mapping
        public Type apply(Type t) {
            return new Type.TypeVar(t.tsym, t.getUpperBound(), t.getLowerBound());
        }
    };
    private static final UnaryVisitor<Integer> hashCode = new UnaryVisitor<Integer>() { // from class: com.sun.tools.javac.code.Types.26
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Integer visitType(Type t, Void ignored) {
            return Integer.valueOf(t.getTag().ordinal());
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Integer visitClassType(Type.ClassType t, Void ignored) {
            int result = visit(t.getEnclosingType()).intValue();
            int result2 = (result * 127) + t.tsym.flatName().hashCode();
            for (Type s : t.getTypeArguments()) {
                result2 = (result2 * 127) + visit(s).intValue();
            }
            return Integer.valueOf(result2);
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Integer visitMethodType(Type.MethodType t, Void ignored) {
            int h = TypeTag.METHOD.ordinal();
            for (List list = t.argtypes; list.tail != null; list = list.tail) {
                h = (h << 5) + visit((Type) list.head).intValue();
            }
            return Integer.valueOf((h << 5) + visit(t.restype).intValue());
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Integer visitWildcardType(Type.WildcardType t, Void ignored) {
            int result = t.kind.hashCode();
            if (t.type != null) {
                result = (result * 127) + visit(t.type).intValue();
            }
            return Integer.valueOf(result);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Integer visitArrayType(Type.ArrayType t, Void ignored) {
            return Integer.valueOf(visit(t.elemtype).intValue() + 12);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Integer visitTypeVar(Type.TypeVar t, Void ignored) {
            return Integer.valueOf(System.identityHashCode(t.tsym));
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Integer visitUndetVar(Type.UndetVar t, Void ignored) {
            return Integer.valueOf(System.identityHashCode(t));
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Integer visitErrorType(Type.ErrorType t, Void ignored) {
            return 0;
        }
    };
    List<Warner> warnStack = List.nil();
    private final UnaryVisitor<Boolean> isUnbounded = new UnaryVisitor<Boolean>() { // from class: com.sun.tools.javac.code.Types.1
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Void ignored) {
            return true;
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitClassType(Type.ClassType t, Void ignored) {
            List listAllparams = t.tsym.type.allparams();
            List listAllparams2 = t.allparams();
            while (listAllparams.nonEmpty()) {
                Type.WildcardType unb = new Type.WildcardType(Types.this.syms.objectType, BoundKind.UNBOUND, Types.this.syms.boundClass, (Type.TypeVar) ((Type) listAllparams.head).unannotatedType());
                if (!Types.this.containsType((Type) listAllparams2.head, unb)) {
                    return false;
                }
                listAllparams = listAllparams.tail;
                listAllparams2 = listAllparams2.tail;
            }
            return true;
        }
    };
    private final SimpleVisitor<Type, Symbol> asSub = new SimpleVisitor<Type, Symbol>() { // from class: com.sun.tools.javac.code.Types.2
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, Symbol sym) {
            return null;
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitClassType(Type.ClassType t, Symbol sym) {
            if (t.tsym == sym) {
                return t;
            }
            Type base = Types.this.asSuper(sym.type, t.tsym);
            if (base == null) {
                return null;
            }
            ListBuffer<Type> from = new ListBuffer<>();
            ListBuffer<Type> to = new ListBuffer<>();
            try {
                Types.this.adapt(base, t, from, to);
                Type res = Types.this.subst(sym.type, from.toList(), to.toList());
                if (!Types.this.isSubtype(res, t)) {
                    return null;
                }
                ListBuffer listBuffer = new ListBuffer();
                for (List listAllparams = sym.type.allparams(); listAllparams.nonEmpty(); listAllparams = listAllparams.tail) {
                    if (res.contains((Type) listAllparams.head) && !t.contains((Type) listAllparams.head)) {
                        listBuffer.append(listAllparams.head);
                    }
                }
                if (listBuffer.nonEmpty()) {
                    if (t.isRaw()) {
                        return Types.this.erasure(res);
                    }
                    List<Type> opens = listBuffer.toList();
                    ListBuffer<Type> qs = new ListBuffer<>();
                    for (List list = opens; list.nonEmpty(); list = list.tail) {
                        qs.append(new Type.WildcardType(Types.this.syms.objectType, BoundKind.UNBOUND, Types.this.syms.boundClass, (Type.TypeVar) ((Type) list.head).unannotatedType()));
                    }
                    return Types.this.subst(res, opens, qs.toList());
                }
                return res;
            } catch (AdaptFailure e) {
                return null;
            }
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitErrorType(Type.ErrorType t, Symbol sym) {
            return t;
        }
    };
    private DescriptorCache descCache = new DescriptorCache();
    private Filter<Symbol> bridgeFilter = new Filter<Symbol>() { // from class: com.sun.tools.javac.code.Types.3
        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol t) {
            return t.kind == 16 && t.name != Types.this.names.init && t.name != Types.this.names.clinit && (t.flags() & 4096) == 0;
        }
    };
    private TypeRelation isSubtype = new TypeRelation() { // from class: com.sun.tools.javac.code.Types.4
        private Set<TypePair> cache = new HashSet();

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Type s) {
            switch (t.getTag()) {
                case BYTE:
                    return Boolean.valueOf(!s.hasTag(TypeTag.CHAR) && t.getTag().isSubRangeOf(s.getTag()));
                case CHAR:
                    return Boolean.valueOf(!s.hasTag(TypeTag.SHORT) && t.getTag().isSubRangeOf(s.getTag()));
                case SHORT:
                case INT:
                case LONG:
                case FLOAT:
                case DOUBLE:
                    return Boolean.valueOf(t.getTag().isSubRangeOf(s.getTag()));
                case BOOLEAN:
                case VOID:
                    return Boolean.valueOf(t.hasTag(s.getTag()));
                case TYPEVAR:
                    return Boolean.valueOf(Types.this.isSubtypeNoCapture(t.getUpperBound(), s));
                case BOT:
                    if (!s.hasTag(TypeTag.BOT) && !s.hasTag(TypeTag.CLASS) && !s.hasTag(TypeTag.ARRAY) && !s.hasTag(TypeTag.TYPEVAR)) {
                        z = false;
                    }
                    return Boolean.valueOf(z);
                case WILDCARD:
                case NONE:
                    return false;
                default:
                    throw new AssertionError("isSubtype " + t.getTag());
            }
        }

        private boolean containsTypeRecursive(Type t, Type s) {
            TypePair pair = new TypePair(Types.this, t, s);
            if (this.cache.add(pair)) {
                try {
                    return Types.this.containsType(t.getTypeArguments(), s.getTypeArguments());
                } finally {
                    this.cache.remove(pair);
                }
            }
            return Types.this.containsType(t.getTypeArguments(), rewriteSupers(s).getTypeArguments());
        }

        private Type rewriteSupers(Type t) {
            if (!t.isParameterized()) {
                return t;
            }
            ListBuffer<Type> from = new ListBuffer<>();
            ListBuffer<Type> to = new ListBuffer<>();
            Types.this.adaptSelf(t, from, to);
            if (from.isEmpty()) {
                return t;
            }
            ListBuffer<Type> rewrite = new ListBuffer<>();
            boolean changed = false;
            for (Type orig : to.toList()) {
                Type s = rewriteSupers(orig);
                if (s.isSuperBound() && !s.isExtendsBound()) {
                    s = new Type.WildcardType(Types.this.syms.objectType, BoundKind.UNBOUND, Types.this.syms.boundClass);
                    changed = true;
                } else if (s != orig) {
                    s = new Type.WildcardType(Types.this.wildUpperBound(s), BoundKind.EXTENDS, Types.this.syms.boundClass);
                    changed = true;
                }
                rewrite.append(s);
            }
            if (changed) {
                return Types.this.subst(t.tsym.type, from.toList(), rewrite.toList());
            }
            return t;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitClassType(Type.ClassType t, Type s) {
            Type sup = Types.this.asSuper(t, s.tsym);
            boolean z = false;
            if (sup == null) {
                return false;
            }
            if (!sup.hasTag(TypeTag.CLASS)) {
                return Boolean.valueOf(Types.this.isSubtypeNoCapture(sup, s));
            }
            if (sup.tsym == s.tsym && ((!s.isParameterized() || containsTypeRecursive(s, sup)) && Types.this.isSubtypeNoCapture(sup.getEnclosingType(), s.getEnclosingType()))) {
                z = true;
            }
            return Boolean.valueOf(z);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitArrayType(Type.ArrayType t, Type s) {
            if (s.hasTag(TypeTag.ARRAY)) {
                if (t.elemtype.isPrimitive()) {
                    return Boolean.valueOf(Types.this.isSameType(t.elemtype, Types.this.elemtype(s)));
                }
                return Boolean.valueOf(Types.this.isSubtypeNoCapture(t.elemtype, Types.this.elemtype(s)));
            }
            if (s.hasTag(TypeTag.CLASS)) {
                Name sname = s.tsym.getQualifiedName();
                return Boolean.valueOf(sname == Types.this.names.java_lang_Object || sname == Types.this.names.java_lang_Cloneable || sname == Types.this.names.java_io_Serializable);
            }
            return false;
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitUndetVar(Type.UndetVar t, Type s) {
            if (t == s || t.qtype == s || s.hasTag(TypeTag.ERROR) || s.hasTag(TypeTag.UNKNOWN)) {
                return true;
            }
            if (s.hasTag(TypeTag.BOT)) {
                return false;
            }
            t.addBound(Type.UndetVar.InferenceBound.UPPER, s, Types.this);
            return true;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitErrorType(Type.ErrorType t, Type s) {
            return true;
        }
    };
    TypeRelation isSameTypeLoose = new LooseSameTypeVisitor();
    TypeRelation isSameTypeStrict = new SameTypeVisitor() { // from class: com.sun.tools.javac.code.Types.5
        @Override // com.sun.tools.javac.code.Types.SameTypeVisitor
        boolean sameTypeVars(Type.TypeVar tv1, Type.TypeVar tv2) {
            return tv1 == tv2;
        }

        @Override // com.sun.tools.javac.code.Types.SameTypeVisitor
        protected boolean containsTypes(List<Type> ts1, List<Type> ts2) {
            return Types.this.isSameTypes(ts1, ts2, true);
        }

        @Override // com.sun.tools.javac.code.Types.SameTypeVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitWildcardType(Type.WildcardType t, Type s) {
            boolean z = false;
            if (!s.hasTag(TypeTag.WILDCARD)) {
                return false;
            }
            Type.WildcardType t2 = (Type.WildcardType) s.unannotatedType();
            if (t.kind == t2.kind && Types.this.isSameType(t.type, t2.type, true)) {
                z = true;
            }
            return Boolean.valueOf(z);
        }
    };
    TypeRelation isSameAnnotatedType = new LooseSameTypeVisitor() { // from class: com.sun.tools.javac.code.Types.6
        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitAnnotatedType(Type.AnnotatedType t, Type s) {
            if (!s.isAnnotated() || !t.getAnnotationMirrors().containsAll(s.getAnnotationMirrors()) || !s.getAnnotationMirrors().containsAll(t.getAnnotationMirrors())) {
                return false;
            }
            return visit(t.unannotatedType(), s);
        }
    };
    private TypeRelation containsType = new TypeRelation() { // from class: com.sun.tools.javac.code.Types.7
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Type s) {
            if (s.isPartial()) {
                return Boolean.valueOf(Types.this.containedBy(s, t));
            }
            return Boolean.valueOf(Types.this.isSameType(t, s));
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitWildcardType(Type.WildcardType t, Type s) {
            if (s.isPartial()) {
                return Boolean.valueOf(Types.this.containedBy(s, t));
            }
            return Boolean.valueOf(Types.this.isSameWildcard(t, s) || t.type == s || Types.this.isCaptureOf(s, t) || ((t.isExtendsBound() || Types.this.isSubtypeNoCapture(Types.this.wildLowerBound(t), Types.this.cvarLowerBound(Types.this.wildLowerBound(s)))) && (t.isSuperBound() || Types.this.isSubtypeNoCapture(Types.this.cvarUpperBound(Types.this.wildUpperBound(s)), Types.this.wildUpperBound(t)))));
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitUndetVar(Type.UndetVar t, Type s) {
            if (!s.hasTag(TypeTag.WILDCARD)) {
                return Boolean.valueOf(Types.this.isSameType(t, s));
            }
            return false;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitErrorType(Type.ErrorType t, Type s) {
            return true;
        }
    };
    private TypeRelation isCastable = new TypeRelation() { // from class: com.sun.tools.javac.code.Types.8
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Type s) {
            if (s.hasTag(TypeTag.ERROR)) {
                return true;
            }
            switch (t.getTag()) {
                case BYTE:
                case CHAR:
                case SHORT:
                case INT:
                case LONG:
                case FLOAT:
                case DOUBLE:
                    return Boolean.valueOf(s.isNumeric());
                case BOOLEAN:
                    return Boolean.valueOf(s.hasTag(TypeTag.BOOLEAN));
                case VOID:
                    return false;
                case TYPEVAR:
                default:
                    throw new AssertionError();
                case BOT:
                    return Boolean.valueOf(Types.this.isSubtype(t, s));
            }
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitWildcardType(Type.WildcardType t, Type s) {
            return Boolean.valueOf(Types.this.isCastable(Types.this.wildUpperBound(t), s, Types.this.warnStack.head));
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitClassType(Type.ClassType t, Type s) {
            Type lowSub;
            Type lowSub2;
            Type lowSub3;
            if (s.hasTag(TypeTag.ERROR) || s.hasTag(TypeTag.BOT)) {
                return true;
            }
            if (s.hasTag(TypeTag.TYPEVAR)) {
                if (Types.this.isCastable(t, s.getUpperBound(), Types.this.noWarnings)) {
                    Types.this.warnStack.head.warn(Lint.LintCategory.UNCHECKED);
                    return true;
                }
                return false;
            }
            if (t.isIntersection() || s.isIntersection()) {
                return Boolean.valueOf(!t.isIntersection() ? visitIntersectionType((Type.IntersectionClassType) s.unannotatedType(), t, true) : visitIntersectionType((Type.IntersectionClassType) t.unannotatedType(), s, false));
            }
            if (s.hasTag(TypeTag.CLASS) || s.hasTag(TypeTag.ARRAY)) {
                boolean upcast = Types.this.isSubtype(Types.this.erasure(t), Types.this.erasure(s));
                if (upcast || Types.this.isSubtype(Types.this.erasure(s), Types.this.erasure(t))) {
                    if (!upcast && s.hasTag(TypeTag.ARRAY)) {
                        if (!Types.this.isReifiable(s)) {
                            Types.this.warnStack.head.warn(Lint.LintCategory.UNCHECKED);
                        }
                        return true;
                    }
                    if (s.isRaw()) {
                        return true;
                    }
                    if (t.isRaw()) {
                        if (!Types.this.isUnbounded(s)) {
                            Types.this.warnStack.head.warn(Lint.LintCategory.UNCHECKED);
                        }
                        return true;
                    }
                    Type a = upcast ? t : s;
                    Type b = upcast ? s : t;
                    Type aHigh = Types.this.rewriteQuantifiers(a, true, false);
                    Type aLow = Types.this.rewriteQuantifiers(a, false, false);
                    Type bHigh = Types.this.rewriteQuantifiers(b, true, false);
                    Type bLow = Types.this.rewriteQuantifiers(b, false, false);
                    Type lowSub4 = Types.this.asSub(bLow, aLow.tsym);
                    if (lowSub4 == null) {
                        lowSub = lowSub4;
                        lowSub2 = null;
                    } else {
                        lowSub = lowSub4;
                        lowSub2 = Types.this.asSub(bHigh, aHigh.tsym);
                    }
                    if (lowSub2 == null) {
                        aHigh = Types.this.rewriteQuantifiers(a, true, true);
                        aLow = Types.this.rewriteQuantifiers(a, false, true);
                        Type bHigh2 = Types.this.rewriteQuantifiers(b, true, true);
                        Type bLow2 = Types.this.rewriteQuantifiers(b, false, true);
                        Type lowSub5 = Types.this.asSub(bLow2, aLow.tsym);
                        lowSub3 = lowSub5;
                        lowSub2 = lowSub5 == null ? null : Types.this.asSub(bHigh2, aHigh.tsym);
                    } else {
                        lowSub3 = lowSub;
                    }
                    if (lowSub2 != null) {
                        if (a.tsym != lowSub2.tsym || a.tsym != lowSub3.tsym) {
                            Assert.error(a.tsym + " != " + lowSub2.tsym + " != " + lowSub3.tsym);
                        }
                        if (!Types.this.disjointTypes(aHigh.allparams(), lowSub2.allparams()) && !Types.this.disjointTypes(aHigh.allparams(), lowSub3.allparams()) && !Types.this.disjointTypes(aLow.allparams(), lowSub2.allparams()) && !Types.this.disjointTypes(aLow.allparams(), lowSub3.allparams())) {
                            Types types = Types.this;
                            if (!upcast ? types.giveWarning(b, a) : types.giveWarning(a, b)) {
                                Types.this.warnStack.head.warn(Lint.LintCategory.UNCHECKED);
                            }
                            return true;
                        }
                    }
                    if (!Types.this.isReifiable(s)) {
                        return Boolean.valueOf(Types.this.isSubtypeUnchecked(a, b, Types.this.warnStack.head));
                    }
                    return Boolean.valueOf(Types.this.isSubtypeUnchecked(a, b));
                }
                if (s.hasTag(TypeTag.CLASS)) {
                    if ((s.tsym.flags() & 512) != 0) {
                        return Boolean.valueOf((t.tsym.flags() & 16) == 0 ? Types.this.sideCast(t, s, Types.this.warnStack.head) : Types.this.sideCastFinal(t, s, Types.this.warnStack.head));
                    }
                    if ((512 & t.tsym.flags()) != 0) {
                        return Boolean.valueOf((s.tsym.flags() & 16) == 0 ? Types.this.sideCast(t, s, Types.this.warnStack.head) : Types.this.sideCastFinal(t, s, Types.this.warnStack.head));
                    }
                    return false;
                }
            }
            return false;
        }

        boolean visitIntersectionType(Type.IntersectionClassType ict, Type s, boolean reverse) {
            Warner warn = Types.this.noWarnings;
            for (Type c : ict.getComponents()) {
                warn.clear();
                Types types = Types.this;
                if (reverse) {
                    if (!types.isCastable(s, c, warn)) {
                        return false;
                    }
                } else if (!types.isCastable(c, s, warn)) {
                    return false;
                }
            }
            if (warn.hasLint(Lint.LintCategory.UNCHECKED)) {
                Types.this.warnStack.head.warn(Lint.LintCategory.UNCHECKED);
                return true;
            }
            return true;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitArrayType(Type.ArrayType t, Type s) {
            switch (s.getTag()) {
                case ARRAY:
                    if (Types.this.elemtype(t).isPrimitive() || Types.this.elemtype(s).isPrimitive()) {
                        return Boolean.valueOf(Types.this.elemtype(t).hasTag(Types.this.elemtype(s).getTag()));
                    }
                    return visit(Types.this.elemtype(t), Types.this.elemtype(s));
                case CLASS:
                    return Boolean.valueOf(Types.this.isSubtype(t, s));
                case TYPEVAR:
                    if (Types.this.isCastable(s, t, Types.this.noWarnings)) {
                        Types.this.warnStack.head.warn(Lint.LintCategory.UNCHECKED);
                        return true;
                    }
                    return false;
                case BOT:
                case ERROR:
                    return true;
                default:
                    return false;
            }
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitTypeVar(Type.TypeVar t, Type s) {
            switch (s.getTag()) {
                case TYPEVAR:
                    if (Types.this.isSubtype(t, s)) {
                        return true;
                    }
                    if (Types.this.isCastable(t.bound, s, Types.this.noWarnings)) {
                        Types.this.warnStack.head.warn(Lint.LintCategory.UNCHECKED);
                        return true;
                    }
                    return false;
                case BOT:
                case ERROR:
                    return true;
                case WILDCARD:
                case NONE:
                default:
                    return Boolean.valueOf(Types.this.isCastable(t.bound, s, Types.this.warnStack.head));
            }
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitErrorType(Type.ErrorType t, Type s) {
            return true;
        }
    };
    private TypeRelation disjointType = new TypeRelation() { // from class: com.sun.tools.javac.code.Types.9
        private Set<TypePair> cache = new HashSet();

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Type s) {
            if (s.hasTag(TypeTag.WILDCARD)) {
                return visit(s, t);
            }
            return Boolean.valueOf(notSoftSubtypeRecursive(t, s) || notSoftSubtypeRecursive(s, t));
        }

        private boolean isCastableRecursive(Type t, Type s) {
            TypePair pair = new TypePair(Types.this, t, s);
            if (this.cache.add(pair)) {
                try {
                    return Types.this.isCastable(t, s);
                } finally {
                    this.cache.remove(pair);
                }
            }
            return true;
        }

        private boolean notSoftSubtypeRecursive(Type t, Type s) {
            TypePair pair = new TypePair(Types.this, t, s);
            if (this.cache.add(pair)) {
                try {
                    return Types.this.notSoftSubtype(t, s);
                } finally {
                    this.cache.remove(pair);
                }
            }
            return false;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitWildcardType(Type.WildcardType t, Type s) {
            if (t.isUnbound()) {
                return false;
            }
            if (!s.hasTag(TypeTag.WILDCARD)) {
                if (t.isExtendsBound()) {
                    return Boolean.valueOf(notSoftSubtypeRecursive(s, t.type));
                }
                return Boolean.valueOf(notSoftSubtypeRecursive(t.type, s));
            }
            if (s.isUnbound()) {
                return false;
            }
            if (t.isExtendsBound()) {
                if (s.isExtendsBound()) {
                    return Boolean.valueOf(!isCastableRecursive(t.type, Types.this.wildUpperBound(s)));
                }
                if (s.isSuperBound()) {
                    return Boolean.valueOf(notSoftSubtypeRecursive(Types.this.wildLowerBound(s), t.type));
                }
            } else if (t.isSuperBound() && s.isExtendsBound()) {
                return Boolean.valueOf(notSoftSubtypeRecursive(t.type, Types.this.wildUpperBound(s)));
            }
            return false;
        }
    };
    private final Type.Mapping cvarLowerBoundMapping = new Type.Mapping("cvarLowerBound") { // from class: com.sun.tools.javac.code.Types.10
        @Override // com.sun.tools.javac.code.Type.Mapping
        public Type apply(Type t) {
            return Types.this.cvarLowerBound(t);
        }
    };
    private UnaryVisitor<Boolean> isReifiable = new UnaryVisitor<Boolean>() { // from class: com.sun.tools.javac.code.Types.11
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Void ignored) {
            return true;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitClassType(Type.ClassType t, Void ignored) {
            if (t.isCompound()) {
                return false;
            }
            if (!t.isParameterized()) {
                return true;
            }
            for (Type param : t.allparams()) {
                if (!param.isUnbound()) {
                    return false;
                }
            }
            return true;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitArrayType(Type.ArrayType t, Void ignored) {
            return visit(t.elemtype);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitTypeVar(Type.TypeVar t, Void ignored) {
            return false;
        }
    };
    private Type.Mapping elemTypeFun = new Type.Mapping("elemTypeFun") { // from class: com.sun.tools.javac.code.Types.12
        @Override // com.sun.tools.javac.code.Type.Mapping
        public Type apply(Type t) {
            while (t.hasTag(TypeTag.TYPEVAR)) {
                t = t.getUpperBound();
            }
            return Types.this.elemtype(t);
        }
    };
    private SimpleVisitor<Type, Symbol> asSuper = new SimpleVisitor<Type, Symbol>() { // from class: com.sun.tools.javac.code.Types.13
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, Symbol sym) {
            return null;
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitClassType(Type.ClassType t, Symbol sym) {
            Type x;
            Type x2;
            if (t.tsym == sym) {
                return t;
            }
            Type st = Types.this.supertype(t);
            if ((st.hasTag(TypeTag.CLASS) || st.hasTag(TypeTag.TYPEVAR)) && (x = Types.this.asSuper(st, sym)) != null) {
                return x;
            }
            if ((sym.flags() & 512) != 0) {
                for (List listInterfaces = Types.this.interfaces(t); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
                    if (!((Type) listInterfaces.head).hasTag(TypeTag.ERROR) && (x2 = Types.this.asSuper((Type) listInterfaces.head, sym)) != null) {
                        return x2;
                    }
                }
                return null;
            }
            return null;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitArrayType(Type.ArrayType t, Symbol sym) {
            if (Types.this.isSubtype(t, sym.type)) {
                return sym.type;
            }
            return null;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitTypeVar(Type.TypeVar t, Symbol sym) {
            if (t.tsym == sym) {
                return t;
            }
            return Types.this.asSuper(t.bound, sym);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitErrorType(Type.ErrorType t, Symbol sym) {
            return t;
        }
    };
    private SimpleVisitor<Type, Symbol> memberType = new SimpleVisitor<Type, Symbol>() { // from class: com.sun.tools.javac.code.Types.14
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, Symbol sym) {
            return sym.type;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitWildcardType(Type.WildcardType t, Symbol sym) {
            return Types.this.memberType(Types.this.wildUpperBound(t), sym);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitClassType(Type.ClassType t, Symbol sym) {
            Symbol owner = sym.owner;
            long flags = sym.flags();
            if ((8 & flags) == 0 && owner.type.isParameterized()) {
                Type base = Types.this.asOuterSuper(t, owner);
                Type base2 = t.isCompound() ? Types.this.capture(base) : base;
                if (base2 != null) {
                    List<Type> ownerParams = owner.type.allparams();
                    List<Type> baseParams = base2.allparams();
                    if (ownerParams.nonEmpty()) {
                        if (baseParams.isEmpty()) {
                            return Types.this.erasure(sym.type);
                        }
                        return Types.this.subst(sym.type, ownerParams, baseParams);
                    }
                }
            }
            return sym.type;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitTypeVar(Type.TypeVar t, Symbol sym) {
            return Types.this.memberType(t.bound, sym);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitErrorType(Type.ErrorType t, Symbol sym) {
            return t;
        }
    };
    private SimpleVisitor<Type, Boolean> erasure = new SimpleVisitor<Type, Boolean>() { // from class: com.sun.tools.javac.code.Types.15
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, Boolean recurse) {
            if (t.isPrimitive()) {
                return t;
            }
            return t.map(recurse.booleanValue() ? Types.this.erasureRecFun : Types.this.erasureFun);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitWildcardType(Type.WildcardType t, Boolean recurse) {
            return Types.this.erasure(Types.this.wildUpperBound(t), recurse.booleanValue());
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitClassType(Type.ClassType t, Boolean recurse) {
            Type erased = t.tsym.erasure(Types.this);
            if (recurse.booleanValue()) {
                return new Type.ErasedClassType(erased.getEnclosingType(), erased.tsym);
            }
            return erased;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitTypeVar(Type.TypeVar t, Boolean recurse) {
            return Types.this.erasure(t.bound, recurse.booleanValue());
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitErrorType(Type.ErrorType t, Boolean recurse) {
            return t;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitAnnotatedType(Type.AnnotatedType t, Boolean recurse) {
            Type erased = Types.this.erasure(t.unannotatedType(), recurse.booleanValue());
            if (erased.isAnnotated()) {
                erased = ((Type.AnnotatedType) erased).unannotatedType();
            }
            return erased.annotatedType(t.getAnnotationMirrors());
        }
    };
    private Type.Mapping erasureFun = new Type.Mapping("erasure") { // from class: com.sun.tools.javac.code.Types.16
        @Override // com.sun.tools.javac.code.Type.Mapping
        public Type apply(Type t) {
            return Types.this.erasure(t);
        }
    };
    private Type.Mapping erasureRecFun = new Type.Mapping("erasureRecursive") { // from class: com.sun.tools.javac.code.Types.17
        @Override // com.sun.tools.javac.code.Type.Mapping
        public Type apply(Type t) {
            return Types.this.erasureRecursive(t);
        }
    };
    private UnaryVisitor<Type> supertype = new UnaryVisitor<Type>() { // from class: com.sun.tools.javac.code.Types.18
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, Void ignored) {
            return Type.noType;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitClassType(Type.ClassType t, Void ignored) {
            if (t.supertype_field == null) {
                Type supertype = ((Symbol.ClassSymbol) t.tsym).getSuperclass();
                if (t.isInterface()) {
                    supertype = ((Type.ClassType) t.tsym.type).supertype_field;
                }
                if (t.supertype_field == null) {
                    List<Type> actuals = Types.this.classBound(t).allparams();
                    List<Type> formals = t.tsym.type.allparams();
                    if (t.hasErasedSupertypes()) {
                        t.supertype_field = Types.this.erasureRecursive(supertype);
                    } else if (formals.nonEmpty()) {
                        t.supertype_field = Types.this.subst(supertype, formals, actuals);
                    } else {
                        t.supertype_field = supertype;
                    }
                }
            }
            Type supertype2 = t.supertype_field;
            return supertype2;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitTypeVar(Type.TypeVar t, Void ignored) {
            if (t.bound.hasTag(TypeTag.TYPEVAR) || (!t.bound.isCompound() && !t.bound.isInterface())) {
                return t.bound;
            }
            return Types.this.supertype(t.bound);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitArrayType(Type.ArrayType t, Void ignored) {
            if (t.elemtype.isPrimitive() || Types.this.isSameType(t.elemtype, Types.this.syms.objectType)) {
                return Types.this.arraySuperType();
            }
            return new Type.ArrayType(Types.this.supertype(t.elemtype), t.tsym);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitErrorType(Type.ErrorType t, Void ignored) {
            return Type.noType;
        }
    };
    private UnaryVisitor<List<Type>> interfaces = new UnaryVisitor<List<Type>>() { // from class: com.sun.tools.javac.code.Types.19
        @Override // com.sun.tools.javac.code.Type.Visitor
        public List<Type> visitType(Type t, Void ignored) {
            return List.nil();
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public List<Type> visitClassType(Type.ClassType t, Void ignored) {
            if (t.interfaces_field == null) {
                List<Type> interfaces = ((Symbol.ClassSymbol) t.tsym).getInterfaces();
                if (t.interfaces_field == null) {
                    Assert.check(t != t.tsym.type, t);
                    List<Type> actuals = t.allparams();
                    List<Type> formals = t.tsym.type.allparams();
                    if (t.hasErasedSupertypes()) {
                        t.interfaces_field = Types.this.erasureRecursive(interfaces);
                    } else if (formals.nonEmpty()) {
                        t.interfaces_field = Types.this.subst(interfaces, formals, actuals);
                    } else {
                        t.interfaces_field = interfaces;
                    }
                }
            }
            return t.interfaces_field;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public List<Type> visitTypeVar(Type.TypeVar t, Void ignored) {
            if (t.bound.isCompound()) {
                return Types.this.interfaces(t.bound);
            }
            if (t.bound.isInterface()) {
                return List.of(t.bound);
            }
            return List.nil();
        }
    };
    private final UnaryVisitor<List<Type>> directSupertypes = new UnaryVisitor<List<Type>>() { // from class: com.sun.tools.javac.code.Types.20
        @Override // com.sun.tools.javac.code.Type.Visitor
        public List<Type> visitType(Type type, Void ignored) {
            if (!type.isIntersection()) {
                Type sup = Types.this.supertype(type);
                if (sup == Type.noType || sup == type || sup == null) {
                    return Types.this.interfaces(type);
                }
                return Types.this.interfaces(type).prepend(sup);
            }
            return visitIntersectionType((Type.IntersectionClassType) type);
        }

        private List<Type> visitIntersectionType(Type.IntersectionClassType it) {
            return it.getExplicitComponents();
        }
    };
    Map<Type, Boolean> isDerivedRawCache = new HashMap();
    private UnaryVisitor<Type> classBound = new UnaryVisitor<Type>() { // from class: com.sun.tools.javac.code.Types.21
        @Override // com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, Void ignored) {
            return t;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitClassType(Type.ClassType t, Void ignored) {
            Type outer1 = Types.this.classBound(t.getEnclosingType());
            if (outer1 != t.getEnclosingType()) {
                return new Type.ClassType(outer1, t.getTypeArguments(), t.tsym);
            }
            return t;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitTypeVar(Type.TypeVar t, Void ignored) {
            return Types.this.classBound(Types.this.supertype(t));
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitErrorType(Type.ErrorType t, Void ignored) {
            return t;
        }
    };
    private ImplementationCache implCache = new ImplementationCache();
    private MembersClosureCache membersCache = new MembersClosureCache();
    TypeRelation hasSameArgs_strict = new HasSameArgs(true);
    TypeRelation hasSameArgs_nonstrict = new HasSameArgs(false);
    private final MapVisitor<List<Type>> methodWithParameters = new MapVisitor<List<Type>>() { // from class: com.sun.tools.javac.code.Types.23
        @Override // com.sun.tools.javac.code.Types.MapVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, List<Type> newParams) {
            throw new IllegalArgumentException("Not a method type: " + t);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitMethodType(Type.MethodType t, List<Type> newParams) {
            return new Type.MethodType(newParams, t.restype, t.thrown, t.tsym);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitForAll(Type.ForAll t, List<Type> newParams) {
            return new Type.ForAll(t.tvars, (Type) t.qtype.accept(this, newParams));
        }
    };
    private final MapVisitor<List<Type>> methodWithThrown = new MapVisitor<List<Type>>() { // from class: com.sun.tools.javac.code.Types.24
        @Override // com.sun.tools.javac.code.Types.MapVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, List<Type> newThrown) {
            throw new IllegalArgumentException("Not a method type: " + t);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitMethodType(Type.MethodType t, List<Type> newThrown) {
            return new Type.MethodType(t.argtypes, t.restype, newThrown, t.tsym);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitForAll(Type.ForAll t, List<Type> newThrown) {
            return new Type.ForAll(t.tvars, (Type) t.qtype.accept(this, newThrown));
        }
    };
    private final MapVisitor<Type> methodWithReturn = new MapVisitor<Type>() { // from class: com.sun.tools.javac.code.Types.25
        @Override // com.sun.tools.javac.code.Types.MapVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, Type newReturn) {
            throw new IllegalArgumentException("Not a method type: " + t);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitMethodType(Type.MethodType t, Type newReturn) {
            return new Type.MethodType(t.argtypes, newReturn, t.thrown, t.tsym);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitForAll(Type.ForAll t, Type newReturn) {
            return new Type.ForAll(t.tvars, (Type) t.qtype.accept(this, newReturn));
        }
    };
    private Map<Type, List<Type>> closureCache = new HashMap();
    Set<TypePair> mergeCache = new HashSet();
    private Type arraySuperType = null;

    public static class AdaptFailure extends RuntimeException {
        static final long serialVersionUID = -7490231548272701566L;
    }

    public static abstract class TypeRelation extends SimpleVisitor<Boolean, Type> {
    }

    public static Types instance(Context context) {
        Types instance = (Types) context.get(typesKey);
        if (instance == null) {
            return new Types(context);
        }
        return instance;
    }

    protected Types(Context context) {
        context.put(typesKey, this);
        this.syms = Symtab.instance(context);
        this.names = Names.instance(context);
        Source source = Source.instance(context);
        this.allowBoxing = source.allowBoxing();
        this.allowCovariantReturns = source.allowCovariantReturns();
        this.allowObjectToPrimitiveCast = source.allowObjectToPrimitiveCast();
        this.allowDefaultMethods = source.allowDefaultMethods();
        this.reader = ClassReader.instance(context);
        this.chk = Check.instance(context);
        this.enter = Enter.instance(context);
        this.capturedName = this.names.fromString("<captured wildcard>");
        this.messages = JavacMessages.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        this.functionDescriptorLookupError = new FunctionDescriptorLookupError();
        this.noWarnings = new Warner(null);
    }

    public Type wildUpperBound(Type t) {
        if (t.hasTag(TypeTag.WILDCARD)) {
            Type.WildcardType w = (Type.WildcardType) t.unannotatedType();
            if (w.isSuperBound()) {
                return w.bound == null ? this.syms.objectType : w.bound.bound;
            }
            return wildUpperBound(w.type);
        }
        return t.unannotatedType();
    }

    public Type cvarUpperBound(Type t) {
        if (t.hasTag(TypeTag.TYPEVAR)) {
            Type.TypeVar v = (Type.TypeVar) t.unannotatedType();
            return v.isCaptured() ? cvarUpperBound(v.bound) : v;
        }
        return t.unannotatedType();
    }

    public Type wildLowerBound(Type t) {
        if (t.hasTag(TypeTag.WILDCARD)) {
            Type.WildcardType w = (Type.WildcardType) t.unannotatedType();
            return w.isExtendsBound() ? this.syms.botType : wildLowerBound(w.type);
        }
        return t.unannotatedType();
    }

    public Type cvarLowerBound(Type t) {
        if (t.hasTag(TypeTag.TYPEVAR)) {
            Type.TypeVar v = (Type.TypeVar) t.unannotatedType();
            return v.isCaptured() ? cvarLowerBound(v.getLowerBound()) : v;
        }
        return t.unannotatedType();
    }

    public boolean isUnbounded(Type t) {
        return this.isUnbounded.visit(t).booleanValue();
    }

    public Type asSub(Type t, Symbol sym) {
        return this.asSub.visit(t, sym);
    }

    public boolean isConvertible(Type t, Type s, Warner warn) {
        if (t.hasTag(TypeTag.ERROR)) {
            return true;
        }
        boolean tPrimitive = t.isPrimitive();
        boolean sPrimitive = s.isPrimitive();
        if (tPrimitive == sPrimitive) {
            return isSubtypeUnchecked(t, s, warn);
        }
        if (!this.allowBoxing) {
            return false;
        }
        if (tPrimitive) {
            return isSubtype(boxedClass(t).type, s);
        }
        return isSubtype(unboxedType(t), s);
    }

    public boolean isConvertible(Type t, Type s) {
        return isConvertible(t, s, this.noWarnings);
    }

    public static class FunctionDescriptorLookupError extends RuntimeException {
        private static final long serialVersionUID = 0;
        JCDiagnostic diagnostic = null;

        FunctionDescriptorLookupError() {
        }

        FunctionDescriptorLookupError setMessage(JCDiagnostic diag) {
            this.diagnostic = diag;
            return this;
        }

        public JCDiagnostic getDiagnostic() {
            return this.diagnostic;
        }
    }

    class DescriptorCache {
        private WeakHashMap<Symbol.TypeSymbol, Entry> _map = new WeakHashMap<>();

        DescriptorCache() {
        }

        class FunctionDescriptor {
            Symbol descSym;

            FunctionDescriptor(Symbol descSym) {
                this.descSym = descSym;
            }

            public Symbol getSymbol() {
                return this.descSym;
            }

            public Type getType(Type site) {
                Type site2 = Types.this.removeWildcards(site);
                if (!Types.this.chk.checkValidGenericType(site2)) {
                    throw DescriptorCache.this.failure(Types.this.diags.fragment("no.suitable.functional.intf.inst", site2));
                }
                return Types.this.memberType(site2, this.descSym);
            }
        }

        class Entry {
            final FunctionDescriptor cachedDescRes;
            final int prevMark;

            public Entry(FunctionDescriptor cachedDescRes, int prevMark) {
                this.cachedDescRes = cachedDescRes;
                this.prevMark = prevMark;
            }

            boolean matches(int mark) {
                return this.prevMark == mark;
            }
        }

        FunctionDescriptor get(Symbol.TypeSymbol origin) throws FunctionDescriptorLookupError {
            Entry e = this._map.get(origin);
            Scope.CompoundScope members = Types.this.membersClosure(origin.type, false);
            if (e == null || !e.matches(members.getMark())) {
                FunctionDescriptor descRes = findDescriptorInternal(origin, members);
                this._map.put(origin, new Entry(descRes, members.getMark()));
                return descRes;
            }
            return e.cachedDescRes;
        }

        public FunctionDescriptor findDescriptorInternal(Symbol.TypeSymbol origin, Scope.CompoundScope membersCache) throws FunctionDescriptorLookupError {
            if (!origin.isInterface() || (origin.flags() & 8192) != 0) {
                throw failure("not.a.functional.intf", origin);
            }
            ListBuffer<Symbol> abstracts = new ListBuffer<>();
            for (Symbol sym : membersCache.getElements(Types.this.new DescriptorFilter(origin))) {
                Type mtype = Types.this.memberType(origin.type, sym);
                if (abstracts.isEmpty() || (sym.name == abstracts.first().name && Types.this.overrideEquivalent(mtype, Types.this.memberType(origin.type, abstracts.first())))) {
                    abstracts.append(sym);
                } else {
                    throw failure("not.a.functional.intf.1", origin, Types.this.diags.fragment("incompatible.abstracts", Kinds.kindName(origin), origin));
                }
            }
            if (abstracts.isEmpty()) {
                throw failure("not.a.functional.intf.1", origin, Types.this.diags.fragment("no.abstracts", Kinds.kindName(origin), origin));
            }
            if (abstracts.size() == 1) {
                return new FunctionDescriptor(abstracts.first());
            }
            FunctionDescriptor descRes = mergeDescriptors(origin, abstracts.toList());
            if (descRes == null) {
                ListBuffer<JCDiagnostic> descriptors = new ListBuffer<>();
                for (Symbol desc : abstracts) {
                    String key = desc.type.mo179getThrownTypes().nonEmpty() ? "descriptor.throws" : "descriptor";
                    descriptors.append(Types.this.diags.fragment(key, desc.name, desc.type.mo176getParameterTypes(), desc.type.mo178getReturnType(), desc.type.mo179getThrownTypes()));
                }
                JCDiagnostic.MultilineDiagnostic incompatibleDescriptors = new JCDiagnostic.MultilineDiagnostic(Types.this.diags.fragment("incompatible.descs.in.functional.intf", Kinds.kindName(origin), origin), descriptors.toList());
                throw failure(incompatibleDescriptors);
            }
            return descRes;
        }

        private FunctionDescriptor mergeDescriptors(Symbol.TypeSymbol origin, List<Symbol> methodSyms) {
            List<Type> thrown_mt2;
            Symbol.TypeSymbol typeSymbol = origin;
            List<Symbol> mostSpecific = List.nil();
            for (Symbol msym1 : methodSyms) {
                Type mt1 = Types.this.memberType(typeSymbol.type, msym1);
                Iterator<Symbol> it = methodSyms.iterator();
                while (true) {
                    if (it.hasNext()) {
                        Symbol msym2 = it.next();
                        if (!Types.this.isSubSignature(mt1, Types.this.memberType(typeSymbol.type, msym2))) {
                            break;
                        }
                    } else {
                        mostSpecific = mostSpecific.prepend(msym1);
                        break;
                    }
                }
            }
            if (mostSpecific.isEmpty()) {
                return null;
            }
            boolean phase2 = false;
            Symbol bestSoFar = null;
            while (bestSoFar == null) {
                for (Symbol msym12 : mostSpecific) {
                    Type mt12 = Types.this.memberType(typeSymbol.type, msym12);
                    Iterator<Symbol> it2 = methodSyms.iterator();
                    while (true) {
                        if (it2.hasNext()) {
                            Symbol msym22 = it2.next();
                            Type mt2 = Types.this.memberType(typeSymbol.type, msym22);
                            if (!phase2) {
                                if (!isSubtypeInternal(mt12.mo178getReturnType(), mt2.mo178getReturnType())) {
                                    break;
                                }
                            } else if (!Types.this.returnTypeSubstitutable(mt12, mt2)) {
                                break;
                            }
                        } else {
                            bestSoFar = msym12;
                            break;
                        }
                    }
                }
                if (phase2) {
                    break;
                }
                phase2 = true;
            }
            if (bestSoFar == null) {
                return null;
            }
            boolean toErase = !bestSoFar.type.hasTag(TypeTag.FORALL);
            List<Type> thrown = null;
            Type mt13 = Types.this.memberType(typeSymbol.type, bestSoFar);
            for (Symbol msym23 : methodSyms) {
                Type mt22 = Types.this.memberType(typeSymbol.type, msym23);
                List<Type> thrown_mt22 = mt22.mo179getThrownTypes();
                if (toErase) {
                    thrown_mt2 = Types.this.erasure(thrown_mt22);
                } else {
                    Type.ForAll fa1 = (Type.ForAll) mt13;
                    Type.ForAll fa2 = (Type.ForAll) mt22;
                    thrown_mt2 = Types.this.subst(thrown_mt22, fa2.tvars, fa1.tvars);
                }
                thrown = thrown == null ? thrown_mt2 : Types.this.chk.intersect(thrown_mt2, thrown);
                typeSymbol = origin;
            }
            final List<Type> thrown1 = thrown;
            return new FunctionDescriptor(bestSoFar) { // from class: com.sun.tools.javac.code.Types.DescriptorCache.1
                @Override // com.sun.tools.javac.code.Types.DescriptorCache.FunctionDescriptor
                public Type getType(Type origin2) {
                    Type mt = Types.this.memberType(origin2, getSymbol());
                    return Types.this.createMethodTypeWithThrown(mt, thrown1);
                }
            };
        }

        boolean isSubtypeInternal(Type s, Type t) {
            if (s.isPrimitive() && t.isPrimitive()) {
                return Types.this.isSameType(t, s);
            }
            return Types.this.isSubtype(s, t);
        }

        FunctionDescriptorLookupError failure(String msg, Object... args) {
            return failure(Types.this.diags.fragment(msg, args));
        }

        FunctionDescriptorLookupError failure(JCDiagnostic diag) {
            return Types.this.functionDescriptorLookupError.setMessage(diag);
        }
    }

    public Symbol findDescriptorSymbol(Symbol.TypeSymbol origin) throws FunctionDescriptorLookupError {
        return this.descCache.get(origin).getSymbol();
    }

    public Type findDescriptorType(Type origin) throws FunctionDescriptorLookupError {
        return this.descCache.get(origin.tsym).getType(origin);
    }

    public boolean isFunctionalInterface(Symbol.TypeSymbol tsym) {
        try {
            findDescriptorSymbol(tsym);
            return true;
        } catch (FunctionDescriptorLookupError e) {
            return false;
        }
    }

    public boolean isFunctionalInterface(Type site) {
        try {
            findDescriptorType(site);
            return true;
        } catch (FunctionDescriptorLookupError e) {
            return false;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public Type removeWildcards(Type site) {
        Type bound;
        Type capturedSite = capture(site);
        if (capturedSite != site) {
            Type formalInterface = site.tsym.type;
            ListBuffer listBuffer = new ListBuffer();
            List typeArguments = site.getTypeArguments();
            List typeArguments2 = capturedSite.getTypeArguments();
            for (Type type : formalInterface.getTypeArguments()) {
                if (((Type) typeArguments.head).hasTag(TypeTag.WILDCARD)) {
                    Type.WildcardType wt = (Type.WildcardType) ((Type) typeArguments.head).unannotatedType();
                    switch (wt.kind) {
                        case EXTENDS:
                        case UNBOUND:
                            Type.CapturedType capVar = (Type.CapturedType) ((Type) typeArguments2.head).unannotatedType();
                            bound = capVar.bound.containsAny(capturedSite.getTypeArguments()) ? wt.type : capVar.bound;
                            break;
                        default:
                            bound = wt.type;
                            break;
                    }
                    listBuffer.append(bound);
                } else {
                    listBuffer.append(typeArguments.head);
                }
                typeArguments = typeArguments.tail;
                typeArguments2 = typeArguments2.tail;
            }
            return subst(formalInterface, formalInterface.getTypeArguments(), listBuffer.toList());
        }
        return site;
    }

    public Symbol.ClassSymbol makeFunctionalInterfaceClass(Env<AttrContext> env, Name name, List<Type> targets, long cflags) {
        if (targets.isEmpty()) {
            return null;
        }
        Symbol descSym = findDescriptorSymbol(targets.head.tsym);
        Type descType = findDescriptorType(targets.head);
        Symbol.ClassSymbol csym = new Symbol.ClassSymbol(cflags, name, env.enclClass.sym.outermostClass());
        csym.completer = null;
        csym.members_field = new Scope(csym);
        Symbol.MethodSymbol instDescSym = new Symbol.MethodSymbol(descSym.flags(), descSym.name, descType, csym);
        csym.members_field.enter(instDescSym);
        Type.ClassType ctype = new Type.ClassType(Type.noType, List.nil(), csym);
        ctype.supertype_field = this.syms.objectType;
        ctype.interfaces_field = targets;
        csym.type = ctype;
        csym.sourcefile = ((Symbol.ClassSymbol) csym.owner).sourcefile;
        return csym;
    }

    public List<Symbol> functionalInterfaceBridges(Symbol.TypeSymbol origin) {
        Assert.check(isFunctionalInterface(origin));
        Symbol descSym = findDescriptorSymbol(origin);
        Scope.CompoundScope members = membersClosure(origin.type, false);
        ListBuffer<Symbol> overridden = new ListBuffer<>();
        for (Symbol m2 : members.getElementsByName(descSym.name, this.bridgeFilter)) {
            if (m2 != descSym && descSym.overrides(m2, origin, this, false)) {
                Iterator<Symbol> it = overridden.iterator();
                while (true) {
                    if (it.hasNext()) {
                        Symbol m3 = it.next();
                        if (isSameType(m3.erasure(this), m2.erasure(this)) || (m3.overrides(m2, origin, this, false) && (pendingBridges((Symbol.ClassSymbol) origin, m3.enclClass()) || ((Symbol.MethodSymbol) m2).binaryImplementation((Symbol.ClassSymbol) m3.owner, this) != null))) {
                            break;
                        }
                    } else {
                        overridden.add(m2);
                        break;
                    }
                }
            }
        }
        return overridden.toList();
    }

    private boolean pendingBridges(Symbol.ClassSymbol origin, Symbol.TypeSymbol s) {
        if (origin.classfile != null && origin.classfile.getKind() == JavaFileObject.Kind.CLASS && this.enter.getEnv(origin) == null) {
            return false;
        }
        if (origin == s) {
            return true;
        }
        for (Type t : interfaces(origin.type)) {
            if (pendingBridges((Symbol.ClassSymbol) t.tsym, s)) {
                return true;
            }
        }
        return false;
    }

    class DescriptorFilter implements Filter<Symbol> {
        Symbol.TypeSymbol origin;

        DescriptorFilter(Symbol.TypeSymbol origin) {
            this.origin = origin;
        }

        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol sym) {
            return sym.kind == 16 && (sym.flags() & 8796093023232L) == 1024 && !Types.this.overridesObjectMethod(this.origin, sym) && (Types.this.interfaceCandidates(this.origin.type, (Symbol.MethodSymbol) sym).head.flags() & Flags.DEFAULT) == 0;
        }
    }

    public boolean isSubtypeUnchecked(Type t, Type s) {
        return isSubtypeUnchecked(t, s, this.noWarnings);
    }

    public boolean isSubtypeUnchecked(Type t, Type s, Warner warn) {
        boolean result = isSubtypeUncheckedInternal(t, s, warn);
        if (result) {
            checkUnsafeVarargsConversion(t, s, warn);
        }
        return result;
    }

    private boolean isSubtypeUncheckedInternal(Type t, Type s, Warner warn) {
        Type t2;
        if (t.hasTag(TypeTag.ARRAY) && s.hasTag(TypeTag.ARRAY)) {
            Type t3 = t.unannotatedType();
            Type s2 = s.unannotatedType();
            if (((Type.ArrayType) t3).elemtype.isPrimitive()) {
                return isSameType(elemtype(t3), elemtype(s2));
            }
            return isSubtypeUnchecked(elemtype(t3), elemtype(s2), warn);
        }
        if (isSubtype(t, s)) {
            return true;
        }
        if (t.hasTag(TypeTag.TYPEVAR)) {
            return isSubtypeUnchecked(t.getUpperBound(), s, warn);
        }
        if (!s.isRaw() && (t2 = asSuper(t, s.tsym)) != null && t2.isRaw()) {
            if (isReifiable(s)) {
                warn.silentWarn(Lint.LintCategory.UNCHECKED);
            } else {
                warn.warn(Lint.LintCategory.UNCHECKED);
            }
            return true;
        }
        return false;
    }

    private void checkUnsafeVarargsConversion(Type t, Type s, Warner warn) {
        if (!t.hasTag(TypeTag.ARRAY) || isReifiable(t)) {
            return;
        }
        Type t2 = t.unannotatedType();
        Type s2 = s.unannotatedType();
        Type.ArrayType from = (Type.ArrayType) t2;
        boolean shouldWarn = false;
        switch (s2.getTag()) {
            case ARRAY:
                Type.ArrayType to = (Type.ArrayType) s2;
                shouldWarn = (!from.isVarargs() || to.isVarargs() || isReifiable(from)) ? false : true;
                break;
            case CLASS:
                shouldWarn = from.isVarargs();
                break;
        }
        if (shouldWarn) {
            warn.warn(Lint.LintCategory.VARARGS);
        }
    }

    public final boolean isSubtype(Type t, Type s) {
        return isSubtype(t, s, true);
    }

    public final boolean isSubtypeNoCapture(Type t, Type s) {
        return isSubtype(t, s, false);
    }

    public boolean isSubtype(Type t, Type s, boolean capture) {
        Type t2;
        Type s2;
        Type lower;
        if (t == s || (t2 = t.unannotatedType()) == (s2 = s.unannotatedType())) {
            return true;
        }
        if (s2.isPartial()) {
            return isSuperType(s2, t2);
        }
        if (s2.isCompound()) {
            for (Type s22 : interfaces(s2).prepend(supertype(s2))) {
                if (!isSubtype(t2, s22, capture)) {
                    return false;
                }
            }
            return true;
        }
        if (t2.hasTag(TypeTag.UNDETVAR) || t2.isCompound() || s2 == (lower = cvarLowerBound(wildLowerBound(s2)))) {
            return this.isSubtype.visit(capture ? capture(t2) : t2, s2).booleanValue();
        }
        return isSubtype(capture ? capture(t2) : t2, lower, false);
    }

    /* JADX WARN: Multi-variable type inference failed */
    public boolean isSubtypeUnchecked(Type t, List<Type> ts, Warner warn) {
        for (List list = ts; list.nonEmpty(); list = list.tail) {
            if (!isSubtypeUnchecked(t, (Type) list.head, warn)) {
                return false;
            }
        }
        return true;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public boolean isSubtypes(List<Type> list, List<Type> list2) {
        List list3 = list;
        List list4 = list2;
        while (list3.tail != null && list4.tail != null && isSubtype((Type) list3.head, (Type) list4.head)) {
            list3 = list3.tail;
            list4 = list4.tail;
        }
        return list3.tail == null && list4.tail == null;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public boolean isSubtypesUnchecked(List<Type> list, List<Type> list2, Warner warner) {
        List list3 = list;
        List list4 = list2;
        while (list3.tail != null && list4.tail != null && isSubtypeUnchecked((Type) list3.head, (Type) list4.head, warner)) {
            list3 = list3.tail;
            list4 = list4.tail;
        }
        return list3.tail == null && list4.tail == null;
    }

    public boolean isSuperType(Type t, Type s) {
        switch (t.getTag()) {
            case ERROR:
                return true;
            case UNDETVAR:
                Type.UndetVar undet = (Type.UndetVar) t;
                if (t == s || undet.qtype == s || s.hasTag(TypeTag.ERROR) || s.hasTag(TypeTag.BOT)) {
                    return true;
                }
                undet.addBound(Type.UndetVar.InferenceBound.LOWER, s, this);
                return true;
            default:
                return isSubtype(s, t);
        }
    }

    public boolean isSameTypes(List<Type> ts, List<Type> ss) {
        return isSameTypes(ts, ss, false);
    }

    /* JADX WARN: Multi-variable type inference failed */
    public boolean isSameTypes(List<Type> list, List<Type> list2, boolean z) {
        List list3 = list;
        List list4 = list2;
        while (list3.tail != null && list4.tail != null && isSameType((Type) list3.head, (Type) list4.head, z)) {
            list3 = list3.tail;
            list4 = list4.tail;
        }
        return list3.tail == null && list4.tail == null;
    }

    public boolean isSignaturePolymorphic(Symbol.MethodSymbol msym) {
        List<Type> argtypes = msym.type.mo176getParameterTypes();
        return (msym.flags_field & 256) != 0 && msym.owner == this.syms.methodHandleType.tsym && argtypes.length() == 1 && argtypes.head.hasTag(TypeTag.ARRAY) && msym.type.mo178getReturnType().tsym == this.syms.objectType.tsym && ((Type.ArrayType) argtypes.head).elemtype.tsym == this.syms.objectType.tsym;
    }

    public boolean isSameType(Type t, Type s) {
        return isSameType(t, s, false);
    }

    public boolean isSameType(Type t, Type s, boolean strict) {
        Boolean boolVisit;
        if (strict) {
            boolVisit = this.isSameTypeStrict.visit(t, s);
        } else {
            boolVisit = this.isSameTypeLoose.visit(t, s);
        }
        return boolVisit.booleanValue();
    }

    public boolean isSameAnnotatedType(Type t, Type s) {
        return this.isSameAnnotatedType.visit(t, s).booleanValue();
    }

    abstract class SameTypeVisitor extends TypeRelation {
        protected abstract boolean containsTypes(List<Type> list, List<Type> list2);

        abstract boolean sameTypeVars(Type.TypeVar typeVar, Type.TypeVar typeVar2);

        SameTypeVisitor() {
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Type s) {
            if (t == s) {
                return true;
            }
            if (s.isPartial()) {
                return visit(s, t);
            }
            switch (t.getTag()) {
                case BYTE:
                case CHAR:
                case SHORT:
                case INT:
                case LONG:
                case FLOAT:
                case DOUBLE:
                case BOOLEAN:
                case VOID:
                case BOT:
                case NONE:
                    return Boolean.valueOf(t.hasTag(s.getTag()));
                case TYPEVAR:
                    if (s.hasTag(TypeTag.TYPEVAR)) {
                        return Boolean.valueOf(sameTypeVars((Type.TypeVar) t.unannotatedType(), (Type.TypeVar) s.unannotatedType()));
                    }
                    return Boolean.valueOf(s.isSuperBound() && !s.isExtendsBound() && visit(t, Types.this.wildUpperBound(s)).booleanValue());
                case WILDCARD:
                default:
                    throw new AssertionError("isSameType " + t.getTag());
            }
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitWildcardType(Type.WildcardType t, Type s) {
            if (s.isPartial()) {
                return visit(s, t);
            }
            return false;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitClassType(Type.ClassType t, Type s) {
            if (t == s) {
                return true;
            }
            if (s.isPartial()) {
                return visit(s, t);
            }
            if (s.isSuperBound() && !s.isExtendsBound()) {
                return Boolean.valueOf(visit(t, Types.this.wildUpperBound(s)).booleanValue() && visit(t, Types.this.wildLowerBound(s)).booleanValue());
            }
            if (t.isCompound() && s.isCompound()) {
                if (!visit(Types.this.supertype(t), Types.this.supertype(s)).booleanValue()) {
                    return false;
                }
                HashSet<UniqueType> set = new HashSet<>();
                for (Type x : Types.this.interfaces(t)) {
                    set.add(new UniqueType(x.unannotatedType(), Types.this));
                }
                for (Type x2 : Types.this.interfaces(s)) {
                    if (!set.remove(new UniqueType(x2.unannotatedType(), Types.this))) {
                        return false;
                    }
                }
                return Boolean.valueOf(set.isEmpty());
            }
            return Boolean.valueOf(t.tsym == s.tsym && visit(t.getEnclosingType(), s.getEnclosingType()).booleanValue() && containsTypes(t.getTypeArguments(), s.getTypeArguments()));
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitArrayType(Type.ArrayType t, Type s) {
            if (t == s) {
                return true;
            }
            if (s.isPartial()) {
                return visit(s, t);
            }
            return Boolean.valueOf(s.hasTag(TypeTag.ARRAY) && Types.this.containsTypeEquivalent(t.elemtype, Types.this.elemtype(s)));
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitMethodType(Type.MethodType t, Type s) {
            return Boolean.valueOf(Types.this.hasSameArgs(t, s) && visit(t.mo178getReturnType(), s.mo178getReturnType()).booleanValue());
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitPackageType(Type.PackageType t, Type s) {
            return Boolean.valueOf(t == s);
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitForAll(Type.ForAll t, Type s) {
            boolean z = false;
            if (!s.hasTag(TypeTag.FORALL)) {
                return false;
            }
            Type.ForAll forAll = (Type.ForAll) s;
            if (Types.this.hasSameBounds(t, forAll) && visit(t.qtype, Types.this.subst(forAll.qtype, forAll.tvars, t.tvars)).booleanValue()) {
                z = true;
            }
            return Boolean.valueOf(z);
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitUndetVar(Type.UndetVar t, Type s) {
            if (s.hasTag(TypeTag.WILDCARD)) {
                return false;
            }
            if (t == s || t.qtype == s || s.hasTag(TypeTag.ERROR) || s.hasTag(TypeTag.UNKNOWN)) {
                return true;
            }
            t.addBound(Type.UndetVar.InferenceBound.EQ, s, Types.this);
            return true;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitErrorType(Type.ErrorType t, Type s) {
            return true;
        }
    }

    private class LooseSameTypeVisitor extends SameTypeVisitor {
        private Set<TypePair> cache;

        private LooseSameTypeVisitor() {
            super();
            this.cache = new HashSet();
        }

        @Override // com.sun.tools.javac.code.Types.SameTypeVisitor
        boolean sameTypeVars(Type.TypeVar tv1, Type.TypeVar tv2) {
            return tv1.tsym == tv2.tsym && checkSameBounds(tv1, tv2);
        }

        @Override // com.sun.tools.javac.code.Types.SameTypeVisitor
        protected boolean containsTypes(List<Type> ts1, List<Type> ts2) {
            return Types.this.containsTypeEquivalent(ts1, ts2);
        }

        private boolean checkSameBounds(Type.TypeVar tv1, Type.TypeVar tv2) {
            TypePair p = Types.this.new TypePair(tv1, tv2, true);
            if (this.cache.add(p)) {
                try {
                    return visit(tv1.getUpperBound(), tv2.getUpperBound()).booleanValue();
                } finally {
                    this.cache.remove(p);
                }
            }
            return false;
        }
    }

    /* JADX WARN: Can't fix incorrect switch cases order, some code will duplicate */
    public boolean containedBy(Type t, Type s) {
        switch (t.getTag()) {
            case ERROR:
                return true;
            case UNDETVAR:
                if (s.hasTag(TypeTag.WILDCARD)) {
                    Type.UndetVar undetvar = (Type.UndetVar) t;
                    switch (wt.kind) {
                        case EXTENDS:
                            Type bound = wildUpperBound(s);
                            undetvar.addBound(Type.UndetVar.InferenceBound.UPPER, bound, this);
                            return true;
                        case UNBOUND:
                        default:
                            return true;
                        case SUPER:
                            Type bound2 = wildLowerBound(s);
                            undetvar.addBound(Type.UndetVar.InferenceBound.LOWER, bound2, this);
                            return true;
                    }
                }
                return isSameType(t, s);
            default:
                return containsType(s, t);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    boolean containsType(List<Type> list, List<Type> list2) {
        List list3 = list;
        List list4 = list2;
        while (list3.nonEmpty() && list4.nonEmpty() && containsType((Type) list3.head, (Type) list4.head)) {
            list3 = list3.tail;
            list4 = list4.tail;
        }
        return list3.isEmpty() && list4.isEmpty();
    }

    public boolean containsType(Type t, Type s) {
        return this.containsType.visit(t, s).booleanValue();
    }

    public boolean isCaptureOf(Type s, Type.WildcardType t) {
        if (!s.hasTag(TypeTag.TYPEVAR) || !((Type.TypeVar) s.unannotatedType()).isCaptured()) {
            return false;
        }
        return isSameWildcard(t, ((Type.CapturedType) s.unannotatedType()).wildcard);
    }

    public boolean isSameWildcard(Type.WildcardType t, Type s) {
        if (!s.hasTag(TypeTag.WILDCARD)) {
            return false;
        }
        Type.WildcardType w = (Type.WildcardType) s.unannotatedType();
        return w.kind == t.kind && w.type == t.type;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public boolean containsTypeEquivalent(List<Type> list, List<Type> list2) {
        List list3 = list;
        List list4 = list2;
        while (list3.nonEmpty() && list4.nonEmpty() && containsTypeEquivalent((Type) list3.head, (Type) list4.head)) {
            list3 = list3.tail;
            list4 = list4.tail;
        }
        return list3.isEmpty() && list4.isEmpty();
    }

    public boolean isEqualityComparable(Type s, Type t, Warner warn) {
        if (t.isNumeric() && s.isNumeric()) {
            return true;
        }
        boolean tPrimitive = t.isPrimitive();
        boolean sPrimitive = s.isPrimitive();
        if (tPrimitive || sPrimitive) {
            return false;
        }
        return isCastable(s, t, warn) || isCastable(t, s, warn);
    }

    public boolean isCastable(Type t, Type s) {
        return isCastable(t, s, this.noWarnings);
    }

    public boolean isCastable(Type t, Type s, Warner warn) {
        if (t == s) {
            return true;
        }
        if (t.isPrimitive() != s.isPrimitive()) {
            if (this.allowBoxing) {
                if (isConvertible(t, s, warn)) {
                    return true;
                }
                if (this.allowObjectToPrimitiveCast && s.isPrimitive() && isSubtype(boxedClass(s).type, t)) {
                    return true;
                }
            }
            return false;
        }
        if (warn != this.warnStack.head) {
            try {
                this.warnStack = this.warnStack.prepend(warn);
                checkUnsafeVarargsConversion(t, s, warn);
                return this.isCastable.visit(t, s).booleanValue();
            } finally {
                this.warnStack = this.warnStack.tail;
            }
        }
        return this.isCastable.visit(t, s).booleanValue();
    }

    /* JADX WARN: Multi-variable type inference failed */
    public boolean disjointTypes(List<Type> list, List<Type> list2) {
        List list3 = list;
        for (List list4 = list2; list3.tail != null && list4.tail != null; list4 = list4.tail) {
            if (disjointType((Type) list3.head, (Type) list4.head)) {
                return true;
            }
            list3 = list3.tail;
        }
        return false;
    }

    public boolean disjointType(Type t, Type s) {
        return this.disjointType.visit(t, s).booleanValue();
    }

    public List<Type> cvarLowerBounds(List<Type> ts) {
        return Type.map(ts, this.cvarLowerBoundMapping);
    }

    public boolean notSoftSubtype(Type t, Type s) {
        if (t == s) {
            return false;
        }
        if (t.hasTag(TypeTag.TYPEVAR)) {
            Type.TypeVar tv = (Type.TypeVar) t;
            return !isCastable(tv.bound, relaxBound(s), this.noWarnings);
        }
        if (!s.hasTag(TypeTag.WILDCARD)) {
            s = cvarUpperBound(s);
        }
        return !isSubtype(t, relaxBound(s));
    }

    private Type relaxBound(Type t) {
        if (t.hasTag(TypeTag.TYPEVAR)) {
            while (t.hasTag(TypeTag.TYPEVAR)) {
                t = t.getUpperBound();
            }
            return rewriteQuantifiers(t, true, true);
        }
        return t;
    }

    public boolean isReifiable(Type t) {
        return this.isReifiable.visit(t).booleanValue();
    }

    public boolean isArray(Type t) {
        while (t.hasTag(TypeTag.WILDCARD)) {
            t = wildUpperBound(t);
        }
        return t.hasTag(TypeTag.ARRAY);
    }

    public Type elemtype(Type t) {
        switch (t.getTag()) {
            case ARRAY:
                return ((Type.ArrayType) t.unannotatedType()).elemtype;
            case WILDCARD:
                return elemtype(wildUpperBound(t));
            case ERROR:
                return t;
            case FORALL:
                return elemtype(((Type.ForAll) t).qtype);
            default:
                return null;
        }
    }

    public Type elemtypeOrType(Type t) {
        Type elemtype = elemtype(t);
        return elemtype != null ? elemtype : t;
    }

    public int dimensions(Type t) {
        int result = 0;
        while (t.hasTag(TypeTag.ARRAY)) {
            result++;
            t = elemtype(t);
        }
        return result;
    }

    public Type.ArrayType makeArrayType(Type t) {
        if (t.hasTag(TypeTag.VOID) || t.hasTag(TypeTag.PACKAGE)) {
            Assert.error("Type t must not be a VOID or PACKAGE type, " + t.toString());
        }
        return new Type.ArrayType(t, this.syms.arrayClass);
    }

    public Type asSuper(Type t, Symbol sym) {
        if (sym.type == this.syms.objectType) {
            return this.syms.objectType;
        }
        return this.asSuper.visit(t, sym);
    }

    public Type asOuterSuper(Type t, Symbol sym) {
        switch (t.getTag()) {
            case ARRAY:
                if (isSubtype(t, sym.type)) {
                    return sym.type;
                }
                return null;
            case CLASS:
                break;
            case TYPEVAR:
                return asSuper(t, sym);
            case ERROR:
                return t;
            default:
                return null;
        }
        do {
            Type s = asSuper(t, sym);
            if (s != null) {
                return s;
            }
            t = t.getEnclosingType();
        } while (t.hasTag(TypeTag.CLASS));
        return null;
    }

    public Type asEnclosingSuper(Type t, Symbol sym) {
        Type type;
        switch (t.getTag()) {
            case ARRAY:
                if (isSubtype(t, sym.type)) {
                    return sym.type;
                }
                return null;
            case CLASS:
                break;
            case TYPEVAR:
                return asSuper(t, sym);
            case ERROR:
                return t;
            default:
                return null;
        }
        do {
            Type s = asSuper(t, sym);
            if (s != null) {
                return s;
            }
            Type outer = t.getEnclosingType();
            if (outer.hasTag(TypeTag.CLASS)) {
                type = outer;
            } else {
                type = t.tsym.owner.enclClass() != null ? t.tsym.owner.enclClass().type : Type.noType;
            }
            t = type;
        } while (t.hasTag(TypeTag.CLASS));
        return null;
    }

    public Type memberType(Type t, Symbol sym) {
        return (sym.flags() & 8) != 0 ? sym.type : this.memberType.visit(t, sym);
    }

    public boolean isAssignable(Type t, Type s) {
        return isAssignable(t, s, this.noWarnings);
    }

    /*  JADX ERROR: UnsupportedOperationException in pass: RegionMakerVisitor
        java.lang.UnsupportedOperationException
        	at java.base/java.util.Collections$UnmodifiableCollection.add(Collections.java:1091)
        	at jadx.core.dex.visitors.regions.maker.SwitchRegionMaker$1.leaveRegion(SwitchRegionMaker.java:390)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseInternal(DepthRegionTraversal.java:70)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.lambda$traverseInternal$0(DepthRegionTraversal.java:68)
        	at java.base/java.util.ArrayList.forEach(ArrayList.java:1596)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseInternal(DepthRegionTraversal.java:68)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverse(DepthRegionTraversal.java:23)
        	at jadx.core.dex.visitors.regions.maker.SwitchRegionMaker.insertBreaksForCase(SwitchRegionMaker.java:370)
        	at jadx.core.dex.visitors.regions.maker.SwitchRegionMaker.insertBreaks(SwitchRegionMaker.java:85)
        	at jadx.core.dex.visitors.regions.PostProcessRegions.leaveRegion(PostProcessRegions.java:33)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseInternal(DepthRegionTraversal.java:70)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.lambda$traverseInternal$0(DepthRegionTraversal.java:68)
        	at java.base/java.util.ArrayList.forEach(ArrayList.java:1596)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseInternal(DepthRegionTraversal.java:68)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.lambda$traverseInternal$0(DepthRegionTraversal.java:68)
        	at java.base/java.util.ArrayList.forEach(ArrayList.java:1596)
        	at java.base/java.util.Collections$UnmodifiableCollection.forEach(Collections.java:1116)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseInternal(DepthRegionTraversal.java:68)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.lambda$traverseInternal$0(DepthRegionTraversal.java:68)
        	at java.base/java.util.ArrayList.forEach(ArrayList.java:1596)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseInternal(DepthRegionTraversal.java:68)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.lambda$traverseInternal$0(DepthRegionTraversal.java:68)
        	at java.base/java.util.ArrayList.forEach(ArrayList.java:1596)
        	at java.base/java.util.Collections$UnmodifiableCollection.forEach(Collections.java:1116)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseInternal(DepthRegionTraversal.java:68)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.lambda$traverseInternal$0(DepthRegionTraversal.java:68)
        	at java.base/java.util.ArrayList.forEach(ArrayList.java:1596)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverseInternal(DepthRegionTraversal.java:68)
        	at jadx.core.dex.visitors.regions.DepthRegionTraversal.traverse(DepthRegionTraversal.java:19)
        	at jadx.core.dex.visitors.regions.PostProcessRegions.process(PostProcessRegions.java:23)
        	at jadx.core.dex.visitors.regions.RegionMakerVisitor.visit(RegionMakerVisitor.java:31)
        */
    public boolean isAssignable(com.sun.tools.javac.code.Type r5, com.sun.tools.javac.code.Type r6, com.sun.tools.javac.util.Warner r7) {
        /*
            r4 = this;
            com.sun.tools.javac.code.TypeTag r0 = com.sun.tools.javac.code.TypeTag.ERROR
            boolean r0 = r5.hasTag(r0)
            r1 = 1
            if (r0 == 0) goto La
            return r1
        La:
            com.sun.tools.javac.code.TypeTag r0 = r5.getTag()
            com.sun.tools.javac.code.TypeTag r2 = com.sun.tools.javac.code.TypeTag.INT
            boolean r0 = r0.isSubRangeOf(r2)
            if (r0 == 0) goto L6e
            java.lang.Object r0 = r5.constValue()
            if (r0 == 0) goto L6e
            java.lang.Object r0 = r5.constValue()
            java.lang.Number r0 = (java.lang.Number) r0
            int r0 = r0.intValue()
            int[] r2 = com.sun.tools.javac.code.Types.AnonymousClass27.$SwitchMap$com$sun$tools$javac$code$TypeTag
            com.sun.tools.javac.code.TypeTag r3 = r6.getTag()
            int r3 = r3.ordinal()
            r2 = r2[r3]
            switch(r2) {
                case 2: goto L51;
                case 3: goto L48;
                case 4: goto L40;
                case 5: goto L37;
                case 6: goto L36;
                default: goto L35;
            }
        L35:
            goto L6e
        L36:
            return r1
        L37:
            r2 = -32768(0xffffffffffff8000, float:NaN)
            if (r2 > r0) goto L6e
            r2 = 32767(0x7fff, float:4.5916E-41)
            if (r0 > r2) goto L6e
            return r1
        L40:
            if (r0 < 0) goto L6e
            r2 = 65535(0xffff, float:9.1834E-41)
            if (r0 > r2) goto L6e
            return r1
        L48:
            r2 = -128(0xffffffffffffff80, float:NaN)
            if (r2 > r0) goto L6e
            r2 = 127(0x7f, float:1.78E-43)
            if (r0 > r2) goto L6e
            return r1
        L51:
            int[] r1 = com.sun.tools.javac.code.Types.AnonymousClass27.$SwitchMap$com$sun$tools$javac$code$TypeTag
            com.sun.tools.javac.code.Type r2 = r4.unboxedType(r6)
            com.sun.tools.javac.code.TypeTag r2 = r2.getTag()
            int r2 = r2.ordinal()
            r1 = r1[r2]
            switch(r1) {
                case 3: goto L65;
                case 4: goto L65;
                case 5: goto L65;
                default: goto L64;
            }
        L64:
            goto L6e
        L65:
            com.sun.tools.javac.code.Type r1 = r4.unboxedType(r6)
            boolean r1 = r4.isAssignable(r5, r1, r7)
            return r1
        L6e:
            boolean r0 = r4.isConvertible(r5, r6, r7)
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.code.Types.isAssignable(com.sun.tools.javac.code.Type, com.sun.tools.javac.code.Type, com.sun.tools.javac.util.Warner):boolean");
    }

    public Type erasure(Type t) {
        return eraseNotNeeded(t) ? t : erasure(t, false);
    }

    private boolean eraseNotNeeded(Type t) {
        return t.isPrimitive() || this.syms.stringType.tsym == t.tsym;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Type erasure(Type t, boolean recurse) {
        if (t.isPrimitive()) {
            return t;
        }
        return this.erasure.visit(t, Boolean.valueOf(recurse));
    }

    public List<Type> erasure(List<Type> ts) {
        return Type.map(ts, this.erasureFun);
    }

    public Type erasureRecursive(Type t) {
        return erasure(t, true);
    }

    public List<Type> erasureRecursive(List<Type> ts) {
        return Type.map(ts, this.erasureRecFun);
    }

    public Type.IntersectionClassType makeIntersectionType(List<Type> bounds) {
        return makeIntersectionType(bounds, bounds.head.tsym.isInterface());
    }

    public Type.IntersectionClassType makeIntersectionType(List<Type> bounds, boolean allInterfaces) {
        Assert.check(bounds.nonEmpty());
        Type firstExplicitBound = bounds.head;
        if (allInterfaces) {
            bounds = bounds.prepend(this.syms.objectType);
        }
        Symbol.ClassSymbol bc = new Symbol.ClassSymbol(1090524161L, Type.moreInfo ? this.names.fromString(bounds.toString()) : this.names.empty, null, this.syms.noSymbol);
        Type.IntersectionClassType intersectionType = new Type.IntersectionClassType(bounds, bc, allInterfaces);
        bc.type = intersectionType;
        bc.erasure_field = bounds.head.hasTag(TypeTag.TYPEVAR) ? this.syms.objectType : erasure(firstExplicitBound);
        bc.members_field = new Scope(bc);
        return intersectionType;
    }

    public Type makeIntersectionType(Type bound1, Type bound2) {
        return makeIntersectionType(List.of(bound1, bound2));
    }

    public Type supertype(Type t) {
        return this.supertype.visit(t);
    }

    public List<Type> interfaces(Type t) {
        return this.interfaces.visit(t);
    }

    public List<Type> directSupertypes(Type t) {
        return this.directSupertypes.visit(t);
    }

    public boolean isDirectSuperInterface(Symbol.TypeSymbol isym, Symbol.TypeSymbol origin) {
        for (Type i2 : interfaces(origin.type)) {
            if (isym == i2.tsym) {
                return true;
            }
        }
        return false;
    }

    public boolean isDerivedRaw(Type t) {
        Boolean result = this.isDerivedRawCache.get(t);
        if (result == null) {
            result = Boolean.valueOf(isDerivedRawInternal(t));
            this.isDerivedRawCache.put(t, result);
        }
        return result.booleanValue();
    }

    public boolean isDerivedRawInternal(Type t) {
        if (t.isErroneous()) {
            return false;
        }
        return t.isRaw() || (supertype(t) != Type.noType && isDerivedRaw(supertype(t))) || isDerivedRaw(interfaces(t));
    }

    /* JADX WARN: Multi-variable type inference failed */
    public boolean isDerivedRaw(List<Type> ts) {
        List list = ts;
        while (list.nonEmpty() && !isDerivedRaw((Type) list.head)) {
            list = list.tail;
        }
        return list.nonEmpty();
    }

    public void setBounds(Type.TypeVar t, List<Type> bounds) {
        setBounds(t, bounds, bounds.head.tsym.isInterface());
    }

    public void setBounds(Type.TypeVar t, List<Type> bounds, boolean allInterfaces) {
        t.bound = bounds.tail.isEmpty() ? bounds.head : makeIntersectionType(bounds, allInterfaces);
        t.rank_field = -1;
    }

    public List<Type> getBounds(Type.TypeVar t) {
        if (t.bound.hasTag(TypeTag.NONE)) {
            return List.nil();
        }
        if (t.bound.isErroneous() || !t.bound.isCompound()) {
            return List.of(t.bound);
        }
        if ((erasure(t).tsym.flags() & 512) == 0) {
            return interfaces(t).prepend(supertype(t));
        }
        return interfaces(t);
    }

    public Type classBound(Type t) {
        return this.classBound.visit(t);
    }

    public boolean isSubSignature(Type t, Type s) {
        return isSubSignature(t, s, true);
    }

    public boolean isSubSignature(Type t, Type s, boolean strict) {
        return hasSameArgs(t, s, strict) || hasSameArgs(t, erasure(s), strict);
    }

    public boolean overrideEquivalent(Type t, Type s) {
        return hasSameArgs(t, s) || hasSameArgs(t, erasure(s)) || hasSameArgs(erasure(t), s);
    }

    public boolean overridesObjectMethod(Symbol.TypeSymbol origin, Symbol msym) {
        for (Scope.Entry e = this.syms.objectType.tsym.members().lookup(msym.name); e.scope != null; e = e.next()) {
            if (msym.overrides(e.sym, origin, this, true)) {
                return true;
            }
        }
        return false;
    }

    class ImplementationCache {
        private WeakHashMap<Symbol.MethodSymbol, SoftReference<Map<Symbol.TypeSymbol, Entry>>> _map = new WeakHashMap<>();

        ImplementationCache() {
        }

        class Entry {
            final Symbol.MethodSymbol cachedImpl;
            final boolean checkResult;
            final Filter<Symbol> implFilter;
            final int prevMark;

            public Entry(Symbol.MethodSymbol cachedImpl, Filter<Symbol> scopeFilter, boolean checkResult, int prevMark) {
                this.cachedImpl = cachedImpl;
                this.implFilter = scopeFilter;
                this.checkResult = checkResult;
                this.prevMark = prevMark;
            }

            boolean matches(Filter<Symbol> scopeFilter, boolean checkResult, int mark) {
                return this.implFilter == scopeFilter && this.checkResult == checkResult && this.prevMark == mark;
            }
        }

        Symbol.MethodSymbol get(Symbol.MethodSymbol ms, Symbol.TypeSymbol origin, boolean checkResult, Filter<Symbol> implFilter) {
            Map<Symbol.TypeSymbol, Entry> cache;
            SoftReference<Map<Symbol.TypeSymbol, Entry>> ref_cache = this._map.get(ms);
            Map<Symbol.TypeSymbol, Entry> cache2 = ref_cache != null ? ref_cache.get() : null;
            if (cache2 != null) {
                cache = cache2;
            } else {
                Map<Symbol.TypeSymbol, Entry> cache3 = new HashMap<>();
                this._map.put(ms, new SoftReference<>(cache3));
                cache = cache3;
            }
            Entry e = cache.get(origin);
            Scope.CompoundScope members = Types.this.membersClosure(origin.type, true);
            if (e != null && e.matches(implFilter, checkResult, members.getMark())) {
                return e.cachedImpl;
            }
            Symbol.MethodSymbol impl = implementationInternal(ms, origin, checkResult, implFilter);
            cache.put(origin, new Entry(impl, implFilter, checkResult, members.getMark()));
            return impl;
        }

        private Symbol.MethodSymbol implementationInternal(Symbol.MethodSymbol ms, Symbol.TypeSymbol origin, boolean checkResult, Filter<Symbol> implFilter) {
            Type t = origin.type;
            while (true) {
                if (t.hasTag(TypeTag.CLASS) || t.hasTag(TypeTag.TYPEVAR)) {
                    while (t.hasTag(TypeTag.TYPEVAR)) {
                        t = t.getUpperBound();
                    }
                    Symbol.TypeSymbol c = t.tsym;
                    Scope.Entry e = c.members().lookup(ms.name, implFilter);
                    while (e.scope != null) {
                        if (e.sym == null || !e.sym.overrides(ms, origin, Types.this, checkResult)) {
                            e = e.next(implFilter);
                        } else {
                            return (Symbol.MethodSymbol) e.sym;
                        }
                    }
                    t = Types.this.supertype(t);
                } else {
                    return null;
                }
            }
        }
    }

    public Symbol.MethodSymbol implementation(Symbol.MethodSymbol ms, Symbol.TypeSymbol origin, boolean checkResult, Filter<Symbol> implFilter) {
        return this.implCache.get(ms, origin, checkResult, implFilter);
    }

    class MembersClosureCache extends SimpleVisitor<Scope.CompoundScope, Void> {
        Scope.CompoundScope nilScope;
        private Map<Symbol.TypeSymbol, Scope.CompoundScope> _map = new HashMap();
        Set<Symbol.TypeSymbol> seenTypes = new HashSet();

        MembersClosureCache() {
        }

        class MembersScope extends Scope.CompoundScope {
            Scope.CompoundScope scope;

            public MembersScope(Scope.CompoundScope scope) {
                super(scope.owner);
                this.scope = scope;
            }

            Filter<Symbol> combine(final Filter<Symbol> sf) {
                return new Filter<Symbol>() { // from class: com.sun.tools.javac.code.Types.MembersClosureCache.MembersScope.1
                    @Override // com.sun.tools.javac.util.Filter
                    public boolean accepts(Symbol s) {
                        return !s.owner.isInterface() && (sf == null || sf.accepts(s));
                    }
                };
            }

            @Override // com.sun.tools.javac.code.Scope.CompoundScope, com.sun.tools.javac.code.Scope
            public Iterable<Symbol> getElements(Filter<Symbol> sf) {
                return this.scope.getElements(combine(sf));
            }

            @Override // com.sun.tools.javac.code.Scope.CompoundScope, com.sun.tools.javac.code.Scope
            public Iterable<Symbol> getElementsByName(Name name, Filter<Symbol> sf) {
                return this.scope.getElementsByName(name, combine(sf));
            }

            @Override // com.sun.tools.javac.code.Scope.CompoundScope
            public int getMark() {
                return this.scope.getMark();
            }
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Scope.CompoundScope visitType(Type t, Void _unused) {
            if (this.nilScope == null) {
                this.nilScope = new Scope.CompoundScope(Types.this.syms.noSymbol);
            }
            return this.nilScope;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Scope.CompoundScope visitClassType(Type.ClassType t, Void _unused) {
            if (!this.seenTypes.add(t.tsym)) {
                return new Scope.CompoundScope(t.tsym);
            }
            try {
                this.seenTypes.add(t.tsym);
                Symbol.ClassSymbol csym = (Symbol.ClassSymbol) t.tsym;
                Scope.CompoundScope membersClosure = this._map.get(csym);
                if (membersClosure == null) {
                    membersClosure = new Scope.CompoundScope(csym);
                    for (Type i : Types.this.interfaces(t)) {
                        membersClosure.addSubScope(visit(i, null));
                    }
                    membersClosure.addSubScope(visit(Types.this.supertype(t), null));
                    membersClosure.addSubScope(csym.members());
                    this._map.put(csym, membersClosure);
                }
                return membersClosure;
            } finally {
                this.seenTypes.remove(t.tsym);
            }
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Scope.CompoundScope visitTypeVar(Type.TypeVar t, Void _unused) {
            return visit(t.getUpperBound(), null);
        }
    }

    public Scope.CompoundScope membersClosure(Type site, boolean skipInterface) {
        Scope.CompoundScope cs = this.membersCache.visit(site, null);
        if (cs == null) {
            Assert.error("type " + site);
        }
        if (!skipInterface) {
            return cs;
        }
        MembersClosureCache membersClosureCache = this.membersCache;
        membersClosureCache.getClass();
        return membersClosureCache.new MembersScope(cs);
    }

    public Symbol.MethodSymbol firstUnimplementedAbstract(Symbol.ClassSymbol sym) {
        try {
            return firstUnimplementedAbstractImpl(sym, sym);
        } catch (Symbol.CompletionFailure ex) {
            this.chk.completionError(this.enter.getEnv(sym).tree.pos(), ex);
            return null;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    private Symbol.MethodSymbol firstUnimplementedAbstractImpl(Symbol.ClassSymbol impl, Symbol.ClassSymbol c) {
        Symbol.MethodSymbol prov;
        Symbol.MethodSymbol undef = null;
        if (c == impl || (c.flags() & 1536) != 0) {
            Scope s = c.members();
            for (Scope.Entry e = s.elems; undef == null && e != null; e = e.sibling) {
                if (e.sym.kind == 16 && (e.sym.flags() & 8796095120384L) == 1024) {
                    Symbol.MethodSymbol absmeth = (Symbol.MethodSymbol) e.sym;
                    Symbol.MethodSymbol implmeth = absmeth.implementation(impl, this, true);
                    if ((implmeth == null || implmeth == absmeth) && this.allowDefaultMethods && (prov = interfaceCandidates(impl.type, absmeth).head) != null && prov.overrides(absmeth, impl, this, true)) {
                        implmeth = prov;
                    }
                    if (implmeth == null || implmeth == absmeth) {
                        undef = absmeth;
                    }
                }
            }
            if (undef == null) {
                Type st = supertype(c.type);
                if (st.hasTag(TypeTag.CLASS)) {
                    undef = firstUnimplementedAbstractImpl(impl, (Symbol.ClassSymbol) st.tsym);
                }
            }
            for (List listInterfaces = interfaces(c.type); undef == null && listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
                undef = firstUnimplementedAbstractImpl(impl, (Symbol.ClassSymbol) ((Type) listInterfaces.head).tsym);
            }
        }
        return undef;
    }

    public List<Symbol.MethodSymbol> interfaceCandidates(Type site, Symbol.MethodSymbol ms) {
        Filter<Symbol> filter = new MethodFilter(ms, site);
        List<Symbol.MethodSymbol> candidates = List.nil();
        for (Symbol s : membersClosure(site, false).getElements(filter)) {
            if (!site.tsym.isInterface() && !s.owner.isInterface()) {
                return List.of((Symbol.MethodSymbol) s);
            }
            if (!candidates.contains(s)) {
                candidates = candidates.prepend((Symbol.MethodSymbol) s);
            }
        }
        return prune(candidates);
    }

    public List<Symbol.MethodSymbol> prune(List<Symbol.MethodSymbol> methods) {
        ListBuffer<Symbol.MethodSymbol> methodsMin = new ListBuffer<>();
        for (Symbol.MethodSymbol m1 : methods) {
            boolean isMin_m1 = true;
            Iterator<Symbol.MethodSymbol> it = methods.iterator();
            while (true) {
                if (!it.hasNext()) {
                    break;
                }
                Symbol.MethodSymbol m2 = it.next();
                if (m1 != m2 && m2.owner != m1.owner && asSuper(m2.owner.type, m1.owner) != null) {
                    isMin_m1 = false;
                    break;
                }
            }
            if (isMin_m1) {
                methodsMin.append(m1);
            }
        }
        return methodsMin.toList();
    }

    private class MethodFilter implements Filter<Symbol> {
        Symbol msym;
        Type site;

        MethodFilter(Symbol msym, Type site) {
            this.msym = msym;
            this.site = site;
        }

        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol s) {
            return s.kind == 16 && s.name == this.msym.name && (s.flags() & 4096) == 0 && s.isInheritedIn(this.site.tsym, Types.this) && Types.this.overrideEquivalent(Types.this.memberType(this.site, s), Types.this.memberType(this.site, this.msym));
        }
    }

    public boolean hasSameArgs(Type t, Type s) {
        return hasSameArgs(t, s, true);
    }

    public boolean hasSameArgs(Type t, Type s, boolean strict) {
        return hasSameArgs(t, s, strict ? this.hasSameArgs_strict : this.hasSameArgs_nonstrict);
    }

    private boolean hasSameArgs(Type t, Type s, TypeRelation hasSameArgs) {
        return hasSameArgs.visit(t, s).booleanValue();
    }

    private class HasSameArgs extends TypeRelation {
        boolean strict;

        public HasSameArgs(boolean strict) {
            this.strict = strict;
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Boolean visitType(Type t, Type s) {
            throw new AssertionError();
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitMethodType(Type.MethodType t, Type s) {
            return Boolean.valueOf(s.hasTag(TypeTag.METHOD) && Types.this.containsTypeEquivalent(t.argtypes, s.mo176getParameterTypes()));
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitForAll(Type.ForAll t, Type s) {
            if (!s.hasTag(TypeTag.FORALL)) {
                return Boolean.valueOf(this.strict ? false : visitMethodType(t.asMethodType(), s).booleanValue());
            }
            Type.ForAll forAll = (Type.ForAll) s;
            if (Types.this.hasSameBounds(t, forAll) && visit(t.qtype, Types.this.subst(forAll.qtype, forAll.tvars, t.tvars)).booleanValue()) {
                z = true;
            }
            return Boolean.valueOf(z);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Boolean visitErrorType(Type.ErrorType t, Type s) {
            return false;
        }
    }

    public List<Type> subst(List<Type> ts, List<Type> from, List<Type> to) {
        return new Subst(from, to).subst(ts);
    }

    public Type subst(Type t, List<Type> from, List<Type> to) {
        return new Subst(from, to).subst(t);
    }

    private class Subst extends UnaryVisitor<Type> {
        List<Type> from;
        List<Type> to;

        /* JADX WARN: Multi-variable type inference failed */
        /* JADX WARN: Type inference failed for: r4v0, types: [com.sun.tools.javac.util.List, com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type>] */
        /* JADX WARN: Type inference failed for: r4v1, types: [com.sun.tools.javac.util.List, com.sun.tools.javac.util.List<com.sun.tools.javac.code.Type>] */
        /* JADX WARN: Type inference failed for: r4v2, types: [com.sun.tools.javac.util.List<A>] */
        public Subst(List<Type> list, List<Type> list2) {
            int length = list.length();
            int length2 = list2.length();
            List list3 = list;
            while (length > length2) {
                length--;
                list3 = list3.tail;
            }
            while (length < length2) {
                length2--;
                list2 = list2.tail;
            }
            this.from = list3;
            this.to = list2;
        }

        Type subst(Type t) {
            if (this.from.tail == null) {
                return t;
            }
            return visit(t);
        }

        List<Type> subst(List<Type> ts) {
            if (this.from.tail != null && ts.nonEmpty() && this.from.nonEmpty()) {
                Type head1 = subst(ts.head);
                List<Type> tail1 = subst(ts.tail);
                if (head1 != ts.head || tail1 != ts.tail) {
                    return tail1.prepend(head1);
                }
            }
            return ts;
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, Void ignored) {
            return t;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitMethodType(Type.MethodType t, Void ignored) {
            List<Type> argtypes = subst(t.argtypes);
            Type restype = subst(t.restype);
            List<Type> thrown = subst(t.thrown);
            if (argtypes == t.argtypes && restype == t.restype && thrown == t.thrown) {
                return t;
            }
            return new Type.MethodType(argtypes, restype, thrown, t.tsym);
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitTypeVar(Type.TypeVar t, Void ignored) {
            List list = this.from;
            List list2 = this.to;
            while (list.nonEmpty()) {
                if (t != list.head) {
                    list = list.tail;
                    list2 = list2.tail;
                } else {
                    return ((Type) list2.head).withTypeVar(t);
                }
            }
            return t;
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitUndetVar(Type.UndetVar t, Void ignored) {
            return t;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitClassType(Type.ClassType t, Void ignored) {
            if (!t.isCompound()) {
                List<Type> typarams = t.getTypeArguments();
                List<Type> typarams1 = subst(typarams);
                Type outer = t.getEnclosingType();
                Type outer1 = subst(outer);
                if (typarams1 == typarams && outer1 == outer) {
                    return t;
                }
                return new Type.ClassType(outer1, typarams1, t.tsym);
            }
            Type st = subst(Types.this.supertype(t));
            List<Type> is = subst(Types.this.interfaces(t));
            if (st == Types.this.supertype(t) && is == Types.this.interfaces(t)) {
                return t;
            }
            return Types.this.makeIntersectionType(is.prepend(st));
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitWildcardType(Type.WildcardType t, Void ignored) {
            Type bound = t.type;
            if (t.kind != BoundKind.UNBOUND) {
                bound = subst(bound);
            }
            if (bound == t.type) {
                return t;
            }
            if (t.isExtendsBound() && bound.isExtendsBound()) {
                bound = Types.this.wildUpperBound(bound);
            }
            return new Type.WildcardType(bound, t.kind, Types.this.syms.boundClass, t.bound);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitArrayType(Type.ArrayType t, Void ignored) {
            Type elemtype = subst(t.elemtype);
            if (elemtype == t.elemtype) {
                return t;
            }
            return new Type.ArrayType(elemtype, t.tsym);
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitForAll(Type.ForAll t, Void ignored) {
            if (Type.containsAny(this.to, t.tvars)) {
                List<Type> freevars = Types.this.newInstances(t.tvars);
                t = new Type.ForAll(freevars, Types.this.subst(t.qtype, t.tvars, freevars));
            }
            List<Type> tvars1 = Types.this.substBounds(t.tvars, this.from, this.to);
            Type qtype1 = subst(t.qtype);
            if (tvars1 == t.tvars && qtype1 == t.qtype) {
                return t;
            }
            if (tvars1 == t.tvars) {
                return new Type.ForAll(tvars1, qtype1);
            }
            return new Type.ForAll(tvars1, Types.this.subst(qtype1, t.tvars, tvars1));
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitErrorType(Type.ErrorType t, Void ignored) {
            return t;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r4v7, types: [A, com.sun.tools.javac.code.Type] */
    public List<Type> substBounds(List<Type> tvars, List<Type> from, List<Type> to) {
        if (tvars.isEmpty()) {
            return tvars;
        }
        ListBuffer<Type> newBoundsBuf = new ListBuffer<>();
        boolean changed = false;
        for (Type t : tvars) {
            Type.TypeVar tv = (Type.TypeVar) t;
            Type bound = subst(tv.bound, from, to);
            if (bound != tv.bound) {
                changed = true;
            }
            newBoundsBuf.append(bound);
        }
        if (!changed) {
            return tvars;
        }
        ListBuffer<Type> newTvars = new ListBuffer<>();
        for (Type t2 : tvars) {
            newTvars.append(new Type.TypeVar(t2.tsym, (Type) null, this.syms.botType));
        }
        List<Type> to2 = newTvars.toList();
        for (List list = newBoundsBuf.toList(); !list.isEmpty(); list = list.tail) {
            list.head = subst((Type) list.head, tvars, to2);
        }
        List list2 = newBoundsBuf.toList();
        for (Type t3 : newTvars.toList()) {
            ((Type.TypeVar) t3).bound = (Type) list2.head;
            list2 = list2.tail;
        }
        return newTvars.toList();
    }

    public Type.TypeVar substBound(Type.TypeVar t, List<Type> from, List<Type> to) {
        Type bound1 = subst(t.bound, from, to);
        if (bound1 == t.bound) {
            return t;
        }
        Type.TypeVar tv = new Type.TypeVar(t.tsym, (Type) null, this.syms.botType);
        tv.bound = subst(bound1, List.of(t), List.of(tv));
        return tv;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public boolean hasSameBounds(Type.ForAll t, Type.ForAll s) {
        List list = t.tvars;
        List list2 = s.tvars;
        while (list.nonEmpty() && list2.nonEmpty() && isSameType(((Type) list.head).getUpperBound(), subst(((Type) list2.head).getUpperBound(), s.tvars, t.tvars))) {
            list = list.tail;
            list2 = list2.tail;
        }
        return list.isEmpty() && list2.isEmpty();
    }

    /* JADX WARN: Multi-variable type inference failed */
    public List<Type> newInstances(List<Type> tvars) {
        List<Type> tvars1 = Type.map(tvars, newInstanceFun);
        for (List list = tvars1; list.nonEmpty(); list = list.tail) {
            Type.TypeVar tv = (Type.TypeVar) list.head;
            tv.bound = subst(tv.bound, tvars, tvars1);
        }
        return tvars1;
    }

    public Type createMethodTypeWithParameters(Type original, List<Type> newParams) {
        return (Type) original.accept(this.methodWithParameters, newParams);
    }

    public Type createMethodTypeWithThrown(Type original, List<Type> newThrown) {
        return (Type) original.accept(this.methodWithThrown, newThrown);
    }

    public Type createMethodTypeWithReturn(Type original, Type newReturn) {
        return (Type) original.accept(this.methodWithReturn, newReturn);
    }

    public Type createErrorType(Type originalType) {
        return new Type.ErrorType(originalType, this.syms.errSymbol);
    }

    public Type createErrorType(Symbol.ClassSymbol c, Type originalType) {
        return new Type.ErrorType(c, originalType);
    }

    public Type createErrorType(Name name, Symbol.TypeSymbol container, Type originalType) {
        return new Type.ErrorType(name, container, originalType);
    }

    /* JADX WARN: Multi-variable type inference failed */
    public int rank(Type t) {
        Type t2 = t.unannotatedType();
        switch (t2.getTag()) {
            case CLASS:
                Type.ClassType cls = (Type.ClassType) t2;
                if (cls.rank_field < 0) {
                    Name fullname = cls.tsym.getQualifiedName();
                    if (fullname == this.names.java_lang_Object) {
                        cls.rank_field = 0;
                    } else {
                        int r = rank(supertype(cls));
                        for (List listInterfaces = interfaces(cls); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
                            if (rank((Type) listInterfaces.head) > r) {
                                r = rank((Type) listInterfaces.head);
                            }
                        }
                        cls.rank_field = r + 1;
                    }
                }
                int r2 = cls.rank_field;
                return r2;
            case TYPEVAR:
                Type.TypeVar tvar = (Type.TypeVar) t2;
                if (tvar.rank_field < 0) {
                    int r3 = rank(supertype(tvar));
                    for (List listInterfaces2 = interfaces(tvar); listInterfaces2.nonEmpty(); listInterfaces2 = listInterfaces2.tail) {
                        if (rank((Type) listInterfaces2.head) > r3) {
                            r3 = rank((Type) listInterfaces2.head);
                        }
                    }
                    tvar.rank_field = r3 + 1;
                }
                int r4 = tvar.rank_field;
                return r4;
            case NONE:
            case ERROR:
                return 0;
            default:
                throw new AssertionError();
        }
    }

    public String toString(Type t, Locale locale) {
        return Printer.createStandardPrinter(this.messages).visit(t, locale);
    }

    public String toString(Symbol t, Locale locale) {
        return Printer.createStandardPrinter(this.messages).visit(t, locale);
    }

    @Deprecated
    public String toString(Type t) {
        if (t.hasTag(TypeTag.FORALL)) {
            Type.ForAll forAll = (Type.ForAll) t;
            return typaramsString(forAll.tvars) + forAll.qtype;
        }
        return "" + t;
    }

    private String typaramsString(List<Type> tvars) {
        StringBuilder s = new StringBuilder();
        s.append('<');
        boolean first = true;
        for (Type t : tvars) {
            if (!first) {
                s.append(", ");
            }
            first = false;
            appendTyparamString((Type.TypeVar) t.unannotatedType(), s);
        }
        s.append('>');
        return s.toString();
    }

    private void appendTyparamString(Type.TypeVar t, StringBuilder buf) {
        buf.append(t);
        if (t.bound == null || t.bound.tsym.getQualifiedName() == this.names.java_lang_Object) {
            return;
        }
        buf.append(" extends ");
        Type bound = t.bound;
        if (!bound.isCompound()) {
            buf.append(bound);
            return;
        }
        if ((erasure(t).tsym.flags() & 512) == 0) {
            buf.append(supertype(t));
            for (Type intf : interfaces(t)) {
                buf.append('&');
                buf.append(intf);
            }
            return;
        }
        boolean first = true;
        for (Type intf2 : interfaces(t)) {
            if (!first) {
                buf.append('&');
            }
            first = false;
            buf.append(intf2);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public List<Type> closure(Type t) {
        List<Type> cl = this.closureCache.get(t);
        if (cl == null) {
            Type st = supertype(t);
            if (!t.isCompound()) {
                if (st.hasTag(TypeTag.CLASS)) {
                    cl = insert(closure(st), t);
                } else if (st.hasTag(TypeTag.TYPEVAR)) {
                    cl = closure(st).prepend(t);
                } else {
                    cl = List.of(t);
                }
            } else {
                cl = closure(supertype(t));
            }
            for (List listInterfaces = interfaces(t); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
                cl = union(cl, closure((Type) listInterfaces.head));
            }
            this.closureCache.put(t, cl);
        }
        return cl;
    }

    public List<Type> insert(List<Type> cl, Type t) {
        if (cl.isEmpty()) {
            return cl.prepend(t);
        }
        if (t.tsym == cl.head.tsym) {
            return cl;
        }
        if (t.tsym.precedes(cl.head.tsym, this)) {
            return cl.prepend(t);
        }
        return insert(cl.tail, t).prepend(cl.head);
    }

    public List<Type> union(List<Type> cl1, List<Type> cl2) {
        if (cl1.isEmpty()) {
            return cl2;
        }
        if (cl2.isEmpty()) {
            return cl1;
        }
        if (cl1.head.tsym == cl2.head.tsym) {
            return union(cl1.tail, cl2.tail).prepend(cl1.head);
        }
        if (cl1.head.tsym.precedes(cl2.head.tsym, this)) {
            return union(cl1.tail, cl2).prepend(cl1.head);
        }
        if (cl2.head.tsym.precedes(cl1.head.tsym, this)) {
            return union(cl1, cl2.tail).prepend(cl2.head);
        }
        return union(cl1.tail, cl2).prepend(cl1.head);
    }

    public List<Type> intersect(List<Type> cl1, List<Type> cl2) {
        if (cl1 == cl2) {
            return cl1;
        }
        if (cl1.isEmpty() || cl2.isEmpty()) {
            return List.nil();
        }
        if (cl1.head.tsym.precedes(cl2.head.tsym, this)) {
            return intersect(cl1.tail, cl2);
        }
        if (cl2.head.tsym.precedes(cl1.head.tsym, this)) {
            return intersect(cl1, cl2.tail);
        }
        if (isSameType(cl1.head, cl2.head)) {
            return intersect(cl1.tail, cl2.tail).prepend(cl1.head);
        }
        if (cl1.head.tsym == cl2.head.tsym && cl1.head.hasTag(TypeTag.CLASS) && cl2.head.hasTag(TypeTag.CLASS)) {
            if (cl1.head.isParameterized() && cl2.head.isParameterized()) {
                Type merge = merge(cl1.head, cl2.head);
                return intersect(cl1.tail, cl2.tail).prepend(merge);
            }
            Type merge2 = cl1.head;
            if (merge2.isRaw() || cl2.head.isRaw()) {
                return intersect(cl1.tail, cl2.tail).prepend(erasure(cl1.head));
            }
        }
        return intersect(cl1.tail, cl2.tail);
    }

    class TypePair {
        boolean strict;
        final Type t1;
        final Type t2;

        TypePair(Types this$0, Type t1, Type t2) {
            this(t1, t2, false);
        }

        TypePair(Type t1, Type t2, boolean strict) {
            this.t1 = t1;
            this.t2 = t2;
            this.strict = strict;
        }

        public int hashCode() {
            return (Types.this.hashCode(this.t1) * 127) + Types.this.hashCode(this.t2);
        }

        public boolean equals(Object obj) {
            if (!(obj instanceof TypePair)) {
                return false;
            }
            TypePair typePair = (TypePair) obj;
            return Types.this.isSameType(this.t1, typePair.t1, this.strict) && Types.this.isSameType(this.t2, typePair.t2, this.strict);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    private Type merge(Type c1, Type c2) {
        Type m;
        Type.ClassType class1 = (Type.ClassType) c1;
        List typeArguments = class1.getTypeArguments();
        Type.ClassType class2 = (Type.ClassType) c2;
        List typeArguments2 = class2.getTypeArguments();
        ListBuffer listBuffer = new ListBuffer();
        List typeArguments3 = class1.tsym.type.getTypeArguments();
        while (true) {
            if (!typeArguments.nonEmpty() || !typeArguments2.nonEmpty() || !typeArguments3.nonEmpty()) {
                break;
            }
            if (containsType((Type) typeArguments.head, (Type) typeArguments2.head)) {
                listBuffer.append(typeArguments.head);
            } else if (containsType((Type) typeArguments2.head, (Type) typeArguments.head)) {
                listBuffer.append(typeArguments2.head);
            } else {
                TypePair pair = new TypePair(this, c1, c2);
                if (this.mergeCache.add(pair)) {
                    m = new Type.WildcardType(lub(wildUpperBound((Type) typeArguments.head), wildUpperBound((Type) typeArguments2.head)), BoundKind.EXTENDS, this.syms.boundClass);
                    this.mergeCache.remove(pair);
                } else {
                    m = new Type.WildcardType(this.syms.objectType, BoundKind.UNBOUND, this.syms.boundClass);
                }
                listBuffer.append(m.withTypeVar((Type) typeArguments3.head));
            }
            typeArguments = typeArguments.tail;
            typeArguments2 = typeArguments2.tail;
            typeArguments3 = typeArguments3.tail;
        }
        Assert.check(typeArguments.isEmpty() && typeArguments2.isEmpty() && typeArguments3.isEmpty());
        return new Type.ClassType(class1.getEnclosingType(), listBuffer.toList(), class1.tsym);
    }

    private Type compoundMin(List<Type> cl) {
        if (cl.isEmpty()) {
            return this.syms.objectType;
        }
        List<Type> compound = closureMin(cl);
        if (compound.isEmpty()) {
            return null;
        }
        if (compound.tail.isEmpty()) {
            return compound.head;
        }
        return makeIntersectionType(compound);
    }

    /* JADX WARN: Multi-variable type inference failed */
    private List<Type> closureMin(List<Type> list) {
        ListBuffer listBuffer = new ListBuffer();
        ListBuffer listBuffer2 = new ListBuffer();
        HashSet hashSet = new HashSet();
        for (List list2 = list; !list2.isEmpty(); list2 = list2.tail) {
            Type type = (Type) list2.head;
            boolean z = !hashSet.contains(type);
            if (z && type.hasTag(TypeTag.TYPEVAR)) {
                Iterator it = list2.tail.iterator();
                while (true) {
                    if (!it.hasNext()) {
                        break;
                    }
                    if (isSubtypeNoCapture((Type) it.next(), type)) {
                        z = false;
                        break;
                    }
                }
            }
            if (z) {
                if (type.isInterface()) {
                    listBuffer2.append(type);
                } else {
                    listBuffer.append(type);
                }
                for (A a : list2.tail) {
                    if (isSubtypeNoCapture(type, a)) {
                        hashSet.add(a);
                    }
                }
            }
        }
        return listBuffer.appendList(listBuffer2).toList();
    }

    public Type lub(List<Type> ts) {
        return lub((Type[]) ts.toArray(new Type[ts.length()]));
    }

    /* JADX WARN: Removed duplicated region for block: B:33:0x008c  */
    /* JADX WARN: Removed duplicated region for block: B:37:0x009b  */
    /* JADX WARN: Removed duplicated region for block: B:46:0x00ca  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public com.sun.tools.javac.code.Type lub(com.sun.tools.javac.code.Type... r18) {
        /*
            Method dump skipped, instruction units count: 386
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.code.Types.lub(com.sun.tools.javac.code.Type[]):com.sun.tools.javac.code.Type");
    }

    List<Type> erasedSupertypes(Type t) {
        ListBuffer<Type> buf = new ListBuffer<>();
        for (Type sup : closure(t)) {
            if (sup.hasTag(TypeTag.TYPEVAR)) {
                buf.append(sup);
            } else {
                buf.append(erasure(sup));
            }
        }
        return buf.toList();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Type arraySuperType() {
        if (this.arraySuperType == null) {
            synchronized (this) {
                if (this.arraySuperType == null) {
                    this.arraySuperType = makeIntersectionType(List.of(this.syms.serializableType, this.syms.cloneableType), true);
                }
            }
        }
        return this.arraySuperType;
    }

    public Type glb(List<Type> ts) {
        Type t1 = ts.head;
        for (Type t2 : ts.tail) {
            if (t1.isErroneous()) {
                return t1;
            }
            t1 = glb(t1, t2);
        }
        return t1;
    }

    public Type glb(Type t, Type s) {
        if (s == null) {
            return t;
        }
        if (t.isPrimitive() || s.isPrimitive()) {
            return this.syms.errType;
        }
        if (isSubtypeNoCapture(t, s)) {
            return t;
        }
        if (isSubtypeNoCapture(s, t)) {
            return s;
        }
        List<Type> closure = union(closure(t), closure(s));
        return glbFlattened(closure, t);
    }

    private Type glbFlattened(List<Type> flatBounds, Type errT) {
        List<Type> bounds = closureMin(flatBounds);
        if (bounds.isEmpty()) {
            return this.syms.objectType;
        }
        if (bounds.tail.isEmpty()) {
            return bounds.head;
        }
        int classCount = 0;
        List<Type> lowers = List.nil();
        for (Type bound : bounds) {
            if (!bound.isInterface()) {
                classCount++;
                Type lower = cvarLowerBound(bound);
                if (bound != lower && !lower.hasTag(TypeTag.BOT)) {
                    lowers = insert(lowers, lower);
                }
            }
        }
        if (classCount > 1) {
            if (lowers.isEmpty()) {
                return createErrorType(errT);
            }
            return glbFlattened(union(bounds, lowers), errT);
        }
        return makeIntersectionType(bounds);
    }

    public int hashCode(Type t) {
        return hashCode.visit(t).intValue();
    }

    public boolean resultSubtype(Type t, Type s, Warner warner) {
        List<Type> tvars = t.getTypeArguments();
        List<Type> svars = s.getTypeArguments();
        Type tres = t.mo178getReturnType();
        Type sres = subst(s.mo178getReturnType(), svars, tvars);
        return covariantReturnType(tres, sres, warner);
    }

    public boolean returnTypeSubstitutable(Type r1, Type r2) {
        if (hasSameArgs(r1, r2)) {
            return resultSubtype(r1, r2, this.noWarnings);
        }
        return covariantReturnType(r1.mo178getReturnType(), erasure(r2.mo178getReturnType()), this.noWarnings);
    }

    public boolean returnTypeSubstitutable(Type r1, Type r2, Type r2res, Warner warner) {
        if (isSameType(r1.mo178getReturnType(), r2res)) {
            return true;
        }
        if (r1.mo178getReturnType().isPrimitive() || r2res.isPrimitive()) {
            return false;
        }
        if (hasSameArgs(r1, r2)) {
            return covariantReturnType(r1.mo178getReturnType(), r2res, warner);
        }
        if (!this.allowCovariantReturns) {
            return false;
        }
        if (isSubtypeUnchecked(r1.mo178getReturnType(), r2res, warner)) {
            return true;
        }
        if (!isSubtype(r1.mo178getReturnType(), erasure(r2res))) {
            return false;
        }
        warner.warn(Lint.LintCategory.UNCHECKED);
        return true;
    }

    public boolean covariantReturnType(Type t, Type s, Warner warner) {
        return isSameType(t, s) || (this.allowCovariantReturns && !t.isPrimitive() && !s.isPrimitive() && isAssignable(t, s, warner));
    }

    public Symbol.ClassSymbol boxedClass(Type t) {
        return this.reader.enterClass(this.syms.boxedName[t.getTag().ordinal()]);
    }

    public Type boxedTypeOrType(Type t) {
        return t.isPrimitive() ? boxedClass(t).type : t;
    }

    public Type unboxedType(Type t) {
        if (this.allowBoxing) {
            for (int i = 0; i < this.syms.boxedName.length; i++) {
                Name box = this.syms.boxedName[i];
                if (box != null && asSuper(t, this.reader.enterClass(box)) != null) {
                    return this.syms.typeOfTag[i];
                }
            }
        }
        return Type.noType;
    }

    public Type unboxedTypeOrType(Type t) {
        Type unboxedType = unboxedType(t);
        return unboxedType.hasTag(TypeTag.NONE) ? t : unboxedType;
    }

    public List<Type> capture(List<Type> ts) {
        List<Type> buf = List.nil();
        for (Type t : ts) {
            buf = buf.prepend(capture(t));
        }
        return buf.reverse();
    }

    /* JADX WARN: Multi-variable type inference failed */
    /* JADX WARN: Type inference failed for: r4v10, types: [A, com.sun.tools.javac.code.Type] */
    public Type capture(Type t) {
        Type.ClassType G;
        List<Type> A;
        Type capturedEncl;
        Type t2 = t;
        if (!t2.hasTag(TypeTag.CLASS)) {
            return t2;
        }
        if (t.getEnclosingType() != Type.noType && (capturedEncl = capture(t.getEnclosingType())) != t.getEnclosingType()) {
            Type type1 = memberType(capturedEncl, t2.tsym);
            t2 = subst(type1, t2.tsym.type.getTypeArguments(), t.getTypeArguments());
        }
        Type t3 = t2.unannotatedType();
        Type.ClassType cls = (Type.ClassType) t3;
        if (cls.isRaw() || !cls.isParameterized()) {
            return cls;
        }
        Type.ClassType G2 = (Type.ClassType) cls.asElement().asType();
        List<Type> A2 = G2.getTypeArguments();
        List<Type> T = cls.getTypeArguments();
        List<Type> S = freshTypeVariables(T);
        List list = A2;
        List list2 = T;
        List list3 = S;
        boolean captured = false;
        while (!list.isEmpty() && !list2.isEmpty() && !list3.isEmpty()) {
            if (list3.head == list2.head) {
                G = G2;
                A = A2;
            } else {
                captured = true;
                Type.WildcardType Ti = (Type.WildcardType) ((Type) list2.head).unannotatedType();
                Type Ui = ((Type) list.head).getUpperBound();
                Type.CapturedType Si = (Type.CapturedType) ((Type) list3.head).unannotatedType();
                if (Ui == null) {
                    Ui = this.syms.objectType;
                }
                switch (Ti.kind) {
                    case EXTENDS:
                        Si.bound = glb(Ti.getExtendsBound(), subst(Ui, A2, S));
                        Si.lower = this.syms.botType;
                        break;
                    case UNBOUND:
                        Si.bound = subst(Ui, A2, S);
                        Si.lower = this.syms.botType;
                        break;
                    case SUPER:
                        Si.bound = subst(Ui, A2, S);
                        Si.lower = Ti.getSuperBound();
                        break;
                }
                Type tmpBound = Si.bound.hasTag(TypeTag.UNDETVAR) ? ((Type.UndetVar) Si.bound).qtype : Si.bound;
                G = G2;
                Type tmpLower = Si.lower.hasTag(TypeTag.UNDETVAR) ? ((Type.UndetVar) Si.lower).qtype : Si.lower;
                A = A2;
                if (!Si.bound.hasTag(TypeTag.ERROR) && !Si.lower.hasTag(TypeTag.ERROR) && isSameType(tmpBound, tmpLower, false)) {
                    list3.head = Si.bound;
                }
            }
            list = list.tail;
            list2 = list2.tail;
            list3 = list3.tail;
            G2 = G;
            A2 = A;
        }
        if (!list.isEmpty() || !list2.isEmpty() || !list3.isEmpty()) {
            return erasure(t3);
        }
        if (captured) {
            return new Type.ClassType(cls.getEnclosingType(), S, cls.tsym);
        }
        return t3;
    }

    public List<Type> freshTypeVariables(List<Type> types) {
        ListBuffer<Type> result = new ListBuffer<>();
        for (Type t : types) {
            if (t.hasTag(TypeTag.WILDCARD)) {
                Type t2 = t.unannotatedType();
                Type bound = ((Type.WildcardType) t2).getExtendsBound();
                if (bound == null) {
                    bound = this.syms.objectType;
                }
                result.append(new Type.CapturedType(this.capturedName, this.syms.noSymbol, bound, this.syms.botType, (Type.WildcardType) t2));
            } else {
                result.append(t);
            }
        }
        return result.toList();
    }

    /* JADX INFO: Access modifiers changed from: private */
    /* JADX WARN: Multi-variable type inference failed */
    public boolean sideCast(Type from, Type to, Warner warn) {
        boolean reverse = false;
        if ((to.tsym.flags() & 512) == 0) {
            Assert.check((512 & from.tsym.flags()) != 0);
            reverse = true;
            to = from;
            from = to;
        }
        List listSuperClosure = superClosure(to, erasure(from));
        boolean giveWarning = listSuperClosure.isEmpty();
        while (listSuperClosure.nonEmpty()) {
            Type t1 = asSuper(from, ((Type) listSuperClosure.head).tsym);
            Type t2 = (Type) listSuperClosure.head;
            if (disjointTypes(t1.getTypeArguments(), t2.getTypeArguments())) {
                return false;
            }
            giveWarning = giveWarning || (!reverse ? !giveWarning(t1, t2) : !giveWarning(t2, t1));
            listSuperClosure = listSuperClosure.tail;
        }
        if (giveWarning) {
            if (!isReifiable(reverse ? from : to)) {
                warn.warn(Lint.LintCategory.UNCHECKED);
            }
        }
        if (!this.allowCovariantReturns) {
            this.chk.checkCompatibleAbstracts(warn.pos(), from, to);
        }
        return true;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public boolean sideCastFinal(Type from, Type to, Warner warn) {
        boolean reverse = false;
        if ((to.tsym.flags() & 512) == 0) {
            Assert.check((512 & from.tsym.flags()) != 0);
            reverse = true;
            to = from;
            from = to;
        }
        Assert.check((from.tsym.flags() & 16) != 0);
        Type t1 = asSuper(from, to.tsym);
        if (t1 == null) {
            return false;
        }
        Type t2 = to;
        if (disjointTypes(t1.getTypeArguments(), t2.getTypeArguments())) {
            return false;
        }
        if (!this.allowCovariantReturns) {
            this.chk.checkCompatibleAbstracts(warn.pos(), from, to);
        }
        if (!isReifiable(to) && (!reverse ? giveWarning(t1, t2) : giveWarning(t2, t1))) {
            warn.warn(Lint.LintCategory.UNCHECKED);
        }
        return true;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public boolean giveWarning(Type from, Type to) {
        List<Type> bounds = to.isCompound() ? ((Type.IntersectionClassType) to.unannotatedType()).getComponents() : List.of(to);
        for (Type b : bounds) {
            Type subFrom = asSub(from, b.tsym);
            if (b.isParameterized() && !isUnbounded(b) && !isSubtype(from, b) && (subFrom == null || !containsType(b.allparams(), subFrom.allparams()))) {
                return true;
            }
        }
        return false;
    }

    /* JADX WARN: Multi-variable type inference failed */
    private List<Type> superClosure(Type t, Type s) {
        List<Type> cl = List.nil();
        for (List listInterfaces = interfaces(t); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
            if (isSubtype(s, erasure((Type) listInterfaces.head))) {
                cl = insert(cl, (Type) listInterfaces.head);
            } else {
                cl = union(cl, superClosure((Type) listInterfaces.head, s));
            }
        }
        return cl;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public boolean containsTypeEquivalent(Type t, Type s) {
        return isSameType(t, s) || (containsType(t, s) && containsType(s, t));
    }

    public void adapt(Type source, Type target, ListBuffer<Type> from, ListBuffer<Type> to) throws AdaptFailure {
        new Adapter(from, to).adapt(source, target);
    }

    class Adapter extends SimpleVisitor<Void, Type> {
        ListBuffer<Type> from;
        ListBuffer<Type> to;
        private Set<TypePair> cache = new HashSet();
        Map<Symbol, Type> mapping = new HashMap();

        Adapter(ListBuffer<Type> from, ListBuffer<Type> to) {
            this.from = from;
            this.to = to;
        }

        /* JADX WARN: Multi-variable type inference failed */
        /* JADX WARN: Type inference failed for: r2v3, types: [A, com.sun.tools.javac.code.Type] */
        public void adapt(Type source, Type target) throws AdaptFailure {
            visit(source, target);
            List list = this.from.toList();
            List list2 = this.to.toList();
            while (!list.isEmpty()) {
                Type type = this.mapping.get(((Type) list.head).tsym);
                if (list2.head != type) {
                    list2.head = type;
                }
                list = list.tail;
                list2 = list2.tail;
            }
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Void visitClassType(Type.ClassType source, Type target) throws AdaptFailure {
            if (target.hasTag(TypeTag.CLASS)) {
                adaptRecursive(source.allparams(), target.allparams());
                return null;
            }
            return null;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Void visitArrayType(Type.ArrayType source, Type target) throws AdaptFailure {
            if (target.hasTag(TypeTag.ARRAY)) {
                adaptRecursive(Types.this.elemtype(source), Types.this.elemtype(target));
                return null;
            }
            return null;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Void visitWildcardType(Type.WildcardType source, Type target) throws AdaptFailure {
            if (source.isExtendsBound()) {
                adaptRecursive(Types.this.wildUpperBound(source), Types.this.wildUpperBound(target));
                return null;
            }
            if (source.isSuperBound()) {
                adaptRecursive(Types.this.wildLowerBound(source), Types.this.wildLowerBound(target));
                return null;
            }
            return null;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Void visitTypeVar(Type.TypeVar source, Type target) throws AdaptFailure {
            Type val = this.mapping.get(source.tsym);
            if (val != null) {
                if (val.isSuperBound() && target.isSuperBound()) {
                    val = Types.this.isSubtype(Types.this.wildLowerBound(val), Types.this.wildLowerBound(target)) ? target : val;
                } else if (val.isExtendsBound() && target.isExtendsBound()) {
                    val = Types.this.isSubtype(Types.this.wildUpperBound(val), Types.this.wildUpperBound(target)) ? val : target;
                } else if (!Types.this.isSameType(val, target)) {
                    throw new AdaptFailure();
                }
            } else {
                val = target;
                this.from.append(source);
                this.to.append(target);
            }
            this.mapping.put(source.tsym, val);
            return null;
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Void visitType(Type source, Type target) {
            return null;
        }

        private void adaptRecursive(Type source, Type target) {
            TypePair pair = new TypePair(Types.this, source, target);
            if (this.cache.add(pair)) {
                try {
                    visit(source, target);
                } finally {
                    this.cache.remove(pair);
                }
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        private void adaptRecursive(List<Type> list, List<Type> list2) {
            int length = list.length();
            int length2 = list2.length();
            List list3 = list;
            List list4 = list2;
            if (length == length2) {
                while (list3.nonEmpty()) {
                    adaptRecursive((Type) list3.head, (Type) list4.head);
                    list3 = list3.tail;
                    list4 = list4.tail;
                }
            }
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void adaptSelf(Type t, ListBuffer<Type> from, ListBuffer<Type> to) {
        try {
            adapt(t.tsym.type, t, from, to);
        } catch (AdaptFailure ex) {
            throw new AssertionError(ex);
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Type rewriteQuantifiers(Type t, boolean high, boolean rewriteTypeVars) {
        return new Rewriter(high, rewriteTypeVars).visit(t);
    }

    class Rewriter extends UnaryVisitor<Type> {
        boolean high;
        boolean rewriteTypeVars;

        Rewriter(boolean high, boolean rewriteTypeVars) {
            this.high = high;
            this.rewriteTypeVars = rewriteTypeVars;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitClassType(Type.ClassType t, Void s) {
            ListBuffer<Type> rewritten = new ListBuffer<>();
            boolean changed = false;
            for (Type arg : t.allparams()) {
                Type bound = visit(arg);
                if (arg != bound) {
                    changed = true;
                }
                rewritten.append(bound);
            }
            if (changed) {
                return Types.this.subst(t.tsym.type, t.tsym.type.allparams(), rewritten.toList());
            }
            return t;
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, Void s) {
            return t;
        }

        @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitCapturedType(Type.CapturedType t, Void s) {
            Type bound;
            Type w_bound = t.wildcard.type;
            if (w_bound.contains(t)) {
                bound = Types.this.erasure(w_bound);
            } else {
                bound = visit(w_bound);
            }
            return rewriteAsWildcardType(visit(bound), t.wildcard.bound, t.wildcard.kind);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitTypeVar(Type.TypeVar t, Void s) {
            Type bound;
            if (this.rewriteTypeVars) {
                if (t.bound.contains(t)) {
                    bound = Types.this.erasure(t.bound);
                } else {
                    bound = visit(t.bound);
                }
                return rewriteAsWildcardType(bound, t, BoundKind.EXTENDS);
            }
            return t;
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public Type visitWildcardType(Type.WildcardType t, Void s) {
            Type bound2 = visit(t.type);
            return t.type == bound2 ? t : rewriteAsWildcardType(bound2, t.bound, t.kind);
        }

        private Type rewriteAsWildcardType(Type bound, Type.TypeVar formal, BoundKind bk) {
            switch (bk) {
                case EXTENDS:
                    return this.high ? Types.this.makeExtendsWildcard(B(bound), formal) : Types.this.makeExtendsWildcard(Types.this.syms.objectType, formal);
                case UNBOUND:
                    return Types.this.makeExtendsWildcard(Types.this.syms.objectType, formal);
                case SUPER:
                    return this.high ? Types.this.makeSuperWildcard(Types.this.syms.botType, formal) : Types.this.makeSuperWildcard(B(bound), formal);
                default:
                    Assert.error("Invalid bound kind " + bk);
                    return null;
            }
        }

        Type B(Type t) {
            Type superBound;
            while (t.hasTag(TypeTag.WILDCARD)) {
                Type.WildcardType w = (Type.WildcardType) t.unannotatedType();
                if (this.high) {
                    superBound = w.getExtendsBound();
                } else {
                    superBound = w.getSuperBound();
                }
                t = superBound;
                if (t == null) {
                    t = this.high ? Types.this.syms.objectType : Types.this.syms.botType;
                }
            }
            return t;
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Type.WildcardType makeExtendsWildcard(Type bound, Type.TypeVar formal) {
        if (bound == this.syms.objectType) {
            return new Type.WildcardType(this.syms.objectType, BoundKind.UNBOUND, this.syms.boundClass, formal);
        }
        return new Type.WildcardType(bound, BoundKind.EXTENDS, this.syms.boundClass, formal);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public Type.WildcardType makeSuperWildcard(Type bound, Type.TypeVar formal) {
        if (bound.hasTag(TypeTag.BOT)) {
            return new Type.WildcardType(this.syms.objectType, BoundKind.UNBOUND, this.syms.boundClass, formal);
        }
        return new Type.WildcardType(bound, BoundKind.SUPER, this.syms.boundClass, formal);
    }

    public static class UniqueType {
        public final Type type;
        final Types types;

        public UniqueType(Type type, Types types) {
            this.type = type;
            this.types = types;
        }

        public int hashCode() {
            return this.types.hashCode(this.type);
        }

        public boolean equals(Object obj) {
            return (obj instanceof UniqueType) && this.types.isSameAnnotatedType(this.type, ((UniqueType) obj).type);
        }

        public String toString() {
            return this.type.toString();
        }
    }

    public static abstract class DefaultTypeVisitor<R, S> implements Type.Visitor<R, S> {
        public final R visit(Type type, S s) {
            return (R) type.accept(this, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitClassType(Type.ClassType t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitWildcardType(Type.WildcardType t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitArrayType(Type.ArrayType t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitMethodType(Type.MethodType t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitPackageType(Type.PackageType t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitTypeVar(Type.TypeVar t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitCapturedType(Type.CapturedType t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitForAll(Type.ForAll t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitUndetVar(Type.UndetVar t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitErrorType(Type.ErrorType t, S s) {
            return visitType(t, s);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public R visitAnnotatedType(Type.AnnotatedType t, S s) {
            return visit(t.unannotatedType(), s);
        }
    }

    public static abstract class DefaultSymbolVisitor<R, S> implements Symbol.Visitor<R, S> {
        public final R visit(Symbol symbol, S s) {
            return (R) symbol.accept(this, s);
        }

        @Override // com.sun.tools.javac.code.Symbol.Visitor
        public R visitClassSymbol(Symbol.ClassSymbol s, S arg) {
            return visitSymbol(s, arg);
        }

        @Override // com.sun.tools.javac.code.Symbol.Visitor
        public R visitMethodSymbol(Symbol.MethodSymbol s, S arg) {
            return visitSymbol(s, arg);
        }

        @Override // com.sun.tools.javac.code.Symbol.Visitor
        public R visitOperatorSymbol(Symbol.OperatorSymbol s, S arg) {
            return visitSymbol(s, arg);
        }

        @Override // com.sun.tools.javac.code.Symbol.Visitor
        public R visitPackageSymbol(Symbol.PackageSymbol s, S arg) {
            return visitSymbol(s, arg);
        }

        @Override // com.sun.tools.javac.code.Symbol.Visitor
        public R visitTypeSymbol(Symbol.TypeSymbol s, S arg) {
            return visitSymbol(s, arg);
        }

        @Override // com.sun.tools.javac.code.Symbol.Visitor
        public R visitVarSymbol(Symbol.VarSymbol s, S arg) {
            return visitSymbol(s, arg);
        }
    }

    public static abstract class SimpleVisitor<R, S> extends DefaultTypeVisitor<R, S> {
        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public R visitCapturedType(Type.CapturedType t, S s) {
            return visitTypeVar(t, s);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public R visitForAll(Type.ForAll t, S s) {
            return visit(t.qtype, s);
        }

        @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
        public R visitUndetVar(Type.UndetVar t, S s) {
            return visit(t.qtype, s);
        }
    }

    public static abstract class UnaryVisitor<R> extends SimpleVisitor<R, Void> {
        public final R visit(Type type) {
            return (R) type.accept(this, (Object) null);
        }
    }

    public static class MapVisitor<S> extends DefaultTypeVisitor<Type, S> {
        public final Type visit(Type t) {
            return (Type) t.accept(this, (Object) null);
        }

        @Override // com.sun.tools.javac.code.Type.Visitor
        public Type visitType(Type t, S s) {
            return t;
        }
    }

    public Attribute.RetentionPolicy getRetention(Attribute.Compound a) {
        return getRetention(a.type.tsym);
    }

    public Attribute.RetentionPolicy getRetention(Symbol sym) {
        Attribute value;
        Attribute.RetentionPolicy vis = Attribute.RetentionPolicy.CLASS;
        Attribute.Compound c = sym.attribute(this.syms.retentionType.tsym);
        if (c != null && (value = c.member(this.names.value)) != null && (value instanceof Attribute.Enum)) {
            Name levelName = ((Attribute.Enum) value).value.name;
            return levelName == this.names.SOURCE ? Attribute.RetentionPolicy.SOURCE : levelName == this.names.CLASS ? Attribute.RetentionPolicy.CLASS : levelName == this.names.RUNTIME ? Attribute.RetentionPolicy.RUNTIME : vis;
        }
        return vis;
    }

    public static abstract class SignatureGenerator {
        private final Types types;

        protected abstract void append(char c);

        protected abstract void append(Name name);

        protected abstract void append(byte[] bArr);

        protected void classReference(Symbol.ClassSymbol c) {
        }

        protected SignatureGenerator(Types types) {
            this.types = types;
        }

        /* JADX WARN: Multi-variable type inference failed */
        public void assembleSig(Type type) {
            Type type2 = type.unannotatedType();
            switch (type2.getTag()) {
                case ARRAY:
                    Type.ArrayType at = (Type.ArrayType) type2;
                    append('[');
                    assembleSig(at.elemtype);
                    return;
                case CLASS:
                    append('L');
                    assembleClassSig(type2);
                    append(';');
                    return;
                case BYTE:
                    append('B');
                    return;
                case CHAR:
                    append('C');
                    return;
                case SHORT:
                    append('S');
                    return;
                case INT:
                    append('I');
                    return;
                case LONG:
                    append('J');
                    return;
                case FLOAT:
                    append('F');
                    return;
                case DOUBLE:
                    append('D');
                    return;
                case BOOLEAN:
                    append('Z');
                    return;
                case VOID:
                    append('V');
                    return;
                case TYPEVAR:
                    append('T');
                    append(type2.tsym.name);
                    append(';');
                    return;
                case BOT:
                case NONE:
                case ERROR:
                case UNDETVAR:
                default:
                    throw new AssertionError("typeSig " + type2.getTag());
                case WILDCARD:
                    Type.WildcardType ta = (Type.WildcardType) type2;
                    switch (ta.kind) {
                        case EXTENDS:
                            append('+');
                            assembleSig(ta.type);
                            return;
                        case UNBOUND:
                            append('*');
                            return;
                        case SUPER:
                            append('-');
                            assembleSig(ta.type);
                            return;
                        default:
                            throw new AssertionError(ta.kind);
                    }
                case FORALL:
                    Type.ForAll ft = (Type.ForAll) type2;
                    assembleParamsSig(ft.tvars);
                    assembleSig(ft.qtype);
                    return;
                case METHOD:
                    Type.MethodType mt = (Type.MethodType) type2;
                    append('(');
                    assembleSig(mt.argtypes);
                    append(')');
                    assembleSig(mt.restype);
                    if (hasTypeVar(mt.thrown)) {
                        for (List list = mt.thrown; list.nonEmpty(); list = list.tail) {
                            append('^');
                            assembleSig((Type) list.head);
                        }
                        return;
                    }
                    return;
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        public boolean hasTypeVar(List<Type> list) {
            for (List list2 = list; list2.nonEmpty(); list2 = list2.tail) {
                if (((Type) list2.head).hasTag(TypeTag.TYPEVAR)) {
                    return true;
                }
            }
            return false;
        }

        public void assembleClassSig(Type type) {
            Type.ClassType ct = (Type.ClassType) type.unannotatedType();
            Symbol.ClassSymbol c = (Symbol.ClassSymbol) ct.tsym;
            classReference(c);
            Type outer = ct.getEnclosingType();
            if (outer.allparams().nonEmpty()) {
                boolean rawOuter = c.owner.kind == 16 || c.name == this.types.names.empty;
                assembleClassSig(rawOuter ? this.types.erasure(outer) : outer);
                append(rawOuter ? '$' : DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
                Assert.check(c.flatname.startsWith(c.owner.enclClass().flatname));
                append(rawOuter ? c.flatname.subName(c.owner.enclClass().flatname.getByteLength() + 1, c.flatname.getByteLength()) : c.name);
            } else {
                append(ClassFile.externalize(c.flatname));
            }
            if (ct.getTypeArguments().nonEmpty()) {
                append('<');
                assembleSig(ct.getTypeArguments());
                append('>');
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        public void assembleParamsSig(List<Type> typarams) {
            append('<');
            for (List list = typarams; list.nonEmpty(); list = list.tail) {
                Type.TypeVar tvar = (Type.TypeVar) list.head;
                append(tvar.tsym.name);
                List<Type> bounds = this.types.getBounds(tvar);
                if ((bounds.head.tsym.flags() & 512) != 0) {
                    append(':');
                }
                for (List list2 = bounds; list2.nonEmpty(); list2 = list2.tail) {
                    append(':');
                    assembleSig((Type) list2.head);
                }
            }
            append('>');
        }

        /* JADX WARN: Multi-variable type inference failed */
        private void assembleSig(List<Type> types) {
            for (List list = types; list.nonEmpty(); list = list.tail) {
                assembleSig((Type) list.head);
            }
        }
    }
}
