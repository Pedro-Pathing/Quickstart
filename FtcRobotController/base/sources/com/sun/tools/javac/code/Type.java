package com.sun.tools.javac.code;

import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import java.lang.annotation.Annotation;
import java.lang.reflect.Array;
import java.util.Collections;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import javax.lang.model.element.Element;
import javax.lang.model.type.DeclaredType;
import javax.lang.model.type.ExecutableType;
import javax.lang.model.type.IntersectionType;
import javax.lang.model.type.NoType;
import javax.lang.model.type.NullType;
import javax.lang.model.type.PrimitiveType;
import javax.lang.model.type.TypeKind;
import javax.lang.model.type.TypeMirror;
import javax.lang.model.type.TypeVariable;
import javax.lang.model.type.TypeVisitor;
import javax.lang.model.type.UnionType;

/* JADX INFO: loaded from: classes.dex */
public abstract class Type extends AnnoConstruct implements TypeMirror {
    public Symbol.TypeSymbol tsym;
    public static final JCNoType noType = new JCNoType() { // from class: com.sun.tools.javac.code.Type.1
        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return "none";
        }
    };
    public static final JCNoType recoveryType = new JCNoType() { // from class: com.sun.tools.javac.code.Type.2
        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return "recovery";
        }
    };
    public static final JCNoType stuckType = new JCNoType() { // from class: com.sun.tools.javac.code.Type.3
        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return "stuck";
        }
    };
    public static boolean moreInfo = false;

    public interface Visitor<R, S> {
        R visitAnnotatedType(AnnotatedType annotatedType, S s);

        R visitArrayType(ArrayType arrayType, S s);

        R visitCapturedType(CapturedType capturedType, S s);

        R visitClassType(ClassType classType, S s);

        R visitErrorType(ErrorType errorType, S s);

        R visitForAll(ForAll forAll, S s);

        R visitMethodType(MethodType methodType, S s);

        R visitPackageType(PackageType packageType, S s);

        R visitType(Type type, S s);

        R visitTypeVar(TypeVar typeVar, S s);

        R visitUndetVar(UndetVar undetVar, S s);

        R visitWildcardType(WildcardType wildcardType, S s);
    }

    public abstract TypeTag getTag();

    public boolean hasTag(TypeTag tag) {
        return tag == getTag();
    }

    public boolean isNumeric() {
        return false;
    }

    public boolean isPrimitive() {
        return false;
    }

    public boolean isPrimitiveOrVoid() {
        return false;
    }

    public boolean isReference() {
        return false;
    }

    public boolean isNullOrReference() {
        return false;
    }

    public boolean isPartial() {
        return false;
    }

    public Object constValue() {
        return null;
    }

    public boolean isFalse() {
        return false;
    }

    public boolean isTrue() {
        return false;
    }

    public Type getModelType() {
        return this;
    }

    public static List<Type> getModelTypes(List<Type> ts) {
        ListBuffer<Type> lb = new ListBuffer<>();
        for (Type t : ts) {
            lb.append(t.getModelType());
        }
        return lb.toList();
    }

    public Type getOriginalType() {
        return this;
    }

    public <R, S> R accept(Visitor<R, S> v, S s) {
        return v.visitType(this, s);
    }

    public Type(Symbol.TypeSymbol tsym) {
        this.tsym = tsym;
    }

    public static abstract class Mapping {
        private String name;

        public abstract Type apply(Type type);

        public Mapping(String name) {
            this.name = name;
        }

        public String toString() {
            return this.name;
        }
    }

    public Type map(Mapping f) {
        return this;
    }

    public static List<Type> map(List<Type> ts, Mapping f) {
        if (ts.nonEmpty()) {
            List<Type> tail1 = map(ts.tail, f);
            Type t = f.apply(ts.head);
            if (tail1 != ts.tail || t != ts.head) {
                return tail1.prepend(t);
            }
        }
        return ts;
    }

    public Type constType(Object constValue) {
        throw new AssertionError();
    }

    public Type baseType() {
        return this;
    }

    public Type annotatedType(List<Attribute.TypeCompound> annos) {
        return new AnnotatedType(annos, this);
    }

    public boolean isAnnotated() {
        return false;
    }

    public Type unannotatedType() {
        return this;
    }

    @Override // com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
    public List<Attribute.TypeCompound> getAnnotationMirrors() {
        return List.nil();
    }

    @Override // com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
    public <A extends Annotation> A getAnnotation(Class<A> annotationType) {
        return null;
    }

    @Override // com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
    public <A extends Annotation> A[] getAnnotationsByType(Class<A> cls) {
        return (A[]) ((Annotation[]) Array.newInstance((Class<?>) cls, 0));
    }

    public static List<Type> baseTypes(List<Type> ts) {
        if (ts.nonEmpty()) {
            Type t = ts.head.baseType();
            List<Type> baseTypes = baseTypes(ts.tail);
            if (t != ts.head || baseTypes != ts.tail) {
                return baseTypes.prepend(t);
            }
        }
        return ts;
    }

    @Override // javax.lang.model.type.TypeMirror
    public String toString() {
        String s;
        if (this.tsym == null || this.tsym.name == null) {
            s = "<none>";
        } else {
            s = this.tsym.name.toString();
        }
        if (moreInfo && hasTag(TypeTag.TYPEVAR)) {
            return s + hashCode();
        }
        return s;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static String toString(List<Type> ts) {
        if (ts.isEmpty()) {
            return "";
        }
        StringBuilder buf = new StringBuilder();
        buf.append(ts.head.toString());
        for (List list = ts.tail; list.nonEmpty(); list = list.tail) {
            buf.append(DocLint.TAGS_SEPARATOR).append(((Type) list.head).toString());
        }
        return buf.toString();
    }

    public String stringValue() {
        Object cv = Assert.checkNonNull(constValue());
        return cv.toString();
    }

    @Override // javax.lang.model.type.TypeMirror
    public boolean equals(Object t) {
        return super.equals(t);
    }

    @Override // javax.lang.model.type.TypeMirror
    public int hashCode() {
        return super.hashCode();
    }

    /* JADX WARN: Multi-variable type inference failed */
    public String argtypes(boolean varargs) {
        List listMo176getParameterTypes = mo176getParameterTypes();
        if (!varargs) {
            return listMo176getParameterTypes.toString();
        }
        StringBuilder buf = new StringBuilder();
        while (listMo176getParameterTypes.tail.nonEmpty()) {
            buf.append(listMo176getParameterTypes.head);
            listMo176getParameterTypes = listMo176getParameterTypes.tail;
            buf.append(',');
        }
        if (((Type) listMo176getParameterTypes.head).unannotatedType().hasTag(TypeTag.ARRAY)) {
            buf.append(((ArrayType) ((Type) listMo176getParameterTypes.head).unannotatedType()).elemtype);
            if (((Type) listMo176getParameterTypes.head).getAnnotationMirrors().nonEmpty()) {
                buf.append(((Type) listMo176getParameterTypes.head).getAnnotationMirrors());
            }
            buf.append("...");
        } else {
            buf.append(listMo176getParameterTypes.head);
        }
        return buf.toString();
    }

    public List<Type> getTypeArguments() {
        return List.nil();
    }

    public Type getEnclosingType() {
        return null;
    }

    /* JADX INFO: renamed from: getParameterTypes */
    public List<Type> mo176getParameterTypes() {
        return List.nil();
    }

    /* JADX INFO: renamed from: getReturnType */
    public Type mo178getReturnType() {
        return null;
    }

    /* JADX INFO: renamed from: getReceiverType */
    public Type mo177getReceiverType() {
        return null;
    }

    /* JADX INFO: renamed from: getThrownTypes */
    public List<Type> mo179getThrownTypes() {
        return List.nil();
    }

    public Type getUpperBound() {
        return null;
    }

    public Type getLowerBound() {
        return null;
    }

    public List<Type> allparams() {
        return List.nil();
    }

    public boolean isErroneous() {
        return false;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static boolean isErroneous(List<Type> ts) {
        for (List list = ts; list.nonEmpty(); list = list.tail) {
            if (((Type) list.head).isErroneous()) {
                return true;
            }
        }
        return false;
    }

    public boolean isParameterized() {
        return false;
    }

    public boolean isRaw() {
        return false;
    }

    public boolean isCompound() {
        return this.tsym.completer == null && (this.tsym.flags() & 16777216) != 0;
    }

    public boolean isIntersection() {
        return false;
    }

    public boolean isUnion() {
        return false;
    }

    public boolean isInterface() {
        return (this.tsym.flags() & 512) != 0;
    }

    public boolean isFinal() {
        return (this.tsym.flags() & 16) != 0;
    }

    public boolean contains(Type t) {
        return t == this;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static boolean contains(List<Type> ts, Type t) {
        for (List list = ts; list.tail != null; list = list.tail) {
            if (((Type) list.head).contains(t)) {
                return true;
            }
        }
        return false;
    }

    public boolean containsAny(List<Type> ts) {
        for (Type t : ts) {
            if (contains(t)) {
                return true;
            }
        }
        return false;
    }

    public static boolean containsAny(List<Type> ts1, List<Type> ts2) {
        for (Type t : ts1) {
            if (t.containsAny(ts2)) {
                return true;
            }
        }
        return false;
    }

    public static List<Type> filter(List<Type> ts, Filter<Type> tf) {
        ListBuffer<Type> buf = new ListBuffer<>();
        for (Type t : ts) {
            if (tf.accepts(t)) {
                buf.append(t);
            }
        }
        return buf.toList();
    }

    public boolean isSuperBound() {
        return false;
    }

    public boolean isExtendsBound() {
        return false;
    }

    public boolean isUnbound() {
        return false;
    }

    public Type withTypeVar(Type t) {
        return this;
    }

    public MethodType asMethodType() {
        throw new AssertionError();
    }

    public void complete() {
    }

    public Symbol.TypeSymbol asElement() {
        return this.tsym;
    }

    @Override // javax.lang.model.type.TypeMirror
    public TypeKind getKind() {
        return TypeKind.OTHER;
    }

    @Override // javax.lang.model.type.TypeMirror
    public <R, P> R accept(TypeVisitor<R, P> v, P p) {
        throw new AssertionError();
    }

    public static class JCPrimitiveType extends Type implements PrimitiveType {
        TypeTag tag;

        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public JCPrimitiveType(TypeTag tag, Symbol.TypeSymbol tsym) {
            super(tsym);
            this.tag = tag;
            Assert.check(tag.isPrimitive);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isNumeric() {
            return this.tag != TypeTag.BOOLEAN;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isPrimitive() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return this.tag;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isPrimitiveOrVoid() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public Type constType(final Object constValue) {
            return new JCPrimitiveType(this.tag, this.tsym) { // from class: com.sun.tools.javac.code.Type.JCPrimitiveType.1
                @Override // com.sun.tools.javac.code.Type.JCPrimitiveType, com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
                public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
                    return super.getAnnotationMirrors();
                }

                @Override // com.sun.tools.javac.code.Type
                public Object constValue() {
                    return constValue;
                }

                @Override // com.sun.tools.javac.code.Type
                public Type baseType() {
                    return this.tsym.type;
                }
            };
        }

        @Override // com.sun.tools.javac.code.Type
        public String stringValue() {
            Object cv = Assert.checkNonNull(constValue());
            if (this.tag == TypeTag.BOOLEAN) {
                return ((Integer) cv).intValue() == 0 ? "false" : "true";
            }
            if (this.tag == TypeTag.CHAR) {
                return String.valueOf((char) ((Integer) cv).intValue());
            }
            return cv.toString();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isFalse() {
            return this.tag == TypeTag.BOOLEAN && constValue() != null && ((Integer) constValue()).intValue() == 0;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isTrue() {
            return (this.tag != TypeTag.BOOLEAN || constValue() == null || ((Integer) constValue()).intValue() == 0) ? false : true;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitPrimitive(this, p);
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            switch (this.tag) {
                case BYTE:
                    return TypeKind.BYTE;
                case CHAR:
                    return TypeKind.CHAR;
                case SHORT:
                    return TypeKind.SHORT;
                case INT:
                    return TypeKind.INT;
                case LONG:
                    return TypeKind.LONG;
                case FLOAT:
                    return TypeKind.FLOAT;
                case DOUBLE:
                    return TypeKind.DOUBLE;
                case BOOLEAN:
                    return TypeKind.BOOLEAN;
                default:
                    throw new AssertionError();
            }
        }
    }

    public static class WildcardType extends Type implements javax.lang.model.type.WildcardType {
        public TypeVar bound;
        boolean isPrintingBound;
        public BoundKind kind;
        public Type type;

        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        @Override // com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitWildcardType(this, s);
        }

        public WildcardType(Type type, BoundKind kind, Symbol.TypeSymbol tsym) {
            super(tsym);
            this.isPrintingBound = false;
            this.type = (Type) Assert.checkNonNull(type);
            this.kind = kind;
        }

        public WildcardType(WildcardType t, TypeVar bound) {
            this(t.type, t.kind, t.tsym, bound);
        }

        public WildcardType(Type type, BoundKind kind, Symbol.TypeSymbol tsym, TypeVar bound) {
            this(type, kind, tsym);
            this.bound = bound;
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.WILDCARD;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean contains(Type t) {
            return this.kind != BoundKind.UNBOUND && this.type.contains(t);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isSuperBound() {
            return this.kind == BoundKind.SUPER || this.kind == BoundKind.UNBOUND;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isExtendsBound() {
            return this.kind == BoundKind.EXTENDS || this.kind == BoundKind.UNBOUND;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isUnbound() {
            return this.kind == BoundKind.UNBOUND;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isReference() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isNullOrReference() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public Type withTypeVar(Type t) {
            if (this.bound == t) {
                return this;
            }
            this.bound = (TypeVar) t;
            return this;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            StringBuilder s = new StringBuilder();
            s.append(this.kind.toString());
            if (this.kind != BoundKind.UNBOUND) {
                s.append(this.type);
            }
            if (moreInfo && this.bound != null && !this.isPrintingBound) {
                try {
                    this.isPrintingBound = true;
                    s.append("{:").append(this.bound.bound).append(":}");
                } finally {
                    this.isPrintingBound = false;
                }
            }
            return s.toString();
        }

        @Override // com.sun.tools.javac.code.Type
        public Type map(Mapping f) {
            Type t = this.type;
            if (t != null) {
                t = f.apply(t);
            }
            if (t == this.type) {
                return this;
            }
            return new WildcardType(t, this.kind, this.tsym, this.bound);
        }

        @Override // javax.lang.model.type.WildcardType
        public Type getExtendsBound() {
            if (this.kind == BoundKind.EXTENDS) {
                return this.type;
            }
            return null;
        }

        @Override // javax.lang.model.type.WildcardType
        public Type getSuperBound() {
            if (this.kind == BoundKind.SUPER) {
                return this.type;
            }
            return null;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.WILDCARD;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitWildcard(this, p);
        }
    }

    public static class ClassType extends Type implements DeclaredType {
        public List<Type> all_interfaces_field;
        public List<Type> allparams_field;
        public List<Type> interfaces_field;
        private Type outer_field;
        int rank_field;
        public Type supertype_field;
        public List<Type> typarams_field;

        @Override // javax.lang.model.type.DeclaredType, javax.lang.model.type.TypeVariable
        public /* bridge */ /* synthetic */ Element asElement() {
            return super.asElement();
        }

        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public ClassType(Type outer, List<Type> typarams, Symbol.TypeSymbol tsym) {
            super(tsym);
            this.rank_field = -1;
            this.outer_field = outer;
            this.typarams_field = typarams;
            this.allparams_field = null;
            this.supertype_field = null;
            this.interfaces_field = null;
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.CLASS;
        }

        @Override // com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitClassType(this, s);
        }

        @Override // com.sun.tools.javac.code.Type
        public Type constType(final Object constValue) {
            return new ClassType(getEnclosingType(), this.typarams_field, this.tsym) { // from class: com.sun.tools.javac.code.Type.ClassType.1
                @Override // com.sun.tools.javac.code.Type.ClassType, javax.lang.model.type.DeclaredType, javax.lang.model.type.TypeVariable
                public /* bridge */ /* synthetic */ Element asElement() {
                    return super.asElement();
                }

                @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
                public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
                    return super.getAnnotationMirrors();
                }

                @Override // com.sun.tools.javac.code.Type.ClassType, javax.lang.model.type.DeclaredType
                public /* bridge */ /* synthetic */ TypeMirror getEnclosingType() {
                    return super.getEnclosingType();
                }

                @Override // com.sun.tools.javac.code.Type.ClassType, javax.lang.model.type.DeclaredType
                public /* bridge */ /* synthetic */ java.util.List getTypeArguments() {
                    return super.getTypeArguments();
                }

                @Override // com.sun.tools.javac.code.Type
                public Object constValue() {
                    return constValue;
                }

                @Override // com.sun.tools.javac.code.Type
                public Type baseType() {
                    return this.tsym.type;
                }
            };
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            StringBuilder buf = new StringBuilder();
            if (getEnclosingType().hasTag(TypeTag.CLASS) && this.tsym.owner.kind == 2) {
                buf.append(getEnclosingType().toString());
                buf.append(".");
                buf.append(className(this.tsym, false));
            } else {
                buf.append(className(this.tsym, true));
            }
            if (getTypeArguments().nonEmpty()) {
                buf.append('<');
                buf.append(getTypeArguments().toString());
                buf.append(">");
            }
            return buf.toString();
        }

        /* JADX WARN: Multi-variable type inference failed */
        private String className(Symbol sym, boolean longform) {
            String s;
            if (sym.name.isEmpty() && (sym.flags() & 16777216) != 0) {
                StringBuilder s2 = new StringBuilder(this.supertype_field.toString());
                for (List list = this.interfaces_field; list.nonEmpty(); list = list.tail) {
                    s2.append("&");
                    s2.append(((Type) list.head).toString());
                }
                return s2.toString();
            }
            if (sym.name.isEmpty()) {
                ClassType norm = (ClassType) this.tsym.type.unannotatedType();
                if (norm == null) {
                    s = Log.getLocalizedString("anonymous.class", null);
                } else {
                    s = (norm.interfaces_field == null || !norm.interfaces_field.nonEmpty()) ? Log.getLocalizedString("anonymous.class", norm.supertype_field) : Log.getLocalizedString("anonymous.class", norm.interfaces_field.head);
                }
                if (moreInfo) {
                    return s + String.valueOf(sym.hashCode());
                }
                return s;
            }
            if (longform) {
                return sym.getQualifiedName().toString();
            }
            return sym.name.toString();
        }

        @Override // javax.lang.model.type.DeclaredType
        public List<Type> getTypeArguments() {
            if (this.typarams_field == null) {
                complete();
                if (this.typarams_field == null) {
                    this.typarams_field = List.nil();
                }
            }
            return this.typarams_field;
        }

        public boolean hasErasedSupertypes() {
            return isRaw();
        }

        @Override // javax.lang.model.type.DeclaredType
        public Type getEnclosingType() {
            return this.outer_field;
        }

        public void setEnclosingType(Type outer) {
            this.outer_field = outer;
        }

        @Override // com.sun.tools.javac.code.Type
        public List<Type> allparams() {
            if (this.allparams_field == null) {
                this.allparams_field = getTypeArguments().prependList(getEnclosingType().allparams());
            }
            return this.allparams_field;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isErroneous() {
            return getEnclosingType().isErroneous() || isErroneous(getTypeArguments()) || (this != this.tsym.type.unannotatedType() && this.tsym.type.isErroneous());
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isParameterized() {
            return allparams().tail != null;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isReference() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isNullOrReference() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isRaw() {
            return this != this.tsym.type && this.tsym.type.allparams().nonEmpty() && allparams().isEmpty();
        }

        @Override // com.sun.tools.javac.code.Type
        public Type map(Mapping f) {
            Type outer = getEnclosingType();
            Type outer1 = f.apply(outer);
            List<Type> typarams = getTypeArguments();
            List<Type> typarams1 = map(typarams, f);
            return (outer1 == outer && typarams1 == typarams) ? this : new ClassType(outer1, typarams1, this.tsym);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean contains(Type elem) {
            return elem == this || (isParameterized() && (getEnclosingType().contains(elem) || contains(getTypeArguments(), elem))) || (isCompound() && (this.supertype_field.contains(elem) || contains(this.interfaces_field, elem)));
        }

        @Override // com.sun.tools.javac.code.Type
        public void complete() {
            if (this.tsym.completer != null) {
                this.tsym.complete();
            }
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.DECLARED;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitDeclared(this, p);
        }
    }

    public static class ErasedClassType extends ClassType {
        public ErasedClassType(Type outer, Symbol.TypeSymbol tsym) {
            super(outer, List.nil(), tsym);
        }

        @Override // com.sun.tools.javac.code.Type.ClassType
        public boolean hasErasedSupertypes() {
            return true;
        }
    }

    public static class UnionClassType extends ClassType implements UnionType {
        final List<? extends Type> alternatives_field;

        public UnionClassType(ClassType ct, List<? extends Type> alternatives) {
            super(ct.outer_field, ct.typarams_field, ct.tsym);
            this.allparams_field = ct.allparams_field;
            this.supertype_field = ct.supertype_field;
            this.interfaces_field = ct.interfaces_field;
            this.all_interfaces_field = ct.interfaces_field;
            this.alternatives_field = alternatives;
        }

        public Type getLub() {
            return this.tsym.type;
        }

        @Override // javax.lang.model.type.UnionType
        public java.util.List<? extends TypeMirror> getAlternatives() {
            return Collections.unmodifiableList(this.alternatives_field);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isUnion() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.UNION;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitUnion(this, p);
        }
    }

    public static class IntersectionClassType extends ClassType implements IntersectionType {
        public boolean allInterfaces;

        public IntersectionClassType(List<Type> bounds, Symbol.ClassSymbol csym, boolean allInterfaces) {
            super(Type.noType, List.nil(), csym);
            this.allInterfaces = allInterfaces;
            boolean z = true;
            Assert.check((csym.flags() & 16777216) != 0);
            this.supertype_field = bounds.head;
            this.interfaces_field = bounds.tail;
            if (this.supertype_field.tsym.completer == null && this.supertype_field.isInterface()) {
                z = false;
            }
            Assert.check(z, this.supertype_field);
        }

        @Override // javax.lang.model.type.IntersectionType
        public java.util.List<? extends TypeMirror> getBounds() {
            return Collections.unmodifiableList(getExplicitComponents());
        }

        public List<Type> getComponents() {
            return this.interfaces_field.prepend(this.supertype_field);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isIntersection() {
            return true;
        }

        public List<Type> getExplicitComponents() {
            return this.allInterfaces ? this.interfaces_field : getComponents();
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.INTERSECTION;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitIntersection(this, p);
        }
    }

    public static class ArrayType extends Type implements javax.lang.model.type.ArrayType {
        public Type elemtype;

        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public ArrayType(Type elemtype, Symbol.TypeSymbol arrayClass) {
            super(arrayClass);
            this.elemtype = elemtype;
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.ARRAY;
        }

        @Override // com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitArrayType(this, s);
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return this.elemtype + "[]";
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public boolean equals(Object obj) {
            return this == obj || ((obj instanceof ArrayType) && this.elemtype.equals(((ArrayType) obj).elemtype));
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public int hashCode() {
            return (TypeTag.ARRAY.ordinal() << 5) + this.elemtype.hashCode();
        }

        public boolean isVarargs() {
            return false;
        }

        @Override // com.sun.tools.javac.code.Type
        public List<Type> allparams() {
            return this.elemtype.allparams();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isErroneous() {
            return this.elemtype.isErroneous();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isParameterized() {
            return this.elemtype.isParameterized();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isReference() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isNullOrReference() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isRaw() {
            return this.elemtype.isRaw();
        }

        public ArrayType makeVarargs() {
            return new ArrayType(this.elemtype, this.tsym) { // from class: com.sun.tools.javac.code.Type.ArrayType.1
                @Override // com.sun.tools.javac.code.Type.ArrayType, com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
                public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
                    return super.getAnnotationMirrors();
                }

                @Override // com.sun.tools.javac.code.Type.ArrayType, javax.lang.model.type.ArrayType
                public /* bridge */ /* synthetic */ TypeMirror getComponentType() {
                    return super.getComponentType();
                }

                @Override // com.sun.tools.javac.code.Type.ArrayType
                public boolean isVarargs() {
                    return true;
                }
            };
        }

        @Override // com.sun.tools.javac.code.Type
        public Type map(Mapping f) {
            Type elemtype1 = f.apply(this.elemtype);
            return elemtype1 == this.elemtype ? this : new ArrayType(elemtype1, this.tsym);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean contains(Type elem) {
            return elem == this || this.elemtype.contains(elem);
        }

        @Override // com.sun.tools.javac.code.Type
        public void complete() {
            this.elemtype.complete();
        }

        @Override // javax.lang.model.type.ArrayType
        public Type getComponentType() {
            return this.elemtype;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.ARRAY;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitArray(this, p);
        }
    }

    public static class MethodType extends Type implements ExecutableType {
        public List<Type> argtypes;
        public Type recvtype;
        public Type restype;
        public List<Type> thrown;

        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public MethodType(List<Type> argtypes, Type restype, List<Type> thrown, Symbol.TypeSymbol methodClass) {
            super(methodClass);
            this.argtypes = argtypes;
            this.restype = restype;
            this.thrown = thrown;
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.METHOD;
        }

        @Override // com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitMethodType(this, s);
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return "(" + this.argtypes + ")" + this.restype;
        }

        @Override // javax.lang.model.type.ExecutableType
        /* JADX INFO: renamed from: getParameterTypes, reason: merged with bridge method [inline-methods] */
        public List<Type> mo176getParameterTypes() {
            return this.argtypes;
        }

        @Override // javax.lang.model.type.ExecutableType
        /* JADX INFO: renamed from: getReturnType, reason: merged with bridge method [inline-methods] */
        public Type mo178getReturnType() {
            return this.restype;
        }

        @Override // javax.lang.model.type.ExecutableType
        /* JADX INFO: renamed from: getReceiverType, reason: merged with bridge method [inline-methods] */
        public Type mo177getReceiverType() {
            return this.recvtype;
        }

        @Override // javax.lang.model.type.ExecutableType
        /* JADX INFO: renamed from: getThrownTypes, reason: merged with bridge method [inline-methods] */
        public List<Type> mo179getThrownTypes() {
            return this.thrown;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isErroneous() {
            return isErroneous(this.argtypes) || (this.restype != null && this.restype.isErroneous());
        }

        @Override // com.sun.tools.javac.code.Type
        public Type map(Mapping f) {
            List<Type> argtypes1 = map(this.argtypes, f);
            Type restype1 = f.apply(this.restype);
            List<Type> thrown1 = map(this.thrown, f);
            if (argtypes1 == this.argtypes && restype1 == this.restype && thrown1 == this.thrown) {
                return this;
            }
            return new MethodType(argtypes1, restype1, thrown1, this.tsym);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean contains(Type elem) {
            return elem == this || contains(this.argtypes, elem) || this.restype.contains(elem) || contains(this.thrown, elem);
        }

        @Override // com.sun.tools.javac.code.Type
        public MethodType asMethodType() {
            return this;
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Type
        public void complete() {
            for (List list = this.argtypes; list.nonEmpty(); list = list.tail) {
                ((Type) list.head).complete();
            }
            this.restype.complete();
            this.recvtype.complete();
            for (List list2 = this.thrown; list2.nonEmpty(); list2 = list2.tail) {
                ((Type) list2.head).complete();
            }
        }

        @Override // javax.lang.model.type.ExecutableType
        public List<TypeVar> getTypeVariables() {
            return List.nil();
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.DeclaredType, javax.lang.model.type.TypeVariable
        public Symbol.TypeSymbol asElement() {
            return null;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.EXECUTABLE;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitExecutable(this, p);
        }
    }

    public static class PackageType extends Type implements NoType {
        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        PackageType(Symbol.TypeSymbol tsym) {
            super(tsym);
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.PACKAGE;
        }

        @Override // com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitPackageType(this, s);
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return this.tsym.getQualifiedName().toString();
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.PACKAGE;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitNoType(this, p);
        }
    }

    public static class TypeVar extends Type implements TypeVariable {
        public Type bound;
        public Type lower;
        int rank_field;

        @Override // javax.lang.model.type.TypeVariable
        public /* bridge */ /* synthetic */ Element asElement() {
            return super.asElement();
        }

        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public TypeVar(Name name, Symbol owner, Type lower) {
            super(null);
            this.bound = null;
            this.rank_field = -1;
            this.tsym = new Symbol.TypeVariableSymbol(0L, name, this, owner);
            this.lower = lower;
        }

        public TypeVar(Symbol.TypeSymbol tsym, Type bound, Type lower) {
            super(tsym);
            this.bound = null;
            this.rank_field = -1;
            this.bound = bound;
            this.lower = lower;
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.TYPEVAR;
        }

        @Override // com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitTypeVar(this, s);
        }

        @Override // javax.lang.model.type.TypeVariable
        public Type getUpperBound() {
            if ((this.bound == null || this.bound.hasTag(TypeTag.NONE)) && this != this.tsym.type) {
                this.bound = this.tsym.type.getUpperBound();
            }
            return this.bound;
        }

        @Override // javax.lang.model.type.TypeVariable
        public Type getLowerBound() {
            return this.lower;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.TYPEVAR;
        }

        public boolean isCaptured() {
            return false;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isReference() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isNullOrReference() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitTypeVariable(this, p);
        }
    }

    public static class CapturedType extends TypeVar {
        public WildcardType wildcard;

        public CapturedType(Name name, Symbol owner, Type upper, Type lower, WildcardType wildcard) {
            super(name, owner, lower);
            this.lower = (Type) Assert.checkNonNull(lower);
            this.bound = upper;
            this.wildcard = wildcard;
        }

        @Override // com.sun.tools.javac.code.Type.TypeVar, com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitCapturedType(this, s);
        }

        @Override // com.sun.tools.javac.code.Type.TypeVar
        public boolean isCaptured() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return "capture#" + ((((long) hashCode()) & 4294967295L) % 997) + " of " + this.wildcard;
        }
    }

    public static abstract class DelegatedType extends Type {
        public Type qtype;
        public TypeTag tag;

        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public DelegatedType(TypeTag tag, Type qtype) {
            super(qtype.tsym);
            this.tag = tag;
            this.qtype = qtype;
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return this.tag;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return this.qtype.toString();
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.DeclaredType
        public List<Type> getTypeArguments() {
            return this.qtype.getTypeArguments();
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.DeclaredType
        public Type getEnclosingType() {
            return this.qtype.getEnclosingType();
        }

        @Override // com.sun.tools.javac.code.Type
        /* JADX INFO: renamed from: getParameterTypes */
        public List<Type> mo176getParameterTypes() {
            return this.qtype.mo176getParameterTypes();
        }

        @Override // com.sun.tools.javac.code.Type
        /* JADX INFO: renamed from: getReturnType */
        public Type mo178getReturnType() {
            return this.qtype.mo178getReturnType();
        }

        @Override // com.sun.tools.javac.code.Type
        /* JADX INFO: renamed from: getReceiverType */
        public Type mo177getReceiverType() {
            return this.qtype.mo177getReceiverType();
        }

        @Override // com.sun.tools.javac.code.Type
        /* JADX INFO: renamed from: getThrownTypes */
        public List<Type> mo179getThrownTypes() {
            return this.qtype.mo179getThrownTypes();
        }

        @Override // com.sun.tools.javac.code.Type
        public List<Type> allparams() {
            return this.qtype.allparams();
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeVariable
        public Type getUpperBound() {
            return this.qtype.getUpperBound();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isErroneous() {
            return this.qtype.isErroneous();
        }
    }

    public static class ForAll extends DelegatedType implements ExecutableType {
        public List<Type> tvars;

        @Override // javax.lang.model.type.ExecutableType
        /* JADX INFO: renamed from: getParameterTypes */
        public /* bridge */ /* synthetic */ java.util.List mo176getParameterTypes() {
            return super.mo176getParameterTypes();
        }

        @Override // javax.lang.model.type.ExecutableType
        /* JADX INFO: renamed from: getReceiverType */
        public /* bridge */ /* synthetic */ TypeMirror mo177getReceiverType() {
            return super.mo177getReceiverType();
        }

        @Override // javax.lang.model.type.ExecutableType
        /* JADX INFO: renamed from: getReturnType */
        public /* bridge */ /* synthetic */ TypeMirror mo178getReturnType() {
            return super.mo178getReturnType();
        }

        @Override // javax.lang.model.type.ExecutableType
        /* JADX INFO: renamed from: getThrownTypes */
        public /* bridge */ /* synthetic */ java.util.List mo179getThrownTypes() {
            return super.mo179getThrownTypes();
        }

        public ForAll(List<Type> tvars, Type qtype) {
            super(TypeTag.FORALL, (MethodType) qtype);
            this.tvars = tvars;
        }

        @Override // com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitForAll(this, s);
        }

        @Override // com.sun.tools.javac.code.Type.DelegatedType, com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return "<" + this.tvars + ">" + this.qtype;
        }

        @Override // com.sun.tools.javac.code.Type.DelegatedType, com.sun.tools.javac.code.Type, javax.lang.model.type.DeclaredType
        public List<Type> getTypeArguments() {
            return this.tvars;
        }

        @Override // com.sun.tools.javac.code.Type.DelegatedType, com.sun.tools.javac.code.Type
        public boolean isErroneous() {
            return this.qtype.isErroneous();
        }

        @Override // com.sun.tools.javac.code.Type
        public Type map(Mapping f) {
            return f.apply(this.qtype);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean contains(Type elem) {
            return this.qtype.contains(elem);
        }

        @Override // com.sun.tools.javac.code.Type
        public MethodType asMethodType() {
            return (MethodType) this.qtype;
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Type
        public void complete() {
            for (List list = this.tvars; list.nonEmpty(); list = list.tail) {
                ((TypeVar) list.head).bound.complete();
            }
            this.qtype.complete();
        }

        @Override // javax.lang.model.type.ExecutableType
        public List<TypeVar> getTypeVariables() {
            return List.convert(TypeVar.class, getTypeArguments());
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.EXECUTABLE;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitExecutable(this, p);
        }
    }

    public static class UndetVar extends DelegatedType {
        protected Map<InferenceBound, List<Type>> bounds;
        public int declaredCount;
        public Type inst;
        public UndetVarListener listener;
        Mapping toTypeVarMap;

        public enum InferenceBound {
            UPPER { // from class: com.sun.tools.javac.code.Type.UndetVar.InferenceBound.1
                @Override // com.sun.tools.javac.code.Type.UndetVar.InferenceBound
                public InferenceBound complement() {
                    return LOWER;
                }
            },
            LOWER { // from class: com.sun.tools.javac.code.Type.UndetVar.InferenceBound.2
                @Override // com.sun.tools.javac.code.Type.UndetVar.InferenceBound
                public InferenceBound complement() {
                    return UPPER;
                }
            },
            EQ { // from class: com.sun.tools.javac.code.Type.UndetVar.InferenceBound.3
                @Override // com.sun.tools.javac.code.Type.UndetVar.InferenceBound
                public InferenceBound complement() {
                    return EQ;
                }
            };

            public abstract InferenceBound complement();
        }

        public interface UndetVarListener {
            void varChanged(UndetVar undetVar, Set<InferenceBound> set);
        }

        @Override // com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitUndetVar(this, s);
        }

        public UndetVar(TypeVar origin, Types types) {
            super(TypeTag.UNDETVAR, origin);
            this.inst = null;
            this.listener = null;
            this.toTypeVarMap = new Mapping("toTypeVarMap") { // from class: com.sun.tools.javac.code.Type.UndetVar.1
                @Override // com.sun.tools.javac.code.Type.Mapping
                public Type apply(Type t) {
                    if (t.hasTag(TypeTag.UNDETVAR)) {
                        UndetVar uv = (UndetVar) t;
                        return uv.inst != null ? uv.inst : uv.qtype;
                    }
                    return t.map(this);
                }
            };
            this.bounds = new EnumMap(InferenceBound.class);
            List<Type> declaredBounds = types.getBounds(origin);
            this.declaredCount = declaredBounds.length();
            this.bounds.put(InferenceBound.UPPER, declaredBounds);
            this.bounds.put(InferenceBound.LOWER, List.nil());
            this.bounds.put(InferenceBound.EQ, List.nil());
        }

        @Override // com.sun.tools.javac.code.Type.DelegatedType, com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            return this.inst == null ? this.qtype + "?" : this.inst.toString();
        }

        public String debugString() {
            String result = "inference var = " + this.qtype + "\n";
            if (this.inst != null) {
                result = result + "inst = " + this.inst + '\n';
            }
            for (InferenceBound bound : InferenceBound.values()) {
                List<Type> aboundList = this.bounds.get(bound);
                if (aboundList.size() > 0) {
                    result = result + bound + " = " + aboundList + '\n';
                }
            }
            return result;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isPartial() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public Type baseType() {
            return this.inst == null ? this : this.inst.baseType();
        }

        public List<Type> getBounds(InferenceBound... ibs) {
            ListBuffer<Type> buf = new ListBuffer<>();
            for (InferenceBound ib : ibs) {
                buf.appendList(this.bounds.get(ib));
            }
            return buf.toList();
        }

        public List<Type> getDeclaredBounds() {
            ListBuffer<Type> buf = new ListBuffer<>();
            int count = 0;
            for (Type b : getBounds(InferenceBound.UPPER)) {
                int count2 = count + 1;
                if (count == this.declaredCount) {
                    break;
                }
                buf.append(b);
                count = count2;
            }
            return buf.toList();
        }

        public void setBounds(InferenceBound ib, List<Type> newBounds) {
            this.bounds.put(ib, newBounds);
        }

        public final void addBound(InferenceBound ib, Type bound, Types types) {
            addBound(ib, bound, types, false);
        }

        protected void addBound(InferenceBound ib, Type bound, Types types, boolean update) {
            Type bound2 = this.toTypeVarMap.apply(bound).baseType();
            List<Type> prevBounds = this.bounds.get(ib);
            for (Type b : prevBounds) {
                if (types.isSameType(b, bound2, true) || bound == this.qtype) {
                    return;
                }
            }
            this.bounds.put(ib, prevBounds.prepend(bound2));
            notifyChange(EnumSet.of(ib));
        }

        public void substBounds(List<Type> from, List<Type> to, Types types) throws Throwable {
            List<Type> instVars = from.diff(to);
            if (instVars.isEmpty()) {
                return;
            }
            final EnumSet<InferenceBound> boundsChanged = EnumSet.noneOf(InferenceBound.class);
            UndetVarListener prevListener = this.listener;
            try {
                this.listener = new UndetVarListener() { // from class: com.sun.tools.javac.code.Type.UndetVar.2
                    @Override // com.sun.tools.javac.code.Type.UndetVar.UndetVarListener
                    public void varChanged(UndetVar uv, Set<InferenceBound> ibs) {
                        boundsChanged.addAll(ibs);
                    }
                };
                Iterator<Map.Entry<InferenceBound, List<Type>>> it = this.bounds.entrySet().iterator();
                while (it.hasNext()) {
                    Map.Entry<InferenceBound, List<Type>> _entry = it.next();
                    InferenceBound ib = _entry.getKey();
                    List<Type> prevBounds = _entry.getValue();
                    ListBuffer<Type> newBounds = new ListBuffer<>();
                    ListBuffer<Type> deps = new ListBuffer<>();
                    for (Type t : prevBounds) {
                        if (!t.containsAny(instVars)) {
                            newBounds.append(t);
                        } else {
                            deps.append(t);
                        }
                    }
                    this.bounds.put(ib, newBounds.toList());
                    for (Type dep : deps) {
                        try {
                            Iterator<Map.Entry<InferenceBound, List<Type>>> it2 = it;
                            addBound(ib, types.subst(dep, from, to), types, true);
                            it = it2;
                        } catch (Throwable th) {
                            th = th;
                            this.listener = prevListener;
                            if (!boundsChanged.isEmpty()) {
                                notifyChange(boundsChanged);
                            }
                            throw th;
                        }
                    }
                }
                this.listener = prevListener;
                if (!boundsChanged.isEmpty()) {
                    notifyChange(boundsChanged);
                }
            } catch (Throwable th2) {
                th = th2;
            }
        }

        private void notifyChange(EnumSet<InferenceBound> ibs) {
            if (this.listener != null) {
                this.listener.varChanged(this, ibs);
            }
        }

        public boolean isCaptured() {
            return false;
        }
    }

    public static class CapturedUndetVar extends UndetVar {
        public CapturedUndetVar(CapturedType origin, Types types) {
            super(origin, types);
            if (!origin.lower.hasTag(TypeTag.BOT)) {
                this.bounds.put(UndetVar.InferenceBound.LOWER, List.of(origin.lower));
            }
        }

        @Override // com.sun.tools.javac.code.Type.UndetVar
        public void addBound(UndetVar.InferenceBound ib, Type bound, Types types, boolean update) {
            if (update) {
                super.addBound(ib, bound, types, update);
            }
        }

        @Override // com.sun.tools.javac.code.Type.UndetVar
        public boolean isCaptured() {
            return true;
        }
    }

    public static class JCNoType extends Type implements NoType {
        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public JCNoType() {
            super(null);
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.NONE;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.NONE;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitNoType(this, p);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isCompound() {
            return false;
        }
    }

    public static class JCVoidType extends Type implements NoType {
        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public JCVoidType() {
            super(null);
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.VOID;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.VOID;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isCompound() {
            return false;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitNoType(this, p);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isPrimitiveOrVoid() {
            return true;
        }
    }

    static class BottomType extends Type implements NullType {
        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public BottomType() {
            super(null);
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.BOT;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.NULL;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isCompound() {
            return false;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitNull(this, p);
        }

        @Override // com.sun.tools.javac.code.Type
        public Type constType(Object value) {
            return this;
        }

        @Override // com.sun.tools.javac.code.Type
        public String stringValue() {
            return "null";
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isNullOrReference() {
            return true;
        }
    }

    public static class ErrorType extends ClassType implements javax.lang.model.type.ErrorType {
        private Type originalType;

        public ErrorType(Type originalType, Symbol.TypeSymbol tsym) {
            super(noType, List.nil(), null);
            this.originalType = null;
            this.tsym = tsym;
            this.originalType = originalType == null ? noType : originalType;
        }

        public ErrorType(Symbol.ClassSymbol c, Type originalType) {
            this(originalType, c);
            c.type = this;
            c.kind = 63;
            c.members_field = new Scope.ErrorScope(c);
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.ERROR;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isPartial() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type
        public boolean isReference() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type
        public boolean isNullOrReference() {
            return true;
        }

        public ErrorType(Name name, Symbol.TypeSymbol container, Type originalType) {
            this(new Symbol.ClassSymbol(1073741833L, name, null, container), originalType);
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitErrorType(this, s);
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type
        public Type constType(Object constValue) {
            return this;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, javax.lang.model.type.DeclaredType
        public Type getEnclosingType() {
            return this;
        }

        @Override // com.sun.tools.javac.code.Type
        /* JADX INFO: renamed from: getReturnType */
        public Type mo178getReturnType() {
            return this;
        }

        public Type asSub(Symbol sym) {
            return this;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type
        public Type map(Mapping f) {
            return this;
        }

        public boolean isGenType(Type t) {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type
        public boolean isErroneous() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isCompound() {
            return false;
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isInterface() {
            return false;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type
        public List<Type> allparams() {
            return List.nil();
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, javax.lang.model.type.DeclaredType
        public List<Type> getTypeArguments() {
            return List.nil();
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return TypeKind.ERROR;
        }

        @Override // com.sun.tools.javac.code.Type
        public Type getOriginalType() {
            return this.originalType;
        }

        @Override // com.sun.tools.javac.code.Type.ClassType, com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitError(this, p);
        }
    }

    public static class AnnotatedType extends Type implements javax.lang.model.type.ArrayType, DeclaredType, PrimitiveType, TypeVariable, javax.lang.model.type.WildcardType {
        private List<Attribute.TypeCompound> typeAnnotations;
        private Type underlyingType;

        protected AnnotatedType(List<Attribute.TypeCompound> typeAnnotations, Type underlyingType) {
            super(underlyingType.tsym);
            this.typeAnnotations = typeAnnotations;
            this.underlyingType = underlyingType;
            Assert.check(typeAnnotations != null && typeAnnotations.nonEmpty(), "Can't create AnnotatedType without annotations: " + underlyingType);
            Assert.check(true ^ underlyingType.isAnnotated(), "Can't annotate already annotated type: " + underlyingType + "; adding: " + typeAnnotations);
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return this.underlyingType.getTag();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isAnnotated() {
            return true;
        }

        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public List<Attribute.TypeCompound> getAnnotationMirrors() {
            return this.typeAnnotations;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public TypeKind getKind() {
            return this.underlyingType.getKind();
        }

        @Override // com.sun.tools.javac.code.Type
        public Type unannotatedType() {
            return this.underlyingType;
        }

        @Override // com.sun.tools.javac.code.Type
        public <R, S> R accept(Visitor<R, S> v, S s) {
            return v.visitAnnotatedType(this, s);
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> typeVisitor, P p) {
            return (R) this.underlyingType.accept(typeVisitor, p);
        }

        @Override // com.sun.tools.javac.code.Type
        public Type map(Mapping f) {
            this.underlyingType.map(f);
            return this;
        }

        @Override // com.sun.tools.javac.code.Type
        public Type constType(Object constValue) {
            return this.underlyingType.constType(constValue);
        }

        @Override // javax.lang.model.type.DeclaredType
        public Type getEnclosingType() {
            return this.underlyingType.getEnclosingType();
        }

        @Override // com.sun.tools.javac.code.Type
        /* JADX INFO: renamed from: getReturnType */
        public Type mo178getReturnType() {
            return this.underlyingType.mo178getReturnType();
        }

        @Override // javax.lang.model.type.DeclaredType
        public List<Type> getTypeArguments() {
            return this.underlyingType.getTypeArguments();
        }

        @Override // com.sun.tools.javac.code.Type
        /* JADX INFO: renamed from: getParameterTypes */
        public List<Type> mo176getParameterTypes() {
            return this.underlyingType.mo176getParameterTypes();
        }

        @Override // com.sun.tools.javac.code.Type
        /* JADX INFO: renamed from: getReceiverType */
        public Type mo177getReceiverType() {
            return this.underlyingType.mo177getReceiverType();
        }

        @Override // com.sun.tools.javac.code.Type
        /* JADX INFO: renamed from: getThrownTypes */
        public List<Type> mo179getThrownTypes() {
            return this.underlyingType.mo179getThrownTypes();
        }

        @Override // javax.lang.model.type.TypeVariable
        public Type getUpperBound() {
            return this.underlyingType.getUpperBound();
        }

        @Override // javax.lang.model.type.TypeVariable
        public Type getLowerBound() {
            return this.underlyingType.getLowerBound();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isErroneous() {
            return this.underlyingType.isErroneous();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isCompound() {
            return this.underlyingType.isCompound();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isInterface() {
            return this.underlyingType.isInterface();
        }

        @Override // com.sun.tools.javac.code.Type
        public List<Type> allparams() {
            return this.underlyingType.allparams();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isPrimitive() {
            return this.underlyingType.isPrimitive();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isPrimitiveOrVoid() {
            return this.underlyingType.isPrimitiveOrVoid();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isNumeric() {
            return this.underlyingType.isNumeric();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isReference() {
            return this.underlyingType.isReference();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isNullOrReference() {
            return this.underlyingType.isNullOrReference();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isPartial() {
            return this.underlyingType.isPartial();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isParameterized() {
            return this.underlyingType.isParameterized();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isRaw() {
            return this.underlyingType.isRaw();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isFinal() {
            return this.underlyingType.isFinal();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isSuperBound() {
            return this.underlyingType.isSuperBound();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isExtendsBound() {
            return this.underlyingType.isExtendsBound();
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isUnbound() {
            return this.underlyingType.isUnbound();
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public String toString() {
            if (this.typeAnnotations != null && !this.typeAnnotations.isEmpty()) {
                return "(" + this.typeAnnotations.toString() + " :: " + this.underlyingType.toString() + ")";
            }
            return "({} :: " + this.underlyingType.toString() + ")";
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean contains(Type t) {
            return this.underlyingType.contains(t);
        }

        @Override // com.sun.tools.javac.code.Type
        public Type withTypeVar(Type t) {
            this.underlyingType = this.underlyingType.withTypeVar(t);
            return this;
        }

        @Override // javax.lang.model.type.DeclaredType, javax.lang.model.type.TypeVariable
        public Symbol.TypeSymbol asElement() {
            return this.underlyingType.asElement();
        }

        @Override // com.sun.tools.javac.code.Type
        public MethodType asMethodType() {
            return this.underlyingType.asMethodType();
        }

        @Override // com.sun.tools.javac.code.Type
        public void complete() {
            this.underlyingType.complete();
        }

        @Override // javax.lang.model.type.ArrayType
        public TypeMirror getComponentType() {
            return ((ArrayType) this.underlyingType).getComponentType();
        }

        public Type makeVarargs() {
            return ((ArrayType) this.underlyingType).makeVarargs().annotatedType(this.typeAnnotations);
        }

        @Override // javax.lang.model.type.WildcardType
        public TypeMirror getExtendsBound() {
            return ((WildcardType) this.underlyingType).getExtendsBound();
        }

        @Override // javax.lang.model.type.WildcardType
        public TypeMirror getSuperBound() {
            return ((WildcardType) this.underlyingType).getSuperBound();
        }
    }

    public static class UnknownType extends Type {
        @Override // com.sun.tools.javac.code.Type, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        public UnknownType() {
            super(null);
        }

        @Override // com.sun.tools.javac.code.Type
        public TypeTag getTag() {
            return TypeTag.UNKNOWN;
        }

        @Override // com.sun.tools.javac.code.Type, javax.lang.model.type.TypeMirror
        public <R, P> R accept(TypeVisitor<R, P> v, P p) {
            return v.visitUnknown(this, p);
        }

        @Override // com.sun.tools.javac.code.Type
        public boolean isPartial() {
            return true;
        }
    }
}
