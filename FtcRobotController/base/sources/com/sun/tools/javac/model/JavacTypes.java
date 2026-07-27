package com.sun.tools.javac.model;

import com.sun.tools.javac.code.BoundKind;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.ListBuffer;
import java.util.Collections;
import java.util.EnumSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.TypeElement;
import javax.lang.model.type.ArrayType;
import javax.lang.model.type.DeclaredType;
import javax.lang.model.type.ExecutableType;
import javax.lang.model.type.NoType;
import javax.lang.model.type.NullType;
import javax.lang.model.type.PrimitiveType;
import javax.lang.model.type.ReferenceType;
import javax.lang.model.type.TypeKind;
import javax.lang.model.type.TypeMirror;
import javax.lang.model.type.WildcardType;
import javax.lang.model.util.Types;

/* JADX INFO: loaded from: classes.dex */
public class JavacTypes implements Types {
    private static final Set<TypeKind> EXEC_OR_PKG = EnumSet.of(TypeKind.EXECUTABLE, TypeKind.PACKAGE);
    private Symtab syms;
    private com.sun.tools.javac.code.Types types;

    public static JavacTypes instance(Context context) {
        JavacTypes instance = (JavacTypes) context.get(JavacTypes.class);
        if (instance == null) {
            return new JavacTypes(context);
        }
        return instance;
    }

    protected JavacTypes(Context context) {
        setContext(context);
    }

    public void setContext(Context context) {
        context.put((Class<JavacTypes>) JavacTypes.class, this);
        this.syms = Symtab.instance(context);
        this.types = com.sun.tools.javac.code.Types.instance(context);
    }

    @Override // javax.lang.model.util.Types
    public Element asElement(TypeMirror t) {
        switch (t.getKind()) {
            case DECLARED:
            case INTERSECTION:
            case ERROR:
            case TYPEVAR:
                Type type = (Type) cast(Type.class, t);
                return type.asElement();
            default:
                return null;
        }
    }

    @Override // javax.lang.model.util.Types
    public boolean isSameType(TypeMirror t1, TypeMirror t2) {
        return this.types.isSameType((Type) t1, (Type) t2);
    }

    @Override // javax.lang.model.util.Types
    public boolean isSubtype(TypeMirror t1, TypeMirror t2) {
        validateTypeNotIn(t1, EXEC_OR_PKG);
        validateTypeNotIn(t2, EXEC_OR_PKG);
        return this.types.isSubtype((Type) t1, (Type) t2);
    }

    @Override // javax.lang.model.util.Types
    public boolean isAssignable(TypeMirror t1, TypeMirror t2) {
        validateTypeNotIn(t1, EXEC_OR_PKG);
        validateTypeNotIn(t2, EXEC_OR_PKG);
        return this.types.isAssignable((Type) t1, (Type) t2);
    }

    @Override // javax.lang.model.util.Types
    public boolean contains(TypeMirror t1, TypeMirror t2) {
        validateTypeNotIn(t1, EXEC_OR_PKG);
        validateTypeNotIn(t2, EXEC_OR_PKG);
        return this.types.containsType((Type) t1, (Type) t2);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // javax.lang.model.util.Types
    public boolean isSubsignature(ExecutableType executableType, ExecutableType executableType2) {
        return this.types.isSubSignature((Type) executableType, (Type) executableType2);
    }

    @Override // javax.lang.model.util.Types
    public List<Type> directSupertypes(TypeMirror t) {
        validateTypeNotIn(t, EXEC_OR_PKG);
        return this.types.directSupertypes((Type) t);
    }

    @Override // javax.lang.model.util.Types
    public TypeMirror erasure(TypeMirror t) {
        if (t.getKind() == TypeKind.PACKAGE) {
            throw new IllegalArgumentException(t.toString());
        }
        return this.types.erasure((Type) t);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // javax.lang.model.util.Types
    public TypeElement boxedClass(PrimitiveType primitiveType) {
        return this.types.boxedClass((Type) primitiveType);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // javax.lang.model.util.Types
    public PrimitiveType unboxedType(TypeMirror t) {
        if (t.getKind() != TypeKind.DECLARED) {
            throw new IllegalArgumentException(t.toString());
        }
        Type typeUnboxedType = this.types.unboxedType((Type) t);
        if (!typeUnboxedType.isPrimitive()) {
            throw new IllegalArgumentException(t.toString());
        }
        return (PrimitiveType) typeUnboxedType;
    }

    @Override // javax.lang.model.util.Types
    public TypeMirror capture(TypeMirror t) {
        validateTypeNotIn(t, EXEC_OR_PKG);
        return this.types.capture((Type) t);
    }

    @Override // javax.lang.model.util.Types
    public PrimitiveType getPrimitiveType(TypeKind kind) {
        switch (kind) {
            case BOOLEAN:
                return this.syms.booleanType;
            case BYTE:
                return this.syms.byteType;
            case SHORT:
                return this.syms.shortType;
            case INT:
                return this.syms.intType;
            case LONG:
                return this.syms.longType;
            case CHAR:
                return this.syms.charType;
            case FLOAT:
                return this.syms.floatType;
            case DOUBLE:
                return this.syms.doubleType;
            default:
                throw new IllegalArgumentException("Not a primitive type: " + kind);
        }
    }

    @Override // javax.lang.model.util.Types
    public NullType getNullType() {
        return (NullType) this.syms.botType;
    }

    @Override // javax.lang.model.util.Types
    public NoType getNoType(TypeKind kind) {
        switch (kind) {
            case VOID:
                return this.syms.voidType;
            case NONE:
                return Type.noType;
            default:
                throw new IllegalArgumentException(kind.toString());
        }
    }

    @Override // javax.lang.model.util.Types
    public ArrayType getArrayType(TypeMirror componentType) {
        switch (componentType.getKind()) {
            case VOID:
            case EXECUTABLE:
            case WILDCARD:
            case PACKAGE:
                throw new IllegalArgumentException(componentType.toString());
            case NONE:
            default:
                return new Type.ArrayType((Type) componentType, this.syms.arrayClass);
        }
    }

    @Override // javax.lang.model.util.Types
    public WildcardType getWildcardType(TypeMirror extendsBound, TypeMirror superBound) {
        BoundKind bkind;
        Type bound;
        if (extendsBound == null && superBound == null) {
            bkind = BoundKind.UNBOUND;
            bound = this.syms.objectType;
        } else if (superBound == null) {
            bkind = BoundKind.EXTENDS;
            bound = (Type) extendsBound;
        } else if (extendsBound == null) {
            bkind = BoundKind.SUPER;
            bound = (Type) superBound;
        } else {
            throw new IllegalArgumentException("Extends and super bounds cannot both be provided");
        }
        switch (bound.getKind()) {
            case DECLARED:
            case ERROR:
            case TYPEVAR:
            case ARRAY:
                return new Type.WildcardType(bound, bkind, this.syms.boundClass);
            default:
                throw new IllegalArgumentException(bound.toString());
        }
    }

    @Override // javax.lang.model.util.Types
    public DeclaredType getDeclaredType(TypeElement typeElem, TypeMirror... typeArgs) {
        Symbol.ClassSymbol sym = (Symbol.ClassSymbol) typeElem;
        if (typeArgs.length == 0) {
            return (DeclaredType) sym.erasure(this.types);
        }
        if (sym.type.getEnclosingType().isParameterized()) {
            throw new IllegalArgumentException(sym.toString());
        }
        return getDeclaredType0(sym.type.getEnclosingType(), sym, typeArgs);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // javax.lang.model.util.Types
    public DeclaredType getDeclaredType(DeclaredType declaredType, TypeElement typeElem, TypeMirror... typeArgs) {
        if (declaredType == 0) {
            return getDeclaredType(typeElem, typeArgs);
        }
        Symbol.ClassSymbol sym = (Symbol.ClassSymbol) typeElem;
        Type outer = (Type) declaredType;
        if (outer.tsym != sym.owner.enclClass()) {
            throw new IllegalArgumentException(declaredType.toString());
        }
        if (!outer.isParameterized()) {
            return getDeclaredType(typeElem, typeArgs);
        }
        return getDeclaredType0(outer, sym, typeArgs);
    }

    private DeclaredType getDeclaredType0(Type outer, Symbol.ClassSymbol sym, TypeMirror... typeArgs) {
        if (typeArgs.length != sym.type.getTypeArguments().length()) {
            throw new IllegalArgumentException("Incorrect number of type arguments");
        }
        ListBuffer<Type> targs = new ListBuffer<>();
        for (TypeMirror t : typeArgs) {
            if (!(t instanceof ReferenceType) && !(t instanceof WildcardType)) {
                throw new IllegalArgumentException(t.toString());
            }
            targs.append((Type) t);
        }
        return new Type.ClassType(outer, targs.toList(), sym);
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // javax.lang.model.util.Types
    public TypeMirror asMemberOf(DeclaredType declaredType, Element element) {
        Type site = (Type) declaredType;
        Symbol sym = (Symbol) element;
        if (this.types.asSuper(site, sym.getEnclosingElement()) == null) {
            throw new IllegalArgumentException(sym + "@" + site);
        }
        return this.types.memberType(site, sym);
    }

    private void validateTypeNotIn(TypeMirror t, Set<TypeKind> invalidKinds) {
        if (invalidKinds.contains(t.getKind())) {
            throw new IllegalArgumentException(t.toString());
        }
    }

    private static <T> T cast(Class<T> clazz, Object o) {
        if (!clazz.isInstance(o)) {
            throw new IllegalArgumentException(o.toString());
        }
        return clazz.cast(o);
    }

    public Set<Symbol.MethodSymbol> getOverriddenMethods(Element elem) {
        if (elem.getKind() != ElementKind.METHOD || elem.getModifiers().contains(Modifier.STATIC) || elem.getModifiers().contains(Modifier.PRIVATE)) {
            return Collections.emptySet();
        }
        if (!(elem instanceof Symbol.MethodSymbol)) {
            throw new IllegalArgumentException();
        }
        Symbol.MethodSymbol m = (Symbol.MethodSymbol) elem;
        Symbol.ClassSymbol origin = (Symbol.ClassSymbol) m.owner;
        Set<Symbol.MethodSymbol> results = new LinkedHashSet<>();
        for (Type t : this.types.closure(origin.type)) {
            if (t != origin.type) {
                Symbol.ClassSymbol c = (Symbol.ClassSymbol) t.tsym;
                for (Scope.Entry e = c.members().lookup(m.name); e.scope != null; e = e.next()) {
                    if (e.sym.kind == 16 && m.overrides(e.sym, origin, this.types, true)) {
                        results.add((Symbol.MethodSymbol) e.sym);
                    }
                }
            }
        }
        return results;
    }
}
