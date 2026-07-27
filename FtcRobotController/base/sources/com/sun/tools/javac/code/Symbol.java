package com.sun.tools.javac.code;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.comp.Annotate;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.jvm.Code;
import com.sun.tools.javac.jvm.Pool;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Constants;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import dk.sgjesse.r8api.DescriptorUtils;
import java.lang.annotation.Annotation;
import java.lang.annotation.Inherited;
import java.util.Set;
import java.util.concurrent.Callable;
import javax.lang.model.element.AnnotationValue;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ElementVisitor;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.NestingKind;
import javax.lang.model.element.PackageElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.TypeParameterElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.type.TypeMirror;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public abstract class Symbol extends AnnoConstruct implements Element {
    public Completer completer = null;
    public Type erasure_field = null;
    public long flags_field;
    public int kind;
    protected SymbolMetadata metadata;
    public Name name;
    public Symbol owner;
    public Type type;

    public interface Completer {
        void complete(Symbol symbol) throws CompletionFailure;
    }

    public interface Visitor<R, P> {
        R visitClassSymbol(ClassSymbol classSymbol, P p);

        R visitMethodSymbol(MethodSymbol methodSymbol, P p);

        R visitOperatorSymbol(OperatorSymbol operatorSymbol, P p);

        R visitPackageSymbol(PackageSymbol packageSymbol, P p);

        R visitSymbol(Symbol symbol, P p);

        R visitTypeSymbol(TypeSymbol typeSymbol, P p);

        R visitVarSymbol(VarSymbol varSymbol, P p);
    }

    public long flags() {
        return this.flags_field;
    }

    public List<Attribute.Compound> getRawAttributes() {
        if (this.metadata == null) {
            return List.nil();
        }
        return this.metadata.getDeclarationAttributes();
    }

    public List<Attribute.TypeCompound> getRawTypeAttributes() {
        if (this.metadata == null) {
            return List.nil();
        }
        return this.metadata.getTypeAttributes();
    }

    public Attribute.Compound attribute(Symbol anno) {
        for (Attribute.Compound a : getRawAttributes()) {
            if (a.type.tsym == anno) {
                return a;
            }
        }
        return null;
    }

    public boolean annotationsPendingCompletion() {
        if (this.metadata == null) {
            return false;
        }
        return this.metadata.pendingCompletion();
    }

    public void appendAttributes(List<Attribute.Compound> l) {
        if (l.nonEmpty()) {
            initedMetadata().append(l);
        }
    }

    public void appendClassInitTypeAttributes(List<Attribute.TypeCompound> l) {
        if (l.nonEmpty()) {
            initedMetadata().appendClassInitTypeAttributes(l);
        }
    }

    public void appendInitTypeAttributes(List<Attribute.TypeCompound> l) {
        if (l.nonEmpty()) {
            initedMetadata().appendInitTypeAttributes(l);
        }
    }

    public void appendTypeAttributesWithCompletion(Annotate.AnnotateRepeatedContext<Attribute.TypeCompound> ctx) {
        initedMetadata().appendTypeAttributesWithCompletion(ctx);
    }

    public void appendUniqueTypeAttributes(List<Attribute.TypeCompound> l) {
        if (l.nonEmpty()) {
            initedMetadata().appendUniqueTypes(l);
        }
    }

    public List<Attribute.TypeCompound> getClassInitTypeAttributes() {
        if (this.metadata == null) {
            return List.nil();
        }
        return this.metadata.getClassInitTypeAttributes();
    }

    public List<Attribute.TypeCompound> getInitTypeAttributes() {
        if (this.metadata == null) {
            return List.nil();
        }
        return this.metadata.getInitTypeAttributes();
    }

    public List<Attribute.Compound> getDeclarationAttributes() {
        if (this.metadata == null) {
            return List.nil();
        }
        return this.metadata.getDeclarationAttributes();
    }

    public boolean hasAnnotations() {
        return (this.metadata == null || this.metadata.isEmpty()) ? false : true;
    }

    public boolean hasTypeAnnotations() {
        return (this.metadata == null || this.metadata.isTypesEmpty()) ? false : true;
    }

    public void prependAttributes(List<Attribute.Compound> l) {
        if (l.nonEmpty()) {
            initedMetadata().prepend(l);
        }
    }

    public void resetAnnotations() {
        initedMetadata().reset();
    }

    public void setAttributes(Symbol other) {
        if (this.metadata != null || other.metadata != null) {
            initedMetadata().setAttributes(other.metadata);
        }
    }

    public void setDeclarationAttributes(List<Attribute.Compound> a) {
        if (this.metadata != null || a.nonEmpty()) {
            initedMetadata().setDeclarationAttributes(a);
        }
    }

    public void setDeclarationAttributesWithCompletion(Annotate.AnnotateRepeatedContext<Attribute.Compound> ctx) {
        initedMetadata().setDeclarationAttributesWithCompletion(ctx);
    }

    public void setTypeAttributes(List<Attribute.TypeCompound> a) {
        if (this.metadata != null || a.nonEmpty()) {
            if (this.metadata == null) {
                this.metadata = new SymbolMetadata(this);
            }
            this.metadata.setTypeAttributes(a);
        }
    }

    private SymbolMetadata initedMetadata() {
        if (this.metadata == null) {
            this.metadata = new SymbolMetadata(this);
        }
        return this.metadata;
    }

    public SymbolMetadata getMetadata() {
        return this.metadata;
    }

    public Symbol(int kind, long flags, Name name, Type type, Symbol owner) {
        this.kind = kind;
        this.flags_field = flags;
        this.type = type;
        this.owner = owner;
        this.name = name;
    }

    public Symbol clone(Symbol newOwner) {
        throw new AssertionError();
    }

    public <R, P> R accept(Visitor<R, P> v, P p) {
        return v.visitSymbol(this, p);
    }

    public String toString() {
        return this.name.toString();
    }

    public Symbol location() {
        if (this.owner.name == null) {
            return null;
        }
        if (this.owner.name.isEmpty() && (this.owner.flags() & 1048576) == 0 && this.owner.kind != 1 && this.owner.kind != 2) {
            return null;
        }
        return this.owner;
    }

    public Symbol location(Type site, Types types) {
        Type ownertype;
        if (this.owner.name == null || this.owner.name.isEmpty()) {
            return location();
        }
        return (!this.owner.type.hasTag(TypeTag.CLASS) || (ownertype = types.asOuterSuper(site, this.owner)) == null) ? this.owner : ownertype.tsym;
    }

    public Symbol baseSymbol() {
        return this;
    }

    public Type erasure(Types types) {
        if (this.erasure_field == null) {
            this.erasure_field = types.erasure(this.type);
        }
        return this.erasure_field;
    }

    public Type externalType(Types types) {
        Type t = erasure(types);
        if (this.name == this.name.table.names.init && this.owner.hasOuterInstance()) {
            Type outerThisType = types.erasure(this.owner.type.getEnclosingType());
            return new Type.MethodType(t.mo176getParameterTypes().prepend(outerThisType), t.mo178getReturnType(), t.mo179getThrownTypes(), t.tsym);
        }
        return t;
    }

    public boolean isDeprecated() {
        return (this.flags_field & 131072) != 0;
    }

    public boolean isStatic() {
        return ((flags() & 8) == 0 && ((this.owner.flags() & 512) == 0 || this.kind == 16 || this.name == this.name.table.names._this)) ? false : true;
    }

    public boolean isInterface() {
        return (flags() & 512) != 0;
    }

    public boolean isPrivate() {
        return (this.flags_field & 7) == 2;
    }

    public boolean isEnum() {
        return (flags() & 16384) != 0;
    }

    public boolean isLocal() {
        return (this.owner.kind & 20) != 0 || (this.owner.kind == 2 && this.owner.isLocal());
    }

    public boolean isAnonymous() {
        return this.name.isEmpty();
    }

    public boolean isConstructor() {
        return this.name == this.name.table.names.init;
    }

    public Name getQualifiedName() {
        return this.name;
    }

    public Name flatName() {
        return getQualifiedName();
    }

    public Scope members() {
        return null;
    }

    public boolean isInner() {
        return this.kind == 2 && this.type.getEnclosingType().hasTag(TypeTag.CLASS);
    }

    public boolean hasOuterInstance() {
        return this.type.getEnclosingType().hasTag(TypeTag.CLASS) && (flags() & 4194816) == 0;
    }

    public ClassSymbol enclClass() {
        Symbol c = this;
        while (c != null && ((c.kind & 2) == 0 || !c.type.hasTag(TypeTag.CLASS))) {
            c = c.owner;
        }
        return (ClassSymbol) c;
    }

    public ClassSymbol outermostClass() {
        Symbol prev = null;
        for (Symbol sym = this; sym.kind != 1; sym = sym.owner) {
            prev = sym;
        }
        return (ClassSymbol) prev;
    }

    public PackageSymbol packge() {
        Symbol sym = this;
        while (sym.kind != 1) {
            sym = sym.owner;
        }
        return (PackageSymbol) sym;
    }

    public boolean isSubClass(Symbol base, Types types) {
        throw new AssertionError("isSubClass " + this);
    }

    public boolean isMemberOf(TypeSymbol clazz, Types types) {
        return this.owner == clazz || (clazz.isSubClass(this.owner, types) && isInheritedIn(clazz, types) && !hiddenIn((ClassSymbol) clazz, types));
    }

    public boolean isEnclosedBy(ClassSymbol clazz) {
        for (Symbol sym = this; sym.kind != 1; sym = sym.owner) {
            if (sym == clazz) {
                return true;
            }
        }
        return false;
    }

    private boolean hiddenIn(ClassSymbol clazz, Types types) {
        Symbol sym = hiddenInInternal(clazz, types);
        Assert.check(sym != null, "the result of hiddenInInternal() can't be null");
        return sym != this;
    }

    private Symbol hiddenInInternal(ClassSymbol currentClass, Types types) {
        if (currentClass == this.owner) {
            return this;
        }
        for (Scope.Entry e = currentClass.members().lookup(this.name); e.scope != null; e = e.next()) {
            if (e.sym.kind == this.kind && (this.kind != 16 || ((e.sym.flags() & 8) != 0 && types.isSubSignature(e.sym.type, this.type)))) {
                return e.sym;
            }
        }
        Symbol hiddenSym = null;
        for (Type st : types.interfaces(currentClass.type).prepend(types.supertype(currentClass.type))) {
            if (st != null && st.hasTag(TypeTag.CLASS)) {
                Symbol sym = hiddenInInternal((ClassSymbol) st.tsym, types);
                if (sym == this) {
                    return this;
                }
                if (sym != null) {
                    hiddenSym = sym;
                }
            }
        }
        return hiddenSym;
    }

    public boolean isInheritedIn(Symbol clazz, Types types) {
        switch ((int) (this.flags_field & 7)) {
            case 0:
                PackageSymbol thisPackage = packge();
                Symbol sup = clazz;
                while (sup != null && sup != this.owner) {
                    while (sup.type.hasTag(TypeTag.TYPEVAR)) {
                        sup = sup.type.getUpperBound().tsym;
                    }
                    if (!sup.type.isErroneous()) {
                        if ((sup.flags() & 16777216) != 0 || sup.packge() == thisPackage) {
                            sup = types.supertype(sup.type).tsym;
                        }
                    }
                    break;
                }
                if ((512 & clazz.flags()) == 0) {
                }
                break;
            case 2:
                if (this.owner == clazz) {
                }
                break;
            case 4:
                if ((clazz.flags() & 512) == 0) {
                }
                break;
        }
        return true;
    }

    public Symbol asMemberOf(Type site, Types types) {
        throw new AssertionError();
    }

    public boolean overrides(Symbol _other, TypeSymbol origin, Types types, boolean checkResult) {
        return false;
    }

    public void complete() throws CompletionFailure {
        if (this.completer != null) {
            Completer c = this.completer;
            this.completer = null;
            c.complete(this);
        }
    }

    public boolean exists() {
        return true;
    }

    @Override // javax.lang.model.element.Element
    public Type asType() {
        return this.type;
    }

    @Override // javax.lang.model.element.Element
    public Symbol getEnclosingElement() {
        return this.owner;
    }

    @Override // javax.lang.model.element.Element
    public ElementKind getKind() {
        return ElementKind.OTHER;
    }

    @Override // javax.lang.model.element.Element
    public Set<Modifier> getModifiers() {
        return Flags.asModifierSet(flags());
    }

    @Override // javax.lang.model.element.Element
    public Name getSimpleName() {
        return this.name;
    }

    @Override // com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
    public List<Attribute.Compound> getAnnotationMirrors() {
        return getRawAttributes();
    }

    @Override // javax.lang.model.element.Element
    public java.util.List<Symbol> getEnclosedElements() {
        return List.nil();
    }

    public List<TypeVariableSymbol> getTypeParameters() {
        ListBuffer<TypeVariableSymbol> l = new ListBuffer<>();
        for (Type t : this.type.getTypeArguments()) {
            Assert.check(t.tsym.getKind() == ElementKind.TYPE_PARAMETER);
            l.append((TypeVariableSymbol) t.tsym);
        }
        return l.toList();
    }

    public static class DelegatedSymbol<T extends Symbol> extends Symbol {
        protected T other;

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ TypeMirror asType() {
            return super.asType();
        }

        @Override // com.sun.tools.javac.code.Symbol, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ Element getEnclosingElement() {
            return super.getEnclosingElement();
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ javax.lang.model.element.Name getSimpleName() {
            return super.getSimpleName();
        }

        public DelegatedSymbol(T other) {
            super(other.kind, other.flags_field, other.name, other.type, other.owner);
            this.other = other;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public String toString() {
            return this.other.toString();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Symbol location() {
            return this.other.location();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Symbol location(Type site, Types types) {
            return this.other.location(site, types);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Symbol baseSymbol() {
            return this.other;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Type erasure(Types types) {
            return this.other.erasure(types);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Type externalType(Types types) {
            return this.other.externalType(types);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean isLocal() {
            return this.other.isLocal();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean isConstructor() {
            return this.other.isConstructor();
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.TypeElement, javax.lang.model.element.QualifiedNameable
        public Name getQualifiedName() {
            return this.other.getQualifiedName();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Name flatName() {
            return this.other.flatName();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Scope members() {
            return this.other.members();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean isInner() {
            return this.other.isInner();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean hasOuterInstance() {
            return this.other.hasOuterInstance();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public ClassSymbol enclClass() {
            return this.other.enclClass();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public ClassSymbol outermostClass() {
            return this.other.outermostClass();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public PackageSymbol packge() {
            return this.other.packge();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean isSubClass(Symbol base, Types types) {
            return this.other.isSubClass(base, types);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean isMemberOf(TypeSymbol clazz, Types types) {
            return this.other.isMemberOf(clazz, types);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean isEnclosedBy(ClassSymbol clazz) {
            return this.other.isEnclosedBy(clazz);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean isInheritedIn(Symbol clazz, Types types) {
            return this.other.isInheritedIn(clazz, types);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Symbol asMemberOf(Type site, Types types) {
            return this.other.asMemberOf(site, types);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public void complete() throws CompletionFailure {
            this.other.complete();
        }

        @Override // javax.lang.model.element.Element
        public <R, P> R accept(ElementVisitor<R, P> elementVisitor, P p) {
            return (R) this.other.accept(elementVisitor, p);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public <R, P> R accept(Visitor<R, P> v, P p) {
            return v.visitSymbol(this.other, p);
        }

        public T getUnderlyingSymbol() {
            return this.other;
        }
    }

    public static abstract class TypeSymbol extends Symbol {
        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ TypeMirror asType() {
            return super.asType();
        }

        @Override // com.sun.tools.javac.code.Symbol, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ Element getEnclosingElement() {
            return super.getEnclosingElement();
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ javax.lang.model.element.Name getSimpleName() {
            return super.getSimpleName();
        }

        public TypeSymbol(int kind, long flags, Name name, Type type, Symbol owner) {
            super(kind, flags, name, type, owner);
        }

        public static Name formFullName(Name name, Symbol owner) {
            Name prefix;
            if (owner == null) {
                return name;
            }
            if ((owner.kind != 63 && ((owner.kind & 20) != 0 || (owner.kind == 2 && owner.type.hasTag(TypeTag.TYPEVAR)))) || (prefix = owner.getQualifiedName()) == null || prefix == prefix.table.names.empty) {
                return name;
            }
            return prefix.append(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, name);
        }

        public static Name formFlatName(Name name, Symbol owner) {
            if (owner == null || (owner.kind & 20) != 0 || (owner.kind == 2 && owner.type.hasTag(TypeTag.TYPEVAR))) {
                return name;
            }
            char sep = owner.kind == 2 ? '$' : DescriptorUtils.JAVA_PACKAGE_SEPARATOR;
            Name prefix = owner.flatName();
            if (prefix == null || prefix == prefix.table.names.empty) {
                return name;
            }
            return prefix.append(sep, name);
        }

        public final boolean precedes(TypeSymbol that, Types types) {
            if (this == that) {
                return false;
            }
            if (this.type.hasTag(that.type.getTag())) {
                if (this.type.hasTag(TypeTag.CLASS)) {
                    return types.rank(that.type) < types.rank(this.type) || (types.rank(that.type) == types.rank(this.type) && that.getQualifiedName().compareTo(getQualifiedName()) < 0);
                }
                if (this.type.hasTag(TypeTag.TYPEVAR)) {
                    return types.isSubtype(this.type, that.type);
                }
            }
            return this.type.hasTag(TypeTag.TYPEVAR);
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public java.util.List<Symbol> getEnclosedElements() {
            List<Symbol> list = List.nil();
            if (this.kind == 2 && this.type.hasTag(TypeTag.TYPEVAR)) {
                return list;
            }
            for (Scope.Entry e = members().elems; e != null; e = e.sibling) {
                if (e.sym != null && (e.sym.flags() & 4096) == 0 && e.sym.owner == this) {
                    list = list.prepend(e.sym);
                }
            }
            return list;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public <R, P> R accept(Visitor<R, P> v, P p) {
            return v.visitTypeSymbol(this, p);
        }
    }

    public static class TypeVariableSymbol extends TypeSymbol implements TypeParameterElement {
        public TypeVariableSymbol(long flags, Name name, Type type, Symbol owner) {
            super(2, flags, name, type, owner);
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public ElementKind getKind() {
            return ElementKind.TYPE_PARAMETER;
        }

        @Override // javax.lang.model.element.TypeParameterElement
        public Symbol getGenericElement() {
            return this.owner;
        }

        @Override // javax.lang.model.element.TypeParameterElement
        public List<Type> getBounds() {
            Type.TypeVar t = (Type.TypeVar) this.type;
            Type bound = t.getUpperBound();
            if (!bound.isCompound()) {
                return List.of(bound);
            }
            Type.ClassType ct = (Type.ClassType) bound;
            if (!ct.tsym.erasure_field.isInterface()) {
                return ct.interfaces_field.prepend(ct.supertype_field);
            }
            return ct.interfaces_field;
        }

        @Override // com.sun.tools.javac.code.Symbol.TypeSymbol, com.sun.tools.javac.code.Symbol, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public List<Attribute.Compound> getAnnotationMirrors() {
            List<Attribute.TypeCompound> candidates = this.owner.getRawTypeAttributes();
            int index = this.owner.getTypeParameters().indexOf(this);
            List<Attribute.Compound> res = List.nil();
            for (Attribute.TypeCompound a : candidates) {
                if (isCurrentSymbolsAnnotation(a, index)) {
                    res = res.prepend(a);
                }
            }
            return res.reverse();
        }

        @Override // com.sun.tools.javac.code.AnnoConstruct
        public <A extends Annotation> Attribute.Compound getAttribute(Class<A> annoType) {
            String name = annoType.getName();
            List<Attribute.TypeCompound> candidates = this.owner.getRawTypeAttributes();
            int index = this.owner.getTypeParameters().indexOf(this);
            for (Attribute.TypeCompound anno : candidates) {
                if (isCurrentSymbolsAnnotation(anno, index) && name.contentEquals(anno.type.tsym.flatName())) {
                    return anno;
                }
            }
            return null;
        }

        boolean isCurrentSymbolsAnnotation(Attribute.TypeCompound anno, int index) {
            return (anno.position.type == TargetType.CLASS_TYPE_PARAMETER || anno.position.type == TargetType.METHOD_TYPE_PARAMETER) && anno.position.parameter_index == index;
        }

        @Override // javax.lang.model.element.Element
        public <R, P> R accept(ElementVisitor<R, P> v, P p) {
            return v.visitTypeParameter(this, p);
        }
    }

    public static class PackageSymbol extends TypeSymbol implements PackageElement {
        public Name fullname;
        public Scope members_field;
        public ClassSymbol package_info;

        public PackageSymbol(Name name, Type type, Symbol owner) {
            super(1, 0L, name, type, owner);
            this.members_field = null;
            this.fullname = formFullName(name, owner);
        }

        public PackageSymbol(Name name, Symbol owner) {
            this(name, null, owner);
            this.type = new Type.PackageType(this);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public String toString() {
            return this.fullname.toString();
        }

        @Override // javax.lang.model.element.PackageElement, javax.lang.model.element.QualifiedNameable
        public Name getQualifiedName() {
            return this.fullname;
        }

        @Override // javax.lang.model.element.PackageElement
        public boolean isUnnamed() {
            return this.name.isEmpty() && this.owner != null;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Scope members() {
            if (this.completer != null) {
                complete();
            }
            return this.members_field;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public long flags() {
            if (this.completer != null) {
                complete();
            }
            return this.flags_field;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public List<Attribute.Compound> getRawAttributes() {
            if (this.completer != null) {
                complete();
            }
            if (this.package_info != null && this.package_info.completer != null) {
                this.package_info.complete();
                mergeAttributes();
            }
            return super.getRawAttributes();
        }

        private void mergeAttributes() {
            if (this.metadata == null && this.package_info.metadata != null) {
                this.metadata = new SymbolMetadata(this);
                this.metadata.setAttributes(this.package_info.metadata);
            }
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean exists() {
            return (this.flags_field & 8388608) != 0;
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public ElementKind getKind() {
            return ElementKind.PACKAGE;
        }

        @Override // com.sun.tools.javac.code.Symbol.TypeSymbol, com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public Symbol getEnclosingElement() {
            return null;
        }

        @Override // javax.lang.model.element.Element
        public <R, P> R accept(ElementVisitor<R, P> v, P p) {
            return v.visitPackage(this, p);
        }

        @Override // com.sun.tools.javac.code.Symbol.TypeSymbol, com.sun.tools.javac.code.Symbol
        public <R, P> R accept(Visitor<R, P> v, P p) {
            return v.visitPackageSymbol(this, p);
        }
    }

    public static class ClassSymbol extends TypeSymbol implements TypeElement {
        public JavaFileObject classfile;
        public Name flatname;
        public Name fullname;
        public Scope members_field;
        public Pool pool;
        public JavaFileObject sourcefile;
        public List<ClassSymbol> trans_local;

        @Override // javax.lang.model.element.TypeElement, javax.lang.model.element.Parameterizable
        public /* bridge */ /* synthetic */ java.util.List getTypeParameters() {
            return super.getTypeParameters();
        }

        public ClassSymbol(long flags, Name name, Type type, Symbol owner) {
            super(2, flags, name, type, owner);
            this.members_field = null;
            this.fullname = formFullName(name, owner);
            this.flatname = formFlatName(name, owner);
            this.sourcefile = null;
            this.classfile = null;
            this.pool = null;
        }

        public ClassSymbol(long flags, Name name, Symbol owner) {
            this(flags, name, new Type.ClassType(Type.noType, null, null), owner);
            this.type.tsym = this;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public String toString() {
            return className();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public long flags() {
            if (this.completer != null) {
                complete();
            }
            return this.flags_field;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Scope members() {
            if (this.completer != null) {
                complete();
            }
            return this.members_field;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public List<Attribute.Compound> getRawAttributes() {
            if (this.completer != null) {
                complete();
            }
            return super.getRawAttributes();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public List<Attribute.TypeCompound> getRawTypeAttributes() {
            if (this.completer != null) {
                complete();
            }
            return super.getRawTypeAttributes();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Type erasure(Types types) {
            if (this.erasure_field == null) {
                this.erasure_field = new Type.ClassType(types.erasure(this.type.getEnclosingType()), List.nil(), this);
            }
            return this.erasure_field;
        }

        public String className() {
            if (this.name.isEmpty()) {
                return Log.getLocalizedString("anonymous.class", this.flatname);
            }
            return this.fullname.toString();
        }

        @Override // javax.lang.model.element.TypeElement, javax.lang.model.element.QualifiedNameable
        public Name getQualifiedName() {
            return this.fullname;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Name flatName() {
            return this.flatname;
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Symbol
        public boolean isSubClass(Symbol base, Types types) {
            if (this == base) {
                return true;
            }
            if ((base.flags() & 512) != 0) {
                Type t = this.type;
                while (t.hasTag(TypeTag.CLASS)) {
                    for (List listInterfaces = types.interfaces(t); listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
                        if (((Type) listInterfaces.head).tsym.isSubClass(base, types)) {
                            return true;
                        }
                    }
                    t = types.supertype(t);
                }
                return false;
            }
            Type t2 = this.type;
            while (t2.hasTag(TypeTag.CLASS)) {
                if (t2.tsym == base) {
                    return true;
                }
                t2 = types.supertype(t2);
            }
            return false;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public void complete() throws CompletionFailure {
            try {
                super.complete();
            } catch (CompletionFailure ex) {
                this.flags_field |= 9;
                this.type = new Type.ErrorType(this, Type.noType);
                throw ex;
            }
        }

        @Override // javax.lang.model.element.TypeElement
        public List<Type> getInterfaces() {
            complete();
            if (this.type instanceof Type.ClassType) {
                Type.ClassType t = (Type.ClassType) this.type;
                if (t.interfaces_field == null) {
                    t.interfaces_field = List.nil();
                }
                if (t.all_interfaces_field != null) {
                    return Type.getModelTypes(t.all_interfaces_field);
                }
                return t.interfaces_field;
            }
            return List.nil();
        }

        @Override // javax.lang.model.element.TypeElement
        public Type getSuperclass() {
            complete();
            if (this.type instanceof Type.ClassType) {
                Type.ClassType t = (Type.ClassType) this.type;
                if (t.supertype_field == null) {
                    t.supertype_field = Type.noType;
                }
                return t.isInterface() ? Type.noType : t.supertype_field.getModelType();
            }
            return Type.noType;
        }

        private ClassSymbol getSuperClassToSearchForAnnotations() {
            Type sup = getSuperclass();
            if (!sup.hasTag(TypeTag.CLASS) || sup.isErroneous()) {
                return null;
            }
            return (ClassSymbol) sup.tsym;
        }

        @Override // com.sun.tools.javac.code.AnnoConstruct
        protected <A extends Annotation> A[] getInheritedAnnotations(Class<A> cls) {
            ClassSymbol superClassToSearchForAnnotations = getSuperClassToSearchForAnnotations();
            return superClassToSearchForAnnotations == null ? (A[]) super.getInheritedAnnotations(cls) : (A[]) superClassToSearchForAnnotations.getAnnotationsByType(cls);
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public ElementKind getKind() {
            long flags = flags();
            if ((8192 & flags) != 0) {
                return ElementKind.ANNOTATION_TYPE;
            }
            if ((512 & flags) != 0) {
                return ElementKind.INTERFACE;
            }
            if ((16384 & flags) != 0) {
                return ElementKind.ENUM;
            }
            return ElementKind.CLASS;
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public Set<Modifier> getModifiers() {
            long flags = flags();
            return Flags.asModifierSet((-8796093022209L) & flags);
        }

        @Override // javax.lang.model.element.TypeElement
        public NestingKind getNestingKind() {
            complete();
            if (this.owner.kind == 1) {
                return NestingKind.TOP_LEVEL;
            }
            if (this.name.isEmpty()) {
                return NestingKind.ANONYMOUS;
            }
            if (this.owner.kind == 16) {
                return NestingKind.LOCAL;
            }
            return NestingKind.MEMBER;
        }

        @Override // com.sun.tools.javac.code.AnnoConstruct
        protected <A extends Annotation> Attribute.Compound getAttribute(Class<A> annoType) {
            Attribute.Compound attrib = super.getAttribute(annoType);
            boolean inherited = annoType.isAnnotationPresent(Inherited.class);
            if (attrib != null || !inherited) {
                return attrib;
            }
            ClassSymbol superType = getSuperClassToSearchForAnnotations();
            if (superType == null) {
                return null;
            }
            return superType.getAttribute(annoType);
        }

        @Override // javax.lang.model.element.Element
        public <R, P> R accept(ElementVisitor<R, P> v, P p) {
            return v.visitType(this, p);
        }

        @Override // com.sun.tools.javac.code.Symbol.TypeSymbol, com.sun.tools.javac.code.Symbol
        public <R, P> R accept(Visitor<R, P> v, P p) {
            return v.visitClassSymbol(this, p);
        }

        public void markAbstractIfNeeded(Types types) {
            if (types.enter.getEnv(this) != null && (flags() & 16384) != 0 && types.supertype(this.type).tsym == types.syms.enumSym && (flags() & 1040) == 0 && types.firstUnimplementedAbstract(this) != null) {
                this.flags_field |= 1024;
            }
        }
    }

    public static class VarSymbol extends Symbol implements VariableElement {
        public int adr;
        private Object data;
        public int pos;

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ TypeMirror asType() {
            return super.asType();
        }

        @Override // com.sun.tools.javac.code.Symbol, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ Element getEnclosingElement() {
            return super.getEnclosingElement();
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ javax.lang.model.element.Name getSimpleName() {
            return super.getSimpleName();
        }

        public VarSymbol(long flags, Name name, Type type, Symbol owner) {
            super(4, flags, name, type, owner);
            this.pos = -1;
            this.adr = -1;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public VarSymbol clone(Symbol newOwner) {
            VarSymbol v = new VarSymbol(this.flags_field, this.name, this.type, newOwner) { // from class: com.sun.tools.javac.code.Symbol.VarSymbol.1
                @Override // com.sun.tools.javac.code.Symbol.VarSymbol, com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
                public /* bridge */ /* synthetic */ TypeMirror asType() {
                    return super.asType();
                }

                @Override // com.sun.tools.javac.code.Symbol.VarSymbol, com.sun.tools.javac.code.Symbol
                public /* bridge */ /* synthetic */ Symbol clone(Symbol symbol) {
                    return super.clone(symbol);
                }

                @Override // com.sun.tools.javac.code.Symbol.VarSymbol, com.sun.tools.javac.code.Symbol, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
                public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
                    return super.getAnnotationMirrors();
                }

                @Override // com.sun.tools.javac.code.Symbol.VarSymbol, com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
                public /* bridge */ /* synthetic */ Element getEnclosingElement() {
                    return super.getEnclosingElement();
                }

                @Override // com.sun.tools.javac.code.Symbol.VarSymbol, com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
                public /* bridge */ /* synthetic */ javax.lang.model.element.Name getSimpleName() {
                    return super.getSimpleName();
                }

                @Override // com.sun.tools.javac.code.Symbol
                public Symbol baseSymbol() {
                    return VarSymbol.this;
                }
            };
            v.pos = this.pos;
            v.adr = this.adr;
            v.data = this.data;
            return v;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public String toString() {
            return this.name.toString();
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Symbol asMemberOf(Type site, Types types) {
            return new VarSymbol(this.flags_field, this.name, types.memberType(site, this), this.owner);
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public ElementKind getKind() {
            long flags = flags();
            if ((8589934592L & flags) != 0) {
                if (isExceptionParameter()) {
                    return ElementKind.EXCEPTION_PARAMETER;
                }
                return ElementKind.PARAMETER;
            }
            if ((16384 & flags) != 0) {
                return ElementKind.ENUM_CONSTANT;
            }
            if (this.owner.kind == 2 || this.owner.kind == 63) {
                return ElementKind.FIELD;
            }
            if (isResourceVariable()) {
                return ElementKind.RESOURCE_VARIABLE;
            }
            return ElementKind.LOCAL_VARIABLE;
        }

        @Override // javax.lang.model.element.Element
        public <R, P> R accept(ElementVisitor<R, P> v, P p) {
            return v.visitVariable(this, p);
        }

        @Override // javax.lang.model.element.VariableElement
        public Object getConstantValue() {
            return Constants.decode(getConstValue(), this.type);
        }

        public void setLazyConstValue(final Env<AttrContext> env, final Attr attr, final JCTree.JCVariableDecl variable) {
            setData(new Callable<Object>() { // from class: com.sun.tools.javac.code.Symbol.VarSymbol.2
                @Override // java.util.concurrent.Callable
                public Object call() {
                    return attr.attribLazyConstantValue(env, variable, VarSymbol.this.type);
                }
            });
        }

        public boolean isExceptionParameter() {
            return this.data == ElementKind.EXCEPTION_PARAMETER;
        }

        public boolean isResourceVariable() {
            return this.data == ElementKind.RESOURCE_VARIABLE;
        }

        public Object getConstValue() {
            if (this.data == ElementKind.EXCEPTION_PARAMETER || this.data == ElementKind.RESOURCE_VARIABLE) {
                return null;
            }
            if (this.data instanceof Callable) {
                Callable<?> eval = (Callable) this.data;
                this.data = null;
                try {
                    this.data = eval.call();
                } catch (Exception ex) {
                    throw new AssertionError(ex);
                }
            }
            return this.data;
        }

        public void setData(Object data) {
            Assert.check(!(data instanceof Env), this);
            this.data = data;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public <R, P> R accept(Visitor<R, P> v, P p) {
            return v.visitVarSymbol(this, p);
        }
    }

    public static class MethodSymbol extends Symbol implements ExecutableElement {
        public static final Filter<Symbol> implementation_filter = new Filter<Symbol>() { // from class: com.sun.tools.javac.code.Symbol.MethodSymbol.2
            @Override // com.sun.tools.javac.util.Filter
            public boolean accepts(Symbol s) {
                return s.kind == 16 && (s.flags() & 4096) == 0;
            }
        };
        public List<VarSymbol> capturedLocals;
        public Code code;
        public Attribute defaultValue;
        public List<VarSymbol> extraParams;
        public List<VarSymbol> params;
        public List<Name> savedParameterNames;

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ TypeMirror asType() {
            return super.asType();
        }

        @Override // com.sun.tools.javac.code.Symbol, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
        public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
            return super.getAnnotationMirrors();
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ Element getEnclosingElement() {
            return super.getEnclosingElement();
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public /* bridge */ /* synthetic */ javax.lang.model.element.Name getSimpleName() {
            return super.getSimpleName();
        }

        @Override // javax.lang.model.element.ExecutableElement, javax.lang.model.element.Parameterizable
        public /* bridge */ /* synthetic */ java.util.List getTypeParameters() {
            return super.getTypeParameters();
        }

        public MethodSymbol(long flags, Name name, Type type, Symbol owner) {
            super(16, flags, name, type, owner);
            this.code = null;
            this.extraParams = List.nil();
            this.capturedLocals = List.nil();
            this.params = null;
            this.defaultValue = null;
            if (owner.type.hasTag(TypeTag.TYPEVAR)) {
                Assert.error(owner + "." + ((Object) name));
            }
        }

        @Override // com.sun.tools.javac.code.Symbol
        public MethodSymbol clone(Symbol newOwner) {
            MethodSymbol m = new MethodSymbol(this.flags_field, this.name, this.type, newOwner) { // from class: com.sun.tools.javac.code.Symbol.MethodSymbol.1
                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
                public /* bridge */ /* synthetic */ TypeMirror asType() {
                    return super.asType();
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, com.sun.tools.javac.code.Symbol
                public /* bridge */ /* synthetic */ Symbol clone(Symbol symbol) {
                    return super.clone(symbol);
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, com.sun.tools.javac.code.Symbol, com.sun.tools.javac.code.AnnoConstruct, javax.lang.model.AnnotatedConstruct
                public /* bridge */ /* synthetic */ java.util.List getAnnotationMirrors() {
                    return super.getAnnotationMirrors();
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, javax.lang.model.element.ExecutableElement
                public /* bridge */ /* synthetic */ AnnotationValue getDefaultValue() {
                    return super.getDefaultValue();
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
                public /* bridge */ /* synthetic */ Element getEnclosingElement() {
                    return super.getEnclosingElement();
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, javax.lang.model.element.ExecutableElement
                public /* bridge */ /* synthetic */ java.util.List getParameters() {
                    return super.getParameters();
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, javax.lang.model.element.ExecutableElement
                public /* bridge */ /* synthetic */ TypeMirror getReceiverType() {
                    return super.getReceiverType();
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, javax.lang.model.element.ExecutableElement
                public /* bridge */ /* synthetic */ TypeMirror getReturnType() {
                    return super.getReturnType();
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
                public /* bridge */ /* synthetic */ javax.lang.model.element.Name getSimpleName() {
                    return super.getSimpleName();
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, javax.lang.model.element.ExecutableElement
                public /* bridge */ /* synthetic */ java.util.List getThrownTypes() {
                    return super.getThrownTypes();
                }

                @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, javax.lang.model.element.ExecutableElement, javax.lang.model.element.Parameterizable
                public /* bridge */ /* synthetic */ java.util.List getTypeParameters() {
                    return super.getTypeParameters();
                }

                @Override // com.sun.tools.javac.code.Symbol
                public Symbol baseSymbol() {
                    return MethodSymbol.this;
                }
            };
            m.code = this.code;
            return m;
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public Set<Modifier> getModifiers() {
            long flags = flags();
            return Flags.asModifierSet((Flags.DEFAULT & flags) != 0 ? (-1025) & flags : flags);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public String toString() {
            String s;
            if ((flags() & 1048576) != 0) {
                return this.owner.name.toString();
            }
            if (this.name == this.name.table.names.init) {
                s = this.owner.name.toString();
            } else {
                s = this.name.toString();
            }
            if (this.type != null) {
                if (this.type.hasTag(TypeTag.FORALL)) {
                    s = "<" + ((Type.ForAll) this.type).getTypeArguments() + ">" + s;
                }
                return s + "(" + this.type.argtypes((flags() & Flags.VARARGS) != 0) + ")";
            }
            return s;
        }

        public boolean isDynamic() {
            return false;
        }

        /* JADX WARN: Multi-variable type inference failed */
        public Symbol implemented(TypeSymbol c, Types types) {
            Symbol impl = null;
            for (List listInterfaces = types.interfaces(c.type); impl == null && listInterfaces.nonEmpty(); listInterfaces = listInterfaces.tail) {
                TypeSymbol i = ((Type) listInterfaces.head).tsym;
                impl = implementedIn(i, types);
                if (impl == null) {
                    impl = implemented(i, types);
                }
            }
            return impl;
        }

        public Symbol implementedIn(TypeSymbol c, Types types) {
            Symbol impl = null;
            for (Scope.Entry e = c.members().lookup(this.name); impl == null && e.scope != null; e = e.next()) {
                if (overrides(e.sym, (TypeSymbol) this.owner, types, true) && types.isSameType(this.type.mo178getReturnType(), types.memberType(this.owner.type, e.sym).mo178getReturnType())) {
                    impl = e.sym;
                }
            }
            return impl;
        }

        public boolean binaryOverrides(Symbol _other, TypeSymbol origin, Types types) {
            if (isConstructor() || _other.kind != 16) {
                return false;
            }
            if (this == _other) {
                return true;
            }
            MethodSymbol other = (MethodSymbol) _other;
            if (other.isOverridableIn((TypeSymbol) this.owner) && types.asSuper(this.owner.type, other.owner) != null && types.isSameType(erasure(types), other.erasure(types))) {
                return true;
            }
            return (flags() & 1024) == 0 && other.isOverridableIn(origin) && isMemberOf(origin, types) && types.isSameType(erasure(types), other.erasure(types));
        }

        public MethodSymbol binaryImplementation(ClassSymbol origin, Types types) {
            TypeSymbol c = origin;
            while (c != null) {
                for (Scope.Entry e = c.members().lookup(this.name); e.scope != null; e = e.next()) {
                    if (e.sym.kind == 16 && ((MethodSymbol) e.sym).binaryOverrides(this, origin, types)) {
                        return (MethodSymbol) e.sym;
                    }
                }
                c = types.supertype(c.type).tsym;
            }
            return null;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean overrides(Symbol _other, TypeSymbol origin, Types types, boolean checkResult) {
            if (isConstructor() || _other.kind != 16) {
                return false;
            }
            if (this == _other) {
                return true;
            }
            MethodSymbol other = (MethodSymbol) _other;
            if (other.isOverridableIn((TypeSymbol) this.owner) && types.asSuper(this.owner.type, other.owner) != null) {
                Type mt = types.memberType(this.owner.type, this);
                Type ot = types.memberType(this.owner.type, other);
                if (types.isSubSignature(mt, ot) && (!checkResult || types.returnTypeSubstitutable(mt, ot))) {
                    return true;
                }
            }
            if ((flags() & 1024) != 0 || (((other.flags() & 1024) == 0 && (other.flags() & Flags.DEFAULT) == 0) || !other.isOverridableIn(origin) || !isMemberOf(origin, types))) {
                return false;
            }
            Type mt2 = types.memberType(origin.type, this);
            Type ot2 = types.memberType(origin.type, other);
            if (types.isSubSignature(mt2, ot2)) {
                return !checkResult || types.resultSubtype(mt2, ot2, types.noWarnings);
            }
            return false;
        }

        private boolean isOverridableIn(TypeSymbol origin) {
            switch ((int) (this.flags_field & 7)) {
                case 0:
                    if (packge() != origin.packge() || (origin.flags() & 512) != 0) {
                    }
                    break;
                case 1:
                    if (this.owner.isInterface() && (this.flags_field & 8) != 0) {
                        break;
                    }
                    break;
                case 4:
                    if ((origin.flags() & 512) != 0) {
                        break;
                    }
                    break;
            }
            return false;
        }

        @Override // com.sun.tools.javac.code.Symbol
        public boolean isInheritedIn(Symbol clazz, Types types) {
            switch ((int) (this.flags_field & 7)) {
                case 1:
                    return !this.owner.isInterface() || clazz == this.owner || (this.flags_field & 8) == 0;
                default:
                    return super.isInheritedIn(clazz, types);
            }
        }

        public MethodSymbol implementation(TypeSymbol origin, Types types, boolean checkResult) {
            return implementation(origin, types, checkResult, implementation_filter);
        }

        public MethodSymbol implementation(TypeSymbol origin, Types types, boolean checkResult, Filter<Symbol> implFilter) {
            MethodSymbol res = types.implementation(this, origin, checkResult, implFilter);
            if (res != null) {
                return res;
            }
            if (types.isDerivedRaw(origin.type) && !origin.isInterface()) {
                return implementation(types.supertype(origin.type).tsym, types, checkResult);
            }
            return null;
        }

        /* JADX WARN: Multi-variable type inference failed */
        public List<VarSymbol> params() {
            Name paramName;
            this.owner.complete();
            if (this.params == null) {
                List<Name> paramNames = this.savedParameterNames;
                this.savedParameterNames = null;
                if (paramNames == null || paramNames.size() != this.type.mo176getParameterTypes().size()) {
                    paramNames = List.nil();
                }
                ListBuffer<VarSymbol> buf = new ListBuffer<>();
                List list = paramNames;
                int i = 0;
                for (Type t : this.type.mo176getParameterTypes()) {
                    if (list.isEmpty()) {
                        paramName = createArgName(i, paramNames);
                    } else {
                        paramName = (Name) list.head;
                        list = list.tail;
                        if (paramName.isEmpty()) {
                            paramName = createArgName(i, paramNames);
                        }
                    }
                    buf.append(new VarSymbol(8589934592L, paramName, t, this));
                    i++;
                }
                this.params = buf.toList();
            }
            return this.params;
        }

        private Name createArgName(int index, List<Name> exclude) {
            String prefix = "arg";
            while (true) {
                Name argName = this.name.table.fromString(prefix + index);
                if (!exclude.contains(argName)) {
                    return argName;
                }
                prefix = prefix + "$";
            }
        }

        @Override // com.sun.tools.javac.code.Symbol
        public Symbol asMemberOf(Type site, Types types) {
            return new MethodSymbol(this.flags_field, this.name, types.memberType(site, this), this.owner);
        }

        @Override // com.sun.tools.javac.code.Symbol, javax.lang.model.element.Element
        public ElementKind getKind() {
            if (this.name == this.name.table.names.init) {
                return ElementKind.CONSTRUCTOR;
            }
            if (this.name == this.name.table.names.clinit) {
                return ElementKind.STATIC_INIT;
            }
            if ((flags() & 1048576) != 0) {
                return isStatic() ? ElementKind.STATIC_INIT : ElementKind.INSTANCE_INIT;
            }
            return ElementKind.METHOD;
        }

        public boolean isStaticOrInstanceInit() {
            return getKind() == ElementKind.STATIC_INIT || getKind() == ElementKind.INSTANCE_INIT;
        }

        @Override // javax.lang.model.element.ExecutableElement
        public Attribute getDefaultValue() {
            return this.defaultValue;
        }

        @Override // javax.lang.model.element.ExecutableElement
        public List<VarSymbol> getParameters() {
            return params();
        }

        @Override // javax.lang.model.element.ExecutableElement
        public boolean isVarArgs() {
            return (flags() & Flags.VARARGS) != 0;
        }

        @Override // javax.lang.model.element.ExecutableElement
        public boolean isDefault() {
            return (flags() & Flags.DEFAULT) != 0;
        }

        @Override // javax.lang.model.element.Element
        public <R, P> R accept(ElementVisitor<R, P> v, P p) {
            return v.visitExecutable(this, p);
        }

        @Override // com.sun.tools.javac.code.Symbol
        public <R, P> R accept(Visitor<R, P> v, P p) {
            return v.visitMethodSymbol(this, p);
        }

        @Override // javax.lang.model.element.ExecutableElement
        public Type getReceiverType() {
            return asType().mo177getReceiverType();
        }

        @Override // javax.lang.model.element.ExecutableElement
        public Type getReturnType() {
            return asType().mo178getReturnType();
        }

        @Override // javax.lang.model.element.ExecutableElement
        public List<Type> getThrownTypes() {
            return asType().mo179getThrownTypes();
        }
    }

    public static class DynamicMethodSymbol extends MethodSymbol {
        public Symbol bsm;
        public int bsmKind;
        public Object[] staticArgs;

        public DynamicMethodSymbol(Name name, Symbol owner, int bsmKind, MethodSymbol bsm, Type type, Object[] staticArgs) {
            super(0L, name, type, owner);
            this.bsm = bsm;
            this.bsmKind = bsmKind;
            this.staticArgs = staticArgs;
        }

        @Override // com.sun.tools.javac.code.Symbol.MethodSymbol
        public boolean isDynamic() {
            return true;
        }
    }

    public static class OperatorSymbol extends MethodSymbol {
        public int opcode;

        public OperatorSymbol(Name name, Type type, int opcode, Symbol owner) {
            super(9L, name, type, owner);
            this.opcode = opcode;
        }

        @Override // com.sun.tools.javac.code.Symbol.MethodSymbol, com.sun.tools.javac.code.Symbol
        public <R, P> R accept(Visitor<R, P> v, P p) {
            return v.visitOperatorSymbol(this, p);
        }
    }

    public static class CompletionFailure extends RuntimeException {
        private static final long serialVersionUID = 0;
        public JCDiagnostic diag;

        @Deprecated
        public String errmsg;
        public Symbol sym;

        public CompletionFailure(Symbol sym, String errmsg) {
            this.sym = sym;
            this.errmsg = errmsg;
        }

        public CompletionFailure(Symbol sym, JCDiagnostic diag) {
            this.sym = sym;
            this.diag = diag;
        }

        public JCDiagnostic getDiagnostic() {
            return this.diag;
        }

        @Override // java.lang.Throwable
        public String getMessage() {
            if (this.diag != null) {
                return this.diag.getMessage(null);
            }
            return this.errmsg;
        }

        public Object getDetailValue() {
            return this.diag != null ? this.diag : this.errmsg;
        }

        @Override // java.lang.Throwable
        public CompletionFailure initCause(Throwable cause) {
            super.initCause(cause);
            return this;
        }
    }
}
