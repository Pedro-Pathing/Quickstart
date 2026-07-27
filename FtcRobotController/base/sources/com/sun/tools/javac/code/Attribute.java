package com.sun.tools.javac.code;

import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Constants;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Pair;
import dk.sgjesse.r8api.FileUtils;
import java.util.LinkedHashMap;
import java.util.Map;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.AnnotationValue;
import javax.lang.model.element.AnnotationValueVisitor;
import javax.lang.model.type.DeclaredType;

/* JADX INFO: loaded from: classes.dex */
public abstract class Attribute implements AnnotationValue {
    public Type type;

    public enum RetentionPolicy {
        SOURCE,
        CLASS,
        RUNTIME
    }

    public interface Visitor {
        void visitArray(Array array);

        void visitClass(Class r1);

        void visitCompound(Compound compound);

        void visitConstant(Constant constant);

        void visitEnum(Enum r1);

        void visitError(Error error);
    }

    public abstract void accept(Visitor visitor);

    public Attribute(Type type) {
        this.type = type;
    }

    @Override // javax.lang.model.element.AnnotationValue
    public Object getValue() {
        throw new UnsupportedOperationException();
    }

    @Override // javax.lang.model.element.AnnotationValue
    public <R, P> R accept(AnnotationValueVisitor<R, P> v, P p) {
        throw new UnsupportedOperationException();
    }

    public boolean isSynthesized() {
        return false;
    }

    public TypeAnnotationPosition getPosition() {
        return null;
    }

    public static class Constant extends Attribute {
        public final Object value;

        @Override // com.sun.tools.javac.code.Attribute
        public void accept(Visitor v) {
            v.visitConstant(this);
        }

        public Constant(Type type, Object value) {
            super(type);
            this.value = value;
        }

        @Override // javax.lang.model.element.AnnotationValue
        public String toString() {
            return Constants.format(this.value, this.type);
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public Object getValue() {
            return Constants.decode(this.value, this.type);
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public <R, P> R accept(AnnotationValueVisitor<R, P> v, P p) {
            if (this.value instanceof String) {
                return v.visitString((String) this.value, p);
            }
            if (this.value instanceof Integer) {
                int i = ((Integer) this.value).intValue();
                switch (this.type.getTag()) {
                    case BOOLEAN:
                        return v.visitBoolean(i != 0, p);
                    case CHAR:
                        return v.visitChar((char) i, p);
                    case BYTE:
                        return v.visitByte((byte) i, p);
                    case SHORT:
                        return v.visitShort((short) i, p);
                    case INT:
                        return v.visitInt(i, p);
                }
            }
            switch (this.type.getTag()) {
                case LONG:
                    return v.visitLong(((Long) this.value).longValue(), p);
                case FLOAT:
                    return v.visitFloat(((Float) this.value).floatValue(), p);
                case DOUBLE:
                    return v.visitDouble(((Double) this.value).doubleValue(), p);
                default:
                    throw new AssertionError("Bad annotation element value: " + this.value);
            }
        }
    }

    public static class Class extends Attribute {
        public final Type classType;

        @Override // com.sun.tools.javac.code.Attribute
        public void accept(Visitor v) {
            v.visitClass(this);
        }

        public Class(Types types, Type type) {
            super(makeClassType(types, type));
            this.classType = type;
        }

        static Type makeClassType(Types types, Type type) {
            Type arg;
            if (type.isPrimitive()) {
                arg = types.boxedClass(type).type;
            } else {
                arg = types.erasure(type);
            }
            return new Type.ClassType(types.syms.classType.getEnclosingType(), List.of(arg), types.syms.classType.tsym);
        }

        @Override // javax.lang.model.element.AnnotationValue
        public String toString() {
            return this.classType + FileUtils.CLASS_EXTENSION;
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public Type getValue() {
            return this.classType;
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public <R, P> R accept(AnnotationValueVisitor<R, P> v, P p) {
            return v.visitType(this.classType, p);
        }
    }

    public static class Compound extends Attribute implements AnnotationMirror {
        private boolean synthesized;
        public final List<Pair<Symbol.MethodSymbol, Attribute>> values;

        @Override // com.sun.tools.javac.code.Attribute
        public boolean isSynthesized() {
            return this.synthesized;
        }

        public void setSynthesized(boolean synthesized) {
            this.synthesized = synthesized;
        }

        public Compound(Type type, List<Pair<Symbol.MethodSymbol, Attribute>> values) {
            super(type);
            this.synthesized = false;
            this.values = values;
        }

        @Override // com.sun.tools.javac.code.Attribute
        public void accept(Visitor v) {
            v.visitCompound(this);
        }

        @Override // javax.lang.model.element.AnnotationValue
        public String toString() {
            StringBuilder buf = new StringBuilder();
            buf.append("@");
            buf.append(this.type);
            int len = this.values.length();
            if (len > 0) {
                buf.append('(');
                boolean first = true;
                for (Pair<Symbol.MethodSymbol, Attribute> value : this.values) {
                    if (!first) {
                        buf.append(", ");
                    }
                    first = false;
                    Name name = value.fst.name;
                    if (len > 1 || name != name.table.names.value) {
                        buf.append((CharSequence) name);
                        buf.append('=');
                    }
                    buf.append(value.snd);
                }
                buf.append(')');
            }
            return buf.toString();
        }

        public Attribute member(Name member) {
            Pair<Symbol.MethodSymbol, Attribute> res = getElemPair(member);
            if (res == null) {
                return null;
            }
            return res.snd;
        }

        private Pair<Symbol.MethodSymbol, Attribute> getElemPair(Name member) {
            for (Pair<Symbol.MethodSymbol, Attribute> pair : this.values) {
                if (pair.fst.name == member) {
                    return pair;
                }
            }
            return null;
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public Compound getValue() {
            return this;
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public <R, P> R accept(AnnotationValueVisitor<R, P> v, P p) {
            return v.visitAnnotation(this, p);
        }

        @Override // javax.lang.model.element.AnnotationMirror
        public DeclaredType getAnnotationType() {
            return (DeclaredType) this.type;
        }

        @Override // com.sun.tools.javac.code.Attribute
        public TypeAnnotationPosition getPosition() {
            if (this.values.size() == 0) {
                return null;
            }
            Name valueName = this.values.head.fst.name.table.names.value;
            Pair<Symbol.MethodSymbol, Attribute> res = getElemPair(valueName);
            if (res == null) {
                return null;
            }
            return res.snd.getPosition();
        }

        @Override // javax.lang.model.element.AnnotationMirror
        public Map<Symbol.MethodSymbol, Attribute> getElementValues() {
            Map<Symbol.MethodSymbol, Attribute> valmap = new LinkedHashMap<>();
            for (Pair<Symbol.MethodSymbol, Attribute> value : this.values) {
                valmap.put(value.fst, value.snd);
            }
            return valmap;
        }
    }

    public static class TypeCompound extends Compound {
        public TypeAnnotationPosition position;

        public TypeCompound(Compound compound, TypeAnnotationPosition position) {
            this(compound.type, compound.values, position);
        }

        public TypeCompound(Type type, List<Pair<Symbol.MethodSymbol, Attribute>> values, TypeAnnotationPosition position) {
            super(type, values);
            this.position = position;
        }

        @Override // com.sun.tools.javac.code.Attribute.Compound, com.sun.tools.javac.code.Attribute
        public TypeAnnotationPosition getPosition() {
            if (hasUnknownPosition()) {
                this.position = super.getPosition();
            }
            return this.position;
        }

        public boolean hasUnknownPosition() {
            return this.position.type == TargetType.UNKNOWN;
        }

        public boolean isContainerTypeCompound() {
            return isSynthesized() && this.values.size() == 1 && getFirstEmbeddedTC() != null;
        }

        private TypeCompound getFirstEmbeddedTC() {
            if (this.values.size() == 1) {
                Pair<Symbol.MethodSymbol, Attribute> val = this.values.get(0);
                if (val.fst.getSimpleName().contentEquals("value") && (val.snd instanceof Array)) {
                    Array arr = (Array) val.snd;
                    if (arr.values.length != 0 && (arr.values[0] instanceof TypeCompound)) {
                        return (TypeCompound) arr.values[0];
                    }
                    return null;
                }
                return null;
            }
            return null;
        }

        public boolean tryFixPosition() {
            TypeCompound from;
            if (!isContainerTypeCompound() || (from = getFirstEmbeddedTC()) == null || from.position == null || from.position.type == TargetType.UNKNOWN) {
                return false;
            }
            this.position = from.position;
            return true;
        }
    }

    public static class Array extends Attribute {
        public final Attribute[] values;

        public Array(Type type, Attribute[] values) {
            super(type);
            this.values = values;
        }

        public Array(Type type, List<Attribute> values) {
            super(type);
            this.values = (Attribute[]) values.toArray(new Attribute[values.size()]);
        }

        @Override // com.sun.tools.javac.code.Attribute
        public void accept(Visitor v) {
            v.visitArray(this);
        }

        @Override // javax.lang.model.element.AnnotationValue
        public String toString() {
            StringBuilder buf = new StringBuilder();
            buf.append('{');
            boolean first = true;
            for (Attribute value : this.values) {
                if (!first) {
                    buf.append(", ");
                }
                first = false;
                buf.append(value);
            }
            buf.append('}');
            return buf.toString();
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public List<Attribute> getValue() {
            return List.from(this.values);
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public <R, P> R accept(AnnotationValueVisitor<R, P> v, P p) {
            return v.visitArray(getValue(), p);
        }

        @Override // com.sun.tools.javac.code.Attribute
        public TypeAnnotationPosition getPosition() {
            if (this.values.length != 0) {
                return this.values[0].getPosition();
            }
            return null;
        }
    }

    public static class Enum extends Attribute {
        public Symbol.VarSymbol value;

        public Enum(Type type, Symbol.VarSymbol value) {
            super(type);
            this.value = (Symbol.VarSymbol) Assert.checkNonNull(value);
        }

        @Override // com.sun.tools.javac.code.Attribute
        public void accept(Visitor v) {
            v.visitEnum(this);
        }

        @Override // javax.lang.model.element.AnnotationValue
        public String toString() {
            return this.value.enclClass() + "." + this.value;
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public Symbol.VarSymbol getValue() {
            return this.value;
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public <R, P> R accept(AnnotationValueVisitor<R, P> v, P p) {
            return v.visitEnumConstant(this.value, p);
        }
    }

    public static class Error extends Attribute {
        public Error(Type type) {
            super(type);
        }

        @Override // com.sun.tools.javac.code.Attribute
        public void accept(Visitor v) {
            v.visitError(this);
        }

        @Override // javax.lang.model.element.AnnotationValue
        public String toString() {
            return "<error>";
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public String getValue() {
            return toString();
        }

        @Override // com.sun.tools.javac.code.Attribute, javax.lang.model.element.AnnotationValue
        public <R, P> R accept(AnnotationValueVisitor<R, P> v, P p) {
            return v.visitString(toString(), p);
        }
    }

    public static class UnresolvedClass extends Error {
        public Type classType;

        public UnresolvedClass(Type type, Type classType) {
            super(type);
            this.classType = classType;
        }
    }
}
