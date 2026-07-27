package com.sun.tools.javac.code;

import com.sun.source.tree.Tree;
import javax.lang.model.type.TypeKind;

/* JADX INFO: loaded from: classes.dex */
public enum TypeTag {
    BYTE(1, 125, true),
    CHAR(2, 122, true),
    SHORT(4, 124, true),
    LONG(16, 112, true),
    FLOAT(32, 96, true),
    INT(8, 120, true),
    DOUBLE(64, 64, true),
    BOOLEAN(0, 0, true),
    VOID,
    CLASS,
    ARRAY,
    METHOD,
    PACKAGE,
    TYPEVAR,
    WILDCARD,
    FORALL,
    DEFERRED,
    BOT,
    NONE,
    ERROR,
    UNKNOWN,
    UNDETVAR,
    UNINITIALIZED_THIS,
    UNINITIALIZED_OBJECT;

    final boolean isPrimitive;
    final int numericClass;
    final int superClasses;

    public static class NumericClasses {
        public static final int BYTE_CLASS = 1;
        static final int BYTE_SUPERCLASSES = 125;
        public static final int CHAR_CLASS = 2;
        static final int CHAR_SUPERCLASSES = 122;
        public static final int DOUBLE_CLASS = 64;
        public static final int FLOAT_CLASS = 32;
        static final int FLOAT_SUPERCLASSES = 96;
        public static final int INT_CLASS = 8;
        static final int INT_SUPERCLASSES = 120;
        public static final int LONG_CLASS = 16;
        static final int LONG_SUPERCLASSES = 112;
        public static final int SHORT_CLASS = 4;
        static final int SHORT_SUPERCLASSES = 124;
    }

    TypeTag() {
        this(0, 0, false);
    }

    TypeTag(int numericClass, int superClasses, boolean isPrimitive) {
        this.superClasses = superClasses;
        this.numericClass = numericClass;
        this.isPrimitive = isPrimitive;
    }

    public boolean isStrictSubRangeOf(TypeTag tag) {
        return ((this.superClasses & tag.numericClass) == 0 || this == tag) ? false : true;
    }

    public boolean isSubRangeOf(TypeTag tag) {
        return (this.superClasses & tag.numericClass) != 0;
    }

    public static int getTypeTagCount() {
        return UNDETVAR.ordinal() + 1;
    }

    public Tree.Kind getKindLiteral() {
        switch (this) {
            case INT:
                return Tree.Kind.INT_LITERAL;
            case LONG:
                return Tree.Kind.LONG_LITERAL;
            case FLOAT:
                return Tree.Kind.FLOAT_LITERAL;
            case DOUBLE:
                return Tree.Kind.DOUBLE_LITERAL;
            case BOOLEAN:
                return Tree.Kind.BOOLEAN_LITERAL;
            case CHAR:
                return Tree.Kind.CHAR_LITERAL;
            case CLASS:
                return Tree.Kind.STRING_LITERAL;
            case BOT:
                return Tree.Kind.NULL_LITERAL;
            default:
                throw new AssertionError("unknown literal kind " + this);
        }
    }

    public TypeKind getPrimitiveTypeKind() {
        switch (this) {
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
            case CHAR:
                return TypeKind.CHAR;
            case CLASS:
            case BOT:
            default:
                throw new AssertionError("unknown primitive type " + this);
            case BYTE:
                return TypeKind.BYTE;
            case SHORT:
                return TypeKind.SHORT;
            case VOID:
                return TypeKind.VOID;
        }
    }
}
