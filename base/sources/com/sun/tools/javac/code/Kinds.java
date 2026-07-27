package com.sun.tools.javac.code;

import com.sun.source.tree.MemberReferenceTree;
import com.sun.tools.javac.api.Formattable;
import com.sun.tools.javac.api.Messages;
import java.util.EnumSet;
import java.util.Locale;

/* JADX INFO: loaded from: classes.dex */
public class Kinds {
    public static final int ABSENT_MTH = 136;
    public static final int ABSENT_TYP = 137;
    public static final int ABSENT_VAR = 133;
    public static final int AMBIGUOUS = 129;
    public static final int AllKinds = 63;
    public static final int ERR = 63;
    public static final int ERRONEOUS = 128;
    public static final int HIDDEN = 130;
    public static final int MISSING_ENCL = 132;
    public static final int MTH = 16;
    public static final int NIL = 0;
    public static final int PCK = 1;
    public static final int POLY = 32;
    public static final int STATICERR = 131;
    public static final int TYP = 2;
    public static final int VAL = 12;
    public static final int VAR = 4;
    public static final int WRONG_MTH = 135;
    public static final int WRONG_MTHS = 134;
    public static final int WRONG_STATICNESS = 138;

    private Kinds() {
    }

    public enum KindName implements Formattable {
        ANNOTATION("kindname.annotation"),
        CONSTRUCTOR("kindname.constructor"),
        INTERFACE("kindname.interface"),
        ENUM("kindname.enum"),
        STATIC("kindname.static"),
        TYPEVAR("kindname.type.variable"),
        BOUND("kindname.type.variable.bound"),
        VAR("kindname.variable"),
        VAL("kindname.value"),
        METHOD("kindname.method"),
        CLASS("kindname.class"),
        STATIC_INIT("kindname.static.init"),
        INSTANCE_INIT("kindname.instance.init"),
        PACKAGE("kindname.package");

        private final String name;

        KindName(String name) {
            this.name = name;
        }

        @Override // java.lang.Enum
        public String toString() {
            return this.name;
        }

        @Override // com.sun.tools.javac.api.Formattable
        public String getKind() {
            return "Kindname";
        }

        @Override // com.sun.tools.javac.api.Formattable
        public String toString(Locale locale, Messages messages) {
            String s = toString();
            return messages.getLocalizedString(locale, "compiler.misc." + s, new Object[0]);
        }
    }

    public static KindName kindName(int kind) {
        switch (kind) {
            case 1:
                return KindName.PACKAGE;
            case 2:
                return KindName.CLASS;
            case 4:
                return KindName.VAR;
            case 12:
                return KindName.VAL;
            case 16:
                return KindName.METHOD;
            default:
                throw new AssertionError("Unexpected kind: " + kind);
        }
    }

    public static KindName kindName(MemberReferenceTree.ReferenceMode mode) {
        switch (mode) {
            case INVOKE:
                return KindName.METHOD;
            case NEW:
                return KindName.CONSTRUCTOR;
            default:
                throw new AssertionError("Unexpected mode: " + mode);
        }
    }

    public static KindName kindName(Symbol sym) {
        switch (sym.getKind()) {
            case PACKAGE:
                return KindName.PACKAGE;
            case ENUM:
                return KindName.ENUM;
            case ANNOTATION_TYPE:
            case CLASS:
                return KindName.CLASS;
            case INTERFACE:
                return KindName.INTERFACE;
            case TYPE_PARAMETER:
                return KindName.TYPEVAR;
            case ENUM_CONSTANT:
            case FIELD:
            case PARAMETER:
            case LOCAL_VARIABLE:
            case EXCEPTION_PARAMETER:
            case RESOURCE_VARIABLE:
                return KindName.VAR;
            case CONSTRUCTOR:
                return KindName.CONSTRUCTOR;
            case METHOD:
                return KindName.METHOD;
            case STATIC_INIT:
                return KindName.STATIC_INIT;
            case INSTANCE_INIT:
                return KindName.INSTANCE_INIT;
            default:
                if (sym.kind == 12) {
                    return KindName.VAL;
                }
                throw new AssertionError("Unexpected kind: " + sym.getKind());
        }
    }

    public static EnumSet<KindName> kindNames(int kind) {
        EnumSet<KindName> kinds = EnumSet.noneOf(KindName.class);
        if ((kind & 12) != 0) {
            kinds.add((kind & 12) == 4 ? KindName.VAR : KindName.VAL);
        }
        if ((kind & 16) != 0) {
            kinds.add(KindName.METHOD);
        }
        if ((kind & 2) != 0) {
            kinds.add(KindName.CLASS);
        }
        if ((kind & 1) != 0) {
            kinds.add(KindName.PACKAGE);
        }
        return kinds;
    }

    public static KindName typeKindName(Type t) {
        if (t.hasTag(TypeTag.TYPEVAR) || (t.hasTag(TypeTag.CLASS) && (t.tsym.flags() & 16777216) != 0)) {
            return KindName.BOUND;
        }
        if (t.hasTag(TypeTag.PACKAGE)) {
            return KindName.PACKAGE;
        }
        if ((t.tsym.flags_field & 8192) != 0) {
            return KindName.ANNOTATION;
        }
        if ((t.tsym.flags_field & 512) != 0) {
            return KindName.INTERFACE;
        }
        return KindName.CLASS;
    }

    public static KindName absentKind(int kind) {
        switch (kind) {
            case 133:
                return KindName.VAR;
            case 134:
            case 135:
            case 136:
            case 138:
                return KindName.METHOD;
            case 137:
                return KindName.CLASS;
            default:
                throw new AssertionError("Unexpected kind: " + kind);
        }
    }
}
