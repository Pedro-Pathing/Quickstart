package com.sun.tools.javac.comp;

import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.jvm.ByteCodes;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;

/* JADX INFO: loaded from: classes.dex */
class ConstFold {
    private Symtab syms;
    protected static final Context.Key<ConstFold> constFoldKey = new Context.Key<>();
    static final Integer minusOne = -1;
    static final Integer zero = 0;
    static final Integer one = 1;

    public static ConstFold instance(Context context) {
        ConstFold instance = (ConstFold) context.get(constFoldKey);
        if (instance == null) {
            return new ConstFold(context);
        }
        return instance;
    }

    private ConstFold(Context context) {
        context.put(constFoldKey, this);
        this.syms = Symtab.instance(context);
    }

    private static Integer b2i(boolean b) {
        return b ? one : zero;
    }

    private static int intValue(Object x) {
        return ((Number) x).intValue();
    }

    private static long longValue(Object x) {
        return ((Number) x).longValue();
    }

    private static float floatValue(Object x) {
        return ((Number) x).floatValue();
    }

    private static double doubleValue(Object x) {
        return ((Number) x).doubleValue();
    }

    Type fold(int opcode, List<Type> argtypes) {
        int argCount = argtypes.length();
        if (argCount == 1) {
            return fold1(opcode, argtypes.head);
        }
        if (argCount == 2) {
            return fold2(opcode, argtypes.head, argtypes.tail.head);
        }
        throw new AssertionError();
    }

    Type fold1(int opcode, Type operand) {
        try {
            Object od = operand.constValue();
            boolean z = true;
            switch (opcode) {
                case 0:
                    return operand;
                case 116:
                    return this.syms.intType.constType(Integer.valueOf(-intValue(od)));
                case 117:
                    return this.syms.longType.constType(new Long(-longValue(od)));
                case 118:
                    return this.syms.floatType.constType(new Float(-floatValue(od)));
                case 119:
                    return this.syms.doubleType.constType(new Double(-doubleValue(od)));
                case 130:
                    return this.syms.intType.constType(Integer.valueOf(~intValue(od)));
                case 131:
                    return this.syms.longType.constType(new Long(~longValue(od)));
                case 153:
                    Type.JCPrimitiveType jCPrimitiveType = this.syms.booleanType;
                    if (intValue(od) != 0) {
                        z = false;
                    }
                    return jCPrimitiveType.constType(b2i(z));
                case 154:
                    Type.JCPrimitiveType jCPrimitiveType2 = this.syms.booleanType;
                    if (intValue(od) == 0) {
                        z = false;
                    }
                    return jCPrimitiveType2.constType(b2i(z));
                case 155:
                    Type.JCPrimitiveType jCPrimitiveType3 = this.syms.booleanType;
                    if (intValue(od) >= 0) {
                        z = false;
                    }
                    return jCPrimitiveType3.constType(b2i(z));
                case ByteCodes.ifge /* 156 */:
                    Type.JCPrimitiveType jCPrimitiveType4 = this.syms.booleanType;
                    if (intValue(od) < 0) {
                        z = false;
                    }
                    return jCPrimitiveType4.constType(b2i(z));
                case ByteCodes.ifgt /* 157 */:
                    Type.JCPrimitiveType jCPrimitiveType5 = this.syms.booleanType;
                    if (intValue(od) <= 0) {
                        z = false;
                    }
                    return jCPrimitiveType5.constType(b2i(z));
                case ByteCodes.ifle /* 158 */:
                    Type.JCPrimitiveType jCPrimitiveType6 = this.syms.booleanType;
                    if (intValue(od) > 0) {
                        z = false;
                    }
                    return jCPrimitiveType6.constType(b2i(z));
                case 257:
                    Type.JCPrimitiveType jCPrimitiveType7 = this.syms.booleanType;
                    if (intValue(od) != 0) {
                        z = false;
                    }
                    return jCPrimitiveType7.constType(b2i(z));
                default:
                    return null;
            }
        } catch (ArithmeticException e) {
            return null;
        }
    }

    Type fold2(int opcode, Type left, Type right) {
        try {
            if (opcode > 511) {
                Type t1 = fold2(opcode >> 9, left, right);
                return t1.constValue() == null ? t1 : fold1(opcode & 511, t1);
            }
            Object l = left.constValue();
            Object r = right.constValue();
            boolean z = true;
            switch (opcode) {
                case 96:
                    return this.syms.intType.constType(Integer.valueOf(intValue(l) + intValue(r)));
                case 97:
                    return this.syms.longType.constType(new Long(longValue(l) + longValue(r)));
                case 98:
                    return this.syms.floatType.constType(new Float(floatValue(l) + floatValue(r)));
                case 99:
                    return this.syms.doubleType.constType(new Double(doubleValue(l) + doubleValue(r)));
                case 100:
                    return this.syms.intType.constType(Integer.valueOf(intValue(l) - intValue(r)));
                case 101:
                    return this.syms.longType.constType(new Long(longValue(l) - longValue(r)));
                case 102:
                    return this.syms.floatType.constType(new Float(floatValue(l) - floatValue(r)));
                case 103:
                    return this.syms.doubleType.constType(new Double(doubleValue(l) - doubleValue(r)));
                case 104:
                    return this.syms.intType.constType(Integer.valueOf(intValue(l) * intValue(r)));
                case 105:
                    return this.syms.longType.constType(new Long(longValue(l) * longValue(r)));
                case 106:
                    return this.syms.floatType.constType(new Float(floatValue(l) * floatValue(r)));
                case 107:
                    return this.syms.doubleType.constType(new Double(doubleValue(l) * doubleValue(r)));
                case 108:
                    return this.syms.intType.constType(Integer.valueOf(intValue(l) / intValue(r)));
                case 109:
                    return this.syms.longType.constType(new Long(longValue(l) / longValue(r)));
                case 110:
                    return this.syms.floatType.constType(new Float(floatValue(l) / floatValue(r)));
                case 111:
                    return this.syms.doubleType.constType(new Double(doubleValue(l) / doubleValue(r)));
                case 112:
                    return this.syms.intType.constType(Integer.valueOf(intValue(l) % intValue(r)));
                case ByteCodes.lmod /* 113 */:
                    return this.syms.longType.constType(new Long(longValue(l) % longValue(r)));
                case ByteCodes.fmod /* 114 */:
                    return this.syms.floatType.constType(new Float(floatValue(l) % floatValue(r)));
                case 115:
                    return this.syms.doubleType.constType(new Double(doubleValue(l) % doubleValue(r)));
                case 120:
                case ByteCodes.ishll /* 270 */:
                    return this.syms.intType.constType(Integer.valueOf(intValue(l) << intValue(r)));
                case 121:
                case ByteCodes.lshll /* 271 */:
                    return this.syms.longType.constType(new Long(longValue(l) << intValue(r)));
                case 122:
                case 272:
                    return this.syms.intType.constType(Integer.valueOf(intValue(l) >> intValue(r)));
                case 123:
                case ByteCodes.lshrl /* 273 */:
                    return this.syms.longType.constType(new Long(longValue(l) >> intValue(r)));
                case 124:
                case ByteCodes.iushrl /* 274 */:
                    return this.syms.intType.constType(Integer.valueOf(intValue(l) >>> intValue(r)));
                case 125:
                    return this.syms.longType.constType(new Long(longValue(l) >>> intValue(r)));
                case 126:
                    return (left.hasTag(TypeTag.BOOLEAN) ? this.syms.booleanType : this.syms.intType).constType(Integer.valueOf(intValue(l) & intValue(r)));
                case 127:
                    return this.syms.longType.constType(new Long(longValue(l) & longValue(r)));
                case 128:
                    return (left.hasTag(TypeTag.BOOLEAN) ? this.syms.booleanType : this.syms.intType).constType(Integer.valueOf(intValue(l) | intValue(r)));
                case 129:
                    return this.syms.longType.constType(new Long(longValue(l) | longValue(r)));
                case 130:
                    return (left.hasTag(TypeTag.BOOLEAN) ? this.syms.booleanType : this.syms.intType).constType(Integer.valueOf(intValue(l) ^ intValue(r)));
                case 131:
                    return this.syms.longType.constType(new Long(longValue(l) ^ longValue(r)));
                case 148:
                    if (longValue(l) < longValue(r)) {
                        return this.syms.intType.constType(minusOne);
                    }
                    if (longValue(l) > longValue(r)) {
                        return this.syms.intType.constType(one);
                    }
                    return this.syms.intType.constType(zero);
                case 149:
                case 150:
                    if (floatValue(l) < floatValue(r)) {
                        return this.syms.intType.constType(minusOne);
                    }
                    if (floatValue(l) > floatValue(r)) {
                        return this.syms.intType.constType(one);
                    }
                    if (floatValue(l) == floatValue(r)) {
                        return this.syms.intType.constType(zero);
                    }
                    if (opcode == 150) {
                        return this.syms.intType.constType(one);
                    }
                    return this.syms.intType.constType(minusOne);
                case 151:
                case 152:
                    if (doubleValue(l) < doubleValue(r)) {
                        return this.syms.intType.constType(minusOne);
                    }
                    if (doubleValue(l) > doubleValue(r)) {
                        return this.syms.intType.constType(one);
                    }
                    if (doubleValue(l) == doubleValue(r)) {
                        return this.syms.intType.constType(zero);
                    }
                    if (opcode == 152) {
                        return this.syms.intType.constType(one);
                    }
                    return this.syms.intType.constType(minusOne);
                case ByteCodes.if_icmpeq /* 159 */:
                    Type.JCPrimitiveType jCPrimitiveType = this.syms.booleanType;
                    if (intValue(l) != intValue(r)) {
                        z = false;
                    }
                    return jCPrimitiveType.constType(b2i(z));
                case ByteCodes.if_icmpne /* 160 */:
                    Type.JCPrimitiveType jCPrimitiveType2 = this.syms.booleanType;
                    if (intValue(l) == intValue(r)) {
                        z = false;
                    }
                    return jCPrimitiveType2.constType(b2i(z));
                case ByteCodes.if_icmplt /* 161 */:
                    Type.JCPrimitiveType jCPrimitiveType3 = this.syms.booleanType;
                    if (intValue(l) >= intValue(r)) {
                        z = false;
                    }
                    return jCPrimitiveType3.constType(b2i(z));
                case ByteCodes.if_icmpge /* 162 */:
                    Type.JCPrimitiveType jCPrimitiveType4 = this.syms.booleanType;
                    if (intValue(l) < intValue(r)) {
                        z = false;
                    }
                    return jCPrimitiveType4.constType(b2i(z));
                case ByteCodes.if_icmpgt /* 163 */:
                    Type.JCPrimitiveType jCPrimitiveType5 = this.syms.booleanType;
                    if (intValue(l) <= intValue(r)) {
                        z = false;
                    }
                    return jCPrimitiveType5.constType(b2i(z));
                case ByteCodes.if_icmple /* 164 */:
                    Type.JCPrimitiveType jCPrimitiveType6 = this.syms.booleanType;
                    if (intValue(l) > intValue(r)) {
                        z = false;
                    }
                    return jCPrimitiveType6.constType(b2i(z));
                case ByteCodes.if_acmpeq /* 165 */:
                    return this.syms.booleanType.constType(b2i(l.equals(r)));
                case ByteCodes.if_acmpne /* 166 */:
                    Type.JCPrimitiveType jCPrimitiveType7 = this.syms.booleanType;
                    if (l.equals(r)) {
                        z = false;
                    }
                    return jCPrimitiveType7.constType(b2i(z));
                case 256:
                    return this.syms.stringType.constType(left.stringValue() + right.stringValue());
                case 258:
                    Type.JCPrimitiveType jCPrimitiveType8 = this.syms.booleanType;
                    if ((intValue(l) & intValue(r)) == 0) {
                        z = false;
                    }
                    return jCPrimitiveType8.constType(b2i(z));
                case 259:
                    Type.JCPrimitiveType jCPrimitiveType9 = this.syms.booleanType;
                    if ((intValue(l) | intValue(r)) == 0) {
                        z = false;
                    }
                    return jCPrimitiveType9.constType(b2i(z));
                default:
                    return null;
            }
        } catch (ArithmeticException e) {
            return null;
        }
    }

    Type coerce(Type etype, Type ttype) {
        if (etype.tsym.type == ttype.tsym.type) {
            return etype;
        }
        if (etype.isNumeric()) {
            Object n = etype.constValue();
            switch (ttype.getTag()) {
                case BYTE:
                    return this.syms.byteType.constType(Integer.valueOf(((byte) intValue(n)) + 0));
                case CHAR:
                    return this.syms.charType.constType(Integer.valueOf(((char) intValue(n)) + 0));
                case SHORT:
                    return this.syms.shortType.constType(Integer.valueOf(((short) intValue(n)) + 0));
                case INT:
                    return this.syms.intType.constType(Integer.valueOf(intValue(n)));
                case LONG:
                    return this.syms.longType.constType(Long.valueOf(longValue(n)));
                case FLOAT:
                    return this.syms.floatType.constType(Float.valueOf(floatValue(n)));
                case DOUBLE:
                    return this.syms.doubleType.constType(Double.valueOf(doubleValue(n)));
            }
        }
        return ttype;
    }
}
