package com.sun.tools.javac.jvm;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeAnnotationPosition;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.jvm.ClassWriter;
import com.sun.tools.javac.jvm.Pool;
import com.sun.tools.javac.util.ArrayUtils;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Bits;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Position;
import java.util.ArrayList;
import java.util.Iterator;

/* JADX INFO: loaded from: classes.dex */
public class Code {
    static final Type jsrReturnValue = new Type.JCPrimitiveType(TypeTag.INT, null);
    public CRTable crt;
    public final boolean debugCode;
    public boolean fatcode;
    boolean lineDebugInfo;
    Position.LineMap lineMap;
    LocalVar[] lvar;
    final Symbol.MethodSymbol meth;
    public final boolean needStackMap;
    final Pool pool;
    StackMapFormat stackMap;
    State state;
    final Symtab syms;
    final Types types;
    LocalVar[] varBuffer;
    int varBufferSize;
    boolean varDebugInfo;
    public int max_stack = 0;
    public int max_locals = 0;
    public byte[] code = new byte[64];
    public int cp = 0;
    ListBuffer<char[]> catchInfo = new ListBuffer<>();
    List<char[]> lineInfo = List.nil();
    private boolean alive = true;
    private boolean fixedPc = false;
    public int nextreg = 0;
    Chain pendingJumps = null;
    int pendingStatPos = -1;
    boolean pendingStackMap = false;
    StackMapFrame[] stackMapBuffer = null;
    ClassWriter.StackMapTableFrame[] stackMapTableBuffer = null;
    int stackMapBufferSize = 0;
    int lastStackMapPC = -1;
    StackMapFrame lastFrame = null;
    StackMapFrame frameBeforeLast = null;

    public enum StackMapFormat {
        NONE,
        CLDC { // from class: com.sun.tools.javac.jvm.Code.StackMapFormat.1
            @Override // com.sun.tools.javac.jvm.Code.StackMapFormat
            Name getAttributeName(Names names) {
                return names.StackMap;
            }
        },
        JSR202 { // from class: com.sun.tools.javac.jvm.Code.StackMapFormat.2
            @Override // com.sun.tools.javac.jvm.Code.StackMapFormat
            Name getAttributeName(Names names) {
                return names.StackMapTable;
            }
        };

        Name getAttributeName(Names names) {
            return names.empty;
        }
    }

    public boolean checkLimits(JCDiagnostic.DiagnosticPosition pos, Log log) {
        if (this.cp > 65535) {
            log.error(pos, "limit.code", new Object[0]);
            return true;
        }
        if (this.max_locals > 65535) {
            log.error(pos, "limit.locals", new Object[0]);
            return true;
        }
        if (this.max_stack <= 65535) {
            return false;
        }
        log.error(pos, "limit.stack", new Object[0]);
        return true;
    }

    public Code(Symbol.MethodSymbol meth, boolean fatcode, Position.LineMap lineMap, boolean varDebugInfo, StackMapFormat stackMap, boolean debugCode, CRTable crt, Symtab syms, Types types, Pool pool) {
        boolean z;
        this.meth = meth;
        this.fatcode = fatcode;
        this.lineMap = lineMap;
        if (lineMap == null) {
            z = false;
        } else {
            z = true;
        }
        this.lineDebugInfo = z;
        this.varDebugInfo = varDebugInfo;
        this.crt = crt;
        this.syms = syms;
        this.types = types;
        this.debugCode = debugCode;
        this.stackMap = stackMap;
        switch (stackMap) {
            case CLDC:
            case JSR202:
                this.needStackMap = true;
                break;
            default:
                this.needStackMap = false;
                break;
        }
        this.state = new State();
        this.lvar = new LocalVar[20];
        this.pool = pool;
    }

    public static int typecode(Type type) {
        switch (type.getTag()) {
            case BYTE:
                return 5;
            case SHORT:
                return 7;
            case CHAR:
                return 6;
            case INT:
                return 0;
            case LONG:
                return 1;
            case FLOAT:
                return 2;
            case DOUBLE:
                return 3;
            case BOOLEAN:
                return 5;
            case VOID:
                return 8;
            case CLASS:
            case ARRAY:
            case METHOD:
            case BOT:
            case TYPEVAR:
            case UNINITIALIZED_THIS:
            case UNINITIALIZED_OBJECT:
                return 4;
            default:
                throw new AssertionError("typecode " + type.getTag());
        }
    }

    public static int truncate(int tc) {
        switch (tc) {
            case 5:
            case 6:
            case 7:
                return 0;
            default:
                return tc;
        }
    }

    public static int width(int typecode) {
        switch (typecode) {
            case 1:
            case 3:
                return 2;
            case 8:
                return 0;
            default:
                return 1;
        }
    }

    public static int width(Type type) {
        if (type == null) {
            return 1;
        }
        return width(typecode(type));
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static int width(List<Type> types) {
        int w = 0;
        for (List list = types; list.nonEmpty(); list = list.tail) {
            w += width((Type) list.head);
        }
        return w;
    }

    public static int arraycode(Type type) {
        switch (type.getTag()) {
            case BYTE:
                return 8;
            case SHORT:
                return 9;
            case CHAR:
                return 5;
            case INT:
                return 10;
            case LONG:
                return 11;
            case FLOAT:
                return 6;
            case DOUBLE:
                return 7;
            case BOOLEAN:
                return 4;
            case VOID:
            default:
                throw new AssertionError("arraycode " + type);
            case CLASS:
                return 0;
            case ARRAY:
                return 1;
        }
    }

    public int curCP() {
        if (this.pendingJumps != null) {
            resolvePending();
        }
        if (this.pendingStatPos != -1) {
            markStatBegin();
        }
        this.fixedPc = true;
        return this.cp;
    }

    private void emit1(int od) {
        if (this.alive) {
            this.code = ArrayUtils.ensureCapacity(this.code, this.cp);
            byte[] bArr = this.code;
            int i = this.cp;
            this.cp = i + 1;
            bArr[i] = (byte) od;
        }
    }

    private void emit2(int od) {
        if (this.alive) {
            if (this.cp + 2 > this.code.length) {
                emit1(od >> 8);
                emit1(od);
                return;
            }
            byte[] bArr = this.code;
            int i = this.cp;
            this.cp = i + 1;
            bArr[i] = (byte) (od >> 8);
            byte[] bArr2 = this.code;
            int i2 = this.cp;
            this.cp = i2 + 1;
            bArr2[i2] = (byte) od;
        }
    }

    public void emit4(int od) {
        if (this.alive) {
            if (this.cp + 4 > this.code.length) {
                emit1(od >> 24);
                emit1(od >> 16);
                emit1(od >> 8);
                emit1(od);
                return;
            }
            byte[] bArr = this.code;
            int i = this.cp;
            this.cp = i + 1;
            bArr[i] = (byte) (od >> 24);
            byte[] bArr2 = this.code;
            int i2 = this.cp;
            this.cp = i2 + 1;
            bArr2[i2] = (byte) (od >> 16);
            byte[] bArr3 = this.code;
            int i3 = this.cp;
            this.cp = i3 + 1;
            bArr3[i3] = (byte) (od >> 8);
            byte[] bArr4 = this.code;
            int i4 = this.cp;
            this.cp = i4 + 1;
            bArr4[i4] = (byte) od;
        }
    }

    private void emitop(int op) {
        if (this.pendingJumps != null) {
            resolvePending();
        }
        if (this.alive) {
            if (this.pendingStatPos != -1) {
                markStatBegin();
            }
            if (this.pendingStackMap) {
                this.pendingStackMap = false;
                emitStackMap();
            }
            if (this.debugCode) {
                System.err.println("emit@" + this.cp + " stack=" + this.state.stacksize + ": " + mnem(op));
            }
            emit1(op);
        }
    }

    void postop() {
        Assert.check(this.alive || this.state.stacksize == 0);
    }

    public void emitLdc(int od) {
        if (od <= 255) {
            emitop1(18, od);
        } else {
            emitop2(19, od);
        }
    }

    public void emitMultianewarray(int ndims, int type, Type arrayType) {
        emitop(ByteCodes.multianewarray);
        if (this.alive) {
            emit2(type);
            emit1(ndims);
            this.state.pop(ndims);
            this.state.push(arrayType);
        }
    }

    public void emitNewarray(int elemcode, Type arrayType) {
        emitop(ByteCodes.newarray);
        if (this.alive) {
            emit1(elemcode);
            this.state.pop(1);
            this.state.push(arrayType);
        }
    }

    public void emitAnewarray(int od, Type arrayType) {
        emitop(ByteCodes.anewarray);
        if (this.alive) {
            emit2(od);
            this.state.pop(1);
            this.state.push(arrayType);
        }
    }

    public void emitInvokeinterface(int meth, Type mtype) {
        int argsize = width(mtype.mo176getParameterTypes());
        emitop(ByteCodes.invokeinterface);
        if (this.alive) {
            emit2(meth);
            emit1(argsize + 1);
            emit1(0);
            this.state.pop(argsize + 1);
            this.state.push(mtype.mo178getReturnType());
        }
    }

    public void emitInvokespecial(int meth, Type mtype) {
        int argsize = width(mtype.mo176getParameterTypes());
        emitop(ByteCodes.invokespecial);
        if (this.alive) {
            emit2(meth);
            Symbol sym = (Symbol) this.pool.pool[meth];
            this.state.pop(argsize);
            if (sym.isConstructor()) {
                this.state.markInitialized((UninitializedType) this.state.peek());
            }
            this.state.pop(1);
            this.state.push(mtype.mo178getReturnType());
        }
    }

    public void emitInvokestatic(int meth, Type mtype) {
        int argsize = width(mtype.mo176getParameterTypes());
        emitop(ByteCodes.invokestatic);
        if (this.alive) {
            emit2(meth);
            this.state.pop(argsize);
            this.state.push(mtype.mo178getReturnType());
        }
    }

    public void emitInvokevirtual(int meth, Type mtype) {
        int argsize = width(mtype.mo176getParameterTypes());
        emitop(ByteCodes.invokevirtual);
        if (this.alive) {
            emit2(meth);
            this.state.pop(argsize + 1);
            this.state.push(mtype.mo178getReturnType());
        }
    }

    public void emitInvokedynamic(int desc, Type mtype) {
        int argsize = width(mtype.mo176getParameterTypes());
        emitop(ByteCodes.invokedynamic);
        if (this.alive) {
            emit2(desc);
            emit2(0);
            this.state.pop(argsize);
            this.state.push(mtype.mo178getReturnType());
        }
    }

    public void emitop0(int op) {
        emitop(op);
        if (this.alive) {
            switch (op) {
                case 0:
                case 116:
                case 117:
                case 118:
                case 119:
                case 145:
                case 146:
                case 147:
                    break;
                case 1:
                    this.state.push(this.syms.botType);
                    break;
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 26:
                case 27:
                case 28:
                case 29:
                    this.state.push(this.syms.intType);
                    break;
                case 9:
                case 10:
                case 30:
                case 31:
                case 32:
                case 33:
                    this.state.push(this.syms.longType);
                    break;
                case 11:
                case 12:
                case 13:
                case 34:
                case 35:
                case 36:
                case 37:
                    this.state.push(this.syms.floatType);
                    break;
                case 14:
                case 15:
                case 38:
                case 39:
                case 40:
                case 41:
                    this.state.push(this.syms.doubleType);
                    break;
                case 16:
                case 17:
                case 18:
                case 19:
                case 20:
                case 21:
                case 22:
                case 23:
                case 24:
                case 25:
                case 54:
                case 55:
                case 56:
                case 57:
                case 58:
                case 132:
                case 153:
                case 154:
                case 155:
                case ByteCodes.ifge /* 156 */:
                case ByteCodes.ifgt /* 157 */:
                case ByteCodes.ifle /* 158 */:
                case ByteCodes.if_icmpeq /* 159 */:
                case ByteCodes.if_icmpne /* 160 */:
                case ByteCodes.if_icmplt /* 161 */:
                case ByteCodes.if_icmpge /* 162 */:
                case ByteCodes.if_icmpgt /* 163 */:
                case ByteCodes.if_icmple /* 164 */:
                case ByteCodes.if_acmpeq /* 165 */:
                case ByteCodes.if_acmpne /* 166 */:
                case 168:
                case ByteCodes.getstatic /* 178 */:
                case ByteCodes.putstatic /* 179 */:
                case ByteCodes.getfield /* 180 */:
                case ByteCodes.putfield /* 181 */:
                case ByteCodes.invokevirtual /* 182 */:
                case ByteCodes.invokespecial /* 183 */:
                case ByteCodes.invokestatic /* 184 */:
                case ByteCodes.invokeinterface /* 185 */:
                case ByteCodes.invokedynamic /* 186 */:
                case ByteCodes.new_ /* 187 */:
                case ByteCodes.newarray /* 188 */:
                case ByteCodes.anewarray /* 189 */:
                case ByteCodes.checkcast /* 192 */:
                case ByteCodes.instanceof_ /* 193 */:
                default:
                    throw new AssertionError(mnem(op));
                case 42:
                    this.state.push(this.lvar[0].sym.type);
                    break;
                case 43:
                    this.state.push(this.lvar[1].sym.type);
                    break;
                case 44:
                    this.state.push(this.lvar[2].sym.type);
                    break;
                case 45:
                    this.state.push(this.lvar[3].sym.type);
                    break;
                case 46:
                case 51:
                case 52:
                case 53:
                    this.state.pop(2);
                    this.state.push(this.syms.intType);
                    break;
                case 47:
                    this.state.pop(2);
                    this.state.push(this.syms.longType);
                    break;
                case 48:
                    this.state.pop(2);
                    this.state.push(this.syms.floatType);
                    break;
                case 49:
                    this.state.pop(2);
                    this.state.push(this.syms.doubleType);
                    break;
                case 50:
                    this.state.pop(1);
                    Type a = this.state.stack[this.state.stacksize - 1];
                    this.state.pop(1);
                    Type stackType = a.hasTag(TypeTag.BOT) ? this.syms.objectType : this.types.erasure(this.types.elemtype(a));
                    this.state.push(stackType);
                    break;
                case 59:
                case 60:
                case 61:
                case 62:
                case 67:
                case 68:
                case 69:
                case 70:
                case 75:
                case 76:
                case 77:
                case 78:
                case 87:
                case 121:
                case 123:
                case 125:
                    this.state.pop(1);
                    break;
                case 63:
                case 64:
                case 65:
                case 66:
                case 71:
                case 72:
                case 73:
                case 74:
                case 88:
                    this.state.pop(2);
                    break;
                case 79:
                case 81:
                case 85:
                case 86:
                    this.state.pop(3);
                    break;
                case 80:
                case 82:
                    this.state.pop(4);
                    break;
                case 83:
                    this.state.pop(3);
                    break;
                case 84:
                    this.state.pop(3);
                    break;
                case 89:
                    this.state.push(this.state.stack[this.state.stacksize - 1]);
                    break;
                case 90:
                    Type val1 = this.state.pop1();
                    Type val2 = this.state.pop1();
                    this.state.push(val1);
                    this.state.push(val2);
                    this.state.push(val1);
                    break;
                case 91:
                    Type value1 = this.state.pop1();
                    if (this.state.stack[this.state.stacksize - 1] != null) {
                        Type value2 = this.state.pop1();
                        Type value3 = this.state.pop1();
                        this.state.push(value1);
                        this.state.push(value3);
                        this.state.push(value2);
                        this.state.push(value1);
                    } else {
                        Type value22 = this.state.pop2();
                        this.state.push(value1);
                        this.state.push(value22);
                        this.state.push(value1);
                    }
                    break;
                case 92:
                    if (this.state.stack[this.state.stacksize - 1] != null) {
                        Type value12 = this.state.pop1();
                        Type value23 = this.state.pop1();
                        this.state.push(value23);
                        this.state.push(value12);
                        this.state.push(value23);
                        this.state.push(value12);
                    } else {
                        Type value = this.state.pop2();
                        this.state.push(value);
                        this.state.push(value);
                    }
                    break;
                case 93:
                    if (this.state.stack[this.state.stacksize - 1] != null) {
                        Type value13 = this.state.pop1();
                        Type value24 = this.state.pop1();
                        Type value32 = this.state.pop1();
                        this.state.push(value24);
                        this.state.push(value13);
                        this.state.push(value32);
                        this.state.push(value24);
                        this.state.push(value13);
                    } else {
                        Type value14 = this.state.pop2();
                        Type value25 = this.state.pop1();
                        this.state.push(value14);
                        this.state.push(value25);
                        this.state.push(value14);
                    }
                    break;
                case 94:
                    if (this.state.stack[this.state.stacksize - 1] != null) {
                        Type value15 = this.state.pop1();
                        Type value26 = this.state.pop1();
                        if (this.state.stack[this.state.stacksize - 1] != null) {
                            Type value33 = this.state.pop1();
                            Type value4 = this.state.pop1();
                            this.state.push(value26);
                            this.state.push(value15);
                            this.state.push(value4);
                            this.state.push(value33);
                            this.state.push(value26);
                            this.state.push(value15);
                        } else {
                            Type value34 = this.state.pop2();
                            this.state.push(value26);
                            this.state.push(value15);
                            this.state.push(value34);
                            this.state.push(value26);
                            this.state.push(value15);
                        }
                    } else {
                        Type value16 = this.state.pop2();
                        if (this.state.stack[this.state.stacksize - 1] != null) {
                            Type value27 = this.state.pop1();
                            Type value35 = this.state.pop1();
                            this.state.push(value16);
                            this.state.push(value35);
                            this.state.push(value27);
                            this.state.push(value16);
                        } else {
                            Type value28 = this.state.pop2();
                            this.state.push(value16);
                            this.state.push(value28);
                            this.state.push(value16);
                        }
                    }
                    break;
                case 95:
                    Type value17 = this.state.pop1();
                    Type value29 = this.state.pop1();
                    this.state.push(value17);
                    this.state.push(value29);
                    break;
                case 96:
                case 100:
                case 104:
                case 108:
                case 112:
                case 120:
                case 122:
                case 124:
                case 126:
                case 128:
                case 130:
                    this.state.pop(1);
                    break;
                case 97:
                case 101:
                case 105:
                case 109:
                case ByteCodes.lmod /* 113 */:
                case 127:
                case 129:
                case 131:
                    this.state.pop(2);
                    break;
                case 98:
                case 102:
                case 106:
                case 110:
                case ByteCodes.fmod /* 114 */:
                    this.state.pop(1);
                    break;
                case 99:
                case 103:
                case 107:
                case 111:
                case 115:
                    this.state.pop(2);
                    break;
                case 133:
                    this.state.pop(1);
                    this.state.push(this.syms.longType);
                    break;
                case 134:
                    this.state.pop(1);
                    this.state.push(this.syms.floatType);
                    break;
                case 135:
                    this.state.pop(1);
                    this.state.push(this.syms.doubleType);
                    break;
                case 136:
                    this.state.pop(2);
                    this.state.push(this.syms.intType);
                    break;
                case 137:
                    this.state.pop(2);
                    this.state.push(this.syms.floatType);
                    break;
                case 138:
                    this.state.pop(2);
                    this.state.push(this.syms.doubleType);
                    break;
                case 139:
                    this.state.pop(1);
                    this.state.push(this.syms.intType);
                    break;
                case 140:
                    this.state.pop(1);
                    this.state.push(this.syms.longType);
                    break;
                case 141:
                    this.state.pop(1);
                    this.state.push(this.syms.doubleType);
                    break;
                case 142:
                    this.state.pop(2);
                    this.state.push(this.syms.intType);
                    break;
                case 143:
                    this.state.pop(2);
                    this.state.push(this.syms.longType);
                    break;
                case 144:
                    this.state.pop(2);
                    this.state.push(this.syms.floatType);
                    break;
                case 148:
                    this.state.pop(4);
                    this.state.push(this.syms.intType);
                    break;
                case 149:
                case 150:
                    this.state.pop(2);
                    this.state.push(this.syms.intType);
                    break;
                case 151:
                case 152:
                    this.state.pop(4);
                    this.state.push(this.syms.intType);
                    break;
                case ByteCodes.goto_ /* 167 */:
                    markDead();
                    break;
                case ByteCodes.ret /* 169 */:
                    markDead();
                    break;
                case ByteCodes.tableswitch /* 170 */:
                case ByteCodes.lookupswitch /* 171 */:
                    this.state.pop(1);
                    break;
                case ByteCodes.ireturn /* 172 */:
                case ByteCodes.freturn /* 174 */:
                case ByteCodes.areturn /* 176 */:
                    Assert.check(this.state.nlocks == 0);
                    this.state.pop(1);
                    markDead();
                    break;
                case 173:
                case ByteCodes.dreturn /* 175 */:
                    Assert.check(this.state.nlocks == 0);
                    this.state.pop(2);
                    markDead();
                    break;
                case ByteCodes.return_ /* 177 */:
                    Assert.check(this.state.nlocks == 0);
                    markDead();
                    break;
                case ByteCodes.arraylength /* 190 */:
                    this.state.pop(1);
                    this.state.push(this.syms.intType);
                    break;
                case ByteCodes.athrow /* 191 */:
                    this.state.pop(1);
                    markDead();
                    break;
                case ByteCodes.monitorenter /* 194 */:
                case ByteCodes.monitorexit /* 195 */:
                    this.state.pop(1);
                    break;
                case ByteCodes.wide /* 196 */:
                    return;
            }
            postop();
        }
    }

    public void emitop1(int op, int od) {
        emitop(op);
        if (this.alive) {
            emit1(od);
            switch (op) {
                case 16:
                    this.state.push(this.syms.intType);
                    break;
                case 17:
                default:
                    throw new AssertionError(mnem(op));
                case 18:
                    this.state.push(typeForPool(this.pool.pool[od]));
                    break;
            }
            postop();
        }
    }

    private Type typeForPool(Object o) {
        if (o instanceof Integer) {
            return this.syms.intType;
        }
        if (o instanceof Float) {
            return this.syms.floatType;
        }
        if (o instanceof String) {
            return this.syms.stringType;
        }
        if (o instanceof Long) {
            return this.syms.longType;
        }
        if (o instanceof Double) {
            return this.syms.doubleType;
        }
        if (o instanceof Symbol.ClassSymbol) {
            return this.syms.classType;
        }
        if (o instanceof Pool.MethodHandle) {
            return this.syms.methodHandleType;
        }
        if (o instanceof Types.UniqueType) {
            return typeForPool(((Types.UniqueType) o).type);
        }
        if (o instanceof Type) {
            Type ty = ((Type) o).unannotatedType();
            if (ty instanceof Type.ArrayType) {
                return this.syms.classType;
            }
            if (ty instanceof Type.MethodType) {
                return this.syms.methodTypeType;
            }
        }
        throw new AssertionError("Invalid type of constant pool entry: " + o.getClass());
    }

    public void emitop1w(int op, int od) {
        if (od > 255) {
            emitop(ByteCodes.wide);
            emitop(op);
            emit2(od);
        } else {
            emitop(op);
            emit1(od);
        }
        if (this.alive) {
            switch (op) {
                case 21:
                    this.state.push(this.syms.intType);
                    break;
                case 22:
                    this.state.push(this.syms.longType);
                    break;
                case 23:
                    this.state.push(this.syms.floatType);
                    break;
                case 24:
                    this.state.push(this.syms.doubleType);
                    break;
                case 25:
                    this.state.push(this.lvar[od].sym.type);
                    break;
                case 54:
                case 56:
                case 58:
                    this.state.pop(1);
                    break;
                case 55:
                case 57:
                    this.state.pop(2);
                    break;
                case ByteCodes.ret /* 169 */:
                    markDead();
                    break;
                default:
                    throw new AssertionError(mnem(op));
            }
            postop();
        }
    }

    public void emitop1w(int op, int od1, int od2) {
        if (od1 > 255 || od2 < -128 || od2 > 127) {
            emitop(ByteCodes.wide);
            emitop(op);
            emit2(od1);
            emit2(od2);
        } else {
            emitop(op);
            emit1(od1);
            emit1(od2);
        }
        if (this.alive) {
            switch (op) {
                case 132:
                    return;
                default:
                    throw new AssertionError(mnem(op));
            }
        }
    }

    public void emitop2(int op, int od) {
        Symbol sym;
        Type t;
        emitop(op);
        if (this.alive) {
            emit2(od);
            switch (op) {
                case 17:
                    this.state.push(this.syms.intType);
                    return;
                case 19:
                    this.state.push(typeForPool(this.pool.pool[od]));
                    return;
                case 20:
                    this.state.push(typeForPool(this.pool.pool[od]));
                    return;
                case 153:
                case 154:
                case 155:
                case ByteCodes.ifge /* 156 */:
                case ByteCodes.ifgt /* 157 */:
                case ByteCodes.ifle /* 158 */:
                case ByteCodes.if_acmp_null /* 198 */:
                case ByteCodes.if_acmp_nonnull /* 199 */:
                    this.state.pop(1);
                    return;
                case ByteCodes.if_icmpeq /* 159 */:
                case ByteCodes.if_icmpne /* 160 */:
                case ByteCodes.if_icmplt /* 161 */:
                case ByteCodes.if_icmpge /* 162 */:
                case ByteCodes.if_icmpgt /* 163 */:
                case ByteCodes.if_icmple /* 164 */:
                case ByteCodes.if_acmpeq /* 165 */:
                case ByteCodes.if_acmpne /* 166 */:
                    this.state.pop(2);
                    return;
                case ByteCodes.goto_ /* 167 */:
                    markDead();
                    return;
                case 168:
                    return;
                case ByteCodes.getstatic /* 178 */:
                    this.state.push(((Symbol) this.pool.pool[od]).erasure(this.types));
                    return;
                case ByteCodes.putstatic /* 179 */:
                    this.state.pop(((Symbol) this.pool.pool[od]).erasure(this.types));
                    return;
                case ByteCodes.getfield /* 180 */:
                    this.state.pop(1);
                    this.state.push(((Symbol) this.pool.pool[od]).erasure(this.types));
                    return;
                case ByteCodes.putfield /* 181 */:
                    this.state.pop(((Symbol) this.pool.pool[od]).erasure(this.types));
                    this.state.pop(1);
                    return;
                case ByteCodes.new_ /* 187 */:
                    if (this.pool.pool[od] instanceof Types.UniqueType) {
                        sym = ((Types.UniqueType) this.pool.pool[od]).type.tsym;
                    } else {
                        sym = (Symbol) this.pool.pool[od];
                    }
                    this.state.push(UninitializedType.uninitializedObject(sym.erasure(this.types), this.cp - 3));
                    return;
                case ByteCodes.checkcast /* 192 */:
                    this.state.pop(1);
                    Object o = this.pool.pool[od];
                    if (o instanceof Symbol) {
                        t = ((Symbol) o).erasure(this.types);
                    } else {
                        t = this.types.erasure(((Types.UniqueType) o).type);
                    }
                    this.state.push(t);
                    return;
                case ByteCodes.instanceof_ /* 193 */:
                    this.state.pop(1);
                    this.state.push(this.syms.intType);
                    return;
                default:
                    throw new AssertionError(mnem(op));
            }
        }
    }

    public void emitop4(int op, int od) {
        emitop(op);
        if (this.alive) {
            emit4(od);
            switch (op) {
                case 200:
                    markDead();
                    return;
                case ByteCodes.jsr_w /* 201 */:
                    return;
                default:
                    throw new AssertionError(mnem(op));
            }
        }
    }

    public void align(int incr) {
        if (this.alive) {
            while (this.cp % incr != 0) {
                emitop0(0);
            }
        }
    }

    private void put1(int pc, int op) {
        this.code[pc] = (byte) op;
    }

    private void put2(int pc, int od) {
        put1(pc, od >> 8);
        put1(pc + 1, od);
    }

    public void put4(int pc, int od) {
        put1(pc, od >> 24);
        put1(pc + 1, od >> 16);
        put1(pc + 2, od >> 8);
        put1(pc + 3, od);
    }

    private int get1(int pc) {
        return this.code[pc] & 255;
    }

    private int get2(int pc) {
        return (get1(pc) << 8) | get1(pc + 1);
    }

    public int get4(int pc) {
        return (get1(pc) << 24) | (get1(pc + 1) << 16) | (get1(pc + 2) << 8) | get1(pc + 3);
    }

    public boolean isAlive() {
        return this.alive || this.pendingJumps != null;
    }

    public void markDead() {
        this.alive = false;
    }

    public int entryPoint() {
        int pc = curCP();
        this.alive = true;
        this.pendingStackMap = this.needStackMap;
        return pc;
    }

    public int entryPoint(State state) {
        int pc = curCP();
        this.alive = true;
        State newState = state.dup();
        setDefined(newState.defined);
        this.state = newState;
        Assert.check(state.stacksize <= this.max_stack);
        if (this.debugCode) {
            System.err.println("entry point " + state);
        }
        this.pendingStackMap = this.needStackMap;
        return pc;
    }

    public int entryPoint(State state, Type pushed) {
        int pc = curCP();
        this.alive = true;
        State newState = state.dup();
        setDefined(newState.defined);
        this.state = newState;
        Assert.check(state.stacksize <= this.max_stack);
        this.state.push(pushed);
        if (this.debugCode) {
            System.err.println("entry point " + state);
        }
        this.pendingStackMap = this.needStackMap;
        return pc;
    }

    static class StackMapFrame {
        Type[] locals;
        int pc;
        Type[] stack;

        StackMapFrame() {
        }
    }

    public void emitStackMap() {
        int pc = curCP();
        if (this.needStackMap) {
            switch (this.stackMap) {
                case CLDC:
                    emitCLDCStackMap(pc, getLocalsSize());
                    break;
                case JSR202:
                    emitStackMapFrame(pc, getLocalsSize());
                    break;
                default:
                    throw new AssertionError("Should have chosen a stackmap format");
            }
            if (this.debugCode) {
                this.state.dump(pc);
            }
        }
    }

    private int getLocalsSize() {
        for (int i = this.max_locals - 1; i >= 0; i--) {
            if (this.state.defined.isMember(i) && this.lvar[i] != null) {
                int nextLocal = i + width(this.lvar[i].sym.erasure(this.types));
                return nextLocal;
            }
        }
        return 0;
    }

    void emitCLDCStackMap(int pc, int localsSize) {
        if (this.lastStackMapPC == pc) {
            StackMapFrame[] stackMapFrameArr = this.stackMapBuffer;
            int i = this.stackMapBufferSize - 1;
            this.stackMapBufferSize = i;
            stackMapFrameArr[i] = null;
        }
        this.lastStackMapPC = pc;
        if (this.stackMapBuffer == null) {
            this.stackMapBuffer = new StackMapFrame[20];
        } else {
            this.stackMapBuffer = (StackMapFrame[]) ArrayUtils.ensureCapacity(this.stackMapBuffer, this.stackMapBufferSize);
        }
        StackMapFrame[] stackMapFrameArr2 = this.stackMapBuffer;
        int i2 = this.stackMapBufferSize;
        this.stackMapBufferSize = i2 + 1;
        StackMapFrame frame = new StackMapFrame();
        stackMapFrameArr2[i2] = frame;
        frame.pc = pc;
        frame.locals = new Type[localsSize];
        for (int i3 = 0; i3 < localsSize; i3++) {
            if (this.state.defined.isMember(i3) && this.lvar[i3] != null) {
                Type vtype = this.lvar[i3].sym.type;
                if (!(vtype instanceof UninitializedType)) {
                    vtype = this.types.erasure(vtype);
                }
                frame.locals[i3] = vtype;
            }
        }
        frame.stack = new Type[this.state.stacksize];
        for (int i4 = 0; i4 < this.state.stacksize; i4++) {
            frame.stack[i4] = this.state.stack[i4];
        }
    }

    void emitStackMapFrame(int pc, int localsSize) {
        if (this.lastFrame == null) {
            this.lastFrame = getInitialFrame();
        } else if (this.lastFrame.pc == pc) {
            ClassWriter.StackMapTableFrame[] stackMapTableFrameArr = this.stackMapTableBuffer;
            int i = this.stackMapBufferSize - 1;
            this.stackMapBufferSize = i;
            stackMapTableFrameArr[i] = null;
            this.lastFrame = this.frameBeforeLast;
            this.frameBeforeLast = null;
        }
        StackMapFrame frame = new StackMapFrame();
        frame.pc = pc;
        int localCount = 0;
        Type[] locals = new Type[localsSize];
        int i2 = 0;
        while (i2 < localsSize) {
            if (this.state.defined.isMember(i2) && this.lvar[i2] != null) {
                Type vtype = this.lvar[i2].sym.type;
                if (!(vtype instanceof UninitializedType)) {
                    vtype = this.types.erasure(vtype);
                }
                locals[i2] = vtype;
                if (width(vtype) > 1) {
                    i2++;
                }
            }
            i2++;
            localCount++;
        }
        frame.locals = new Type[localCount];
        int i3 = 0;
        int j = 0;
        while (i3 < localsSize) {
            Assert.check(j < localCount);
            frame.locals[j] = locals[i3];
            if (width(locals[i3]) > 1) {
                i3++;
            }
            i3++;
            j++;
        }
        int stackCount = 0;
        for (int i4 = 0; i4 < this.state.stacksize; i4++) {
            if (this.state.stack[i4] != null) {
                stackCount++;
            }
        }
        frame.stack = new Type[stackCount];
        int stackCount2 = 0;
        for (int i5 = 0; i5 < this.state.stacksize; i5++) {
            if (this.state.stack[i5] != null) {
                frame.stack[stackCount2] = this.types.erasure(this.state.stack[i5]);
                stackCount2++;
            }
        }
        if (this.stackMapTableBuffer == null) {
            this.stackMapTableBuffer = new ClassWriter.StackMapTableFrame[20];
        } else {
            this.stackMapTableBuffer = (ClassWriter.StackMapTableFrame[]) ArrayUtils.ensureCapacity(this.stackMapTableBuffer, this.stackMapBufferSize);
        }
        ClassWriter.StackMapTableFrame[] stackMapTableFrameArr2 = this.stackMapTableBuffer;
        int i6 = this.stackMapBufferSize;
        this.stackMapBufferSize = i6 + 1;
        stackMapTableFrameArr2[i6] = ClassWriter.StackMapTableFrame.getInstance(frame, this.lastFrame.pc, this.lastFrame.locals, this.types);
        this.frameBeforeLast = this.lastFrame;
        this.lastFrame = frame;
    }

    StackMapFrame getInitialFrame() {
        StackMapFrame frame = new StackMapFrame();
        List<Type> arg_types = ((Type.MethodType) this.meth.externalType(this.types)).argtypes;
        int len = arg_types.length();
        int count = 0;
        if (!this.meth.isStatic()) {
            Type thisType = this.meth.owner.type;
            frame.locals = new Type[len + 1];
            if (!this.meth.isConstructor() || thisType == this.syms.objectType) {
                int count2 = 0 + 1;
                frame.locals[0] = this.types.erasure(thisType);
                count = count2;
            } else {
                int count3 = 0 + 1;
                frame.locals[0] = UninitializedType.uninitializedThis(thisType);
                count = count3;
            }
        } else {
            frame.locals = new Type[len];
        }
        for (Type arg_type : arg_types) {
            frame.locals[count] = this.types.erasure(arg_type);
            count++;
        }
        frame.pc = -1;
        frame.stack = null;
        return frame;
    }

    public static class Chain {
        public final Chain next;
        public final int pc;
        State state;

        public Chain(int pc, Chain next, State state) {
            this.pc = pc;
            this.next = next;
            this.state = state;
        }
    }

    public static int negate(int opcode) {
        if (opcode == 198) {
            return ByteCodes.if_acmp_nonnull;
        }
        return opcode == 199 ? ByteCodes.if_acmp_null : ((opcode + 1) ^ 1) - 1;
    }

    public int emitJump(int opcode) {
        if (this.fatcode) {
            if (opcode == 167 || opcode == 168) {
                emitop4((opcode + 200) - ByteCodes.goto_, 0);
            } else {
                emitop2(negate(opcode), 8);
                emitop4(200, 0);
                this.alive = true;
                this.pendingStackMap = this.needStackMap;
            }
            return this.cp - 5;
        }
        emitop2(opcode, 0);
        return this.cp - 3;
    }

    public Chain branch(int opcode) {
        Chain result = null;
        if (opcode == 167) {
            result = this.pendingJumps;
            this.pendingJumps = null;
        }
        if (opcode != 168 && isAlive()) {
            result = new Chain(emitJump(opcode), result, this.state.dup());
            this.fixedPc = this.fatcode;
            if (opcode == 167) {
                this.alive = false;
            }
        }
        return result;
    }

    public void resolve(Chain chain, int target) {
        boolean changed = false;
        State newState = this.state;
        while (true) {
            if (chain == null) {
                break;
            }
            Assert.check(this.state != chain.state && (target > chain.pc || this.state.stacksize == 0));
            if (target >= this.cp) {
                target = this.cp;
            } else if (get1(target) == 167) {
                target = this.fatcode ? target + get4(target + 1) : target + get2(target + 1);
            }
            if (get1(chain.pc) == 167 && chain.pc + 3 == target && target == this.cp && !this.fixedPc) {
                if (this.varDebugInfo) {
                    adjustAliveRanges(this.cp, -3);
                }
                this.cp -= 3;
                target -= 3;
                if (chain.next == null) {
                    this.alive = true;
                    break;
                }
            } else {
                if (this.fatcode) {
                    put4(chain.pc + 1, target - chain.pc);
                } else if (target - chain.pc < -32768 || target - chain.pc > 32767) {
                    this.fatcode = true;
                } else {
                    put2(chain.pc + 1, target - chain.pc);
                }
                if (!this.alive || (chain.state.stacksize == newState.stacksize && chain.state.nlocks == newState.nlocks)) {
                    z = true;
                }
                Assert.check(z);
            }
            this.fixedPc = true;
            if (this.cp == target) {
                changed = true;
                if (this.debugCode) {
                    System.err.println("resolving chain state=" + chain.state);
                }
                if (this.alive) {
                    newState = chain.state.join(newState);
                } else {
                    newState = chain.state;
                    this.alive = true;
                }
            }
            chain = chain.next;
        }
        Assert.check((changed && this.state == newState) ? false : true);
        if (this.state != newState) {
            setDefined(newState.defined);
            this.state = newState;
            this.pendingStackMap = this.needStackMap;
        }
    }

    public void resolve(Chain chain) {
        Assert.check(!this.alive || chain == null || (this.state.stacksize == chain.state.stacksize && this.state.nlocks == chain.state.nlocks));
        this.pendingJumps = mergeChains(chain, this.pendingJumps);
    }

    public void resolvePending() {
        Chain x = this.pendingJumps;
        this.pendingJumps = null;
        resolve(x, this.cp);
    }

    public static Chain mergeChains(Chain chain1, Chain chain2) {
        if (chain2 == null) {
            return chain1;
        }
        if (chain1 == null) {
            return chain2;
        }
        Assert.check(chain1.state.stacksize == chain2.state.stacksize && chain1.state.nlocks == chain2.state.nlocks);
        if (chain1.pc < chain2.pc) {
            return new Chain(chain2.pc, mergeChains(chain1, chain2.next), chain2.state);
        }
        return new Chain(chain1.pc, mergeChains(chain1.next, chain2), chain1.state);
    }

    public void addCatch(char startPc, char endPc, char handlerPc, char catchType) {
        this.catchInfo.append(new char[]{startPc, endPc, handlerPc, catchType});
    }

    public void compressCatchTable() {
        ListBuffer<char[]> compressedCatchInfo = new ListBuffer<>();
        List<Integer> handlerPcs = List.nil();
        Iterator<char[]> it = this.catchInfo.iterator();
        while (it.hasNext()) {
            handlerPcs = handlerPcs.prepend(Integer.valueOf(it.next()[2]));
        }
        for (char[] catchEntry : this.catchInfo) {
            char c = catchEntry[0];
            char c2 = catchEntry[1];
            if (c != c2 && (c != c2 - 1 || !handlerPcs.contains(Integer.valueOf(c)))) {
                compressedCatchInfo.append(catchEntry);
            }
        }
        this.catchInfo = compressedCatchInfo;
    }

    public void addLineNumber(char startPc, char lineNumber) {
        if (this.lineDebugInfo) {
            if (this.lineInfo.nonEmpty() && this.lineInfo.head[0] == startPc) {
                this.lineInfo = this.lineInfo.tail;
            }
            if (this.lineInfo.isEmpty() || this.lineInfo.head[1] != lineNumber) {
                this.lineInfo = this.lineInfo.prepend(new char[]{startPc, lineNumber});
            }
        }
    }

    public void statBegin(int pos) {
        if (pos != -1) {
            this.pendingStatPos = pos;
        }
    }

    public void markStatBegin() {
        if (this.alive && this.lineDebugInfo) {
            int line = this.lineMap.getLineNumber(this.pendingStatPos);
            char cp1 = (char) this.cp;
            char line1 = (char) line;
            if (cp1 == this.cp && line1 == line) {
                addLineNumber(cp1, line1);
            }
        }
        this.pendingStatPos = -1;
    }

    class State implements Cloneable {
        int[] locks;
        int nlocks;
        int stacksize;
        Bits defined = new Bits();
        Type[] stack = new Type[16];

        State() {
        }

        State dup() {
            try {
                State state = (State) super.clone();
                state.defined = new Bits(this.defined);
                state.stack = (Type[]) this.stack.clone();
                if (this.locks != null) {
                    state.locks = (int[]) this.locks.clone();
                }
                if (Code.this.debugCode) {
                    System.err.println("duping state " + this);
                    dump();
                }
                return state;
            } catch (CloneNotSupportedException ex) {
                throw new AssertionError(ex);
            }
        }

        void lock(int register) {
            if (this.locks == null) {
                this.locks = new int[20];
            } else {
                this.locks = ArrayUtils.ensureCapacity(this.locks, this.nlocks);
            }
            this.locks[this.nlocks] = register;
            this.nlocks++;
        }

        void unlock(int register) {
            this.nlocks--;
            Assert.check(this.locks[this.nlocks] == register);
            this.locks[this.nlocks] = -1;
        }

        void push(Type t) {
            if (Code.this.debugCode) {
                System.err.println("   pushing " + t);
            }
            switch (t.getTag()) {
                case BYTE:
                case SHORT:
                case CHAR:
                case BOOLEAN:
                    t = Code.this.syms.intType;
                    break;
                case VOID:
                    return;
            }
            this.stack = (Type[]) ArrayUtils.ensureCapacity(this.stack, this.stacksize + 2);
            Type[] typeArr = this.stack;
            int i = this.stacksize;
            this.stacksize = i + 1;
            typeArr[i] = t;
            switch (Code.width(t)) {
                case 1:
                    break;
                case 2:
                    Type[] typeArr2 = this.stack;
                    int i2 = this.stacksize;
                    this.stacksize = i2 + 1;
                    typeArr2[i2] = null;
                    break;
                default:
                    throw new AssertionError(t);
            }
            if (this.stacksize > Code.this.max_stack) {
                Code.this.max_stack = this.stacksize;
            }
        }

        Type pop1() {
            if (Code.this.debugCode) {
                System.err.println("   popping 1");
            }
            this.stacksize--;
            Type result = this.stack[this.stacksize];
            this.stack[this.stacksize] = null;
            Assert.check(result != null && Code.width(result) == 1);
            return result;
        }

        Type peek() {
            return this.stack[this.stacksize - 1];
        }

        Type pop2() {
            if (Code.this.debugCode) {
                System.err.println("   popping 2");
            }
            this.stacksize -= 2;
            Type result = this.stack[this.stacksize];
            this.stack[this.stacksize] = null;
            Assert.check(this.stack[this.stacksize + 1] == null && result != null && Code.width(result) == 2);
            return result;
        }

        void pop(int n) {
            if (Code.this.debugCode) {
                System.err.println("   popping " + n);
            }
            while (n > 0) {
                Type[] typeArr = this.stack;
                int i = this.stacksize - 1;
                this.stacksize = i;
                typeArr[i] = null;
                n--;
            }
        }

        void pop(Type t) {
            pop(Code.width(t));
        }

        void forceStackTop(Type t) {
            if (Code.this.alive) {
                switch (t.getTag()) {
                    case CLASS:
                    case ARRAY:
                        int width = Code.width(t);
                        Type old = this.stack[this.stacksize - width];
                        Assert.check(Code.this.types.isSubtype(Code.this.types.erasure(old), Code.this.types.erasure(t)));
                        this.stack[this.stacksize - width] = t;
                        break;
                }
            }
        }

        void markInitialized(UninitializedType old) {
            Type newtype = old.initializedType();
            for (int i = 0; i < this.stacksize; i++) {
                if (this.stack[i] == old) {
                    this.stack[i] = newtype;
                }
            }
            for (int i2 = 0; i2 < Code.this.lvar.length; i2++) {
                LocalVar lv = Code.this.lvar[i2];
                if (lv != null && lv.sym.type == old) {
                    Symbol.VarSymbol sym = lv.sym;
                    Symbol.VarSymbol sym2 = sym.clone(sym.owner);
                    sym2.type = newtype;
                    LocalVar[] localVarArr = Code.this.lvar;
                    LocalVar newlv = new LocalVar(sym2);
                    localVarArr[i2] = newlv;
                    newlv.aliveRanges = lv.aliveRanges;
                }
            }
        }

        /* JADX WARN: Removed duplicated region for block: B:19:0x003f  */
        /*
            Code decompiled incorrectly, please refer to instructions dump.
            To view partially-correct code enable 'Show inconsistent code' option in preferences
        */
        com.sun.tools.javac.jvm.Code.State join(com.sun.tools.javac.jvm.Code.State r8) {
            /*
                r7 = this;
                com.sun.tools.javac.util.Bits r0 = r7.defined
                com.sun.tools.javac.util.Bits r1 = r8.defined
                r0.andSet(r1)
                int r0 = r7.stacksize
                int r1 = r8.stacksize
                if (r0 != r1) goto L15
                int r0 = r7.nlocks
                int r1 = r8.nlocks
                if (r0 != r1) goto L15
                r0 = 1
                goto L16
            L15:
                r0 = 0
            L16:
                com.sun.tools.javac.util.Assert.check(r0)
                r0 = 0
            L1a:
                int r1 = r7.stacksize
                if (r0 >= r1) goto L5c
                com.sun.tools.javac.code.Type[] r1 = r7.stack
                r1 = r1[r0]
                com.sun.tools.javac.code.Type[] r2 = r8.stack
                r2 = r2[r0]
                if (r1 != r2) goto L29
                goto L3f
            L29:
                com.sun.tools.javac.jvm.Code r3 = com.sun.tools.javac.jvm.Code.this
                com.sun.tools.javac.code.Types r3 = r3.types
                boolean r3 = r3.isSubtype(r1, r2)
                if (r3 == 0) goto L35
                r3 = r2
                goto L45
            L35:
                com.sun.tools.javac.jvm.Code r3 = com.sun.tools.javac.jvm.Code.this
                com.sun.tools.javac.code.Types r3 = r3.types
                boolean r3 = r3.isSubtype(r2, r1)
                if (r3 == 0) goto L41
            L3f:
                r3 = r1
                goto L45
            L41:
                com.sun.tools.javac.code.Type r3 = r7.error()
            L45:
                int r4 = com.sun.tools.javac.jvm.Code.width(r3)
                com.sun.tools.javac.code.Type[] r5 = r7.stack
                r5[r0] = r3
                r5 = 2
                if (r4 != r5) goto L5a
                com.sun.tools.javac.code.Type[] r5 = r7.stack
                int r6 = r0 + 1
                r5 = r5[r6]
                com.sun.tools.javac.util.Assert.checkNull(r5)
            L5a:
                int r0 = r0 + r4
                goto L1a
            L5c:
                return r7
            */
            throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.jvm.Code.State.join(com.sun.tools.javac.jvm.Code$State):com.sun.tools.javac.jvm.Code$State");
        }

        Type error() {
            throw new AssertionError("inconsistent stack types at join point");
        }

        void dump() {
            dump(-1);
        }

        void dump(int pc) {
            System.err.print("stackMap for " + Code.this.meth.owner + "." + Code.this.meth);
            if (pc == -1) {
                System.out.println();
            } else {
                System.out.println(" at " + pc);
            }
            System.err.println(" stack (from bottom):");
            for (int i = 0; i < this.stacksize; i++) {
                System.err.println("  " + i + ": " + this.stack[i]);
            }
            int lastLocal = 0;
            int i2 = Code.this.max_locals - 1;
            while (true) {
                if (i2 < 0) {
                    break;
                }
                if (this.defined.isMember(i2)) {
                    lastLocal = i2;
                    break;
                }
                i2--;
            }
            if (lastLocal >= 0) {
                System.err.println(" locals:");
            }
            for (int i3 = 0; i3 <= lastLocal; i3++) {
                System.err.print("  " + i3 + ": ");
                if (this.defined.isMember(i3)) {
                    LocalVar var = Code.this.lvar[i3];
                    if (var == null) {
                        System.err.println("(none)");
                    } else if (var.sym == null) {
                        System.err.println("UNKNOWN!");
                    } else {
                        System.err.println("" + var.sym + " of type " + var.sym.erasure(Code.this.types));
                    }
                } else {
                    System.err.println("undefined");
                }
            }
            int i4 = this.nlocks;
            if (i4 != 0) {
                System.err.print(" locks:");
                for (int i5 = 0; i5 < this.nlocks; i5++) {
                    System.err.print(" " + this.locks[i5]);
                }
                System.err.println();
            }
        }
    }

    static class LocalVar {
        java.util.List<Range> aliveRanges = new ArrayList();
        final char reg;
        final Symbol.VarSymbol sym;

        class Range {
            char length;
            char start_pc;

            Range() {
                this.start_pc = (char) 65535;
                this.length = (char) 65535;
            }

            Range(char start) {
                this.start_pc = (char) 65535;
                this.length = (char) 65535;
                this.start_pc = start;
            }

            Range(char start, char length) {
                this.start_pc = (char) 65535;
                this.length = (char) 65535;
                this.start_pc = start;
                this.length = length;
            }

            boolean closed() {
                return (this.start_pc == 65535 || this.length == 65535) ? false : true;
            }

            public String toString() {
                int currentStartPC = this.start_pc;
                int currentLength = this.length;
                return "startpc = " + currentStartPC + " length " + currentLength;
            }
        }

        LocalVar(Symbol.VarSymbol v) {
            this.sym = v;
            this.reg = (char) v.adr;
        }

        public LocalVar dup() {
            return new LocalVar(this.sym);
        }

        Range firstRange() {
            if (this.aliveRanges.isEmpty()) {
                return null;
            }
            return this.aliveRanges.get(0);
        }

        Range lastRange() {
            if (this.aliveRanges.isEmpty()) {
                return null;
            }
            return this.aliveRanges.get(this.aliveRanges.size() - 1);
        }

        void removeLastRange() {
            Range lastRange = lastRange();
            if (lastRange != null) {
                this.aliveRanges.remove(lastRange);
            }
        }

        public String toString() {
            if (this.aliveRanges == null) {
                return "empty local var";
            }
            StringBuilder sb = new StringBuilder().append(this.sym).append(" in register ").append((int) this.reg).append(" \n");
            for (Range r : this.aliveRanges) {
                sb.append(" starts at pc=").append(Integer.toString(r.start_pc)).append(" length=").append(Integer.toString(r.length)).append("\n");
            }
            return sb.toString();
        }

        public void openRange(char start) {
            if (!hasOpenRange()) {
                this.aliveRanges.add(new Range(start));
            }
        }

        public void closeRange(char length) {
            if (isLastRangeInitialized() && length > 0) {
                Range range = lastRange();
                if (range != null && range.length == 65535) {
                    range.length = length;
                    return;
                }
                return;
            }
            removeLastRange();
        }

        public boolean hasOpenRange() {
            return !this.aliveRanges.isEmpty() && lastRange().length == 65535;
        }

        public boolean isLastRangeInitialized() {
            return (this.aliveRanges.isEmpty() || lastRange().start_pc == 65535) ? false : true;
        }

        public Range getWidestRange() {
            if (this.aliveRanges.isEmpty()) {
                return new Range();
            }
            Range firstRange = firstRange();
            Range lastRange = lastRange();
            char length = (char) (lastRange.length + (lastRange.start_pc - firstRange.start_pc));
            return new Range(firstRange.start_pc, length);
        }
    }

    private void addLocalVar(Symbol.VarSymbol v) {
        int adr = v.adr;
        this.lvar = (LocalVar[]) ArrayUtils.ensureCapacity(this.lvar, adr + 1);
        Assert.checkNull(this.lvar[adr]);
        if (this.pendingJumps != null) {
            resolvePending();
        }
        this.lvar[adr] = new LocalVar(v);
        this.state.defined.excl(adr);
    }

    void adjustAliveRanges(int oldCP, int delta) {
        for (LocalVar localVar : this.lvar) {
            if (localVar != null) {
                for (LocalVar.Range range : localVar.aliveRanges) {
                    if (range.closed() && range.start_pc + range.length >= oldCP) {
                        range.length = (char) (range.length + delta);
                    }
                }
            }
        }
    }

    public int getLVTSize() {
        int result = this.varBufferSize;
        for (int i = 0; i < this.varBufferSize; i++) {
            LocalVar var = this.varBuffer[i];
            result += var.aliveRanges.size() - 1;
        }
        return result;
    }

    public void setDefined(Bits newDefined) {
        if (this.alive && newDefined != this.state.defined) {
            Bits diff = new Bits(this.state.defined).xorSet(newDefined);
            for (int adr = diff.nextBit(0); adr >= 0; adr = diff.nextBit(adr + 1)) {
                if (adr >= this.nextreg) {
                    this.state.defined.excl(adr);
                } else if (this.state.defined.isMember(adr)) {
                    setUndefined(adr);
                } else {
                    setDefined(adr);
                }
            }
        }
    }

    public void setDefined(int adr) {
        LocalVar v = this.lvar[adr];
        if (v == null) {
            this.state.defined.excl(adr);
            return;
        }
        this.state.defined.incl(adr);
        if (this.cp < 65535) {
            v.openRange((char) this.cp);
        }
    }

    public void setUndefined(int adr) {
        this.state.defined.excl(adr);
        if (adr < this.lvar.length && this.lvar[adr] != null && this.lvar[adr].isLastRangeInitialized()) {
            LocalVar v = this.lvar[adr];
            char length = (char) (curCP() - v.lastRange().start_pc);
            if (length < 65535) {
                this.lvar[adr] = v.dup();
                v.closeRange(length);
                putVar(v);
                return;
            }
            v.removeLastRange();
        }
    }

    private void endScope(int adr) {
        char length;
        LocalVar v = this.lvar[adr];
        if (v != null) {
            if (v.isLastRangeInitialized() && (length = (char) (curCP() - v.lastRange().start_pc)) < 65535) {
                v.closeRange(length);
                putVar(v);
                fillLocalVarPosition(v);
            }
            this.lvar[adr] = null;
        }
        this.state.defined.excl(adr);
    }

    private void fillLocalVarPosition(LocalVar lv) {
        if (lv == null || lv.sym == null || !lv.sym.hasTypeAnnotations()) {
            return;
        }
        for (Attribute.TypeCompound ta : lv.sym.getRawTypeAttributes()) {
            TypeAnnotationPosition p = ta.position;
            LocalVar.Range widestRange = lv.getWidestRange();
            p.lvarOffset = new int[]{widestRange.start_pc};
            p.lvarLength = new int[]{widestRange.length};
            p.lvarIndex = new int[]{lv.reg};
            p.isValidOffset = true;
        }
    }

    public void fillExceptionParameterPositions() {
        for (int i = 0; i < this.varBufferSize; i++) {
            LocalVar lv = this.varBuffer[i];
            if (lv != null && lv.sym != null && lv.sym.hasTypeAnnotations() && lv.sym.isExceptionParameter()) {
                for (Attribute.TypeCompound ta : lv.sym.getRawTypeAttributes()) {
                    TypeAnnotationPosition p = ta.position;
                    if (p.type_index != -666) {
                        p.exception_index = findExceptionIndex(p.type_index);
                        p.type_index = -666;
                    }
                }
            }
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    private int findExceptionIndex(int catchType) {
        if (catchType == Integer.MIN_VALUE) {
            return -1;
        }
        List list = this.catchInfo.toList();
        int len = this.catchInfo.length();
        for (int i = 0; i < len; i++) {
            char[] catchEntry = (char[]) list.head;
            list = list.tail;
            char ct = catchEntry[3];
            if (catchType == ct) {
                return i;
            }
        }
        return -1;
    }

    void putVar(LocalVar var) {
        boolean ignoredSyntheticVar = false;
        boolean keepLocalVariables = this.varDebugInfo || (var.sym.isExceptionParameter() && var.sym.hasTypeAnnotations());
        if (keepLocalVariables) {
            if ((var.sym.flags() & 4096) != 0 && ((var.sym.owner.flags() & Flags.LAMBDA_METHOD) == 0 || (var.sym.flags() & 8589934592L) == 0)) {
                ignoredSyntheticVar = true;
            }
            if (ignoredSyntheticVar) {
                return;
            }
            if (this.varBuffer == null) {
                this.varBuffer = new LocalVar[20];
            } else {
                this.varBuffer = (LocalVar[]) ArrayUtils.ensureCapacity(this.varBuffer, this.varBufferSize);
            }
            LocalVar[] localVarArr = this.varBuffer;
            int i = this.varBufferSize;
            this.varBufferSize = i + 1;
            localVarArr[i] = var;
        }
    }

    private int newLocal(int typecode) {
        int reg = this.nextreg;
        int w = width(typecode);
        this.nextreg = reg + w;
        if (this.nextreg > this.max_locals) {
            this.max_locals = this.nextreg;
        }
        return reg;
    }

    private int newLocal(Type type) {
        return newLocal(typecode(type));
    }

    public int newLocal(Symbol.VarSymbol v) {
        int reg = newLocal(v.erasure(this.types));
        v.adr = reg;
        addLocalVar(v);
        return reg;
    }

    public void newRegSegment() {
        this.nextreg = this.max_locals;
    }

    public void endScopes(int first) {
        int prevNextReg = this.nextreg;
        this.nextreg = first;
        for (int i = this.nextreg; i < prevNextReg; i++) {
            endScope(i);
        }
    }

    public static String mnem(int opcode) {
        return Mneumonics.mnem[opcode];
    }

    private static class Mneumonics {
        private static final String[] mnem = new String[ByteCodes.ByteCodeCount];

        private Mneumonics() {
        }

        static {
            mnem[0] = "nop";
            mnem[1] = "aconst_null";
            mnem[2] = "iconst_m1";
            mnem[3] = "iconst_0";
            mnem[4] = "iconst_1";
            mnem[5] = "iconst_2";
            mnem[6] = "iconst_3";
            mnem[7] = "iconst_4";
            mnem[8] = "iconst_5";
            mnem[9] = "lconst_0";
            mnem[10] = "lconst_1";
            mnem[11] = "fconst_0";
            mnem[12] = "fconst_1";
            mnem[13] = "fconst_2";
            mnem[14] = "dconst_0";
            mnem[15] = "dconst_1";
            mnem[16] = "bipush";
            mnem[17] = "sipush";
            mnem[18] = "ldc1";
            mnem[19] = "ldc2";
            mnem[20] = "ldc2w";
            mnem[21] = "iload";
            mnem[22] = "lload";
            mnem[23] = "fload";
            mnem[24] = "dload";
            mnem[25] = "aload";
            mnem[26] = "iload_0";
            mnem[30] = "lload_0";
            mnem[34] = "fload_0";
            mnem[38] = "dload_0";
            mnem[42] = "aload_0";
            mnem[27] = "iload_1";
            mnem[31] = "lload_1";
            mnem[35] = "fload_1";
            mnem[39] = "dload_1";
            mnem[43] = "aload_1";
            mnem[28] = "iload_2";
            mnem[32] = "lload_2";
            mnem[36] = "fload_2";
            mnem[40] = "dload_2";
            mnem[44] = "aload_2";
            mnem[29] = "iload_3";
            mnem[33] = "lload_3";
            mnem[37] = "fload_3";
            mnem[41] = "dload_3";
            mnem[45] = "aload_3";
            mnem[46] = "iaload";
            mnem[47] = "laload";
            mnem[48] = "faload";
            mnem[49] = "daload";
            mnem[50] = "aaload";
            mnem[51] = "baload";
            mnem[52] = "caload";
            mnem[53] = "saload";
            mnem[54] = "istore";
            mnem[55] = "lstore";
            mnem[56] = "fstore";
            mnem[57] = "dstore";
            mnem[58] = "astore";
            mnem[59] = "istore_0";
            mnem[63] = "lstore_0";
            mnem[67] = "fstore_0";
            mnem[71] = "dstore_0";
            mnem[75] = "astore_0";
            mnem[60] = "istore_1";
            mnem[64] = "lstore_1";
            mnem[68] = "fstore_1";
            mnem[72] = "dstore_1";
            mnem[76] = "astore_1";
            mnem[61] = "istore_2";
            mnem[65] = "lstore_2";
            mnem[69] = "fstore_2";
            mnem[73] = "dstore_2";
            mnem[77] = "astore_2";
            mnem[62] = "istore_3";
            mnem[66] = "lstore_3";
            mnem[70] = "fstore_3";
            mnem[74] = "dstore_3";
            mnem[78] = "astore_3";
            mnem[79] = "iastore";
            mnem[80] = "lastore";
            mnem[81] = "fastore";
            mnem[82] = "dastore";
            mnem[83] = "aastore";
            mnem[84] = "bastore";
            mnem[85] = "castore";
            mnem[86] = "sastore";
            mnem[87] = "pop";
            mnem[88] = "pop2";
            mnem[89] = "dup";
            mnem[90] = "dup_x1";
            mnem[91] = "dup_x2";
            mnem[92] = "dup2";
            mnem[93] = "dup2_x1";
            mnem[94] = "dup2_x2";
            mnem[95] = "swap";
            mnem[96] = "iadd";
            mnem[97] = "ladd";
            mnem[98] = "fadd";
            mnem[99] = "dadd";
            mnem[100] = "isub";
            mnem[101] = "lsub";
            mnem[102] = "fsub";
            mnem[103] = "dsub";
            mnem[104] = "imul";
            mnem[105] = "lmul";
            mnem[106] = "fmul";
            mnem[107] = "dmul";
            mnem[108] = "idiv";
            mnem[109] = "ldiv";
            mnem[110] = "fdiv";
            mnem[111] = "ddiv";
            mnem[112] = "imod";
            mnem[113] = "lmod";
            mnem[114] = "fmod";
            mnem[115] = "dmod";
            mnem[116] = "ineg";
            mnem[117] = "lneg";
            mnem[118] = "fneg";
            mnem[119] = "dneg";
            mnem[120] = "ishl";
            mnem[121] = "lshl";
            mnem[122] = "ishr";
            mnem[123] = "lshr";
            mnem[124] = "iushr";
            mnem[125] = "lushr";
            mnem[126] = "iand";
            mnem[127] = "land";
            mnem[128] = "ior";
            mnem[129] = "lor";
            mnem[130] = "ixor";
            mnem[131] = "lxor";
            mnem[132] = "iinc";
            mnem[133] = "i2l";
            mnem[134] = "i2f";
            mnem[135] = "i2d";
            mnem[136] = "l2i";
            mnem[137] = "l2f";
            mnem[138] = "l2d";
            mnem[139] = "f2i";
            mnem[140] = "f2l";
            mnem[141] = "f2d";
            mnem[142] = "d2i";
            mnem[143] = "d2l";
            mnem[144] = "d2f";
            mnem[145] = "int2byte";
            mnem[146] = "int2char";
            mnem[147] = "int2short";
            mnem[148] = "lcmp";
            mnem[149] = "fcmpl";
            mnem[150] = "fcmpg";
            mnem[151] = "dcmpl";
            mnem[152] = "dcmpg";
            mnem[153] = "ifeq";
            mnem[154] = "ifne";
            mnem[155] = "iflt";
            mnem[156] = "ifge";
            mnem[157] = "ifgt";
            mnem[158] = "ifle";
            mnem[159] = "if_icmpeq";
            mnem[160] = "if_icmpne";
            mnem[161] = "if_icmplt";
            mnem[162] = "if_icmpge";
            mnem[163] = "if_icmpgt";
            mnem[164] = "if_icmple";
            mnem[165] = "if_acmpeq";
            mnem[166] = "if_acmpne";
            mnem[167] = "goto_";
            mnem[168] = "jsr";
            mnem[169] = "ret";
            mnem[170] = "tableswitch";
            mnem[171] = "lookupswitch";
            mnem[172] = "ireturn";
            mnem[173] = "lreturn";
            mnem[174] = "freturn";
            mnem[175] = "dreturn";
            mnem[176] = "areturn";
            mnem[177] = "return_";
            mnem[178] = "getstatic";
            mnem[179] = "putstatic";
            mnem[180] = "getfield";
            mnem[181] = "putfield";
            mnem[182] = "invokevirtual";
            mnem[183] = "invokespecial";
            mnem[184] = "invokestatic";
            mnem[185] = "invokeinterface";
            mnem[186] = "invokedynamic";
            mnem[187] = "new_";
            mnem[188] = "newarray";
            mnem[189] = "anewarray";
            mnem[190] = "arraylength";
            mnem[191] = "athrow";
            mnem[192] = "checkcast";
            mnem[193] = "instanceof_";
            mnem[194] = "monitorenter";
            mnem[195] = "monitorexit";
            mnem[196] = "wide";
            mnem[197] = "multianewarray";
            mnem[198] = "if_acmp_null";
            mnem[199] = "if_acmp_nonnull";
            mnem[200] = "goto_w";
            mnem[201] = "jsr_w";
            mnem[202] = "breakpoint";
        }
    }
}
