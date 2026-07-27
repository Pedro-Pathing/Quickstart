package com.sun.tools.javac.jvm;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeAnnotationPosition;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.file.BaseFileObject;
import com.sun.tools.javac.jvm.ClassFile;
import com.sun.tools.javac.jvm.Code;
import com.sun.tools.javac.jvm.Pool;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.ByteBuffer;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.Pair;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import javax.tools.FileObject;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.StandardLocation;

/* JADX INFO: loaded from: classes.dex */
public class ClassWriter extends ClassFile {
    static final int DATA_BUF_SIZE = 65520;
    static final int FULL_FRAME = 255;
    static final int MAX_LOCAL_LENGTH_DIFF = 4;
    static final int POOL_BUF_SIZE = 131056;
    static final int SAME_FRAME_EXTENDED = 251;
    static final int SAME_FRAME_SIZE = 64;
    static final int SAME_LOCALS_1_STACK_ITEM_EXTENDED = 247;
    protected static final Context.Key<ClassWriter> classWriterKey = new Context.Key<>();
    private static final String[] flagName = {"PUBLIC", "PRIVATE", "PROTECTED", "STATIC", "FINAL", "SUPER", "VOLATILE", "TRANSIENT", "NATIVE", "INTERFACE", "ABSTRACT", "STRICTFP"};
    Map<Pool.DynamicMethod, Pool.MethodHandle> bootstrapMethods;
    boolean debugstackmap;
    private final boolean dumpClassModifiers;
    private final boolean dumpFieldModifiers;
    private final boolean dumpInnerClassModifiers;
    private final boolean dumpMethodModifiers;
    private boolean emitSourceFile;
    private final JavaFileManager fileManager;
    private boolean genCrt;
    Set<Symbol.ClassSymbol> innerClasses;
    ListBuffer<Symbol.ClassSymbol> innerClassesQueue;
    private final Log log;
    private final Names names;
    private final Options options;
    Pool pool;
    private boolean retrofit;
    private boolean scramble;
    private boolean scrambleAll;
    private final CWSignatureGenerator signatureGen;
    private Source source;
    private Target target;
    private Types types;
    private boolean verbose;
    ByteBuffer databuf = new ByteBuffer(65520);
    ByteBuffer poolbuf = new ByteBuffer(POOL_BUF_SIZE);
    AttributeWriter awriter = new AttributeWriter();

    public static class PoolOverflow extends Exception {
        private static final long serialVersionUID = 0;
    }

    public static ClassWriter instance(Context context) {
        ClassWriter instance = (ClassWriter) context.get(classWriterKey);
        if (instance == null) {
            return new ClassWriter(context);
        }
        return instance;
    }

    protected ClassWriter(Context context) {
        context.put(classWriterKey, this);
        this.log = Log.instance(context);
        this.names = Names.instance(context);
        this.options = Options.instance(context);
        this.target = Target.instance(context);
        this.source = Source.instance(context);
        this.types = Types.instance(context);
        this.fileManager = (JavaFileManager) context.get(JavaFileManager.class);
        this.signatureGen = new CWSignatureGenerator(this.types);
        this.verbose = this.options.isSet(Option.VERBOSE);
        this.scramble = this.options.isSet("-scramble");
        this.scrambleAll = this.options.isSet("-scrambleAll");
        this.retrofit = this.options.isSet("-retrofit");
        this.genCrt = this.options.isSet(Option.XJCOV);
        this.debugstackmap = this.options.isSet("debugstackmap");
        boolean z = false;
        this.emitSourceFile = this.options.isUnset(Option.G_CUSTOM) || this.options.isSet(Option.G_CUSTOM, "source");
        String dumpModFlags = this.options.get("dumpmodifiers");
        this.dumpClassModifiers = (dumpModFlags == null || dumpModFlags.indexOf(99) == -1) ? false : true;
        this.dumpFieldModifiers = (dumpModFlags == null || dumpModFlags.indexOf(102) == -1) ? false : true;
        this.dumpInnerClassModifiers = (dumpModFlags == null || dumpModFlags.indexOf(105) == -1) ? false : true;
        if (dumpModFlags != null && dumpModFlags.indexOf(109) != -1) {
            z = true;
        }
        this.dumpMethodModifiers = z;
    }

    public static String flagNames(long flags) {
        StringBuilder sbuf = new StringBuilder();
        int i = 0;
        long f = 4095 & flags;
        while (f != 0) {
            if ((1 & f) != 0) {
                sbuf.append(" ");
                sbuf.append(flagName[i]);
            }
            f >>= 1;
            i++;
        }
        return sbuf.toString();
    }

    void putChar(ByteBuffer buf, int op, int x) {
        buf.elems[op] = (byte) ((x >> 8) & 255);
        buf.elems[op + 1] = (byte) (x & 255);
    }

    void putInt(ByteBuffer buf, int adr, int x) {
        buf.elems[adr] = (byte) ((x >> 24) & 255);
        buf.elems[adr + 1] = (byte) ((x >> 16) & 255);
        buf.elems[adr + 2] = (byte) ((x >> 8) & 255);
        buf.elems[adr + 3] = (byte) (x & 255);
    }

    private class CWSignatureGenerator extends Types.SignatureGenerator {
        ByteBuffer sigbuf;

        CWSignatureGenerator(Types types) {
            super(types);
            this.sigbuf = new ByteBuffer();
        }

        @Override // com.sun.tools.javac.code.Types.SignatureGenerator
        public void assembleSig(Type type) {
            Type type2 = type.unannotatedType();
            switch (type2.getTag()) {
                case UNINITIALIZED_THIS:
                case UNINITIALIZED_OBJECT:
                    assembleSig(ClassWriter.this.types.erasure(((UninitializedType) type2).qtype));
                    break;
                default:
                    super.assembleSig(type2);
                    break;
            }
        }

        @Override // com.sun.tools.javac.code.Types.SignatureGenerator
        protected void append(char ch) {
            this.sigbuf.appendByte(ch);
        }

        @Override // com.sun.tools.javac.code.Types.SignatureGenerator
        protected void append(byte[] ba) {
            this.sigbuf.appendBytes(ba);
        }

        @Override // com.sun.tools.javac.code.Types.SignatureGenerator
        protected void append(Name name) {
            this.sigbuf.appendName(name);
        }

        @Override // com.sun.tools.javac.code.Types.SignatureGenerator
        protected void classReference(Symbol.ClassSymbol c) {
            ClassWriter.this.enterInner(c);
        }

        /* JADX INFO: Access modifiers changed from: private */
        public void reset() {
            this.sigbuf.reset();
        }

        /* JADX INFO: Access modifiers changed from: private */
        public Name toName() {
            return this.sigbuf.toName(ClassWriter.this.names);
        }

        /* JADX INFO: Access modifiers changed from: private */
        public boolean isEmpty() {
            return this.sigbuf.length == 0;
        }
    }

    Name typeSig(Type type) {
        Assert.check(this.signatureGen.isEmpty());
        this.signatureGen.assembleSig(type);
        Name n = this.signatureGen.toName();
        this.signatureGen.reset();
        return n;
    }

    public Name xClassName(Type t) {
        if (t.hasTag(TypeTag.CLASS)) {
            return this.names.fromUtf(externalize(t.tsym.flatName()));
        }
        if (t.hasTag(TypeTag.ARRAY)) {
            return typeSig(this.types.erasure(t));
        }
        throw new AssertionError("xClassName");
    }

    public static class StringOverflow extends Exception {
        private static final long serialVersionUID = 0;
        public final String value;

        public StringOverflow(String s) {
            this.value = s;
        }
    }

    void writePool(Pool pool) throws StringOverflow, PoolOverflow {
        int poolCountIdx = this.poolbuf.length;
        this.poolbuf.appendChar(0);
        int i = 1;
        while (i < pool.pp) {
            Object value = pool.pool[i];
            Assert.checkNonNull(value);
            if ((value instanceof Pool.Method) || (value instanceof Pool.Variable)) {
                value = ((Symbol.DelegatedSymbol) value).getUnderlyingSymbol();
            }
            if (value instanceof Symbol.MethodSymbol) {
                Symbol.MethodSymbol m = (Symbol.MethodSymbol) value;
                if (!m.isDynamic()) {
                    this.poolbuf.appendByte((m.owner.flags() & 512) != 0 ? 11 : 10);
                    this.poolbuf.appendChar(pool.put(m.owner));
                    this.poolbuf.appendChar(pool.put(nameType(m)));
                } else {
                    Symbol.DynamicMethodSymbol dynSym = (Symbol.DynamicMethodSymbol) m;
                    Pool.MethodHandle handle = new Pool.MethodHandle(dynSym.bsmKind, dynSym.bsm, this.types);
                    Pool.DynamicMethod dynMeth = new Pool.DynamicMethod(dynSym, this.types);
                    this.bootstrapMethods.put(dynMeth, handle);
                    pool.put(this.names.BootstrapMethods);
                    pool.put(handle);
                    for (Object staticArg : dynSym.staticArgs) {
                        pool.put(staticArg);
                    }
                    this.poolbuf.appendByte(18);
                    this.poolbuf.appendChar(this.bootstrapMethods.size() - 1);
                    this.poolbuf.appendChar(pool.put(nameType(dynSym)));
                }
            } else if (value instanceof Symbol.VarSymbol) {
                Symbol.VarSymbol v = (Symbol.VarSymbol) value;
                this.poolbuf.appendByte(9);
                this.poolbuf.appendChar(pool.put(v.owner));
                this.poolbuf.appendChar(pool.put(nameType(v)));
            } else if (value instanceof Name) {
                this.poolbuf.appendByte(1);
                byte[] bs = ((Name) value).toUtf();
                this.poolbuf.appendChar(bs.length);
                this.poolbuf.appendBytes(bs, 0, bs.length);
                if (bs.length > 65535) {
                    throw new StringOverflow(value.toString());
                }
            } else if (value instanceof Symbol.ClassSymbol) {
                Symbol.ClassSymbol c = (Symbol.ClassSymbol) value;
                if (c.owner.kind == 2) {
                    pool.put(c.owner);
                }
                this.poolbuf.appendByte(7);
                if (c.type.hasTag(TypeTag.ARRAY)) {
                    this.poolbuf.appendChar(pool.put(typeSig(c.type)));
                } else {
                    this.poolbuf.appendChar(pool.put(this.names.fromUtf(externalize(c.flatname))));
                    enterInner(c);
                }
            } else if (value instanceof ClassFile.NameAndType) {
                ClassFile.NameAndType nt = (ClassFile.NameAndType) value;
                this.poolbuf.appendByte(12);
                this.poolbuf.appendChar(pool.put(nt.name));
                this.poolbuf.appendChar(pool.put(typeSig(nt.uniqueType.type)));
            } else if (value instanceof Integer) {
                this.poolbuf.appendByte(3);
                this.poolbuf.appendInt(((Integer) value).intValue());
            } else if (value instanceof Long) {
                this.poolbuf.appendByte(5);
                this.poolbuf.appendLong(((Long) value).longValue());
                i++;
            } else if (value instanceof Float) {
                this.poolbuf.appendByte(4);
                this.poolbuf.appendFloat(((Float) value).floatValue());
            } else if (value instanceof Double) {
                this.poolbuf.appendByte(6);
                this.poolbuf.appendDouble(((Double) value).doubleValue());
                i++;
            } else if (value instanceof String) {
                this.poolbuf.appendByte(8);
                this.poolbuf.appendChar(pool.put(this.names.fromString((String) value)));
            } else if (value instanceof Types.UniqueType) {
                Type type = ((Types.UniqueType) value).type;
                if (type instanceof Type.MethodType) {
                    this.poolbuf.appendByte(16);
                    this.poolbuf.appendChar(pool.put(typeSig((Type.MethodType) type)));
                } else {
                    if (type.hasTag(TypeTag.CLASS)) {
                        enterInner((Symbol.ClassSymbol) type.tsym);
                    }
                    this.poolbuf.appendByte(7);
                    this.poolbuf.appendChar(pool.put(xClassName(type)));
                }
            } else if (value instanceof Pool.MethodHandle) {
                Pool.MethodHandle ref = (Pool.MethodHandle) value;
                this.poolbuf.appendByte(15);
                this.poolbuf.appendByte(ref.refKind);
                this.poolbuf.appendChar(pool.put(ref.refSym));
            } else {
                Assert.error("writePool " + value);
            }
            i++;
        }
        if (pool.pp > 65535) {
            throw new PoolOverflow();
        }
        putChar(this.poolbuf, poolCountIdx, pool.pp);
    }

    Name fieldName(Symbol sym) {
        if ((this.scramble && (sym.flags() & 2) != 0) || (this.scrambleAll && (sym.flags() & 5) == 0)) {
            return this.names.fromString("_$" + sym.name.getIndex());
        }
        return sym.name;
    }

    ClassFile.NameAndType nameType(Symbol sym) {
        Type typeExternalType;
        Name nameFieldName = fieldName(sym);
        if (this.retrofit) {
            typeExternalType = sym.erasure(this.types);
        } else {
            typeExternalType = sym.externalType(this.types);
        }
        return new ClassFile.NameAndType(nameFieldName, typeExternalType, this.types);
    }

    int writeAttr(Name attrName) {
        this.databuf.appendChar(this.pool.put(attrName));
        this.databuf.appendInt(0);
        return this.databuf.length;
    }

    void endAttr(int index) {
        putInt(this.databuf, index - 4, this.databuf.length - index);
    }

    int beginAttrs() {
        this.databuf.appendChar(0);
        return this.databuf.length;
    }

    void endAttrs(int index, int count) {
        putChar(this.databuf, index - 2, count);
    }

    int writeEnclosingMethodAttribute(Symbol.ClassSymbol c) {
        if (!this.target.hasEnclosingMethodAttribute()) {
            return 0;
        }
        return writeEnclosingMethodAttribute(this.names.EnclosingMethod, c);
    }

    protected int writeEnclosingMethodAttribute(Name attributeName, Symbol.ClassSymbol c) {
        if (c.owner.kind != 16 && c.name != this.names.empty) {
            return 0;
        }
        int alenIdx = writeAttr(attributeName);
        Symbol.ClassSymbol enclClass = c.owner.enclClass();
        Symbol.MethodSymbol enclMethod = (c.owner.type == null || c.owner.kind != 16) ? null : (Symbol.MethodSymbol) c.owner;
        this.databuf.appendChar(this.pool.put(enclClass));
        this.databuf.appendChar(enclMethod != null ? this.pool.put(nameType(c.owner)) : 0);
        endAttr(alenIdx);
        return 1;
    }

    int writeFlagAttrs(long flags) {
        int acount = 0;
        if ((131072 & flags) != 0) {
            int alenIdx = writeAttr(this.names.Deprecated);
            endAttr(alenIdx);
            acount = 0 + 1;
        }
        if ((16384 & flags) != 0 && !this.target.useEnumFlag()) {
            int alenIdx2 = writeAttr(this.names.Enum);
            endAttr(alenIdx2);
            acount++;
        }
        if ((4096 & flags) != 0 && !this.target.useSyntheticFlag()) {
            int alenIdx3 = writeAttr(this.names.Synthetic);
            endAttr(alenIdx3);
            acount++;
        }
        if ((Flags.BRIDGE & flags) != 0 && !this.target.useBridgeFlag()) {
            int alenIdx4 = writeAttr(this.names.Bridge);
            endAttr(alenIdx4);
            acount++;
        }
        if ((Flags.VARARGS & flags) != 0 && !this.target.useVarargsFlag()) {
            int alenIdx5 = writeAttr(this.names.Varargs);
            endAttr(alenIdx5);
            acount++;
        }
        if ((8192 & flags) != 0 && !this.target.useAnnotationFlag()) {
            int alenIdx6 = writeAttr(this.names.Annotation);
            endAttr(alenIdx6);
            return acount + 1;
        }
        return acount;
    }

    int writeMemberAttrs(Symbol sym) {
        int acount = writeFlagAttrs(sym.flags());
        long flags = sym.flags();
        if (this.source.allowGenerics() && (2147487744L & flags) != 4096 && (536870912 & flags) == 0 && (!this.types.isSameType(sym.type, sym.erasure(this.types)) || this.signatureGen.hasTypeVar(sym.type.mo179getThrownTypes()))) {
            int alenIdx = writeAttr(this.names.Signature);
            this.databuf.appendChar(this.pool.put(typeSig(sym.type)));
            endAttr(alenIdx);
            acount++;
        }
        return acount + writeJavaAnnotations(sym.getRawAttributes()) + writeTypeAnnotations(sym.getRawTypeAttributes(), false);
    }

    int writeMethodParametersAttr(Symbol.MethodSymbol m) {
        Type.MethodType ty = m.externalType(this.types).asMethodType();
        int allparams = ty.argtypes.size();
        if (m.params != null && allparams != 0) {
            int attrIndex = writeAttr(this.names.MethodParameters);
            this.databuf.appendByte(allparams);
            for (Symbol.VarSymbol s : m.extraParams) {
                int flags = (36880 & ((int) s.flags())) | (((int) m.flags()) & 4096);
                this.databuf.appendChar(this.pool.put(s.name));
                this.databuf.appendChar(flags);
            }
            for (Symbol.VarSymbol s2 : m.params) {
                int flags2 = (((int) s2.flags()) & 36880) | (((int) m.flags()) & 4096);
                this.databuf.appendChar(this.pool.put(s2.name));
                this.databuf.appendChar(flags2);
            }
            for (Symbol.VarSymbol s3 : m.capturedLocals) {
                int flags3 = (((int) s3.flags()) & 36880) | (((int) m.flags()) & 4096);
                this.databuf.appendChar(this.pool.put(s3.name));
                this.databuf.appendChar(flags3);
            }
            endAttr(attrIndex);
            return 1;
        }
        return 0;
    }

    int writeParameterAttrs(Symbol.MethodSymbol m) {
        boolean hasVisible = false;
        boolean hasInvisible = false;
        if (m.params != null) {
            for (Symbol.VarSymbol s : m.params) {
                Iterator<Attribute.Compound> it = s.getRawAttributes().iterator();
                while (it.hasNext()) {
                    switch (this.types.getRetention(it.next())) {
                        case CLASS:
                            hasInvisible = true;
                            break;
                        case RUNTIME:
                            hasVisible = true;
                            break;
                    }
                }
            }
        }
        int attrCount = 0;
        if (hasVisible) {
            int attrIndex = writeAttr(this.names.RuntimeVisibleParameterAnnotations);
            this.databuf.appendByte(m.params.length());
            for (Symbol.VarSymbol s2 : m.params) {
                ListBuffer<Attribute.Compound> buf = new ListBuffer<>();
                for (Attribute.Compound a : s2.getRawAttributes()) {
                    if (this.types.getRetention(a) == Attribute.RetentionPolicy.RUNTIME) {
                        buf.append(a);
                    }
                }
                this.databuf.appendChar(buf.length());
                Iterator<Attribute.Compound> it2 = buf.iterator();
                while (it2.hasNext()) {
                    writeCompoundAttribute(it2.next());
                }
            }
            endAttr(attrIndex);
            attrCount = 0 + 1;
        }
        if (hasInvisible) {
            int attrIndex2 = writeAttr(this.names.RuntimeInvisibleParameterAnnotations);
            this.databuf.appendByte(m.params.length());
            for (Symbol.VarSymbol s3 : m.params) {
                ListBuffer<Attribute.Compound> buf2 = new ListBuffer<>();
                for (Attribute.Compound a2 : s3.getRawAttributes()) {
                    if (this.types.getRetention(a2) == Attribute.RetentionPolicy.CLASS) {
                        buf2.append(a2);
                    }
                }
                this.databuf.appendChar(buf2.length());
                Iterator<Attribute.Compound> it3 = buf2.iterator();
                while (it3.hasNext()) {
                    writeCompoundAttribute(it3.next());
                }
            }
            endAttr(attrIndex2);
            return attrCount + 1;
        }
        return attrCount;
    }

    int writeJavaAnnotations(List<Attribute.Compound> attrs) {
        if (attrs.isEmpty()) {
            return 0;
        }
        ListBuffer<Attribute.Compound> visibles = new ListBuffer<>();
        ListBuffer<Attribute.Compound> invisibles = new ListBuffer<>();
        for (Attribute.Compound a : attrs) {
            switch (this.types.getRetention(a)) {
                case CLASS:
                    invisibles.append(a);
                    break;
                case RUNTIME:
                    visibles.append(a);
                    break;
            }
        }
        int attrCount = 0;
        if (visibles.length() != 0) {
            int attrIndex = writeAttr(this.names.RuntimeVisibleAnnotations);
            this.databuf.appendChar(visibles.length());
            for (Attribute.Compound a2 : visibles) {
                writeCompoundAttribute(a2);
            }
            endAttr(attrIndex);
            attrCount = 0 + 1;
        }
        int attrIndex2 = invisibles.length();
        if (attrIndex2 != 0) {
            int attrIndex3 = writeAttr(this.names.RuntimeInvisibleAnnotations);
            this.databuf.appendChar(invisibles.length());
            for (Attribute.Compound a3 : invisibles) {
                writeCompoundAttribute(a3);
            }
            endAttr(attrIndex3);
            return attrCount + 1;
        }
        return attrCount;
    }

    int writeTypeAnnotations(List<Attribute.TypeCompound> typeAnnos, boolean inCode) {
        if (typeAnnos.isEmpty()) {
            return 0;
        }
        ListBuffer<Attribute.TypeCompound> visibles = new ListBuffer<>();
        ListBuffer<Attribute.TypeCompound> invisibles = new ListBuffer<>();
        for (Attribute.TypeCompound tc : typeAnnos) {
            if (tc.hasUnknownPosition()) {
                boolean fixed = tc.tryFixPosition();
                if (!fixed) {
                    PrintWriter pw = this.log.getWriter(Log.WriterKind.ERROR);
                    pw.println("ClassWriter: Position UNKNOWN in type annotation: " + tc);
                }
            }
            if (tc.position.type.isLocal() == inCode && tc.position.emitToClassfile()) {
                switch (this.types.getRetention(tc)) {
                    case CLASS:
                        invisibles.append(tc);
                        break;
                    case RUNTIME:
                        visibles.append(tc);
                        break;
                }
            }
        }
        int attrCount = 0;
        if (visibles.length() != 0) {
            int attrIndex = writeAttr(this.names.RuntimeVisibleTypeAnnotations);
            this.databuf.appendChar(visibles.length());
            for (Attribute.TypeCompound p : visibles) {
                writeTypeAnnotation(p);
            }
            endAttr(attrIndex);
            attrCount = 0 + 1;
        }
        int attrIndex2 = invisibles.length();
        if (attrIndex2 != 0) {
            int attrIndex3 = writeAttr(this.names.RuntimeInvisibleTypeAnnotations);
            this.databuf.appendChar(invisibles.length());
            for (Attribute.TypeCompound p2 : invisibles) {
                writeTypeAnnotation(p2);
            }
            endAttr(attrIndex3);
            return attrCount + 1;
        }
        return attrCount;
    }

    class AttributeWriter implements Attribute.Visitor {
        AttributeWriter() {
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitConstant(Attribute.Constant _value) {
            Object value = _value.value;
            switch (_value.type.getTag()) {
                case BYTE:
                    ClassWriter.this.databuf.appendByte(66);
                    break;
                case CHAR:
                    ClassWriter.this.databuf.appendByte(67);
                    break;
                case SHORT:
                    ClassWriter.this.databuf.appendByte(83);
                    break;
                case INT:
                    ClassWriter.this.databuf.appendByte(73);
                    break;
                case LONG:
                    ClassWriter.this.databuf.appendByte(74);
                    break;
                case FLOAT:
                    ClassWriter.this.databuf.appendByte(70);
                    break;
                case DOUBLE:
                    ClassWriter.this.databuf.appendByte(68);
                    break;
                case BOOLEAN:
                    ClassWriter.this.databuf.appendByte(90);
                    break;
                case CLASS:
                    Assert.check(value instanceof String);
                    ClassWriter.this.databuf.appendByte(115);
                    value = ClassWriter.this.names.fromString(value.toString());
                    break;
                default:
                    throw new AssertionError(_value.type);
            }
            ClassWriter.this.databuf.appendChar(ClassWriter.this.pool.put(value));
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitEnum(Attribute.Enum e) {
            ClassWriter.this.databuf.appendByte(101);
            ClassWriter.this.databuf.appendChar(ClassWriter.this.pool.put(ClassWriter.this.typeSig(e.value.type)));
            ClassWriter.this.databuf.appendChar(ClassWriter.this.pool.put(e.value.name));
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitClass(Attribute.Class clazz) {
            ClassWriter.this.databuf.appendByte(99);
            ClassWriter.this.databuf.appendChar(ClassWriter.this.pool.put(ClassWriter.this.typeSig(clazz.classType)));
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitCompound(Attribute.Compound compound) {
            ClassWriter.this.databuf.appendByte(64);
            ClassWriter.this.writeCompoundAttribute(compound);
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitError(Attribute.Error x) {
            throw new AssertionError(x);
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitArray(Attribute.Array array) {
            ClassWriter.this.databuf.appendByte(91);
            ClassWriter.this.databuf.appendChar(array.values.length);
            for (Attribute a : array.values) {
                a.accept(this);
            }
        }
    }

    void writeCompoundAttribute(Attribute.Compound c) {
        this.databuf.appendChar(this.pool.put(typeSig(c.type)));
        this.databuf.appendChar(c.values.length());
        for (Pair<Symbol.MethodSymbol, Attribute> p : c.values) {
            this.databuf.appendChar(this.pool.put(p.fst.name));
            p.snd.accept(this.awriter);
        }
    }

    void writeTypeAnnotation(Attribute.TypeCompound c) {
        writePosition(c.position);
        writeCompoundAttribute(c);
    }

    void writePosition(TypeAnnotationPosition p) {
        this.databuf.appendByte(p.type.targetTypeValue());
        switch (p.type) {
            case INSTANCEOF:
            case NEW:
            case CONSTRUCTOR_REFERENCE:
            case METHOD_REFERENCE:
                this.databuf.appendChar(p.offset);
                break;
            case LOCAL_VARIABLE:
            case RESOURCE_VARIABLE:
                this.databuf.appendChar(p.lvarOffset.length);
                for (int i = 0; i < p.lvarOffset.length; i++) {
                    this.databuf.appendChar(p.lvarOffset[i]);
                    this.databuf.appendChar(p.lvarLength[i]);
                    this.databuf.appendChar(p.lvarIndex[i]);
                }
                break;
            case EXCEPTION_PARAMETER:
                this.databuf.appendChar(p.exception_index);
                break;
            case METHOD_RECEIVER:
            case METHOD_RETURN:
            case FIELD:
                break;
            case CLASS_TYPE_PARAMETER:
            case METHOD_TYPE_PARAMETER:
                this.databuf.appendByte(p.parameter_index);
                break;
            case CLASS_TYPE_PARAMETER_BOUND:
            case METHOD_TYPE_PARAMETER_BOUND:
                this.databuf.appendByte(p.parameter_index);
                this.databuf.appendByte(p.bound_index);
                break;
            case CLASS_EXTENDS:
                this.databuf.appendChar(p.type_index);
                break;
            case THROWS:
                this.databuf.appendChar(p.type_index);
                break;
            case METHOD_FORMAL_PARAMETER:
                this.databuf.appendByte(p.parameter_index);
                break;
            case CAST:
            case CONSTRUCTOR_INVOCATION_TYPE_ARGUMENT:
            case METHOD_INVOCATION_TYPE_ARGUMENT:
            case CONSTRUCTOR_REFERENCE_TYPE_ARGUMENT:
            case METHOD_REFERENCE_TYPE_ARGUMENT:
                this.databuf.appendChar(p.offset);
                this.databuf.appendByte(p.type_index);
                break;
            case UNKNOWN:
                throw new AssertionError("jvm.ClassWriter: UNKNOWN target type should never occur!");
            default:
                throw new AssertionError("jvm.ClassWriter: Unknown target type for position: " + p);
        }
        this.databuf.appendByte(p.location.size());
        java.util.List<Integer> loc = TypeAnnotationPosition.getBinaryFromTypePath(p.location);
        Iterator<Integer> it = loc.iterator();
        while (it.hasNext()) {
            int i2 = it.next().intValue();
            this.databuf.appendByte((byte) i2);
        }
    }

    void enterInner(Symbol.ClassSymbol c) {
        if (c.type.isCompound()) {
            throw new AssertionError("Unexpected intersection type: " + c.type);
        }
        try {
            c.complete();
            if (c.type.hasTag(TypeTag.CLASS) && this.pool != null && c.owner.enclClass() != null) {
                if (this.innerClasses == null || !this.innerClasses.contains(c)) {
                    enterInner(c.owner.enclClass());
                    this.pool.put(c);
                    if (c.name != this.names.empty) {
                        this.pool.put(c.name);
                    }
                    if (this.innerClasses == null) {
                        this.innerClasses = new HashSet();
                        this.innerClassesQueue = new ListBuffer<>();
                        this.pool.put(this.names.InnerClasses);
                    }
                    this.innerClasses.add(c);
                    this.innerClassesQueue.append(c);
                }
            }
        } catch (Symbol.CompletionFailure ex) {
            System.err.println("error: " + c + ": " + ex.getMessage());
            throw ex;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void writeInnerClasses() {
        int alenIdx = writeAttr(this.names.InnerClasses);
        this.databuf.appendChar(this.innerClassesQueue.length());
        for (List list = this.innerClassesQueue.toList(); list.nonEmpty(); list = list.tail) {
            Symbol.ClassSymbol inner = (Symbol.ClassSymbol) list.head;
            inner.markAbstractIfNeeded(this.types);
            char flags = (char) adjustFlags(inner.flags_field);
            if ((flags & 512) != 0) {
                flags = (char) (flags | 1024);
            }
            if (inner.name.isEmpty()) {
                flags = (char) (flags & (-17));
            }
            char flags2 = (char) (flags & (-2049));
            if (this.dumpInnerClassModifiers) {
                PrintWriter pw = this.log.getWriter(Log.WriterKind.ERROR);
                pw.println("INNERCLASS  " + ((Object) inner.name));
                pw.println("---" + flagNames(flags2));
            }
            this.databuf.appendChar(this.pool.get(inner));
            int i = 0;
            this.databuf.appendChar((inner.owner.kind != 2 || inner.name.isEmpty()) ? 0 : this.pool.get(inner.owner));
            ByteBuffer byteBuffer = this.databuf;
            if (!inner.name.isEmpty()) {
                i = this.pool.get(inner.name);
            }
            byteBuffer.appendChar(i);
            this.databuf.appendChar(flags2);
        }
        endAttr(alenIdx);
    }

    void writeBootstrapMethods() {
        int alenIdx = writeAttr(this.names.BootstrapMethods);
        this.databuf.appendChar(this.bootstrapMethods.size());
        for (Map.Entry<Pool.DynamicMethod, Pool.MethodHandle> entry : this.bootstrapMethods.entrySet()) {
            Pool.DynamicMethod dmeth = entry.getKey();
            Symbol.DynamicMethodSymbol dsym = (Symbol.DynamicMethodSymbol) dmeth.baseSymbol();
            this.databuf.appendChar(this.pool.get(entry.getValue()));
            this.databuf.appendChar(dsym.staticArgs.length);
            Object[] uniqueArgs = dmeth.uniqueStaticArgs;
            for (Object o : uniqueArgs) {
                this.databuf.appendChar(this.pool.get(o));
            }
        }
        endAttr(alenIdx);
    }

    void writeField(Symbol.VarSymbol v) {
        int flags = adjustFlags(v.flags());
        this.databuf.appendChar(flags);
        if (this.dumpFieldModifiers) {
            PrintWriter pw = this.log.getWriter(Log.WriterKind.ERROR);
            pw.println("FIELD  " + ((Object) fieldName(v)));
            pw.println("---" + flagNames(v.flags()));
        }
        this.databuf.appendChar(this.pool.put(fieldName(v)));
        this.databuf.appendChar(this.pool.put(typeSig(v.erasure(this.types))));
        int acountIdx = beginAttrs();
        int acount = 0;
        if (v.getConstValue() != null) {
            int alenIdx = writeAttr(this.names.ConstantValue);
            this.databuf.appendChar(this.pool.put(v.getConstValue()));
            endAttr(alenIdx);
            acount = 0 + 1;
        }
        int alenIdx2 = writeMemberAttrs(v);
        endAttrs(acountIdx, acount + alenIdx2);
    }

    /* JADX WARN: Multi-variable type inference failed */
    void writeMethod(Symbol.MethodSymbol m) {
        int flags = adjustFlags(m.flags());
        this.databuf.appendChar(flags);
        if (this.dumpMethodModifiers) {
            PrintWriter pw = this.log.getWriter(Log.WriterKind.ERROR);
            pw.println("METHOD  " + ((Object) fieldName(m)));
            pw.println("---" + flagNames(m.flags()));
        }
        this.databuf.appendChar(this.pool.put(fieldName(m)));
        this.databuf.appendChar(this.pool.put(typeSig(m.externalType(this.types))));
        int acountIdx = beginAttrs();
        int acount = 0;
        if (m.code != null) {
            int alenIdx = writeAttr(this.names.Code);
            writeCode(m.code);
            m.code = null;
            endAttr(alenIdx);
            acount = 0 + 1;
        }
        List<Type> thrown = m.erasure(this.types).mo179getThrownTypes();
        if (thrown.nonEmpty()) {
            int alenIdx2 = writeAttr(this.names.Exceptions);
            this.databuf.appendChar(thrown.length());
            for (List list = thrown; list.nonEmpty(); list = list.tail) {
                this.databuf.appendChar(this.pool.put(((Type) list.head).tsym));
            }
            endAttr(alenIdx2);
            acount++;
        }
        if (m.defaultValue != null) {
            int alenIdx3 = writeAttr(this.names.AnnotationDefault);
            m.defaultValue.accept(this.awriter);
            endAttr(alenIdx3);
            acount++;
        }
        if (this.options.isSet(Option.PARAMETERS)) {
            acount += writeMethodParametersAttr(m);
        }
        endAttrs(acountIdx, acount + writeMemberAttrs(m) + writeParameterAttrs(m));
    }

    /* JADX WARN: Multi-variable type inference failed */
    void writeCode(Code code) {
        this.databuf.appendChar(code.max_stack);
        this.databuf.appendChar(code.max_locals);
        this.databuf.appendInt(code.cp);
        boolean z = false;
        this.databuf.appendBytes(code.code, 0, code.cp);
        this.databuf.appendChar(code.catchInfo.length());
        for (List list = code.catchInfo.toList(); list.nonEmpty(); list = list.tail) {
            for (int i = 0; i < ((char[]) list.head).length; i++) {
                this.databuf.appendChar(((char[]) list.head)[i]);
            }
        }
        int acountIdx = beginAttrs();
        int acount = 0;
        if (code.lineInfo.nonEmpty()) {
            int alenIdx = writeAttr(this.names.LineNumberTable);
            this.databuf.appendChar(code.lineInfo.length());
            for (List listReverse = code.lineInfo.reverse(); listReverse.nonEmpty(); listReverse = listReverse.tail) {
                for (int i2 = 0; i2 < ((char[]) listReverse.head).length; i2++) {
                    this.databuf.appendChar(((char[]) listReverse.head)[i2]);
                }
            }
            endAttr(alenIdx);
            acount = 0 + 1;
        }
        if (this.genCrt && code.crt != null) {
            CRTable crt = code.crt;
            int alenIdx2 = writeAttr(this.names.CharacterRangeTable);
            int crtIdx = beginAttrs();
            int crtEntries = crt.writeCRT(this.databuf, code.lineMap, this.log);
            endAttrs(crtIdx, crtEntries);
            endAttr(alenIdx2);
            acount++;
        }
        if (code.varDebugInfo && code.varBufferSize > 0) {
            int nGenericVars = 0;
            int alenIdx3 = writeAttr(this.names.LocalVariableTable);
            this.databuf.appendChar(code.getLVTSize());
            int i3 = 0;
            while (i3 < code.varBufferSize) {
                Code.LocalVar var = code.varBuffer[i3];
                for (Code.LocalVar.Range r : var.aliveRanges) {
                    Assert.check((r.start_pc < 0 || r.start_pc > code.cp) ? z : true);
                    this.databuf.appendChar(r.start_pc);
                    Assert.check((r.length <= 0 || r.start_pc + r.length > code.cp) ? z : true);
                    this.databuf.appendChar(r.length);
                    Symbol.VarSymbol sym = var.sym;
                    this.databuf.appendChar(this.pool.put(sym.name));
                    Type vartype = sym.erasure(this.types);
                    this.databuf.appendChar(this.pool.put(typeSig(vartype)));
                    this.databuf.appendChar(var.reg);
                    if (needsLocalVariableTypeEntry(var.sym.type)) {
                        nGenericVars++;
                    }
                    z = false;
                }
                i3++;
                z = false;
            }
            endAttr(alenIdx3);
            acount++;
            if (nGenericVars > 0) {
                int alenIdx4 = writeAttr(this.names.LocalVariableTypeTable);
                this.databuf.appendChar(nGenericVars);
                int count = 0;
                for (int i4 = 0; i4 < code.varBufferSize; i4++) {
                    Code.LocalVar var2 = code.varBuffer[i4];
                    Symbol.VarSymbol sym2 = var2.sym;
                    if (needsLocalVariableTypeEntry(sym2.type)) {
                        for (Code.LocalVar.Range r2 : var2.aliveRanges) {
                            this.databuf.appendChar(r2.start_pc);
                            this.databuf.appendChar(r2.length);
                            this.databuf.appendChar(this.pool.put(sym2.name));
                            this.databuf.appendChar(this.pool.put(typeSig(sym2.type)));
                            this.databuf.appendChar(var2.reg);
                            count++;
                        }
                    }
                }
                Assert.check(count == nGenericVars);
                endAttr(alenIdx4);
                acount++;
            }
        }
        int nGenericVars2 = code.stackMapBufferSize;
        if (nGenericVars2 > 0) {
            if (this.debugstackmap) {
                System.out.println("Stack map for " + code.meth);
            }
            int alenIdx5 = writeAttr(code.stackMap.getAttributeName(this.names));
            writeStackMap(code);
            endAttr(alenIdx5);
            acount++;
        }
        endAttrs(acountIdx, acount + writeTypeAnnotations(code.meth.getRawTypeAttributes(), true));
    }

    private boolean needsLocalVariableTypeEntry(Type t) {
        return (this.types.isSameType(t, this.types.erasure(t)) || t.isCompound()) ? false : true;
    }

    void writeStackMap(Code code) {
        int nframes = code.stackMapBufferSize;
        if (this.debugstackmap) {
            System.out.println(" nframes = " + nframes);
        }
        this.databuf.appendChar(nframes);
        switch (code.stackMap) {
            case CLDC:
                for (int i = 0; i < nframes; i++) {
                    if (this.debugstackmap) {
                        System.out.print("  " + i + ":");
                    }
                    Code.StackMapFrame frame = code.stackMapBuffer[i];
                    if (this.debugstackmap) {
                        System.out.print(" pc=" + frame.pc);
                    }
                    this.databuf.appendChar(frame.pc);
                    int localCount = 0;
                    int j = 0;
                    while (true) {
                        int iWidth = 1;
                        if (j < frame.locals.length) {
                            localCount++;
                            if (!this.target.generateEmptyAfterBig()) {
                                iWidth = Code.width(frame.locals[j]);
                            }
                            j += iWidth;
                        } else {
                            if (this.debugstackmap) {
                                System.out.print(" nlocals=" + localCount);
                            }
                            this.databuf.appendChar(localCount);
                            int j2 = 0;
                            while (j2 < frame.locals.length) {
                                if (this.debugstackmap) {
                                    System.out.print(" local[" + j2 + "]=");
                                }
                                writeStackMapType(frame.locals[j2]);
                                j2 += this.target.generateEmptyAfterBig() ? 1 : Code.width(frame.locals[j2]);
                            }
                            int stackCount = 0;
                            int j3 = 0;
                            while (j3 < frame.stack.length) {
                                stackCount++;
                                j3 += this.target.generateEmptyAfterBig() ? 1 : Code.width(frame.stack[j3]);
                            }
                            if (this.debugstackmap) {
                                System.out.print(" nstack=" + stackCount);
                            }
                            this.databuf.appendChar(stackCount);
                            int j4 = 0;
                            while (j4 < frame.stack.length) {
                                if (this.debugstackmap) {
                                    System.out.print(" stack[" + j4 + "]=");
                                }
                                writeStackMapType(frame.stack[j4]);
                                j4 += this.target.generateEmptyAfterBig() ? 1 : Code.width(frame.stack[j4]);
                            }
                            if (this.debugstackmap) {
                                System.out.println();
                            }
                        }
                    }
                }
                return;
            case JSR202:
                Assert.checkNull(code.stackMapBuffer);
                for (int i2 = 0; i2 < nframes; i2++) {
                    if (this.debugstackmap) {
                        System.out.print("  " + i2 + ":");
                    }
                    code.stackMapTableBuffer[i2].write(this);
                    if (this.debugstackmap) {
                        System.out.println();
                    }
                }
                return;
            default:
                throw new AssertionError("Unexpected stackmap format value");
        }
    }

    void writeStackMapType(Type t) {
        if (t == null) {
            if (this.debugstackmap) {
                System.out.print("empty");
            }
            this.databuf.appendByte(0);
            return;
        }
        switch (t.getTag()) {
            case UNINITIALIZED_THIS:
                if (this.debugstackmap) {
                    System.out.print("uninit_this");
                }
                this.databuf.appendByte(6);
                return;
            case UNINITIALIZED_OBJECT:
                UninitializedType uninitType = (UninitializedType) t;
                this.databuf.appendByte(8);
                if (this.debugstackmap) {
                    System.out.print("uninit_object@" + uninitType.offset);
                }
                this.databuf.appendChar(uninitType.offset);
                return;
            case BYTE:
            case CHAR:
            case SHORT:
            case INT:
            case BOOLEAN:
                if (this.debugstackmap) {
                    System.out.print("int");
                }
                this.databuf.appendByte(1);
                return;
            case LONG:
                if (this.debugstackmap) {
                    System.out.print("long");
                }
                this.databuf.appendByte(4);
                return;
            case FLOAT:
                if (this.debugstackmap) {
                    System.out.print("float");
                }
                this.databuf.appendByte(2);
                return;
            case DOUBLE:
                if (this.debugstackmap) {
                    System.out.print("double");
                }
                this.databuf.appendByte(3);
                return;
            case CLASS:
            case ARRAY:
                if (this.debugstackmap) {
                    System.out.print("object(" + t + ")");
                }
                this.databuf.appendByte(7);
                this.databuf.appendChar(this.pool.put(t));
                return;
            case BOT:
                if (this.debugstackmap) {
                    System.out.print("null");
                }
                this.databuf.appendByte(5);
                return;
            case TYPEVAR:
                if (this.debugstackmap) {
                    System.out.print("object(" + this.types.erasure(t).tsym + ")");
                }
                this.databuf.appendByte(7);
                this.databuf.appendChar(this.pool.put(this.types.erasure(t).tsym));
                return;
            default:
                throw new AssertionError();
        }
    }

    static abstract class StackMapTableFrame {
        abstract int getFrameType();

        StackMapTableFrame() {
        }

        void write(ClassWriter writer) {
            int frameType = getFrameType();
            writer.databuf.appendByte(frameType);
            if (writer.debugstackmap) {
                System.out.print(" frame_type=" + frameType);
            }
        }

        static class SameFrame extends StackMapTableFrame {
            final int offsetDelta;

            SameFrame(int offsetDelta) {
                this.offsetDelta = offsetDelta;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            int getFrameType() {
                return this.offsetDelta < 64 ? this.offsetDelta : ClassWriter.SAME_FRAME_EXTENDED;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            void write(ClassWriter writer) {
                super.write(writer);
                if (getFrameType() == ClassWriter.SAME_FRAME_EXTENDED) {
                    writer.databuf.appendChar(this.offsetDelta);
                    if (writer.debugstackmap) {
                        System.out.print(" offset_delta=" + this.offsetDelta);
                    }
                }
            }
        }

        static class SameLocals1StackItemFrame extends StackMapTableFrame {
            final int offsetDelta;
            final Type stack;

            SameLocals1StackItemFrame(int offsetDelta, Type stack) {
                this.offsetDelta = offsetDelta;
                this.stack = stack;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            int getFrameType() {
                return this.offsetDelta < 64 ? this.offsetDelta + 64 : ClassWriter.SAME_LOCALS_1_STACK_ITEM_EXTENDED;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            void write(ClassWriter writer) {
                super.write(writer);
                if (getFrameType() == ClassWriter.SAME_LOCALS_1_STACK_ITEM_EXTENDED) {
                    writer.databuf.appendChar(this.offsetDelta);
                    if (writer.debugstackmap) {
                        System.out.print(" offset_delta=" + this.offsetDelta);
                    }
                }
                if (writer.debugstackmap) {
                    System.out.print(" stack[0]=");
                }
                writer.writeStackMapType(this.stack);
            }
        }

        static class ChopFrame extends StackMapTableFrame {
            final int frameType;
            final int offsetDelta;

            ChopFrame(int frameType, int offsetDelta) {
                this.frameType = frameType;
                this.offsetDelta = offsetDelta;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            int getFrameType() {
                return this.frameType;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            void write(ClassWriter writer) {
                super.write(writer);
                writer.databuf.appendChar(this.offsetDelta);
                if (writer.debugstackmap) {
                    System.out.print(" offset_delta=" + this.offsetDelta);
                }
            }
        }

        static class AppendFrame extends StackMapTableFrame {
            final int frameType;
            final Type[] locals;
            final int offsetDelta;

            AppendFrame(int frameType, int offsetDelta, Type[] locals) {
                this.frameType = frameType;
                this.offsetDelta = offsetDelta;
                this.locals = locals;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            int getFrameType() {
                return this.frameType;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            void write(ClassWriter writer) {
                super.write(writer);
                writer.databuf.appendChar(this.offsetDelta);
                if (writer.debugstackmap) {
                    System.out.print(" offset_delta=" + this.offsetDelta);
                }
                for (int i = 0; i < this.locals.length; i++) {
                    if (writer.debugstackmap) {
                        System.out.print(" locals[" + i + "]=");
                    }
                    writer.writeStackMapType(this.locals[i]);
                }
            }
        }

        static class FullFrame extends StackMapTableFrame {
            final Type[] locals;
            final int offsetDelta;
            final Type[] stack;

            FullFrame(int offsetDelta, Type[] locals, Type[] stack) {
                this.offsetDelta = offsetDelta;
                this.locals = locals;
                this.stack = stack;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            int getFrameType() {
                return 255;
            }

            @Override // com.sun.tools.javac.jvm.ClassWriter.StackMapTableFrame
            void write(ClassWriter writer) {
                super.write(writer);
                writer.databuf.appendChar(this.offsetDelta);
                writer.databuf.appendChar(this.locals.length);
                if (writer.debugstackmap) {
                    System.out.print(" offset_delta=" + this.offsetDelta);
                    System.out.print(" nlocals=" + this.locals.length);
                }
                for (int i = 0; i < this.locals.length; i++) {
                    if (writer.debugstackmap) {
                        System.out.print(" locals[" + i + "]=");
                    }
                    writer.writeStackMapType(this.locals[i]);
                }
                writer.databuf.appendChar(this.stack.length);
                if (writer.debugstackmap) {
                    System.out.print(" nstack=" + this.stack.length);
                }
                for (int i2 = 0; i2 < this.stack.length; i2++) {
                    if (writer.debugstackmap) {
                        System.out.print(" stack[" + i2 + "]=");
                    }
                    writer.writeStackMapType(this.stack[i2]);
                }
            }
        }

        static StackMapTableFrame getInstance(Code.StackMapFrame this_frame, int prev_pc, Type[] prev_locals, Types types) {
            Type[] locals = this_frame.locals;
            Type[] stack = this_frame.stack;
            int offset_delta = (this_frame.pc - prev_pc) - 1;
            if (stack.length == 1) {
                if (locals.length == prev_locals.length && compare(prev_locals, locals, types) == 0) {
                    return new SameLocals1StackItemFrame(offset_delta, stack[0]);
                }
            } else if (stack.length == 0) {
                int diff_length = compare(prev_locals, locals, types);
                if (diff_length == 0) {
                    return new SameFrame(offset_delta);
                }
                if (-4 < diff_length && diff_length < 0) {
                    Type[] local_diff = new Type[-diff_length];
                    int i = prev_locals.length;
                    int j = 0;
                    while (i < locals.length) {
                        local_diff[j] = locals[i];
                        i++;
                        j++;
                    }
                    return new AppendFrame(251 - diff_length, offset_delta, local_diff);
                }
                if (diff_length > 0 && diff_length < 4) {
                    return new ChopFrame(251 - diff_length, offset_delta);
                }
            }
            return new FullFrame(offset_delta, locals, stack);
        }

        static boolean isInt(Type t) {
            return t.getTag().isStrictSubRangeOf(TypeTag.INT) || t.hasTag(TypeTag.BOOLEAN);
        }

        static boolean isSameType(Type t1, Type t2, Types types) {
            if (t1 == null) {
                return t2 == null;
            }
            if (t2 == null) {
                return false;
            }
            if (isInt(t1) && isInt(t2)) {
                return true;
            }
            if (t1.hasTag(TypeTag.UNINITIALIZED_THIS)) {
                return t2.hasTag(TypeTag.UNINITIALIZED_THIS);
            }
            if (t1.hasTag(TypeTag.UNINITIALIZED_OBJECT)) {
                return t2.hasTag(TypeTag.UNINITIALIZED_OBJECT) && ((UninitializedType) t1).offset == ((UninitializedType) t2).offset;
            }
            if (t2.hasTag(TypeTag.UNINITIALIZED_THIS) || t2.hasTag(TypeTag.UNINITIALIZED_OBJECT)) {
                return false;
            }
            return types.isSameType(t1, t2);
        }

        static int compare(Type[] arr1, Type[] arr2, Types types) {
            int diff_length = arr1.length - arr2.length;
            if (diff_length > 4 || diff_length < -4) {
                return Integer.MAX_VALUE;
            }
            int len = diff_length > 0 ? arr2.length : arr1.length;
            for (int i = 0; i < len; i++) {
                if (!isSameType(arr1[i], arr2[i], types)) {
                    return Integer.MAX_VALUE;
                }
            }
            return diff_length;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void writeFields(Scope.Entry e) {
        List listNil = List.nil();
        for (Scope.Entry i = e; i != null; i = i.sibling) {
            if (i.sym.kind == 4) {
                listNil = listNil.prepend((Symbol.VarSymbol) i.sym);
            }
        }
        while (listNil.nonEmpty()) {
            writeField((Symbol.VarSymbol) listNil.head);
            listNil = listNil.tail;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void writeMethods(Scope.Entry e) {
        List listNil = List.nil();
        for (Scope.Entry i = e; i != null; i = i.sibling) {
            if (i.sym.kind == 16 && (i.sym.flags() & Flags.HYPOTHETICAL) == 0) {
                listNil = listNil.prepend((Symbol.MethodSymbol) i.sym);
            }
        }
        while (listNil.nonEmpty()) {
            writeMethod((Symbol.MethodSymbol) listNil.head);
            listNil = listNil.tail;
        }
    }

    public JavaFileObject writeClass(Symbol.ClassSymbol c) throws StringOverflow, IOException, PoolOverflow {
        JavaFileObject outFile = this.fileManager.getJavaFileForOutput(StandardLocation.CLASS_OUTPUT, c.flatname.toString(), JavaFileObject.Kind.CLASS, c.sourcefile);
        OutputStream out = outFile.openOutputStream();
        try {
            writeClassFile(out, c);
            if (this.verbose) {
                this.log.printVerbose("wrote.file", outFile);
            }
            out.close();
            out = null;
            if (out != null) {
                return null;
            }
            return outFile;
        } finally {
            if (out != null) {
                out.close();
                outFile.delete();
            }
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    public void writeClassFile(OutputStream out, Symbol.ClassSymbol c) throws StringOverflow, PoolOverflow, IOException {
        Assert.check((c.flags() & 16777216) == 0);
        this.databuf.reset();
        this.poolbuf.reset();
        this.signatureGen.reset();
        this.pool = c.pool;
        this.innerClasses = null;
        this.innerClassesQueue = null;
        this.bootstrapMethods = new LinkedHashMap();
        Type supertype = this.types.supertype(c.type);
        List<Type> interfaces = this.types.interfaces(c.type);
        List<Type> typarams = c.type.getTypeArguments();
        int flags = adjustFlags(c.flags() & (-8796093022209L));
        if ((flags & 4) != 0) {
            flags |= 1;
        }
        int flags2 = flags & Flags.ClassFlags & (-2049);
        if ((flags2 & 512) == 0) {
            flags2 |= 32;
        }
        if (c.isInner() && c.name.isEmpty()) {
            flags2 &= -17;
        }
        if (this.dumpClassModifiers) {
            PrintWriter pw = this.log.getWriter(Log.WriterKind.ERROR);
            pw.println();
            pw.println("CLASSFILE  " + ((Object) c.getQualifiedName()));
            pw.println("---" + flagNames(flags2));
        }
        this.databuf.appendChar(flags2);
        this.databuf.appendChar(this.pool.put(c));
        this.databuf.appendChar(supertype.hasTag(TypeTag.CLASS) ? this.pool.put(supertype.tsym) : 0);
        this.databuf.appendChar(interfaces.length());
        for (List list = interfaces; list.nonEmpty(); list = list.tail) {
            this.databuf.appendChar(this.pool.put(((Type) list.head).tsym));
        }
        int fieldsCount = 0;
        int methodsCount = 0;
        for (Scope.Entry e = c.members().elems; e != null; e = e.sibling) {
            switch (e.sym.kind) {
                case 2:
                    enterInner((Symbol.ClassSymbol) e.sym);
                    break;
                case 4:
                    fieldsCount++;
                    break;
                case 16:
                    if ((e.sym.flags() & Flags.HYPOTHETICAL) == 0) {
                        methodsCount++;
                    }
                    break;
                default:
                    Assert.error();
                    break;
            }
        }
        if (c.trans_local != null) {
            for (Symbol.ClassSymbol local : c.trans_local) {
                enterInner(local);
            }
        }
        this.databuf.appendChar(fieldsCount);
        writeFields(c.members().elems);
        this.databuf.appendChar(methodsCount);
        writeMethods(c.members().elems);
        int acountIdx = beginAttrs();
        int acount = 0;
        boolean sigReq = (typarams.length() == 0 && supertype.allparams().length() == 0) ? false : true;
        for (List list2 = interfaces; !sigReq && list2.nonEmpty(); list2 = list2.tail) {
            sigReq = ((Type) list2.head).allparams().length() != 0;
        }
        if (sigReq) {
            Assert.check(this.source.allowGenerics());
            int alenIdx = writeAttr(this.names.Signature);
            if (typarams.length() != 0) {
                this.signatureGen.assembleParamsSig(typarams);
            }
            this.signatureGen.assembleSig(supertype);
            for (List list3 = interfaces; list3.nonEmpty(); list3 = list3.tail) {
                this.signatureGen.assembleSig((Type) list3.head);
            }
            this.databuf.appendChar(this.pool.put(this.signatureGen.toName()));
            this.signatureGen.reset();
            endAttr(alenIdx);
            acount = 0 + 1;
        }
        if (c.sourcefile != null && this.emitSourceFile) {
            int alenIdx2 = writeAttr(this.names.SourceFile);
            String simpleName = BaseFileObject.getSimpleName(c.sourcefile);
            this.databuf.appendChar(c.pool.put(this.names.fromString(simpleName)));
            endAttr(alenIdx2);
            acount++;
        }
        if (this.genCrt) {
            int alenIdx3 = writeAttr(this.names.SourceID);
            this.databuf.appendChar(c.pool.put(this.names.fromString(Long.toString(getLastModified(c.sourcefile)))));
            endAttr(alenIdx3);
            int alenIdx4 = writeAttr(this.names.CompilationID);
            this.databuf.appendChar(c.pool.put(this.names.fromString(Long.toString(System.currentTimeMillis()))));
            endAttr(alenIdx4);
            acount = acount + 1 + 1;
        }
        int acount2 = acount + writeFlagAttrs(c.flags()) + writeJavaAnnotations(c.getRawAttributes()) + writeTypeAnnotations(c.getRawTypeAttributes(), false) + writeEnclosingMethodAttribute(c) + writeExtraClassAttributes(c);
        this.poolbuf.appendInt(ClassFile.JAVA_MAGIC);
        this.poolbuf.appendChar(this.target.minorVersion);
        this.poolbuf.appendChar(this.target.majorVersion);
        writePool(c.pool);
        if (this.innerClasses != null) {
            writeInnerClasses();
            acount2++;
        }
        if (!this.bootstrapMethods.isEmpty()) {
            writeBootstrapMethods();
            acount2++;
        }
        endAttrs(acountIdx, acount2);
        this.poolbuf.appendBytes(this.databuf.elems, 0, this.databuf.length);
        out.write(this.poolbuf.elems, 0, this.poolbuf.length);
        c.pool = null;
        this.pool = null;
    }

    protected int writeExtraClassAttributes(Symbol.ClassSymbol c) {
        return 0;
    }

    int adjustFlags(long flags) {
        int result = (int) flags;
        if ((4096 & flags) != 0 && !this.target.useSyntheticFlag()) {
            result &= -4097;
        }
        if ((16384 & flags) != 0 && !this.target.useEnumFlag()) {
            result &= -16385;
        }
        if ((8192 & flags) != 0 && !this.target.useAnnotationFlag()) {
            result &= -8193;
        }
        if ((Flags.BRIDGE & flags) != 0 && this.target.useBridgeFlag()) {
            result |= 64;
        }
        if ((Flags.VARARGS & flags) != 0 && this.target.useVarargsFlag()) {
            result |= 128;
        }
        if ((Flags.DEFAULT & flags) != 0) {
            return result & (-1025);
        }
        return result;
    }

    long getLastModified(FileObject filename) {
        try {
            long mod = filename.getLastModified();
            return mod;
        } catch (SecurityException e) {
            throw new AssertionError("CRT: couldn't get source file modification date: " + e.getMessage());
        }
    }
}
