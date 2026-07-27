package com.sun.tools.javac.jvm;

import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.BoundKind;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.TargetType;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeAnnotationPosition;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.comp.Annotate;
import com.sun.tools.javac.file.BaseFileObject;
import com.sun.tools.javac.jvm.ClassFile;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Convert;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.Pair;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.Reader;
import java.io.Writer;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.CharBuffer;
import java.util.Arrays;
import java.util.Collection;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import javax.lang.model.SourceVersion;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.StandardLocation;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class ClassReader {
    public static final int INITIAL_BUFFER_SIZE = 65520;
    protected static final Context.Key<ClassReader> classReaderKey = new Context.Key<>();
    boolean allowAnnotations;
    boolean allowGenerics;
    boolean allowSimplifiedVarargs;
    boolean allowVarargs;
    Annotate annotate;
    protected int bp;
    private boolean cacheCompletionFailure;
    boolean checkClassFile;
    private Map<Name, Symbol.ClassSymbol> classes;
    final Name completionFailureName;
    protected JavaFileManager.Location currentLoc;
    JCDiagnostic.Factory diagFactory;
    private final JavaFileManager fileManager;
    boolean haveParameterNameIndices;
    boolean lintClassfile;
    final Log log;
    int majorVersion;
    int minorVersion;
    final Names names;
    private Map<Name, Symbol.PackageSymbol> packages;
    int[] parameterNameIndices;
    int[] poolIdx;
    Object[] poolObj;
    public boolean preferSource;
    public final Profile profile;
    public boolean saveParameterNames;
    boolean sawMethodParameters;
    int siglimit;
    byte[] signature;
    int sigp;
    Symtab syms;
    Types types;
    protected Scope typevars;
    boolean verbose;
    private boolean verbosePath;
    public boolean readAllOfClassFile = false;
    public SourceCompleter sourceCompleter = null;
    protected JavaFileObject currentClassFile = null;
    protected Symbol currentOwner = null;
    byte[] buf = new byte[65520];
    Set<Name> warnedAttrs = new HashSet();
    private final Symbol.Completer thisCompleter = new Symbol.Completer() { // from class: com.sun.tools.javac.jvm.ClassReader.1
        @Override // com.sun.tools.javac.code.Symbol.Completer
        public void complete(Symbol sym) throws Symbol.CompletionFailure {
            ClassReader.this.complete(sym);
        }
    };
    boolean sigEnterPhase = false;
    byte[] signatureBuffer = new byte[0];
    int sbp = 0;
    protected Set<AttributeKind> CLASS_ATTRIBUTE = EnumSet.of(AttributeKind.CLASS);
    protected Set<AttributeKind> MEMBER_ATTRIBUTE = EnumSet.of(AttributeKind.MEMBER);
    protected Set<AttributeKind> CLASS_OR_MEMBER_ATTRIBUTE = EnumSet.of(AttributeKind.CLASS, AttributeKind.MEMBER);
    protected Map<Name, AttributeReader> attributeReaders = new HashMap();
    private boolean readingClassAttr = false;
    private List<Type> missingTypeVariables = List.nil();
    private List<Type> foundTypeVariables = List.nil();
    private boolean filling = false;
    private Symbol.CompletionFailure cachedCompletionFailure = new Symbol.CompletionFailure((Symbol) null, (JCDiagnostic) null);

    protected enum AttributeKind {
        CLASS,
        MEMBER
    }

    interface ProxyVisitor extends Attribute.Visitor {
        void visitArrayAttributeProxy(ArrayAttributeProxy arrayAttributeProxy);

        void visitCompoundAnnotationProxy(CompoundAnnotationProxy compoundAnnotationProxy);

        void visitEnumAttributeProxy(EnumAttributeProxy enumAttributeProxy);
    }

    public interface SourceCompleter {
        void complete(Symbol.ClassSymbol classSymbol) throws Symbol.CompletionFailure;
    }

    public static ClassReader instance(Context context) {
        ClassReader instance = (ClassReader) context.get(classReaderKey);
        if (instance == null) {
            return new ClassReader(context, true);
        }
        return instance;
    }

    public void init(Symtab syms) {
        init(syms, true);
    }

    private void init(Symtab syms, boolean definitive) {
        if (this.classes != null) {
            return;
        }
        if (definitive) {
            Assert.check(this.packages == null || this.packages == syms.packages);
            this.packages = syms.packages;
            Assert.check(this.classes == null || this.classes == syms.classes);
            this.classes = syms.classes;
        } else {
            this.packages = new HashMap();
            this.classes = new HashMap();
        }
        this.packages.put(this.names.empty, syms.rootPackage);
        syms.rootPackage.completer = this.thisCompleter;
        syms.unnamedPackage.completer = this.thisCompleter;
    }

    protected ClassReader(Context context, boolean definitive) {
        this.cachedCompletionFailure.setStackTrace(new StackTraceElement[0]);
        this.verbosePath = true;
        if (definitive) {
            context.put(classReaderKey, this);
        }
        this.names = Names.instance(context);
        this.syms = Symtab.instance(context);
        this.types = Types.instance(context);
        this.fileManager = (JavaFileManager) context.get(JavaFileManager.class);
        if (this.fileManager == null) {
            throw new AssertionError("FileManager initialization error");
        }
        this.diagFactory = JCDiagnostic.Factory.instance(context);
        init(this.syms, definitive);
        this.log = Log.instance(context);
        Options options = Options.instance(context);
        this.annotate = Annotate.instance(context);
        this.verbose = options.isSet(Option.VERBOSE);
        this.checkClassFile = options.isSet("-checkclassfile");
        Source source = Source.instance(context);
        this.allowGenerics = source.allowGenerics();
        this.allowVarargs = source.allowVarargs();
        this.allowAnnotations = source.allowAnnotations();
        this.allowSimplifiedVarargs = source.allowSimplifiedVarargs();
        this.saveParameterNames = options.isSet("save-parameter-names");
        this.cacheCompletionFailure = options.isUnset("dev");
        this.preferSource = "source".equals(options.get("-Xprefer"));
        this.profile = Profile.instance(context);
        this.completionFailureName = options.isSet("failcomplete") ? this.names.fromString(options.get("failcomplete")) : null;
        this.typevars = new Scope(this.syms.noSymbol);
        this.lintClassfile = Lint.instance(context).isEnabled(Lint.LintCategory.CLASSFILE);
        initAttributeReaders();
    }

    private void enterMember(Symbol.ClassSymbol c, Symbol sym) {
        if ((sym.flags_field & 2147487744L) != 4096 || sym.name.startsWith(this.names.lambda)) {
            c.members_field.enter(sym);
        }
    }

    public class BadClassFile extends Symbol.CompletionFailure {
        private static final long serialVersionUID = 0;

        public BadClassFile(Symbol.TypeSymbol sym, JavaFileObject file, JCDiagnostic diag) {
            super(sym, ClassReader.this.createBadClassFileDiagnostic(file, diag));
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public JCDiagnostic createBadClassFileDiagnostic(JavaFileObject file, JCDiagnostic diag) {
        String key = file.getKind() == JavaFileObject.Kind.SOURCE ? "bad.source.file.header" : "bad.class.file.header";
        return this.diagFactory.fragment(key, file, diag);
    }

    public BadClassFile badClassFile(String key, Object... args) {
        return new BadClassFile(this.currentOwner.enclClass(), this.currentClassFile, this.diagFactory.fragment(key, args));
    }

    char nextChar() {
        byte[] bArr = this.buf;
        int i = this.bp;
        this.bp = i + 1;
        int i2 = (bArr[i] & 255) << 8;
        byte[] bArr2 = this.buf;
        int i3 = this.bp;
        this.bp = i3 + 1;
        return (char) (i2 + (bArr2[i3] & 255));
    }

    int nextByte() {
        byte[] bArr = this.buf;
        int i = this.bp;
        this.bp = i + 1;
        return bArr[i] & 255;
    }

    int nextInt() {
        byte[] bArr = this.buf;
        int i = this.bp;
        this.bp = i + 1;
        int i2 = (bArr[i] & 255) << 24;
        byte[] bArr2 = this.buf;
        int i3 = this.bp;
        this.bp = i3 + 1;
        int i4 = i2 + ((bArr2[i3] & 255) << 16);
        byte[] bArr3 = this.buf;
        int i5 = this.bp;
        this.bp = i5 + 1;
        int i6 = i4 + ((bArr3[i5] & 255) << 8);
        byte[] bArr4 = this.buf;
        int i7 = this.bp;
        this.bp = i7 + 1;
        return i6 + (bArr4[i7] & 255);
    }

    char getChar(int bp) {
        return (char) (((this.buf[bp] & 255) << 8) + (this.buf[bp + 1] & 255));
    }

    int getInt(int bp) {
        return ((this.buf[bp] & 255) << 24) + ((this.buf[bp + 1] & 255) << 16) + ((this.buf[bp + 2] & 255) << 8) + (this.buf[bp + 3] & 255);
    }

    long getLong(int bp) {
        DataInputStream bufin = new DataInputStream(new ByteArrayInputStream(this.buf, bp, 8));
        try {
            return bufin.readLong();
        } catch (IOException e) {
            throw new AssertionError(e);
        }
    }

    float getFloat(int bp) {
        DataInputStream bufin = new DataInputStream(new ByteArrayInputStream(this.buf, bp, 4));
        try {
            return bufin.readFloat();
        } catch (IOException e) {
            throw new AssertionError(e);
        }
    }

    double getDouble(int bp) {
        DataInputStream bufin = new DataInputStream(new ByteArrayInputStream(this.buf, bp, 8));
        try {
            return bufin.readDouble();
        } catch (IOException e) {
            throw new AssertionError(e);
        }
    }

    void indexPool() {
        this.poolIdx = new int[nextChar()];
        this.poolObj = new Object[this.poolIdx.length];
        int i = 1;
        while (i < this.poolIdx.length) {
            int i2 = i + 1;
            this.poolIdx[i] = this.bp;
            byte[] bArr = this.buf;
            int i3 = this.bp;
            this.bp = i3 + 1;
            byte tag = bArr[i3];
            switch (tag) {
                case 1:
                case 2:
                    int len = nextChar();
                    this.bp += len;
                    break;
                case 3:
                case 4:
                case 9:
                case 10:
                case 11:
                case 12:
                case 18:
                    this.bp += 4;
                    break;
                case 5:
                case 6:
                    this.bp += 8;
                    i = i2 + 1;
                    continue;
                case 7:
                case 8:
                case 16:
                    this.bp += 2;
                    break;
                case 13:
                case 14:
                case 17:
                default:
                    throw badClassFile("bad.const.pool.tag.at", Byte.toString(tag), Integer.toString(this.bp - 1));
                case 15:
                    this.bp += 3;
                    break;
            }
            i = i2;
        }
    }

    Object readPool(int i) {
        Object result = this.poolObj[i];
        if (result != null) {
            return result;
        }
        int index = this.poolIdx[i];
        if (index == 0) {
            return null;
        }
        byte tag = this.buf[index];
        switch (tag) {
            case 1:
                this.poolObj[i] = this.names.fromUtf(this.buf, index + 3, getChar(index + 1));
                break;
            case 2:
                throw badClassFile("unicode.str.not.supported", new Object[0]);
            case 3:
                this.poolObj[i] = Integer.valueOf(getInt(index + 1));
                break;
            case 4:
                this.poolObj[i] = new Float(getFloat(index + 1));
                break;
            case 5:
                this.poolObj[i] = new Long(getLong(index + 1));
                break;
            case 6:
                this.poolObj[i] = new Double(getDouble(index + 1));
                break;
            case 7:
                this.poolObj[i] = readClassOrType(getChar(index + 1));
                break;
            case 8:
                this.poolObj[i] = readName(getChar(index + 1)).toString();
                break;
            case 9:
                Symbol.ClassSymbol owner = readClassSymbol(getChar(index + 1));
                ClassFile.NameAndType nt = readNameAndType(getChar(index + 3));
                this.poolObj[i] = new Symbol.VarSymbol(0L, nt.name, nt.uniqueType.type, owner);
                break;
            case 10:
            case 11:
                Symbol.ClassSymbol owner2 = readClassSymbol(getChar(index + 1));
                ClassFile.NameAndType nt2 = readNameAndType(getChar(index + 3));
                this.poolObj[i] = new Symbol.MethodSymbol(0L, nt2.name, nt2.uniqueType.type, owner2);
                break;
            case 12:
                this.poolObj[i] = new ClassFile.NameAndType(readName(getChar(index + 1)), readType(getChar(index + 3)), this.types);
                break;
            case 13:
            case 14:
            case 17:
            default:
                throw badClassFile("bad.const.pool.tag", Byte.toString(tag));
            case 15:
                skipBytes(4);
                break;
            case 16:
                skipBytes(3);
                break;
            case 18:
                skipBytes(5);
                break;
        }
        return this.poolObj[i];
    }

    Type readType(int i) {
        int index = this.poolIdx[i];
        return sigToType(this.buf, index + 3, getChar(index + 1));
    }

    Object readClassOrType(int i) {
        int index = this.poolIdx[i];
        int len = getChar(index + 1);
        int start = index + 3;
        Assert.check(this.buf[start] == 91 || this.buf[(start + len) - 1] != 59);
        if (this.buf[start] == 91 || this.buf[(start + len) - 1] == 59) {
            return sigToType(this.buf, start, len);
        }
        return enterClass(this.names.fromUtf(ClassFile.internalize(this.buf, start, len)));
    }

    List<Type> readTypeParams(int i) {
        int index = this.poolIdx[i];
        return sigToTypeParams(this.buf, index + 3, getChar(index + 1));
    }

    Symbol.ClassSymbol readClassSymbol(int i) {
        Object obj = readPool(i);
        if (obj != null && !(obj instanceof Symbol.ClassSymbol)) {
            throw badClassFile("bad.const.pool.entry", this.currentClassFile.toString(), "CONSTANT_Class_info", Integer.valueOf(i));
        }
        return (Symbol.ClassSymbol) obj;
    }

    Name readName(int i) {
        Object obj = readPool(i);
        if (obj != null && !(obj instanceof Name)) {
            throw badClassFile("bad.const.pool.entry", this.currentClassFile.toString(), "CONSTANT_Utf8_info or CONSTANT_String_info", Integer.valueOf(i));
        }
        return (Name) obj;
    }

    ClassFile.NameAndType readNameAndType(int i) {
        Object obj = readPool(i);
        if (obj != null && !(obj instanceof ClassFile.NameAndType)) {
            throw badClassFile("bad.const.pool.entry", this.currentClassFile.toString(), "CONSTANT_NameAndType_info", Integer.valueOf(i));
        }
        return (ClassFile.NameAndType) obj;
    }

    Type sigToType(byte[] sig, int offset, int len) {
        this.signature = sig;
        this.sigp = offset;
        this.siglimit = offset + len;
        return sigToType();
    }

    /* JADX WARN: Multi-variable type inference failed */
    Type sigToType() {
        switch ((char) this.signature[this.sigp]) {
            case '(':
                this.sigp++;
                List<Type> argtypes = sigToTypes(')');
                Type restype = sigToType();
                List<Type> thrown = List.nil();
                while (this.signature[this.sigp] == 94) {
                    this.sigp++;
                    thrown = thrown.prepend(sigToType());
                }
                for (List list = thrown; list.nonEmpty(); list = list.tail) {
                    if (((Type) list.head).hasTag(TypeTag.TYPEVAR)) {
                        ((Type) list.head).tsym.flags_field |= Flags.THROWS;
                    }
                }
                return new Type.MethodType(argtypes, restype, thrown.reverse(), this.syms.methodClass);
            case '*':
                this.sigp++;
                return new Type.WildcardType(this.syms.objectType, BoundKind.UNBOUND, this.syms.boundClass);
            case '+':
                this.sigp++;
                Type t = sigToType();
                return new Type.WildcardType(t, BoundKind.EXTENDS, this.syms.boundClass);
            case '-':
                this.sigp++;
                Type t2 = sigToType();
                return new Type.WildcardType(t2, BoundKind.SUPER, this.syms.boundClass);
            case '<':
                this.typevars = this.typevars.dup(this.currentOwner);
                Type poly = new Type.ForAll(sigToTypeParams(), sigToType());
                this.typevars = this.typevars.leave();
                return poly;
            case 'B':
                this.sigp++;
                return this.syms.byteType;
            case 'C':
                this.sigp++;
                return this.syms.charType;
            case 'D':
                this.sigp++;
                return this.syms.doubleType;
            case 'F':
                this.sigp++;
                return this.syms.floatType;
            case 'I':
                this.sigp++;
                return this.syms.intType;
            case 'J':
                this.sigp++;
                return this.syms.longType;
            case 'L':
                Type t3 = classSigToType();
                if (this.sigp < this.siglimit && this.signature[this.sigp] == 46) {
                    throw badClassFile("deprecated inner class signature syntax (please recompile from source)", new Object[0]);
                }
                return t3;
            case 'S':
                this.sigp++;
                return this.syms.shortType;
            case 'T':
                this.sigp++;
                int start = this.sigp;
                while (this.signature[this.sigp] != 59) {
                    this.sigp++;
                }
                this.sigp++;
                return this.sigEnterPhase ? Type.noType : findTypeVar(this.names.fromUtf(this.signature, start, (this.sigp - 1) - start));
            case 'V':
                this.sigp++;
                return this.syms.voidType;
            case 'Z':
                this.sigp++;
                return this.syms.booleanType;
            case '[':
                this.sigp++;
                return new Type.ArrayType(sigToType(), this.syms.arrayClass);
            default:
                throw badClassFile("bad.signature", Convert.utf2string(this.signature, this.sigp, 10));
        }
    }

    Type classSigToType() {
        Type classType;
        if (this.signature[this.sigp] != 76) {
            throw badClassFile("bad.class.signature", Convert.utf2string(this.signature, this.sigp, 10));
        }
        this.sigp++;
        Type outer = Type.noType;
        int startSbp = this.sbp;
        while (true) {
            byte[] bArr = this.signature;
            int i = this.sigp;
            this.sigp = i + 1;
            byte c = bArr[i];
            switch (c) {
                case 46:
                    if (outer != Type.noType) {
                        outer = new Type.ClassType(outer, List.nil(), enterClass(this.names.fromUtf(this.signatureBuffer, startSbp, this.sbp - startSbp)));
                    }
                    byte[] bArr2 = this.signatureBuffer;
                    int i2 = this.sbp;
                    this.sbp = i2 + 1;
                    bArr2[i2] = 36;
                    break;
                case 47:
                    byte[] bArr3 = this.signatureBuffer;
                    int i3 = this.sbp;
                    this.sbp = i3 + 1;
                    bArr3[i3] = 46;
                    break;
                case 59:
                    Symbol.ClassSymbol t = enterClass(this.names.fromUtf(this.signatureBuffer, startSbp, this.sbp - startSbp));
                    try {
                        if (outer == Type.noType) {
                            classType = t.erasure(this.types);
                        } else {
                            classType = new Type.ClassType(outer, List.nil(), t);
                        }
                        return classType;
                    } finally {
                        this.sbp = startSbp;
                    }
                case 60:
                    outer = new Type.ClassType(outer, sigToTypes('>'), enterClass(this.names.fromUtf(this.signatureBuffer, startSbp, this.sbp - startSbp))) { // from class: com.sun.tools.javac.jvm.ClassReader.2
                        boolean completed = false;

                        @Override // com.sun.tools.javac.code.Type.ClassType, javax.lang.model.type.DeclaredType
                        public Type getEnclosingType() {
                            if (!this.completed) {
                                this.completed = true;
                                this.tsym.complete();
                                Type enclosingType = this.tsym.type.getEnclosingType();
                                if (enclosingType != Type.noType) {
                                    List<Type> typeArgs = super.getEnclosingType().allparams();
                                    List<Type> typeParams = enclosingType.allparams();
                                    if (typeParams.length() != typeArgs.length()) {
                                        super.setEnclosingType(ClassReader.this.types.erasure(enclosingType));
                                    } else {
                                        super.setEnclosingType(ClassReader.this.types.subst(enclosingType, typeParams, typeArgs));
                                    }
                                } else {
                                    super.setEnclosingType(Type.noType);
                                }
                            }
                            return super.getEnclosingType();
                        }

                        @Override // com.sun.tools.javac.code.Type.ClassType
                        public void setEnclosingType(Type outer2) {
                            throw new UnsupportedOperationException();
                        }
                    };
                    byte[] bArr4 = this.signature;
                    int i4 = this.sigp;
                    this.sigp = i4 + 1;
                    switch (bArr4[i4]) {
                        case 46:
                            byte[] bArr5 = this.signatureBuffer;
                            int i5 = this.sbp;
                            this.sbp = i5 + 1;
                            bArr5[i5] = 36;
                            break;
                        case 59:
                            if (this.sigp < this.signature.length && this.signature[this.sigp] == 46) {
                                this.sigp += (this.sbp - startSbp) + 3;
                                byte[] bArr6 = this.signatureBuffer;
                                int i6 = this.sbp;
                                this.sbp = i6 + 1;
                                bArr6[i6] = 36;
                            }
                            break;
                        default:
                            throw new AssertionError((int) this.signature[this.sigp - 1]);
                    }
                    break;
                default:
                    byte[] bArr7 = this.signatureBuffer;
                    int i7 = this.sbp;
                    this.sbp = i7 + 1;
                    bArr7[i7] = c;
                    break;
            }
        }
        return outer;
    }

    List<Type> sigToTypes(char terminator) {
        List<Type> head = List.of((Object) null);
        List<Type> tail = head;
        while (this.signature[this.sigp] != terminator) {
            tail = tail.setTail(List.of(sigToType()));
        }
        this.sigp++;
        return head.tail;
    }

    List<Type> sigToTypeParams(byte[] sig, int offset, int len) {
        this.signature = sig;
        this.sigp = offset;
        this.siglimit = offset + len;
        return sigToTypeParams();
    }

    List<Type> sigToTypeParams() {
        List<Type> tvars = List.nil();
        if (this.signature[this.sigp] == 60) {
            this.sigp++;
            int start = this.sigp;
            this.sigEnterPhase = true;
            while (this.signature[this.sigp] != 62) {
                tvars = tvars.prepend(sigToTypeParam());
            }
            this.sigEnterPhase = false;
            this.sigp = start;
            while (this.signature[this.sigp] != 62) {
                sigToTypeParam();
            }
            this.sigp++;
        }
        return tvars.reverse();
    }

    Type sigToTypeParam() {
        Type.TypeVar tvar;
        int start = this.sigp;
        while (this.signature[this.sigp] != 58) {
            this.sigp++;
        }
        Name name = this.names.fromUtf(this.signature, start, this.sigp - start);
        if (this.sigEnterPhase) {
            tvar = new Type.TypeVar(name, this.currentOwner, this.syms.botType);
            this.typevars.enter(tvar.tsym);
        } else {
            tvar = (Type.TypeVar) findTypeVar(name);
        }
        List<Type> bounds = List.nil();
        boolean allInterfaces = false;
        if (this.signature[this.sigp] == 58 && this.signature[this.sigp + 1] == 58) {
            this.sigp++;
            allInterfaces = true;
        }
        while (this.signature[this.sigp] == 58) {
            this.sigp++;
            bounds = bounds.prepend(sigToType());
        }
        if (!this.sigEnterPhase) {
            this.types.setBounds(tvar, bounds.reverse(), allInterfaces);
        }
        return tvar;
    }

    Type findTypeVar(Name name) {
        Scope.Entry e = this.typevars.lookup(name);
        if (e.scope != null) {
            return e.sym.type;
        }
        if (this.readingClassAttr) {
            Type.TypeVar t = new Type.TypeVar(name, this.currentOwner, this.syms.botType);
            this.missingTypeVariables = this.missingTypeVariables.prepend(t);
            return t;
        }
        throw badClassFile("undecl.type.var", name);
    }

    protected abstract class AttributeReader {
        protected final Set<AttributeKind> kinds;
        protected final Name name;
        protected final ClassFile.Version version;

        protected abstract void read(Symbol symbol, int i);

        protected AttributeReader(Name name, ClassFile.Version version, Set<AttributeKind> kinds) {
            this.name = name;
            this.version = version;
            this.kinds = kinds;
        }

        protected boolean accepts(AttributeKind kind) {
            if (this.kinds.contains(kind)) {
                if (ClassReader.this.majorVersion > this.version.major) {
                    return true;
                }
                if (ClassReader.this.majorVersion == this.version.major && ClassReader.this.minorVersion >= this.version.minor) {
                    return true;
                }
                if (ClassReader.this.lintClassfile && !ClassReader.this.warnedAttrs.contains(this.name)) {
                    JavaFileObject prev = ClassReader.this.log.useSource(ClassReader.this.currentClassFile);
                    try {
                        ClassReader.this.log.warning(Lint.LintCategory.CLASSFILE, null, "future.attr", this.name, Integer.valueOf(this.version.major), Integer.valueOf(this.version.minor), Integer.valueOf(ClassReader.this.majorVersion), Integer.valueOf(ClassReader.this.minorVersion));
                        ClassReader.this.log.useSource(prev);
                        ClassReader.this.warnedAttrs.add(this.name);
                        return false;
                    } catch (Throwable th) {
                        ClassReader.this.log.useSource(prev);
                        throw th;
                    }
                }
                return false;
            }
            return false;
        }
    }

    private void initAttributeReaders() {
        AttributeReader[] readers = {new AttributeReader(this.names.Code, ClassFile.Version.V45_3, this.MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.3
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                if (ClassReader.this.readAllOfClassFile || ClassReader.this.saveParameterNames) {
                    ((Symbol.MethodSymbol) sym).code = ClassReader.this.readCode(sym);
                } else {
                    ClassReader.this.bp += attrLen;
                }
            }
        }, new AttributeReader(this.names.ConstantValue, ClassFile.Version.V45_3, this.MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.4
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                Object v = ClassReader.this.readPool(ClassReader.this.nextChar());
                if ((sym.flags() & 16) != 0) {
                    ((Symbol.VarSymbol) sym).setData(v);
                }
            }
        }, new AttributeReader(this.names.Deprecated, ClassFile.Version.V45_3, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.5
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                sym.flags_field |= 131072;
            }
        }, new AttributeReader(this.names.Exceptions, ClassFile.Version.V45_3, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.6
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                int nexceptions = ClassReader.this.nextChar();
                List<Type> thrown = List.nil();
                for (int j = 0; j < nexceptions; j++) {
                    thrown = thrown.prepend(ClassReader.this.readClassSymbol(ClassReader.this.nextChar()).type);
                }
                if (sym.type.mo179getThrownTypes().isEmpty()) {
                    sym.type.asMethodType().thrown = thrown.reverse();
                }
            }
        }, new AttributeReader(this.names.InnerClasses, ClassFile.Version.V45_3, this.CLASS_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.7
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                Symbol.ClassSymbol c = (Symbol.ClassSymbol) sym;
                ClassReader.this.readInnerClasses(c);
            }
        }, new AttributeReader(this.names.LocalVariableTable, ClassFile.Version.V45_3, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.8
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                int newbp = ClassReader.this.bp + attrLen;
                if (ClassReader.this.saveParameterNames && !ClassReader.this.sawMethodParameters) {
                    int numEntries = ClassReader.this.nextChar();
                    for (int i = 0; i < numEntries; i++) {
                        int start_pc = ClassReader.this.nextChar();
                        ClassReader.this.nextChar();
                        int nameIndex = ClassReader.this.nextChar();
                        ClassReader.this.nextChar();
                        int register = ClassReader.this.nextChar();
                        if (start_pc == 0) {
                            if (register >= ClassReader.this.parameterNameIndices.length) {
                                int newSize = Math.max(register, ClassReader.this.parameterNameIndices.length + 8);
                                ClassReader.this.parameterNameIndices = Arrays.copyOf(ClassReader.this.parameterNameIndices, newSize);
                            }
                            ClassReader.this.parameterNameIndices[register] = nameIndex;
                            ClassReader.this.haveParameterNameIndices = true;
                        }
                    }
                }
                ClassReader.this.bp = newbp;
            }
        }, new AttributeReader(this.names.MethodParameters, ClassFile.Version.V52, this.MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.9
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrlen) {
                int newbp = ClassReader.this.bp + attrlen;
                if (ClassReader.this.saveParameterNames) {
                    ClassReader.this.sawMethodParameters = true;
                    int numEntries = ClassReader.this.nextByte();
                    ClassReader.this.parameterNameIndices = new int[numEntries];
                    ClassReader.this.haveParameterNameIndices = true;
                    for (int i = 0; i < numEntries; i++) {
                        int nameIndex = ClassReader.this.nextChar();
                        ClassReader.this.nextChar();
                        ClassReader.this.parameterNameIndices[i] = nameIndex;
                    }
                }
                ClassReader.this.bp = newbp;
            }
        }, new AttributeReader(this.names.SourceFile, ClassFile.Version.V45_3, this.CLASS_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.10
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                Symbol.ClassSymbol c = (Symbol.ClassSymbol) sym;
                Name n = ClassReader.this.readName(ClassReader.this.nextChar());
                c.sourcefile = new SourceFileObject(n, c.flatname);
                String sn = n.toString();
                if (c.owner.kind == 1 && sn.endsWith(OnBotJavaFileSystemUtils.EXT_JAVA_FILE) && !sn.equals(c.name.toString() + OnBotJavaFileSystemUtils.EXT_JAVA_FILE)) {
                    c.flags_field |= Flags.AUXILIARY;
                }
            }
        }, new AttributeReader(this.names.Synthetic, ClassFile.Version.V45_3, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.11
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                if (ClassReader.this.allowGenerics || (sym.flags_field & Flags.BRIDGE) == 0) {
                    sym.flags_field |= 4096;
                }
            }
        }, new AttributeReader(this.names.EnclosingMethod, ClassFile.Version.V49, this.CLASS_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.12
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                int newbp = ClassReader.this.bp + attrLen;
                ClassReader.this.readEnclosingMethodAttr(sym);
                ClassReader.this.bp = newbp;
            }
        }, new AttributeReader(this.names.Signature, ClassFile.Version.V49, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.13
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected boolean accepts(AttributeKind kind) {
                return super.accepts(kind) && ClassReader.this.allowGenerics;
            }

            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                if (sym.kind == 2) {
                    Symbol.ClassSymbol c = (Symbol.ClassSymbol) sym;
                    boolean z = true;
                    ClassReader.this.readingClassAttr = true;
                    try {
                        Type.ClassType ct1 = (Type.ClassType) c.type;
                        if (c != ClassReader.this.currentOwner) {
                            z = false;
                        }
                        Assert.check(z);
                        ct1.typarams_field = ClassReader.this.readTypeParams(ClassReader.this.nextChar());
                        ct1.supertype_field = ClassReader.this.sigToType();
                        ListBuffer<Type> is = new ListBuffer<>();
                        while (ClassReader.this.sigp != ClassReader.this.siglimit) {
                            is.append(ClassReader.this.sigToType());
                        }
                        ct1.interfaces_field = is.toList();
                        return;
                    } finally {
                        ClassReader.this.readingClassAttr = false;
                    }
                }
                List<Type> thrown = sym.type.mo179getThrownTypes();
                sym.type = ClassReader.this.readType(ClassReader.this.nextChar());
                if (sym.kind == 16 && sym.type.mo179getThrownTypes().isEmpty()) {
                    sym.type.asMethodType().thrown = thrown;
                }
            }
        }, new AttributeReader(this.names.AnnotationDefault, ClassFile.Version.V49, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.14
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                ClassReader.this.attachAnnotationDefault(sym);
            }
        }, new AttributeReader(this.names.RuntimeInvisibleAnnotations, ClassFile.Version.V49, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.15
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                ClassReader.this.attachAnnotations(sym);
            }
        }, new AttributeReader(this.names.RuntimeInvisibleParameterAnnotations, ClassFile.Version.V49, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.16
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                ClassReader.this.attachParameterAnnotations(sym);
            }
        }, new AttributeReader(this.names.RuntimeVisibleAnnotations, ClassFile.Version.V49, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.17
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                ClassReader.this.attachAnnotations(sym);
            }
        }, new AttributeReader(this.names.RuntimeVisibleParameterAnnotations, ClassFile.Version.V49, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.18
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                ClassReader.this.attachParameterAnnotations(sym);
            }
        }, new AttributeReader(this.names.Annotation, ClassFile.Version.V49, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.19
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                if (ClassReader.this.allowAnnotations) {
                    sym.flags_field |= 8192;
                }
            }
        }, new AttributeReader(this.names.Bridge, ClassFile.Version.V49, this.MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.20
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                sym.flags_field |= Flags.BRIDGE;
                if (!ClassReader.this.allowGenerics) {
                    sym.flags_field &= -4097;
                }
            }
        }, new AttributeReader(this.names.Enum, ClassFile.Version.V49, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.21
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                sym.flags_field |= 16384;
            }
        }, new AttributeReader(this.names.Varargs, ClassFile.Version.V49, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.22
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                if (ClassReader.this.allowVarargs) {
                    sym.flags_field |= Flags.VARARGS;
                }
            }
        }, new AttributeReader(this.names.RuntimeVisibleTypeAnnotations, ClassFile.Version.V52, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.23
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                ClassReader.this.attachTypeAnnotations(sym);
            }
        }, new AttributeReader(this.names.RuntimeInvisibleTypeAnnotations, ClassFile.Version.V52, this.CLASS_OR_MEMBER_ATTRIBUTE) { // from class: com.sun.tools.javac.jvm.ClassReader.24
            @Override // com.sun.tools.javac.jvm.ClassReader.AttributeReader
            protected void read(Symbol sym, int attrLen) {
                ClassReader.this.attachTypeAnnotations(sym);
            }
        }};
        for (AttributeReader r : readers) {
            this.attributeReaders.put(r.name, r);
        }
    }

    void unrecognized(Name attrName) {
        if (this.checkClassFile) {
            printCCF("ccf.unrecognized.attribute", attrName);
        }
    }

    protected void readEnclosingMethodAttr(Symbol sym) {
        sym.owner.members().remove(sym);
        Symbol.ClassSymbol self = (Symbol.ClassSymbol) sym;
        Symbol.ClassSymbol c = readClassSymbol(nextChar());
        ClassFile.NameAndType nt = readNameAndType(nextChar());
        if (c.members_field == null) {
            throw badClassFile("bad.enclosing.class", self, c);
        }
        Symbol.MethodSymbol m = findMethod(nt, c.members_field, self.flags());
        if (nt != null && m == null) {
            throw badClassFile("bad.enclosing.method", self);
        }
        self.name = simpleBinaryName(self.flatname, c.flatname);
        self.owner = m != null ? m : c;
        if (self.name.isEmpty()) {
            self.fullname = this.names.empty;
        } else {
            self.fullname = Symbol.ClassSymbol.formFullName(self.name, self.owner);
        }
        if (m != null) {
            ((Type.ClassType) sym.type).setEnclosingType(m.type);
        } else if ((self.flags_field & 8) == 0) {
            ((Type.ClassType) sym.type).setEnclosingType(c.type);
        } else {
            ((Type.ClassType) sym.type).setEnclosingType(Type.noType);
        }
        enterTypevars(self);
        if (!this.missingTypeVariables.isEmpty()) {
            ListBuffer<Type> typeVars = new ListBuffer<>();
            for (Type typevar : this.missingTypeVariables) {
                typeVars.append(findTypeVar(typevar.tsym.name));
            }
            this.foundTypeVariables = typeVars.toList();
            return;
        }
        this.foundTypeVariables = List.nil();
    }

    private Name simpleBinaryName(Name self, Name enclosing) {
        String simpleBinaryName = self.toString().substring(enclosing.toString().length());
        if (simpleBinaryName.length() < 1 || simpleBinaryName.charAt(0) != '$') {
            throw badClassFile("bad.enclosing.method", self);
        }
        int index = 1;
        while (index < simpleBinaryName.length() && isAsciiDigit(simpleBinaryName.charAt(index))) {
            index++;
        }
        return this.names.fromString(simpleBinaryName.substring(index));
    }

    private Symbol.MethodSymbol findMethod(ClassFile.NameAndType nt, Scope scope, long flags) {
        if (nt == null) {
            return null;
        }
        Type.MethodType type = nt.uniqueType.type.asMethodType();
        for (Scope.Entry e = scope.lookup(nt.name); e.scope != null; e = e.next()) {
            if (e.sym.kind == 16 && isSameBinaryType(e.sym.type.asMethodType(), type)) {
                return (Symbol.MethodSymbol) e.sym;
            }
        }
        if (nt.name != this.names.init || (512 & flags) != 0 || nt.uniqueType.type.mo176getParameterTypes().isEmpty()) {
            return null;
        }
        nt.setType(new Type.MethodType(nt.uniqueType.type.mo176getParameterTypes().tail, nt.uniqueType.type.mo178getReturnType(), nt.uniqueType.type.mo179getThrownTypes(), this.syms.methodClass));
        return findMethod(nt, scope, flags);
    }

    /* JADX WARN: Multi-variable type inference failed */
    private boolean isSameBinaryType(Type.MethodType mt1, Type.MethodType mt2) {
        List listPrepend = this.types.erasure(mt1.mo176getParameterTypes()).prepend(this.types.erasure(mt1.mo178getReturnType()));
        List listPrepend2 = mt2.mo176getParameterTypes().prepend(mt2.mo178getReturnType());
        while (!listPrepend.isEmpty() && !listPrepend2.isEmpty()) {
            if (((Type) listPrepend.head).tsym != ((Type) listPrepend2.head).tsym) {
                return false;
            }
            listPrepend = listPrepend.tail;
            listPrepend2 = listPrepend2.tail;
        }
        return listPrepend.isEmpty() && listPrepend2.isEmpty();
    }

    private static boolean isAsciiDigit(char c) {
        return '0' <= c && c <= '9';
    }

    void readMemberAttrs(Symbol sym) {
        readAttrs(sym, AttributeKind.MEMBER);
    }

    void readAttrs(Symbol sym, AttributeKind kind) {
        char ac = nextChar();
        for (int i = 0; i < ac; i++) {
            Name attrName = readName(nextChar());
            int attrLen = nextInt();
            AttributeReader r = this.attributeReaders.get(attrName);
            if (r != null && r.accepts(kind)) {
                r.read(sym, attrLen);
            } else {
                unrecognized(attrName);
                this.bp += attrLen;
            }
        }
    }

    void readClassAttrs(Symbol.ClassSymbol c) {
        readAttrs(c, AttributeKind.CLASS);
    }

    Code readCode(Symbol owner) {
        nextChar();
        nextChar();
        int code_length = nextInt();
        this.bp += code_length;
        char exception_table_length = nextChar();
        this.bp += exception_table_length * '\b';
        readMemberAttrs(owner);
        return null;
    }

    void attachAnnotations(Symbol sym) {
        int numAttributes = nextChar();
        if (numAttributes != 0) {
            ListBuffer<CompoundAnnotationProxy> proxies = new ListBuffer<>();
            for (int i = 0; i < numAttributes; i++) {
                CompoundAnnotationProxy proxy = readCompoundAnnotation();
                if (proxy.type.tsym == this.syms.proprietaryType.tsym) {
                    sym.flags_field |= Flags.PROPRIETARY;
                } else if (proxy.type.tsym == this.syms.profileType.tsym) {
                    if (this.profile != Profile.DEFAULT) {
                        for (Pair<Name, Attribute> v : proxy.values) {
                            if (v.fst == this.names.value && (v.snd instanceof Attribute.Constant)) {
                                Attribute.Constant c = (Attribute.Constant) v.snd;
                                if (c.type == this.syms.intType && ((Integer) c.value).intValue() > this.profile.value) {
                                    sym.flags_field |= 35184372088832L;
                                }
                            }
                        }
                    }
                } else {
                    proxies.append(proxy);
                }
            }
            this.annotate.normal(new AnnotationCompleter(sym, proxies.toList()));
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    void attachParameterAnnotations(Symbol method) {
        Symbol.MethodSymbol meth = (Symbol.MethodSymbol) method;
        byte[] bArr = this.buf;
        int i = this.bp;
        this.bp = i + 1;
        int numParameters = bArr[i] & 255;
        List listParams = meth.params();
        int pnum = 0;
        while (listParams.tail != null) {
            attachAnnotations((Symbol) listParams.head);
            listParams = listParams.tail;
            pnum++;
        }
        if (pnum != numParameters) {
            throw badClassFile("bad.runtime.invisible.param.annotations", meth);
        }
    }

    void attachTypeAnnotations(Symbol sym) {
        int numAttributes = nextChar();
        if (numAttributes != 0) {
            ListBuffer<TypeAnnotationProxy> proxies = new ListBuffer<>();
            for (int i = 0; i < numAttributes; i++) {
                proxies.append(readTypeAnnotation());
            }
            this.annotate.normal(new TypeAnnotationCompleter(sym, proxies.toList()));
        }
    }

    void attachAnnotationDefault(Symbol sym) {
        Symbol.MethodSymbol meth = (Symbol.MethodSymbol) sym;
        Attribute value = readAttributeValue();
        meth.defaultValue = value;
        this.annotate.normal(new AnnotationDefaultCompleter(meth, value));
    }

    Type readTypeOrClassSymbol(int i) {
        if (this.buf[this.poolIdx[i]] == 7) {
            return readClassSymbol(i).type;
        }
        return readType(i);
    }

    Type readEnumType(int i) {
        int index = this.poolIdx[i];
        int length = getChar(index + 1);
        if (this.buf[index + length + 2] != 59) {
            return enterClass(readName(i)).type;
        }
        return readType(i);
    }

    CompoundAnnotationProxy readCompoundAnnotation() {
        Type t = readTypeOrClassSymbol(nextChar());
        int numFields = nextChar();
        ListBuffer<Pair<Name, Attribute>> pairs = new ListBuffer<>();
        for (int i = 0; i < numFields; i++) {
            Name name = readName(nextChar());
            Attribute value = readAttributeValue();
            pairs.append(new Pair<>(name, value));
        }
        return new CompoundAnnotationProxy(t, pairs.toList());
    }

    TypeAnnotationProxy readTypeAnnotation() {
        TypeAnnotationPosition position = readPosition();
        CompoundAnnotationProxy proxy = readCompoundAnnotation();
        return new TypeAnnotationProxy(proxy, position);
    }

    TypeAnnotationPosition readPosition() {
        int tag = nextByte();
        if (!TargetType.isValidTargetTypeValue(tag)) {
            throw badClassFile("bad.type.annotation.value", String.format("0x%02X", Integer.valueOf(tag)));
        }
        TypeAnnotationPosition position = new TypeAnnotationPosition();
        TargetType type = TargetType.fromTargetTypeValue(tag);
        position.type = type;
        switch (type) {
            case INSTANCEOF:
            case NEW:
            case CONSTRUCTOR_REFERENCE:
            case METHOD_REFERENCE:
                position.offset = nextChar();
                break;
            case LOCAL_VARIABLE:
            case RESOURCE_VARIABLE:
                int table_length = nextChar();
                position.lvarOffset = new int[table_length];
                position.lvarLength = new int[table_length];
                position.lvarIndex = new int[table_length];
                for (int i = 0; i < table_length; i++) {
                    position.lvarOffset[i] = nextChar();
                    position.lvarLength[i] = nextChar();
                    position.lvarIndex[i] = nextChar();
                }
                break;
            case EXCEPTION_PARAMETER:
                position.exception_index = nextChar();
                break;
            case METHOD_RECEIVER:
            case METHOD_RETURN:
            case FIELD:
                break;
            case CLASS_TYPE_PARAMETER:
            case METHOD_TYPE_PARAMETER:
                position.parameter_index = nextByte();
                break;
            case CLASS_TYPE_PARAMETER_BOUND:
            case METHOD_TYPE_PARAMETER_BOUND:
                position.parameter_index = nextByte();
                position.bound_index = nextByte();
                break;
            case CLASS_EXTENDS:
                position.type_index = nextChar();
                break;
            case THROWS:
                position.type_index = nextChar();
                break;
            case METHOD_FORMAL_PARAMETER:
                position.parameter_index = nextByte();
                break;
            case CAST:
            case CONSTRUCTOR_INVOCATION_TYPE_ARGUMENT:
            case METHOD_INVOCATION_TYPE_ARGUMENT:
            case CONSTRUCTOR_REFERENCE_TYPE_ARGUMENT:
            case METHOD_REFERENCE_TYPE_ARGUMENT:
                position.offset = nextChar();
                position.type_index = nextByte();
                break;
            case UNKNOWN:
                throw new AssertionError("jvm.ClassReader: UNKNOWN target type should never occur!");
            default:
                throw new AssertionError("jvm.ClassReader: Unknown target type for position: " + position);
        }
        int len = nextByte();
        ListBuffer<Integer> loc = new ListBuffer<>();
        for (int i2 = 0; i2 < len * 2; i2++) {
            loc = loc.append(Integer.valueOf(nextByte()));
        }
        position.location = TypeAnnotationPosition.getTypePathFromBinary(loc.toList());
        return position;
    }

    Attribute readAttributeValue() {
        byte[] bArr = this.buf;
        int i = this.bp;
        this.bp = i + 1;
        char c = (char) bArr[i];
        switch (c) {
            case '@':
                return readCompoundAnnotation();
            case 'B':
                return new Attribute.Constant(this.syms.byteType, readPool(nextChar()));
            case 'C':
                return new Attribute.Constant(this.syms.charType, readPool(nextChar()));
            case 'D':
                return new Attribute.Constant(this.syms.doubleType, readPool(nextChar()));
            case 'F':
                return new Attribute.Constant(this.syms.floatType, readPool(nextChar()));
            case 'I':
                return new Attribute.Constant(this.syms.intType, readPool(nextChar()));
            case 'J':
                return new Attribute.Constant(this.syms.longType, readPool(nextChar()));
            case 'S':
                return new Attribute.Constant(this.syms.shortType, readPool(nextChar()));
            case 'Z':
                return new Attribute.Constant(this.syms.booleanType, readPool(nextChar()));
            case '[':
                int n = nextChar();
                ListBuffer<Attribute> l = new ListBuffer<>();
                for (int i2 = 0; i2 < n; i2++) {
                    l.append(readAttributeValue());
                }
                return new ArrayAttributeProxy(l.toList());
            case 'c':
                return new Attribute.Class(this.types, readTypeOrClassSymbol(nextChar()));
            case 'e':
                return new EnumAttributeProxy(readEnumType(nextChar()), readName(nextChar()));
            case 's':
                return new Attribute.Constant(this.syms.stringType, readPool(nextChar()).toString());
            default:
                throw new AssertionError("unknown annotation tag '" + c + "'");
        }
    }

    static class EnumAttributeProxy extends Attribute {
        Type enumType;
        Name enumerator;

        public EnumAttributeProxy(Type enumType, Name enumerator) {
            super(null);
            this.enumType = enumType;
            this.enumerator = enumerator;
        }

        @Override // com.sun.tools.javac.code.Attribute
        public void accept(Attribute.Visitor v) {
            ((ProxyVisitor) v).visitEnumAttributeProxy(this);
        }

        @Override // javax.lang.model.element.AnnotationValue
        public String toString() {
            return "/*proxy enum*/" + this.enumType + "." + ((Object) this.enumerator);
        }
    }

    static class ArrayAttributeProxy extends Attribute {
        List<Attribute> values;

        ArrayAttributeProxy(List<Attribute> values) {
            super(null);
            this.values = values;
        }

        @Override // com.sun.tools.javac.code.Attribute
        public void accept(Attribute.Visitor v) {
            ((ProxyVisitor) v).visitArrayAttributeProxy(this);
        }

        @Override // javax.lang.model.element.AnnotationValue
        public String toString() {
            return "{" + this.values + "}";
        }
    }

    static class CompoundAnnotationProxy extends Attribute {
        final List<Pair<Name, Attribute>> values;

        public CompoundAnnotationProxy(Type type, List<Pair<Name, Attribute>> values) {
            super(type);
            this.values = values;
        }

        @Override // com.sun.tools.javac.code.Attribute
        public void accept(Attribute.Visitor v) {
            ((ProxyVisitor) v).visitCompoundAnnotationProxy(this);
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // javax.lang.model.element.AnnotationValue
        public String toString() {
            StringBuilder buf = new StringBuilder();
            buf.append("@");
            buf.append((CharSequence) this.type.tsym.getQualifiedName());
            buf.append("/*proxy*/{");
            boolean first = true;
            for (List list = this.values; list.nonEmpty(); list = list.tail) {
                Pair<Name, Attribute> value = (Pair) list.head;
                if (!first) {
                    buf.append(DocLint.TAGS_SEPARATOR);
                }
                first = false;
                buf.append((CharSequence) value.fst);
                buf.append("=");
                buf.append(value.snd);
            }
            buf.append("}");
            return buf.toString();
        }
    }

    static class TypeAnnotationProxy {
        final CompoundAnnotationProxy compound;
        final TypeAnnotationPosition position;

        public TypeAnnotationProxy(CompoundAnnotationProxy compound, TypeAnnotationPosition position) {
            this.compound = compound;
            this.position = position;
        }
    }

    class AnnotationDeproxy implements ProxyVisitor {
        private Symbol.ClassSymbol requestingOwner;
        Attribute result;
        Type type;

        AnnotationDeproxy() {
            this.requestingOwner = ClassReader.this.currentOwner.kind == 16 ? ClassReader.this.currentOwner.enclClass() : (Symbol.ClassSymbol) ClassReader.this.currentOwner;
        }

        /* JADX WARN: Multi-variable type inference failed */
        List<Attribute.Compound> deproxyCompoundList(List<CompoundAnnotationProxy> pl) {
            ListBuffer<Attribute.Compound> buf = new ListBuffer<>();
            for (List list = pl; list.nonEmpty(); list = list.tail) {
                buf.append(deproxyCompound((CompoundAnnotationProxy) list.head));
            }
            return buf.toList();
        }

        /* JADX WARN: Multi-variable type inference failed */
        Attribute.Compound deproxyCompound(CompoundAnnotationProxy a) {
            ListBuffer<Pair<Symbol.MethodSymbol, Attribute>> buf = new ListBuffer<>();
            for (List list = a.values; list.nonEmpty(); list = list.tail) {
                Symbol.MethodSymbol meth = findAccessMethod(a.type, (Name) ((Pair) list.head).fst);
                buf.append(new Pair<>(meth, deproxy(meth.type.mo178getReturnType(), (Attribute) ((Pair) list.head).snd)));
            }
            return new Attribute.Compound(a.type, buf.toList());
        }

        /* JADX WARN: Removed duplicated region for block: B:20:0x0042  */
        /*
            Code decompiled incorrectly, please refer to instructions dump.
            To view partially-correct code enable 'Show inconsistent code' option in preferences
        */
        com.sun.tools.javac.code.Symbol.MethodSymbol findAccessMethod(com.sun.tools.javac.code.Type r10, com.sun.tools.javac.util.Name r11) {
            /*
                r9 = this;
                r0 = 0
                com.sun.tools.javac.code.Symbol$TypeSymbol r1 = r10.tsym     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                com.sun.tools.javac.code.Scope r1 = r1.members()     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                com.sun.tools.javac.code.Scope$Entry r1 = r1.lookup(r11)     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
            Lb:
                com.sun.tools.javac.code.Scope r2 = r1.scope     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                if (r2 == 0) goto L2d
                com.sun.tools.javac.code.Symbol r2 = r1.sym     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                int r3 = r2.kind     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                r4 = 16
                if (r3 != r4) goto L27
                com.sun.tools.javac.code.Type r3 = r2.type     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                com.sun.tools.javac.util.List r3 = r3.mo176getParameterTypes()     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                int r3 = r3.length()     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                if (r3 != 0) goto L27
                r3 = r2
                com.sun.tools.javac.code.Symbol$MethodSymbol r3 = (com.sun.tools.javac.code.Symbol.MethodSymbol) r3     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                return r3
            L27:
                com.sun.tools.javac.code.Scope$Entry r2 = r1.next()     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2e
                r1 = r2
                goto Lb
            L2d:
                goto L30
            L2e:
                r1 = move-exception
                r0 = r1
            L30:
                com.sun.tools.javac.jvm.ClassReader r1 = com.sun.tools.javac.jvm.ClassReader.this
                com.sun.tools.javac.util.Log r1 = r1.log
                com.sun.tools.javac.code.Symbol$ClassSymbol r2 = r9.requestingOwner
                javax.tools.JavaFileObject r2 = r2.classfile
                javax.tools.JavaFileObject r1 = r1.useSource(r2)
                com.sun.tools.javac.jvm.ClassReader r2 = com.sun.tools.javac.jvm.ClassReader.this     // Catch: java.lang.Throwable -> L90
                boolean r2 = r2.lintClassfile     // Catch: java.lang.Throwable -> L90
                if (r2 == 0) goto L63
                if (r0 != 0) goto L52
                com.sun.tools.javac.jvm.ClassReader r2 = com.sun.tools.javac.jvm.ClassReader.this     // Catch: java.lang.Throwable -> L90
                com.sun.tools.javac.util.Log r2 = r2.log     // Catch: java.lang.Throwable -> L90
                java.lang.String r3 = "annotation.method.not.found"
                java.lang.Object[] r4 = new java.lang.Object[]{r10, r11}     // Catch: java.lang.Throwable -> L90
                r2.warning(r3, r4)     // Catch: java.lang.Throwable -> L90
                goto L63
            L52:
                com.sun.tools.javac.jvm.ClassReader r2 = com.sun.tools.javac.jvm.ClassReader.this     // Catch: java.lang.Throwable -> L90
                com.sun.tools.javac.util.Log r2 = r2.log     // Catch: java.lang.Throwable -> L90
                java.lang.String r3 = "annotation.method.not.found.reason"
                java.lang.Object r4 = r0.getDetailValue()     // Catch: java.lang.Throwable -> L90
                java.lang.Object[] r4 = new java.lang.Object[]{r10, r11, r4}     // Catch: java.lang.Throwable -> L90
                r2.warning(r3, r4)     // Catch: java.lang.Throwable -> L90
            L63:
                com.sun.tools.javac.jvm.ClassReader r2 = com.sun.tools.javac.jvm.ClassReader.this
                com.sun.tools.javac.util.Log r2 = r2.log
                r2.useSource(r1)
                com.sun.tools.javac.code.Type$MethodType r7 = new com.sun.tools.javac.code.Type$MethodType
                com.sun.tools.javac.util.List r2 = com.sun.tools.javac.util.List.nil()
                com.sun.tools.javac.jvm.ClassReader r3 = com.sun.tools.javac.jvm.ClassReader.this
                com.sun.tools.javac.code.Symtab r3 = r3.syms
                com.sun.tools.javac.code.Type r3 = r3.botType
                com.sun.tools.javac.util.List r4 = com.sun.tools.javac.util.List.nil()
                com.sun.tools.javac.jvm.ClassReader r5 = com.sun.tools.javac.jvm.ClassReader.this
                com.sun.tools.javac.code.Symtab r5 = r5.syms
                com.sun.tools.javac.code.Symbol$ClassSymbol r5 = r5.methodClass
                r7.<init>(r2, r3, r4, r5)
                com.sun.tools.javac.code.Symbol$MethodSymbol r2 = new com.sun.tools.javac.code.Symbol$MethodSymbol
                r4 = 1025(0x401, double:5.064E-321)
                com.sun.tools.javac.code.Symbol$TypeSymbol r8 = r10.tsym
                r3 = r2
                r6 = r11
                r3.<init>(r4, r6, r7, r8)
                return r2
            L90:
                r2 = move-exception
                com.sun.tools.javac.jvm.ClassReader r3 = com.sun.tools.javac.jvm.ClassReader.this
                com.sun.tools.javac.util.Log r3 = r3.log
                r3.useSource(r1)
                throw r2
            */
            throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.jvm.ClassReader.AnnotationDeproxy.findAccessMethod(com.sun.tools.javac.code.Type, com.sun.tools.javac.util.Name):com.sun.tools.javac.code.Symbol$MethodSymbol");
        }

        Attribute deproxy(Type t, Attribute a) {
            Type oldType = this.type;
            try {
                this.type = t;
                a.accept(this);
                return this.result;
            } finally {
                this.type = oldType;
            }
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitConstant(Attribute.Constant value) {
            this.result = value;
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitClass(Attribute.Class clazz) {
            this.result = clazz;
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitEnum(Attribute.Enum e) {
            throw new AssertionError();
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitCompound(Attribute.Compound compound) {
            throw new AssertionError();
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitArray(Attribute.Array array) {
            throw new AssertionError();
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitError(Attribute.Error e) {
            throw new AssertionError();
        }

        /* JADX WARN: Code restructure failed: missing block: B:8:0x001b, code lost:
        
            r1 = (com.sun.tools.javac.code.Symbol.VarSymbol) r3.sym;
         */
        /* JADX WARN: Removed duplicated region for block: B:15:0x0030  */
        /* JADX WARN: Removed duplicated region for block: B:19:0x0078  */
        @Override // com.sun.tools.javac.jvm.ClassReader.ProxyVisitor
        /*
            Code decompiled incorrectly, please refer to instructions dump.
            To view partially-correct code enable 'Show inconsistent code' option in preferences
        */
        public void visitEnumAttributeProxy(com.sun.tools.javac.jvm.ClassReader.EnumAttributeProxy r13) {
            /*
                r12 = this;
                com.sun.tools.javac.code.Type r0 = r13.enumType
                com.sun.tools.javac.code.Symbol$TypeSymbol r0 = r0.tsym
                r1 = 0
                r2 = 0
                com.sun.tools.javac.code.Scope r3 = r0.members()     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2a
                com.sun.tools.javac.util.Name r4 = r13.enumerator     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2a
                com.sun.tools.javac.code.Scope$Entry r3 = r3.lookup(r4)     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2a
            L10:
                com.sun.tools.javac.code.Scope r4 = r3.scope     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2a
                if (r4 == 0) goto L27
                com.sun.tools.javac.code.Symbol r4 = r3.sym     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2a
                int r4 = r4.kind     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2a
                r5 = 4
                if (r4 != r5) goto L21
                com.sun.tools.javac.code.Symbol r4 = r3.sym     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2a
                com.sun.tools.javac.code.Symbol$VarSymbol r4 = (com.sun.tools.javac.code.Symbol.VarSymbol) r4     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2a
                r1 = r4
                goto L27
            L21:
                com.sun.tools.javac.code.Scope$Entry r4 = r3.next()     // Catch: com.sun.tools.javac.code.Symbol.CompletionFailure -> L2a
                r3 = r4
                goto L10
            L27:
                r7 = r1
                r8 = r2
                goto L2e
            L2a:
                r3 = move-exception
                r2 = r3
                r7 = r1
                r8 = r2
            L2e:
                if (r7 != 0) goto L78
                if (r8 == 0) goto L4a
                com.sun.tools.javac.jvm.ClassReader r1 = com.sun.tools.javac.jvm.ClassReader.this
                com.sun.tools.javac.util.Log r1 = r1.log
                com.sun.tools.javac.jvm.ClassReader r2 = com.sun.tools.javac.jvm.ClassReader.this
                javax.tools.JavaFileObject r2 = r2.currentClassFile
                com.sun.tools.javac.util.Name r3 = r13.enumerator
                com.sun.tools.javac.util.JCDiagnostic r4 = r8.getDiagnostic()
                java.lang.Object[] r2 = new java.lang.Object[]{r2, r0, r3, r4}
                java.lang.String r3 = "unknown.enum.constant.reason"
                r1.warning(r3, r2)
                goto L5d
            L4a:
                com.sun.tools.javac.jvm.ClassReader r1 = com.sun.tools.javac.jvm.ClassReader.this
                com.sun.tools.javac.util.Log r1 = r1.log
                com.sun.tools.javac.jvm.ClassReader r2 = com.sun.tools.javac.jvm.ClassReader.this
                javax.tools.JavaFileObject r2 = r2.currentClassFile
                com.sun.tools.javac.util.Name r3 = r13.enumerator
                java.lang.Object[] r2 = new java.lang.Object[]{r2, r0, r3}
                java.lang.String r3 = "unknown.enum.constant"
                r1.warning(r3, r2)
            L5d:
                com.sun.tools.javac.code.Attribute$Enum r9 = new com.sun.tools.javac.code.Attribute$Enum
                com.sun.tools.javac.code.Type r10 = r0.type
                com.sun.tools.javac.code.Symbol$VarSymbol r11 = new com.sun.tools.javac.code.Symbol$VarSymbol
                com.sun.tools.javac.util.Name r4 = r13.enumerator
                com.sun.tools.javac.jvm.ClassReader r1 = com.sun.tools.javac.jvm.ClassReader.this
                com.sun.tools.javac.code.Symtab r1 = r1.syms
                com.sun.tools.javac.code.Type r5 = r1.botType
                r2 = 0
                r1 = r11
                r6 = r0
                r1.<init>(r2, r4, r5, r6)
                r9.<init>(r10, r11)
                r12.result = r9
                goto L81
            L78:
                com.sun.tools.javac.code.Attribute$Enum r1 = new com.sun.tools.javac.code.Attribute$Enum
                com.sun.tools.javac.code.Type r2 = r0.type
                r1.<init>(r2, r7)
                r12.result = r1
            L81:
                return
            */
            throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.jvm.ClassReader.AnnotationDeproxy.visitEnumAttributeProxy(com.sun.tools.javac.jvm.ClassReader$EnumAttributeProxy):void");
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.jvm.ClassReader.ProxyVisitor
        public void visitArrayAttributeProxy(ArrayAttributeProxy proxy) {
            int length = proxy.values.length();
            Attribute[] ats = new Attribute[length];
            Type elemtype = ClassReader.this.types.elemtype(this.type);
            int i = 0;
            List list = proxy.values;
            while (list.nonEmpty()) {
                ats[i] = deproxy(elemtype, (Attribute) list.head);
                list = list.tail;
                i++;
            }
            this.result = new Attribute.Array(this.type, ats);
        }

        @Override // com.sun.tools.javac.jvm.ClassReader.ProxyVisitor
        public void visitCompoundAnnotationProxy(CompoundAnnotationProxy proxy) {
            this.result = deproxyCompound(proxy);
        }
    }

    class AnnotationDefaultCompleter extends AnnotationDeproxy implements Annotate.Worker {
        final JavaFileObject classFile;
        final Symbol.MethodSymbol sym;
        final Attribute value;

        @Override // com.sun.tools.javac.comp.Annotate.Worker
        public String toString() {
            return " ClassReader store default for " + this.sym.owner + "." + this.sym + " is " + this.value;
        }

        AnnotationDefaultCompleter(Symbol.MethodSymbol sym, Attribute value) {
            super();
            this.classFile = ClassReader.this.currentClassFile;
            this.sym = sym;
            this.value = value;
        }

        @Override // com.sun.tools.javac.comp.Annotate.Worker
        public void run() {
            JavaFileObject previousClassFile = ClassReader.this.currentClassFile;
            try {
                this.sym.defaultValue = null;
                ClassReader.this.currentClassFile = this.classFile;
                this.sym.defaultValue = deproxy(this.sym.type.mo178getReturnType(), this.value);
            } finally {
                ClassReader.this.currentClassFile = previousClassFile;
            }
        }
    }

    class AnnotationCompleter extends AnnotationDeproxy implements Annotate.Worker {
        final JavaFileObject classFile;
        final List<CompoundAnnotationProxy> l;
        final Symbol sym;

        @Override // com.sun.tools.javac.comp.Annotate.Worker
        public String toString() {
            return " ClassReader annotate " + this.sym.owner + "." + this.sym + " with " + this.l;
        }

        AnnotationCompleter(Symbol sym, List<CompoundAnnotationProxy> l) {
            super();
            this.sym = sym;
            this.l = l;
            this.classFile = ClassReader.this.currentClassFile;
        }

        @Override // com.sun.tools.javac.comp.Annotate.Worker
        public void run() {
            JavaFileObject previousClassFile = ClassReader.this.currentClassFile;
            try {
                ClassReader.this.currentClassFile = this.classFile;
                List<Attribute.Compound> newList = deproxyCompoundList(this.l);
                if (this.sym.annotationsPendingCompletion()) {
                    this.sym.setDeclarationAttributes(newList);
                } else {
                    this.sym.appendAttributes(newList);
                }
            } finally {
                ClassReader.this.currentClassFile = previousClassFile;
            }
        }
    }

    class TypeAnnotationCompleter extends AnnotationCompleter {
        List<TypeAnnotationProxy> proxies;

        TypeAnnotationCompleter(Symbol sym, List<TypeAnnotationProxy> proxies) {
            super(sym, List.nil());
            this.proxies = proxies;
        }

        List<Attribute.TypeCompound> deproxyTypeCompoundList(List<TypeAnnotationProxy> proxies) {
            ListBuffer<Attribute.TypeCompound> buf = new ListBuffer<>();
            for (TypeAnnotationProxy proxy : proxies) {
                Attribute.Compound compound = deproxyCompound(proxy.compound);
                Attribute.TypeCompound typeCompound = new Attribute.TypeCompound(compound, proxy.position);
                buf.add(typeCompound);
            }
            return buf.toList();
        }

        @Override // com.sun.tools.javac.jvm.ClassReader.AnnotationCompleter, com.sun.tools.javac.comp.Annotate.Worker
        public void run() {
            JavaFileObject previousClassFile = ClassReader.this.currentClassFile;
            try {
                ClassReader.this.currentClassFile = this.classFile;
                List<Attribute.TypeCompound> newList = deproxyTypeCompoundList(this.proxies);
                this.sym.setTypeAttributes(newList.prependList(this.sym.getRawTypeAttributes()));
            } finally {
                ClassReader.this.currentClassFile = previousClassFile;
            }
        }
    }

    Symbol.VarSymbol readField() {
        long flags = adjustFieldFlags(nextChar());
        Name name = readName(nextChar());
        Type type = readType(nextChar());
        Symbol.VarSymbol v = new Symbol.VarSymbol(flags, name, type, this.currentOwner);
        readMemberAttrs(v);
        return v;
    }

    Symbol.MethodSymbol readMethod() {
        Type type;
        long flags = adjustMethodFlags(nextChar());
        Name name = readName(nextChar());
        Type type2 = readType(nextChar());
        if (this.currentOwner.isInterface() && (1024 & flags) == 0 && !name.equals(this.names.clinit)) {
            if (this.majorVersion <= Target.JDK1_8.majorVersion && (this.majorVersion != Target.JDK1_8.majorVersion || this.minorVersion < Target.JDK1_8.minorVersion)) {
                throw badClassFile((flags & 8) == 0 ? "invalid.default.interface" : "invalid.static.interface", Integer.toString(this.majorVersion), Integer.toString(this.minorVersion));
            }
            if ((flags & 8) == 0) {
                this.currentOwner.flags_field |= Flags.DEFAULT;
                flags |= 8796093023232L;
            }
        }
        if (name == this.names.init && this.currentOwner.hasOuterInstance() && !this.currentOwner.name.isEmpty()) {
            type = new Type.MethodType(adjustMethodParams(flags, type2.mo176getParameterTypes()), type2.mo178getReturnType(), type2.mo179getThrownTypes(), this.syms.methodClass);
        } else {
            type = type2;
        }
        Symbol.MethodSymbol m = new Symbol.MethodSymbol(flags, name, type, this.currentOwner);
        if (this.types.isSignaturePolymorphic(m)) {
            m.flags_field |= Flags.SIGNATURE_POLYMORPHIC;
        }
        if (this.saveParameterNames) {
            initParameterNames(m);
        }
        Symbol prevOwner = this.currentOwner;
        this.currentOwner = m;
        try {
            readMemberAttrs(m);
            this.currentOwner = prevOwner;
            if (this.saveParameterNames) {
                setParameterNames(m, type);
            }
            return m;
        } catch (Throwable th) {
            this.currentOwner = prevOwner;
            throw th;
        }
    }

    private List<Type> adjustMethodParams(long flags, List<Type> args) {
        boolean isVarargs = (Flags.VARARGS & flags) != 0;
        if (isVarargs) {
            Type varargsElem = args.last();
            ListBuffer<Type> adjustedArgs = new ListBuffer<>();
            Iterator<Type> it = args.iterator();
            while (it.hasNext()) {
                Type t = it.next();
                adjustedArgs.append(t != varargsElem ? t : ((Type.ArrayType) t).makeVarargs());
            }
            args = adjustedArgs.toList();
        }
        return args.tail;
    }

    void initParameterNames(Symbol.MethodSymbol sym) {
        int expectedParameterSlots = Code.width(sym.type.mo176getParameterTypes()) + 4;
        if (this.parameterNameIndices == null || this.parameterNameIndices.length < expectedParameterSlots) {
            this.parameterNameIndices = new int[expectedParameterSlots];
        } else {
            Arrays.fill(this.parameterNameIndices, 0);
        }
        this.haveParameterNameIndices = false;
        this.sawMethodParameters = false;
    }

    void setParameterNames(Symbol.MethodSymbol sym, Type jvmType) {
        if (!this.haveParameterNameIndices) {
            return;
        }
        int firstParam = 0;
        if (!this.sawMethodParameters) {
            firstParam = (sym.flags() & 8) == 0 ? 1 : 0;
            if (sym.name == this.names.init && this.currentOwner.hasOuterInstance() && !this.currentOwner.name.isEmpty()) {
                firstParam++;
            }
            if (sym.type != jvmType) {
                int skip = Code.width(jvmType.mo176getParameterTypes()) - Code.width(sym.type.mo176getParameterTypes());
                firstParam += skip;
            }
        }
        List<Name> paramNames = List.nil();
        int index = firstParam;
        for (Type t : sym.type.mo176getParameterTypes()) {
            int nameIdx = index < this.parameterNameIndices.length ? this.parameterNameIndices[index] : 0;
            Name name = nameIdx == 0 ? this.names.empty : readName(nameIdx);
            paramNames = paramNames.prepend(name);
            index += Code.width(t);
        }
        sym.savedParameterNames = paramNames.reverse();
    }

    void skipBytes(int n) {
        this.bp += n;
    }

    void skipMember() {
        this.bp += 6;
        char ac = nextChar();
        for (int i = 0; i < ac; i++) {
            this.bp += 2;
            int attrLen = nextInt();
            this.bp += attrLen;
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    protected void enterTypevars(Type t) {
        if (t.getEnclosingType() != null && t.getEnclosingType().hasTag(TypeTag.CLASS)) {
            enterTypevars(t.getEnclosingType());
        }
        for (List typeArguments = t.getTypeArguments(); typeArguments.nonEmpty(); typeArguments = typeArguments.tail) {
            this.typevars.enter(((Type) typeArguments.head).tsym);
        }
    }

    protected void enterTypevars(Symbol sym) {
        if (sym.owner.kind == 16) {
            enterTypevars(sym.owner);
            enterTypevars(sym.owner.owner);
        }
        enterTypevars(sym.type);
    }

    void readClass(Symbol.ClassSymbol c) {
        Type.ClassType ct = (Type.ClassType) c.type;
        c.members_field = new Scope(c);
        this.typevars = this.typevars.dup(this.currentOwner);
        if (ct.getEnclosingType().hasTag(TypeTag.CLASS)) {
            enterTypevars(ct.getEnclosingType());
        }
        long flags = adjustClassFlags(nextChar());
        if (c.owner.kind == 1) {
            c.flags_field = flags;
        }
        Symbol.ClassSymbol self = readClassSymbol(nextChar());
        if (c != self) {
            throw badClassFile("class.file.wrong.class", self.flatname);
        }
        int startbp = this.bp;
        nextChar();
        char interfaceCount = nextChar();
        this.bp += interfaceCount * 2;
        char fieldCount = nextChar();
        for (int i = 0; i < fieldCount; i++) {
            skipMember();
        }
        int methodCount = nextChar();
        for (int i2 = 0; i2 < methodCount; i2++) {
            skipMember();
        }
        readClassAttrs(c);
        if (this.readAllOfClassFile) {
            for (int i3 = 1; i3 < this.poolObj.length; i3++) {
                readPool(i3);
            }
            c.pool = new Pool(this.poolObj.length, this.poolObj, this.types);
        }
        this.bp = startbp;
        int n = nextChar();
        if (ct.supertype_field == null) {
            ct.supertype_field = n == 0 ? Type.noType : readClassSymbol(n).erasure(this.types);
        }
        int n2 = nextChar();
        List<Type> is = List.nil();
        for (int i4 = 0; i4 < n2; i4++) {
            Type _inter = readClassSymbol(nextChar()).erasure(this.types);
            is = is.prepend(_inter);
        }
        if (ct.interfaces_field == null) {
            ct.interfaces_field = is.reverse();
        }
        Assert.check(fieldCount == nextChar());
        for (int i5 = 0; i5 < fieldCount; i5++) {
            enterMember(c, readField());
        }
        int i6 = nextChar();
        Assert.check(methodCount == i6);
        for (int i7 = 0; i7 < methodCount; i7++) {
            enterMember(c, readMethod());
        }
        this.typevars = this.typevars.leave();
    }

    void readInnerClasses(Symbol.ClassSymbol c) {
        int n = nextChar();
        for (int i = 0; i < n; i++) {
            nextChar();
            Symbol.ClassSymbol outer = readClassSymbol(nextChar());
            Name name = readName(nextChar());
            if (name == null) {
                name = this.names.empty;
            }
            long flags = adjustClassFlags(nextChar());
            if (outer != null) {
                if (name == this.names.empty) {
                    name = this.names.one;
                }
                Symbol.ClassSymbol member = enterClass(name, outer);
                if ((8 & flags) == 0) {
                    ((Type.ClassType) member.type).setEnclosingType(outer.type);
                    if (member.erasure_field != null) {
                        ((Type.ClassType) member.erasure_field).setEnclosingType(this.types.erasure(outer.type));
                    }
                }
                if (c == outer) {
                    member.flags_field = flags;
                    enterMember(c, member);
                }
            }
        }
    }

    private void readClassFile(Symbol.ClassSymbol c) throws IOException {
        int magic = nextInt();
        if (magic != -889275714) {
            throw badClassFile("illegal.start.of.class.file", new Object[0]);
        }
        this.minorVersion = nextChar();
        this.majorVersion = nextChar();
        int maxMajor = Target.MAX().majorVersion;
        int maxMinor = Target.MAX().minorVersion;
        if (this.majorVersion > maxMajor || (this.majorVersion * 1000) + this.minorVersion < (Target.MIN().majorVersion * 1000) + Target.MIN().minorVersion) {
            if (this.majorVersion == maxMajor + 1) {
                this.log.warning("big.major.version", this.currentClassFile, Integer.valueOf(this.majorVersion), Integer.valueOf(maxMajor));
            } else {
                throw badClassFile("wrong.version", Integer.toString(this.majorVersion), Integer.toString(this.minorVersion), Integer.toString(maxMajor), Integer.toString(maxMinor));
            }
        } else if (this.checkClassFile && this.majorVersion == maxMajor && this.minorVersion > maxMinor) {
            printCCF("found.later.version", Integer.toString(this.minorVersion));
        }
        indexPool();
        if (this.signatureBuffer.length < this.bp) {
            int ns = Integer.highestOneBit(this.bp) << 1;
            this.signatureBuffer = new byte[ns];
        }
        readClass(c);
    }

    long adjustFieldFlags(long flags) {
        return flags;
    }

    long adjustMethodFlags(long flags) {
        if ((64 & flags) != 0) {
            flags = (flags & (-65)) | Flags.BRIDGE;
            if (!this.allowGenerics) {
                flags &= -4097;
            }
        }
        if ((128 & flags) != 0) {
            return (flags & (-129)) | Flags.VARARGS;
        }
        return flags;
    }

    long adjustClassFlags(long flags) {
        return (-33) & flags;
    }

    public Symbol.ClassSymbol defineClass(Name name, Symbol owner) {
        Symbol.ClassSymbol c = new Symbol.ClassSymbol(0L, name, owner);
        if (owner.kind == 1) {
            Assert.checkNull(this.classes.get(c.flatname), c);
        }
        c.completer = this.thisCompleter;
        return c;
    }

    public Symbol.ClassSymbol enterClass(Name name, Symbol.TypeSymbol owner) {
        Name flatname = Symbol.TypeSymbol.formFlatName(name, owner);
        Symbol.ClassSymbol c = this.classes.get(flatname);
        if (c == null) {
            Symbol.ClassSymbol c2 = defineClass(name, owner);
            this.classes.put(flatname, c2);
            return c2;
        }
        if ((c.name != name || c.owner != owner) && owner.kind == 2 && c.owner.kind == 1) {
            c.owner.members().remove(c);
            c.name = name;
            c.owner = owner;
            c.fullname = Symbol.ClassSymbol.formFullName(name, owner);
            return c;
        }
        return c;
    }

    public Symbol.ClassSymbol enterClass(Name flatName, JavaFileObject classFile) {
        Symbol.ClassSymbol cs = this.classes.get(flatName);
        if (cs != null) {
            String msg = Log.format("%s: completer = %s; class file = %s; source file = %s", cs.fullname, cs.completer, cs.classfile, cs.sourcefile);
            throw new AssertionError(msg);
        }
        Name packageName = Convert.packagePart(flatName);
        Symbol.PackageSymbol owner = packageName.isEmpty() ? this.syms.unnamedPackage : enterPackage(packageName);
        Symbol.ClassSymbol cs2 = defineClass(Convert.shortName(flatName), owner);
        cs2.classfile = classFile;
        this.classes.put(flatName, cs2);
        return cs2;
    }

    public Symbol.ClassSymbol enterClass(Name flatname) {
        Symbol.ClassSymbol c = this.classes.get(flatname);
        if (c == null) {
            return enterClass(flatname, (JavaFileObject) null);
        }
        return c;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void complete(Symbol sym) throws Symbol.CompletionFailure {
        if (sym.kind == 2) {
            Symbol.ClassSymbol c = (Symbol.ClassSymbol) sym;
            c.members_field = new Scope.ErrorScope(c);
            this.annotate.enterStart();
            try {
                completeOwners(c.owner);
                completeEnclosing(c);
                this.annotate.enterDoneWithoutFlush();
                fillIn(c);
            } catch (Throwable th) {
                this.annotate.enterDoneWithoutFlush();
                throw th;
            }
        } else if (sym.kind == 1) {
            Symbol.PackageSymbol p = (Symbol.PackageSymbol) sym;
            try {
                fillIn(p);
            } catch (IOException ex) {
                throw new Symbol.CompletionFailure(sym, ex.getLocalizedMessage()).initCause((Throwable) ex);
            }
        }
        if (!this.filling) {
            this.annotate.flush();
        }
    }

    private void completeOwners(Symbol o) {
        if (o.kind != 1) {
            completeOwners(o.owner);
        }
        o.complete();
    }

    private void completeEnclosing(Symbol.ClassSymbol c) {
        if (c.owner.kind == 1) {
            Symbol owner = c.owner;
            for (Name name : Convert.enclosingCandidates(Convert.shortName(c.name))) {
                Symbol encl = owner.members().lookup(name).sym;
                if (encl == null) {
                    encl = this.classes.get(Symbol.TypeSymbol.formFlatName(name, owner));
                }
                if (encl != null) {
                    encl.complete();
                }
            }
        }
    }

    private void fillIn(Symbol.ClassSymbol c) {
        if (this.completionFailureName == c.fullname) {
            throw new Symbol.CompletionFailure(c, "user-selected completion failure by class name");
        }
        this.currentOwner = c;
        this.warnedAttrs.clear();
        JavaFileObject classfile = c.classfile;
        if (classfile != null) {
            JavaFileObject previousClassFile = this.currentClassFile;
            try {
                try {
                    if (this.filling) {
                        Assert.error("Filling " + classfile.toUri() + " during " + previousClassFile);
                    }
                    this.currentClassFile = classfile;
                    if (this.verbose) {
                        this.log.printVerbose("loading", this.currentClassFile.toString());
                    }
                    if (classfile.getKind() == JavaFileObject.Kind.CLASS) {
                        this.filling = true;
                        try {
                            this.bp = 0;
                            this.buf = readInputStream(this.buf, classfile.openInputStream());
                            readClassFile(c);
                            if (!this.missingTypeVariables.isEmpty() && !this.foundTypeVariables.isEmpty()) {
                                List<Type> missing = this.missingTypeVariables;
                                List<Type> found = this.foundTypeVariables;
                                this.missingTypeVariables = List.nil();
                                this.foundTypeVariables = List.nil();
                                this.filling = false;
                                Type.ClassType ct = (Type.ClassType) this.currentOwner.type;
                                ct.supertype_field = this.types.subst(ct.supertype_field, missing, found);
                                ct.interfaces_field = this.types.subst(ct.interfaces_field, missing, found);
                            } else if (this.missingTypeVariables.isEmpty() != this.foundTypeVariables.isEmpty()) {
                                Name name = this.missingTypeVariables.head.tsym.name;
                                throw badClassFile("undecl.type.var", name);
                            }
                        } finally {
                            this.missingTypeVariables = List.nil();
                            this.foundTypeVariables = List.nil();
                            this.filling = false;
                        }
                    } else if (this.sourceCompleter != null) {
                        this.sourceCompleter.complete(c);
                    } else {
                        throw new IllegalStateException("Source completer required to read " + classfile.toUri());
                    }
                    return;
                } catch (IOException ex) {
                    throw badClassFile("unable.to.access.file", ex.getMessage());
                }
            } finally {
                this.currentClassFile = previousClassFile;
            }
        }
        JCDiagnostic diag = this.diagFactory.fragment("class.file.not.found", c.flatname);
        throw newCompletionFailure(c, diag);
    }

    private static byte[] readInputStream(byte[] buf, InputStream s) throws IOException {
        try {
            byte[] buf2 = ensureCapacity(buf, s.available());
            int r = s.read(buf2);
            int bp = 0;
            while (r != -1) {
                bp += r;
                buf2 = ensureCapacity(buf2, bp);
                r = s.read(buf2, bp, buf2.length - bp);
            }
            return buf2;
        } finally {
            try {
                s.close();
            } catch (IOException e) {
            }
        }
    }

    private static byte[] ensureCapacity(byte[] buf, int needed) {
        if (buf.length <= needed) {
            byte[] buf2 = new byte[Integer.highestOneBit(needed) << 1];
            System.arraycopy(buf, 0, buf2, 0, buf.length);
            return buf2;
        }
        return buf;
    }

    private Symbol.CompletionFailure newCompletionFailure(Symbol.TypeSymbol c, JCDiagnostic diag) {
        if (!this.cacheCompletionFailure) {
            return new Symbol.CompletionFailure(c, diag);
        }
        Symbol.CompletionFailure result = this.cachedCompletionFailure;
        result.sym = c;
        result.diag = diag;
        return result;
    }

    public Symbol.ClassSymbol loadClass(Name flatname) throws Symbol.CompletionFailure {
        boolean absent = this.classes.get(flatname) == null;
        Symbol.ClassSymbol c = enterClass(flatname);
        if (c.members_field == null && c.completer != null) {
            try {
                c.complete();
            } catch (Symbol.CompletionFailure ex) {
                if (absent) {
                    this.classes.remove(flatname);
                }
                throw ex;
            }
        }
        return c;
    }

    public boolean packageExists(Name fullname) {
        return enterPackage(fullname).exists();
    }

    public Symbol.PackageSymbol enterPackage(Name fullname) {
        Symbol.PackageSymbol p = this.packages.get(fullname);
        if (p == null) {
            Assert.check(!fullname.isEmpty(), "rootPackage missing!");
            Symbol.PackageSymbol p2 = new Symbol.PackageSymbol(Convert.shortName(fullname), enterPackage(Convert.packagePart(fullname)));
            p2.completer = this.thisCompleter;
            this.packages.put(fullname, p2);
            return p2;
        }
        return p;
    }

    public Symbol.PackageSymbol enterPackage(Name name, Symbol.PackageSymbol owner) {
        return enterPackage(Symbol.TypeSymbol.formFullName(name, owner));
    }

    protected void includeClassFile(Symbol.PackageSymbol p, JavaFileObject file) {
        int seen;
        if ((p.flags_field & 8388608) == 0) {
            for (Symbol q = p; q != null && q.kind == 1; q = q.owner) {
                q.flags_field |= 8388608;
            }
        }
        JavaFileObject.Kind kind = file.getKind();
        if (kind == JavaFileObject.Kind.CLASS) {
            seen = Flags.CLASS_SEEN;
        } else {
            seen = Flags.SOURCE_SEEN;
        }
        String binaryName = this.fileManager.inferBinaryName(this.currentLoc, file);
        int lastDot = binaryName.lastIndexOf(".");
        Name classname = this.names.fromString(binaryName.substring(lastDot + 1));
        boolean isPkgInfo = classname == this.names.package_info;
        Symbol.ClassSymbol c = isPkgInfo ? p.package_info : (Symbol.ClassSymbol) p.members_field.lookup(classname).sym;
        if (c == null) {
            c = enterClass(classname, p);
            if (c.classfile == null) {
                c.classfile = file;
            }
            if (isPkgInfo) {
                p.package_info = c;
            } else if (c.owner == p) {
                p.members_field.enter(c);
            }
        } else if (c.classfile != null && (c.flags_field & ((long) seen)) == 0 && (c.flags_field & 100663296) != 0) {
            c.classfile = preferredFileObject(file, c.classfile);
        }
        c.flags_field |= (long) seen;
    }

    protected JavaFileObject preferredFileObject(JavaFileObject a, JavaFileObject b) {
        if (this.preferSource) {
            return a.getKind() == JavaFileObject.Kind.SOURCE ? a : b;
        }
        long adate = a.getLastModified();
        long bdate = b.getLastModified();
        return adate > bdate ? a : b;
    }

    protected EnumSet<JavaFileObject.Kind> getPackageFileKinds() {
        return EnumSet.of(JavaFileObject.Kind.CLASS, JavaFileObject.Kind.SOURCE);
    }

    protected void extraFileActions(Symbol.PackageSymbol pack, JavaFileObject fe) {
    }

    private void fillIn(Symbol.PackageSymbol p) throws IOException {
        if (p.members_field == null) {
            p.members_field = new Scope(p);
        }
        String packageName = p.fullname.toString();
        Set<JavaFileObject.Kind> kinds = getPackageFileKinds();
        fillIn(p, StandardLocation.PLATFORM_CLASS_PATH, this.fileManager.list(StandardLocation.PLATFORM_CLASS_PATH, packageName, EnumSet.of(JavaFileObject.Kind.CLASS), false));
        Set<JavaFileObject.Kind> classKinds = EnumSet.copyOf((Collection) kinds);
        classKinds.remove(JavaFileObject.Kind.SOURCE);
        boolean wantClassFiles = !classKinds.isEmpty();
        Set<JavaFileObject.Kind> sourceKinds = EnumSet.copyOf((Collection) kinds);
        sourceKinds.remove(JavaFileObject.Kind.CLASS);
        boolean wantSourceFiles = !sourceKinds.isEmpty();
        boolean haveSourcePath = this.fileManager.hasLocation(StandardLocation.SOURCE_PATH);
        if (this.verbose && this.verbosePath && (this.fileManager instanceof StandardJavaFileManager)) {
            StandardJavaFileManager fm = (StandardJavaFileManager) this.fileManager;
            if (haveSourcePath && wantSourceFiles) {
                List<File> path = List.nil();
                for (File file : fm.getLocation(StandardLocation.SOURCE_PATH)) {
                    path = path.prepend(file);
                }
                this.log.printVerbose("sourcepath", path.reverse().toString());
            } else if (wantSourceFiles) {
                List<File> path2 = List.nil();
                for (File file2 : fm.getLocation(StandardLocation.CLASS_PATH)) {
                    path2 = path2.prepend(file2);
                }
                this.log.printVerbose("sourcepath", path2.reverse().toString());
            }
            if (wantClassFiles) {
                List<File> path3 = List.nil();
                for (File file3 : fm.getLocation(StandardLocation.PLATFORM_CLASS_PATH)) {
                    path3 = path3.prepend(file3);
                }
                for (File file4 : fm.getLocation(StandardLocation.CLASS_PATH)) {
                    path3 = path3.prepend(file4);
                }
                this.log.printVerbose("classpath", path3.reverse().toString());
            }
        }
        if (wantSourceFiles && !haveSourcePath) {
            fillIn(p, StandardLocation.CLASS_PATH, this.fileManager.list(StandardLocation.CLASS_PATH, packageName, kinds, false));
        } else {
            if (wantClassFiles) {
                fillIn(p, StandardLocation.CLASS_PATH, this.fileManager.list(StandardLocation.CLASS_PATH, packageName, classKinds, false));
            }
            if (wantSourceFiles) {
                fillIn(p, StandardLocation.SOURCE_PATH, this.fileManager.list(StandardLocation.SOURCE_PATH, packageName, sourceKinds, false));
            }
        }
        this.verbosePath = false;
    }

    private void fillIn(Symbol.PackageSymbol p, JavaFileManager.Location location, Iterable<JavaFileObject> files) {
        this.currentLoc = location;
        for (JavaFileObject fo : files) {
            switch (fo.getKind()) {
                case CLASS:
                case SOURCE:
                    String binaryName = this.fileManager.inferBinaryName(this.currentLoc, fo);
                    String simpleName = binaryName.substring(binaryName.lastIndexOf(".") + 1);
                    if (SourceVersion.isIdentifier(simpleName) || simpleName.equals("package-info")) {
                        includeClassFile(p, fo);
                    }
                    break;
                default:
                    extraFileActions(p, fo);
                    break;
            }
        }
    }

    private void printCCF(String key, Object arg) {
        this.log.printLines(key, arg);
    }

    private static class SourceFileObject extends BaseFileObject {
        private Name flatname;
        private Name name;

        public SourceFileObject(Name name, Name flatname) {
            super(null);
            this.name = name;
            this.flatname = flatname;
        }

        @Override // javax.tools.FileObject
        public URI toUri() {
            try {
                return new URI(null, this.name.toString(), null);
            } catch (URISyntaxException e) {
                throw new BaseFileObject.CannotCreateUriError(this.name.toString(), e);
            }
        }

        @Override // javax.tools.FileObject
        public String getName() {
            return this.name.toString();
        }

        @Override // com.sun.tools.javac.file.BaseFileObject
        public String getShortName() {
            return getName();
        }

        @Override // javax.tools.JavaFileObject
        public JavaFileObject.Kind getKind() {
            return getKind(getName());
        }

        @Override // javax.tools.FileObject
        public InputStream openInputStream() {
            throw new UnsupportedOperationException();
        }

        @Override // javax.tools.FileObject
        public OutputStream openOutputStream() {
            throw new UnsupportedOperationException();
        }

        @Override // javax.tools.FileObject
        public CharBuffer getCharContent(boolean ignoreEncodingErrors) {
            throw new UnsupportedOperationException();
        }

        @Override // com.sun.tools.javac.file.BaseFileObject, javax.tools.FileObject
        public Reader openReader(boolean ignoreEncodingErrors) {
            throw new UnsupportedOperationException();
        }

        @Override // javax.tools.FileObject
        public Writer openWriter() {
            throw new UnsupportedOperationException();
        }

        @Override // javax.tools.FileObject
        public long getLastModified() {
            throw new UnsupportedOperationException();
        }

        @Override // javax.tools.FileObject
        public boolean delete() {
            throw new UnsupportedOperationException();
        }

        @Override // com.sun.tools.javac.file.BaseFileObject
        protected String inferBinaryName(Iterable<? extends File> path) {
            return this.flatname.toString();
        }

        @Override // javax.tools.JavaFileObject
        public boolean isNameCompatible(String simpleName, JavaFileObject.Kind kind) {
            return true;
        }

        @Override // com.sun.tools.javac.file.BaseFileObject
        public boolean equals(Object other) {
            if (this == other) {
                return true;
            }
            if (!(other instanceof SourceFileObject)) {
                return false;
            }
            SourceFileObject o = (SourceFileObject) other;
            return this.name.equals(o.name);
        }

        @Override // com.sun.tools.javac.file.BaseFileObject
        public int hashCode() {
            return this.name.hashCode();
        }
    }
}
