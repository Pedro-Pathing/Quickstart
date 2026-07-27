package com.sun.tools.javac.code;

import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.jvm.ByteCodes;
import com.sun.tools.javac.jvm.ClassReader;
import com.sun.tools.javac.jvm.Target;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.JavacMessages;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import javax.lang.model.element.ElementVisitor;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes.dex */
public class Symtab {
    protected static final Context.Key<Symtab> symtabKey = new Context.Key<>();
    public final Type annotationTargetType;
    public final Type annotationType;
    public final Symbol.ClassSymbol arrayClass;
    public final Symbol.MethodSymbol arrayCloneMethod;
    public final Type arraysType;
    public final Type assertionErrorType;
    public final Symbol.MethodSymbol autoCloseableClose;
    public final Type autoCloseableType;
    public final Symbol.ClassSymbol boundClass;
    public final Type classLoaderType;
    public final Type classNotFoundExceptionType;
    public final Type classType;
    public final Type cloneNotSupportedExceptionType;
    public final Type cloneableType;
    public final Type collectionsType;
    public final Type comparableType;
    public final Type comparatorType;
    public final Type deprecatedType;
    public final Type documentedType;
    public final Type elementTypeType;
    public final Symbol.MethodSymbol enumFinalFinalize;
    public final Symbol.TypeSymbol enumSym;
    public final Symbol.ClassSymbol errSymbol;
    public final Type errType;
    public final Type errorType;
    public final Type exceptionType;
    public final Type functionalInterfaceType;
    public final Type illegalArgumentExceptionType;
    public final Type inheritedType;
    public final Type interruptedExceptionType;
    public final Type iterableType;
    public final Type iteratorType;
    public final Type lambdaMetafactory;
    public final Symbol.VarSymbol lengthVar;
    public final Type listType;
    public final Symbol.ClassSymbol methodClass;
    public final Type methodHandleLookupType;
    public final Type methodHandleType;
    public final Type methodTypeType;
    private final Names names;
    public final Type nativeHeaderType;
    public final Type noClassDefFoundErrorType;
    public final Type noSuchFieldErrorType;
    public final Symbol.TypeSymbol noSymbol;
    public final Symbol.OperatorSymbol nullcheck;
    public final Type objectType;
    public final Type overrideType;
    public final Symbol.ClassSymbol predefClass;
    public final Type profileType;
    public final Type proprietaryType;
    private final ClassReader reader;
    public final Type repeatableType;
    public final Type retentionType;
    public final Symbol.PackageSymbol rootPackage;
    public final Type runtimeExceptionType;
    public final Type serializableType;
    public final Type serializedLambdaType;
    public final Type stringBufferType;
    public final Type stringBuilderType;
    public final Type stringType;
    public final Type suppressWarningsType;
    public final Type systemType;
    private final Target target;
    public final Type throwableType;
    public final Type trustMeType;
    public final Symbol.ClassSymbol unknownSymbol;
    public final Type unknownType;
    public final Symbol.PackageSymbol unnamedPackage;
    public final Type.JCPrimitiveType byteType = new Type.JCPrimitiveType(TypeTag.BYTE, null);
    public final Type.JCPrimitiveType charType = new Type.JCPrimitiveType(TypeTag.CHAR, null);
    public final Type.JCPrimitiveType shortType = new Type.JCPrimitiveType(TypeTag.SHORT, null);
    public final Type.JCPrimitiveType intType = new Type.JCPrimitiveType(TypeTag.INT, null);
    public final Type.JCPrimitiveType longType = new Type.JCPrimitiveType(TypeTag.LONG, null);
    public final Type.JCPrimitiveType floatType = new Type.JCPrimitiveType(TypeTag.FLOAT, null);
    public final Type.JCPrimitiveType doubleType = new Type.JCPrimitiveType(TypeTag.DOUBLE, null);
    public final Type.JCPrimitiveType booleanType = new Type.JCPrimitiveType(TypeTag.BOOLEAN, null);
    public final Type botType = new Type.BottomType();
    public final Type.JCVoidType voidType = new Type.JCVoidType();
    public final Type[] typeOfTag = new Type[TypeTag.getTypeTagCount()];
    public final Name[] boxedName = new Name[TypeTag.getTypeTagCount()];
    public final Set<Name> operatorNames = new HashSet();
    public final Map<Name, Symbol.ClassSymbol> classes = new HashMap();
    public final Map<Name, Symbol.PackageSymbol> packages = new HashMap();

    public static Symtab instance(Context context) {
        Symtab instance = (Symtab) context.get(symtabKey);
        if (instance == null) {
            return new Symtab(context);
        }
        return instance;
    }

    public void initType(Type type, Symbol.ClassSymbol c) {
        type.tsym = c;
        this.typeOfTag[type.getTag().ordinal()] = type;
    }

    public void initType(Type type, String name) {
        initType(type, new Symbol.ClassSymbol(1L, this.names.fromString(name), type, this.rootPackage));
    }

    public void initType(Type type, String name, String bname) {
        initType(type, name);
        this.boxedName[type.getTag().ordinal()] = this.names.fromString("java.lang." + bname);
    }

    private Symbol.VarSymbol enterConstant(String name, Type type) {
        Symbol.VarSymbol c = new Symbol.VarSymbol(25L, this.names.fromString(name), type, this.predefClass);
        c.setData(type.constValue());
        this.predefClass.members().enter(c);
        return c;
    }

    private void enterBinop(String name, Type left, Type right, Type res, int opcode) {
        this.predefClass.members().enter(new Symbol.OperatorSymbol(makeOperatorName(name), new Type.MethodType(List.of(left, right), res, List.nil(), this.methodClass), opcode, this.predefClass));
    }

    private void enterBinop(String name, Type left, Type right, Type res, int opcode1, int opcode2) {
        enterBinop(name, left, right, res, (opcode1 << 9) | opcode2);
    }

    private Symbol.OperatorSymbol enterUnop(String name, Type arg, Type res, int opcode) {
        Symbol.OperatorSymbol sym = new Symbol.OperatorSymbol(makeOperatorName(name), new Type.MethodType(List.of(arg), res, List.nil(), this.methodClass), opcode, this.predefClass);
        this.predefClass.members().enter(sym);
        return sym;
    }

    private Name makeOperatorName(String name) {
        Name opName = this.names.fromString(name);
        this.operatorNames.add(opName);
        return opName;
    }

    private Type enterClass(String s) {
        return this.reader.enterClass(this.names.fromString(s)).type;
    }

    public void synthesizeEmptyInterfaceIfMissing(Type type) {
        final Symbol.Completer completer = type.tsym.completer;
        if (completer != null) {
            type.tsym.completer = new Symbol.Completer() { // from class: com.sun.tools.javac.code.Symtab.1
                @Override // com.sun.tools.javac.code.Symbol.Completer
                public void complete(Symbol sym) throws Symbol.CompletionFailure {
                    try {
                        completer.complete(sym);
                    } catch (Symbol.CompletionFailure e) {
                        sym.flags_field |= 513;
                        ((Type.ClassType) sym.type).supertype_field = Symtab.this.objectType;
                    }
                }
            };
        }
    }

    public void synthesizeBoxTypeIfMissing(final Type type) {
        Symbol.ClassSymbol sym = this.reader.enterClass(this.boxedName[type.getTag().ordinal()]);
        final Symbol.Completer completer = sym.completer;
        if (completer != null) {
            sym.completer = new Symbol.Completer() { // from class: com.sun.tools.javac.code.Symtab.2
                @Override // com.sun.tools.javac.code.Symbol.Completer
                public void complete(Symbol sym2) throws Symbol.CompletionFailure {
                    try {
                        completer.complete(sym2);
                    } catch (Symbol.CompletionFailure e) {
                        sym2.flags_field |= 1;
                        ((Type.ClassType) sym2.type).supertype_field = Symtab.this.objectType;
                        Name n = Symtab.this.target.boxWithConstructors() ? Symtab.this.names.init : Symtab.this.names.valueOf;
                        Symbol.MethodSymbol boxMethod = new Symbol.MethodSymbol(9L, n, new Type.MethodType(List.of(type), sym2.type, List.nil(), Symtab.this.methodClass), sym2);
                        sym2.members().enter(boxMethod);
                        Symbol.MethodSymbol unboxMethod = new Symbol.MethodSymbol(1L, type.tsym.name.append(Symtab.this.names.Value), new Type.MethodType(List.nil(), type, List.nil(), Symtab.this.methodClass), sym2);
                        sym2.members().enter(unboxMethod);
                    }
                }
            };
        }
    }

    private Type enterSyntheticAnnotation(String name) {
        Type.ClassType type = (Type.ClassType) enterClass(name);
        Symbol.ClassSymbol sym = (Symbol.ClassSymbol) type.tsym;
        sym.completer = null;
        sym.flags_field = 1073750529L;
        sym.erasure_field = type;
        sym.members_field = new Scope(sym);
        type.typarams_field = List.nil();
        type.allparams_field = List.nil();
        type.supertype_field = this.annotationType;
        type.interfaces_field = List.nil();
        return type;
    }

    protected Symtab(Context context) throws Symbol.CompletionFailure {
        context.put(symtabKey, this);
        this.names = Names.instance(context);
        this.target = Target.instance(context);
        this.unknownType = new Type.UnknownType();
        this.rootPackage = new Symbol.PackageSymbol(this.names.empty, null);
        final JavacMessages messages = JavacMessages.instance(context);
        this.unnamedPackage = new Symbol.PackageSymbol(this.names.empty, this.rootPackage) { // from class: com.sun.tools.javac.code.Symtab.3
            @Override // com.sun.tools.javac.code.Symbol.PackageSymbol, com.sun.tools.javac.code.Symbol
            public String toString() {
                return messages.getLocalizedString("compiler.misc.unnamed.package", new Object[0]);
            }
        };
        this.noSymbol = new Symbol.TypeSymbol(0, 0L, this.names.empty, Type.noType, this.rootPackage) { // from class: com.sun.tools.javac.code.Symtab.4
            @Override // javax.lang.model.element.Element
            public <R, P> R accept(ElementVisitor<R, P> v, P p) {
                return v.visitUnknown(this, p);
            }
        };
        this.errSymbol = new Symbol.ClassSymbol(1073741833L, this.names.any, null, this.rootPackage);
        this.errType = new Type.ErrorType(this.errSymbol, Type.noType);
        this.unknownSymbol = new Symbol.ClassSymbol(1073741833L, this.names.fromString("<any?>"), null, this.rootPackage);
        this.unknownSymbol.members_field = new Scope.ErrorScope(this.unknownSymbol);
        this.unknownSymbol.type = this.unknownType;
        initType(this.byteType, "byte", "Byte");
        initType(this.shortType, "short", "Short");
        initType(this.charType, "char", "Character");
        initType(this.intType, "int", "Integer");
        initType(this.longType, "long", "Long");
        initType(this.floatType, "float", "Float");
        initType(this.doubleType, "double", "Double");
        initType(this.booleanType, "boolean", "Boolean");
        initType(this.voidType, "void", "Void");
        initType(this.botType, "<nulltype>");
        initType(this.errType, this.errSymbol);
        initType(this.unknownType, this.unknownSymbol);
        this.arrayClass = new Symbol.ClassSymbol(1073741825L, this.names.Array, this.noSymbol);
        this.boundClass = new Symbol.ClassSymbol(1073741825L, this.names.Bound, this.noSymbol);
        this.boundClass.members_field = new Scope.ErrorScope(this.boundClass);
        this.methodClass = new Symbol.ClassSymbol(1073741825L, this.names.Method, this.noSymbol);
        this.methodClass.members_field = new Scope.ErrorScope(this.boundClass);
        this.predefClass = new Symbol.ClassSymbol(1073741825L, this.names.empty, this.rootPackage);
        Scope scope = new Scope(this.predefClass);
        this.predefClass.members_field = scope;
        scope.enter(this.byteType.tsym);
        scope.enter(this.shortType.tsym);
        scope.enter(this.charType.tsym);
        scope.enter(this.intType.tsym);
        scope.enter(this.longType.tsym);
        scope.enter(this.floatType.tsym);
        scope.enter(this.doubleType.tsym);
        scope.enter(this.booleanType.tsym);
        scope.enter(this.errType.tsym);
        scope.enter(this.errSymbol);
        this.classes.put(this.predefClass.fullname, this.predefClass);
        this.reader = ClassReader.instance(context);
        this.reader.init(this);
        this.objectType = enterClass("java.lang.Object");
        this.classType = enterClass("java.lang.Class");
        this.stringType = enterClass("java.lang.String");
        this.stringBufferType = enterClass("java.lang.StringBuffer");
        this.stringBuilderType = enterClass("java.lang.StringBuilder");
        this.cloneableType = enterClass("java.lang.Cloneable");
        this.throwableType = enterClass("java.lang.Throwable");
        this.serializableType = enterClass("java.io.Serializable");
        this.serializedLambdaType = enterClass("java.lang.invoke.SerializedLambda");
        this.methodHandleType = enterClass("java.lang.invoke.MethodHandle");
        this.methodHandleLookupType = enterClass("java.lang.invoke.MethodHandles$Lookup");
        this.methodTypeType = enterClass("java.lang.invoke.MethodType");
        this.errorType = enterClass("java.lang.Error");
        this.illegalArgumentExceptionType = enterClass("java.lang.IllegalArgumentException");
        this.interruptedExceptionType = enterClass("java.lang.InterruptedException");
        this.exceptionType = enterClass("java.lang.Exception");
        this.runtimeExceptionType = enterClass("java.lang.RuntimeException");
        this.classNotFoundExceptionType = enterClass("java.lang.ClassNotFoundException");
        this.noClassDefFoundErrorType = enterClass("java.lang.NoClassDefFoundError");
        this.noSuchFieldErrorType = enterClass("java.lang.NoSuchFieldError");
        this.assertionErrorType = enterClass("java.lang.AssertionError");
        this.cloneNotSupportedExceptionType = enterClass("java.lang.CloneNotSupportedException");
        this.annotationType = enterClass("java.lang.annotation.Annotation");
        this.classLoaderType = enterClass("java.lang.ClassLoader");
        this.enumSym = this.reader.enterClass(this.names.java_lang_Enum);
        this.enumFinalFinalize = new Symbol.MethodSymbol(137438953492L, this.names.finalize, new Type.MethodType(List.nil(), this.voidType, List.nil(), this.methodClass), this.enumSym);
        this.listType = enterClass("java.util.List");
        this.collectionsType = enterClass("java.util.Collections");
        this.comparableType = enterClass("java.lang.Comparable");
        this.comparatorType = enterClass("java.util.Comparator");
        this.arraysType = enterClass("java.util.Arrays");
        this.iterableType = this.target.hasIterable() ? enterClass("java.lang.Iterable") : enterClass("java.util.Collection");
        this.iteratorType = enterClass("java.util.Iterator");
        this.annotationTargetType = enterClass("java.lang.annotation.Target");
        this.overrideType = enterClass("java.lang.Override");
        this.retentionType = enterClass("java.lang.annotation.Retention");
        this.deprecatedType = enterClass("java.lang.Deprecated");
        this.suppressWarningsType = enterClass("java.lang.SuppressWarnings");
        this.inheritedType = enterClass("java.lang.annotation.Inherited");
        this.repeatableType = enterClass("java.lang.annotation.Repeatable");
        this.documentedType = enterClass("java.lang.annotation.Documented");
        this.elementTypeType = enterClass("java.lang.annotation.ElementType");
        this.systemType = enterClass("java.lang.System");
        this.autoCloseableType = enterClass("java.lang.AutoCloseable");
        this.autoCloseableClose = new Symbol.MethodSymbol(1L, this.names.close, new Type.MethodType(List.nil(), this.voidType, List.of(this.exceptionType), this.methodClass), this.autoCloseableType.tsym);
        this.trustMeType = enterClass("java.lang.SafeVarargs");
        this.nativeHeaderType = enterClass("java.lang.annotation.Native");
        this.lambdaMetafactory = enterClass("java.lang.invoke.LambdaMetafactory");
        this.functionalInterfaceType = enterClass("java.lang.FunctionalInterface");
        synthesizeEmptyInterfaceIfMissing(this.autoCloseableType);
        synthesizeEmptyInterfaceIfMissing(this.cloneableType);
        synthesizeEmptyInterfaceIfMissing(this.serializableType);
        synthesizeEmptyInterfaceIfMissing(this.lambdaMetafactory);
        synthesizeEmptyInterfaceIfMissing(this.serializedLambdaType);
        synthesizeBoxTypeIfMissing(this.doubleType);
        synthesizeBoxTypeIfMissing(this.floatType);
        synthesizeBoxTypeIfMissing(this.voidType);
        this.proprietaryType = enterSyntheticAnnotation("sun.Proprietary+Annotation");
        this.profileType = enterSyntheticAnnotation("jdk.Profile+Annotation");
        Symbol.MethodSymbol m = new Symbol.MethodSymbol(Flags.AnnotationTypeElementMask, this.names.value, this.intType, this.profileType.tsym);
        this.profileType.tsym.members().enter(m);
        Type.ClassType arrayClassType = (Type.ClassType) this.arrayClass.type;
        arrayClassType.supertype_field = this.objectType;
        arrayClassType.interfaces_field = List.of(this.cloneableType, this.serializableType);
        this.arrayClass.members_field = new Scope(this.arrayClass);
        this.lengthVar = new Symbol.VarSymbol(17L, this.names.length, this.intType, this.arrayClass);
        this.arrayClass.members().enter(this.lengthVar);
        this.arrayCloneMethod = new Symbol.MethodSymbol(1L, this.names.clone, new Type.MethodType(List.nil(), this.objectType, List.nil(), this.methodClass), this.arrayClass);
        this.arrayClass.members().enter(this.arrayCloneMethod);
        enterUnop("+++", this.doubleType, this.doubleType, 0);
        enterUnop("+++", this.floatType, this.floatType, 0);
        enterUnop("+++", this.longType, this.longType, 0);
        enterUnop("+++", this.intType, this.intType, 0);
        enterUnop("---", this.doubleType, this.doubleType, 119);
        enterUnop("---", this.floatType, this.floatType, 118);
        enterUnop("---", this.longType, this.longType, 117);
        enterUnop("---", this.intType, this.intType, 116);
        enterUnop("~", this.longType, this.longType, 131);
        enterUnop("~", this.intType, this.intType, 130);
        enterUnop("++", this.doubleType, this.doubleType, 99);
        enterUnop("++", this.floatType, this.floatType, 98);
        enterUnop("++", this.longType, this.longType, 97);
        enterUnop("++", this.intType, this.intType, 96);
        enterUnop("++", this.charType, this.charType, 96);
        enterUnop("++", this.shortType, this.shortType, 96);
        enterUnop("++", this.byteType, this.byteType, 96);
        enterUnop("--", this.doubleType, this.doubleType, 103);
        enterUnop("--", this.floatType, this.floatType, 102);
        enterUnop("--", this.longType, this.longType, 101);
        enterUnop("--", this.intType, this.intType, 100);
        enterUnop("--", this.charType, this.charType, 100);
        enterUnop("--", this.shortType, this.shortType, 100);
        enterUnop("--", this.byteType, this.byteType, 100);
        enterUnop("!", this.booleanType, this.booleanType, 257);
        this.nullcheck = enterUnop("<*nullchk*>", this.objectType, this.objectType, ByteCodes.nullchk);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.stringType, this.objectType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.objectType, this.stringType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.stringType, this.stringType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.stringType, this.intType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.stringType, this.longType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.stringType, this.floatType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.stringType, this.doubleType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.stringType, this.booleanType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.stringType, this.botType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.intType, this.stringType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.longType, this.stringType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.floatType, this.stringType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.doubleType, this.stringType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.booleanType, this.stringType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.botType, this.stringType, this.stringType, 256);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.botType, this.botType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.botType, this.intType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.botType, this.longType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.botType, this.floatType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.botType, this.doubleType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.botType, this.booleanType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.botType, this.objectType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.intType, this.botType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.longType, this.botType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.floatType, this.botType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.doubleType, this.botType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.booleanType, this.botType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.objectType, this.botType, this.botType, ByteCodes.error);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.doubleType, this.doubleType, this.doubleType, 99);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.floatType, this.floatType, this.floatType, 98);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.longType, this.longType, this.longType, 97);
        enterBinop(Marker.ANY_NON_NULL_MARKER, this.intType, this.intType, this.intType, 96);
        enterBinop("-", this.doubleType, this.doubleType, this.doubleType, 103);
        enterBinop("-", this.floatType, this.floatType, this.floatType, 102);
        enterBinop("-", this.longType, this.longType, this.longType, 101);
        enterBinop("-", this.intType, this.intType, this.intType, 100);
        enterBinop(Marker.ANY_MARKER, this.doubleType, this.doubleType, this.doubleType, 107);
        enterBinop(Marker.ANY_MARKER, this.floatType, this.floatType, this.floatType, 106);
        enterBinop(Marker.ANY_MARKER, this.longType, this.longType, this.longType, 105);
        enterBinop(Marker.ANY_MARKER, this.intType, this.intType, this.intType, 104);
        enterBinop(OnBotJavaFileSystemUtils.PATH_SEPARATOR, this.doubleType, this.doubleType, this.doubleType, 111);
        enterBinop(OnBotJavaFileSystemUtils.PATH_SEPARATOR, this.floatType, this.floatType, this.floatType, 110);
        enterBinop(OnBotJavaFileSystemUtils.PATH_SEPARATOR, this.longType, this.longType, this.longType, 109);
        enterBinop(OnBotJavaFileSystemUtils.PATH_SEPARATOR, this.intType, this.intType, this.intType, 108);
        enterBinop("%", this.doubleType, this.doubleType, this.doubleType, 115);
        enterBinop("%", this.floatType, this.floatType, this.floatType, ByteCodes.fmod);
        enterBinop("%", this.longType, this.longType, this.longType, ByteCodes.lmod);
        enterBinop("%", this.intType, this.intType, this.intType, 112);
        enterBinop("&", this.booleanType, this.booleanType, this.booleanType, 126);
        enterBinop("&", this.longType, this.longType, this.longType, 127);
        enterBinop("&", this.intType, this.intType, this.intType, 126);
        enterBinop("|", this.booleanType, this.booleanType, this.booleanType, 128);
        enterBinop("|", this.longType, this.longType, this.longType, 129);
        enterBinop("|", this.intType, this.intType, this.intType, 128);
        enterBinop("^", this.booleanType, this.booleanType, this.booleanType, 130);
        enterBinop("^", this.longType, this.longType, this.longType, 131);
        enterBinop("^", this.intType, this.intType, this.intType, 130);
        enterBinop("<<", this.longType, this.longType, this.longType, ByteCodes.lshll);
        enterBinop("<<", this.intType, this.longType, this.intType, ByteCodes.ishll);
        enterBinop("<<", this.longType, this.intType, this.longType, 121);
        enterBinop("<<", this.intType, this.intType, this.intType, 120);
        enterBinop(">>", this.longType, this.longType, this.longType, ByteCodes.lshrl);
        enterBinop(">>", this.intType, this.longType, this.intType, 272);
        enterBinop(">>", this.longType, this.intType, this.longType, 123);
        enterBinop(">>", this.intType, this.intType, this.intType, 122);
        enterBinop(">>>", this.longType, this.longType, this.longType, ByteCodes.lushrl);
        enterBinop(">>>", this.intType, this.longType, this.intType, ByteCodes.iushrl);
        enterBinop(">>>", this.longType, this.intType, this.longType, 125);
        enterBinop(">>>", this.intType, this.intType, this.intType, 124);
        enterBinop("<", this.doubleType, this.doubleType, this.booleanType, 152, 155);
        enterBinop("<", this.floatType, this.floatType, this.booleanType, 150, 155);
        enterBinop("<", this.longType, this.longType, this.booleanType, 148, 155);
        enterBinop("<", this.intType, this.intType, this.booleanType, ByteCodes.if_icmplt);
        enterBinop(">", this.doubleType, this.doubleType, this.booleanType, 151, ByteCodes.ifgt);
        enterBinop(">", this.floatType, this.floatType, this.booleanType, 149, ByteCodes.ifgt);
        enterBinop(">", this.longType, this.longType, this.booleanType, 148, ByteCodes.ifgt);
        enterBinop(">", this.intType, this.intType, this.booleanType, ByteCodes.if_icmpgt);
        enterBinop("<=", this.doubleType, this.doubleType, this.booleanType, 152, ByteCodes.ifle);
        enterBinop("<=", this.floatType, this.floatType, this.booleanType, 150, ByteCodes.ifle);
        enterBinop("<=", this.longType, this.longType, this.booleanType, 148, ByteCodes.ifle);
        enterBinop("<=", this.intType, this.intType, this.booleanType, ByteCodes.if_icmple);
        enterBinop(">=", this.doubleType, this.doubleType, this.booleanType, 151, ByteCodes.ifge);
        enterBinop(">=", this.floatType, this.floatType, this.booleanType, 149, ByteCodes.ifge);
        enterBinop(">=", this.longType, this.longType, this.booleanType, 148, ByteCodes.ifge);
        enterBinop(">=", this.intType, this.intType, this.booleanType, ByteCodes.if_icmpge);
        enterBinop("==", this.objectType, this.objectType, this.booleanType, ByteCodes.if_acmpeq);
        enterBinop("==", this.booleanType, this.booleanType, this.booleanType, ByteCodes.if_icmpeq);
        enterBinop("==", this.doubleType, this.doubleType, this.booleanType, 151, 153);
        enterBinop("==", this.floatType, this.floatType, this.booleanType, 149, 153);
        enterBinop("==", this.longType, this.longType, this.booleanType, 148, 153);
        enterBinop("==", this.intType, this.intType, this.booleanType, ByteCodes.if_icmpeq);
        enterBinop("!=", this.objectType, this.objectType, this.booleanType, ByteCodes.if_acmpne);
        enterBinop("!=", this.booleanType, this.booleanType, this.booleanType, ByteCodes.if_icmpne);
        enterBinop("!=", this.doubleType, this.doubleType, this.booleanType, 151, 154);
        enterBinop("!=", this.floatType, this.floatType, this.booleanType, 149, 154);
        enterBinop("!=", this.longType, this.longType, this.booleanType, 148, 154);
        enterBinop("!=", this.intType, this.intType, this.booleanType, ByteCodes.if_icmpne);
        enterBinop("&&", this.booleanType, this.booleanType, this.booleanType, 258);
        enterBinop("||", this.booleanType, this.booleanType, this.booleanType, 259);
    }
}
