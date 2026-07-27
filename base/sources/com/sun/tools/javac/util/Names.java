package com.sun.tools.javac.util;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Name;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes.dex */
public class Names {
    public static final Context.Key<Names> namesKey = new Context.Key<>();
    public final Name ANNOTATION_TYPE;
    public final Name Annotation;
    public final Name AnnotationDefault;
    public final Name Array;
    public final Name BootstrapMethods;
    public final Name Bound;
    public final Name Bridge;
    public final Name CLASS;
    public final Name CONSTRUCTOR;
    public final Name CharacterRangeTable;
    public final Name Code;
    public final Name CompilationID;
    public final Name ConstantValue;
    public final Name Deprecated;
    public final Name EnclosingMethod;
    public final Name Enum;
    public final Name Exceptions;
    public final Name FIELD;
    public final Name InnerClasses;
    public final Name LOCAL_VARIABLE;
    public final Name LineNumberTable;
    public final Name LocalVariableTable;
    public final Name LocalVariableTypeTable;
    public final Name METHOD;
    public final Name Method;
    public final Name MethodParameters;
    public final Name PACKAGE;
    public final Name PARAMETER;
    public final Name RUNTIME;
    public final Name RuntimeInvisibleAnnotations;
    public final Name RuntimeInvisibleParameterAnnotations;
    public final Name RuntimeInvisibleTypeAnnotations;
    public final Name RuntimeVisibleAnnotations;
    public final Name RuntimeVisibleParameterAnnotations;
    public final Name RuntimeVisibleTypeAnnotations;
    public final Name SOURCE;
    public final Name Signature;
    public final Name SourceFile;
    public final Name SourceID;
    public final Name StackMap;
    public final Name StackMapTable;
    public final Name Synthetic;
    public final Name T;
    public final Name TYPE;
    public final Name TYPE_PARAMETER;
    public final Name TYPE_USE;
    public final Name Value;
    public final Name Varargs;
    public final Name _class;
    public final Name _default;
    public final Name _name;
    public final Name _super;
    public final Name _this;
    public final Name addSuppressed;
    public final Name altMetafactory;
    public final Name any;
    public final Name append;
    public final Name asterisk;
    public final Name clinit;
    public final Name clone;
    public final Name close;
    public final Name comma;
    public final Name compareTo;
    public final Name deprecated;
    public final Name deserializeLambda;
    public final Name desiredAssertionStatus;
    public final Name dollarThis;
    public final Name empty;
    public final Name equals;
    public final Name error;
    public final Name ex;
    public final Name family;
    public final Name finalize;
    public final Name forName;
    public final Name getClass;
    public final Name getClassLoader;
    public final Name getComponentType;
    public final Name getDeclaringClass;
    public final Name getMessage;
    public final Name hasNext;
    public final Name hashCode;
    public final Name hyphen;
    public final Name init;
    public final Name initCause;
    public final Name iterator;
    public final Name java_io_Serializable;
    public final Name java_lang;
    public final Name java_lang_AutoCloseable;
    public final Name java_lang_Class;
    public final Name java_lang_Cloneable;
    public final Name java_lang_Enum;
    public final Name java_lang_Object;
    public final Name java_lang_invoke_MethodHandle;
    public final Name lambda;
    public final Name length;
    public final Name metafactory;
    public final Name next;
    public final Name one;
    public final Name ordinal;
    public final Name package_info;
    public final Name period;
    public final Name semicolon;
    public final Name serialVersionUID;
    public final Name slash;
    public final Name slashequals;
    public final Name.Table table;
    public final Name toString;
    public final Name value;
    public final Name valueOf;
    public final Name values;

    public static Names instance(Context context) {
        Names instance = (Names) context.get(namesKey);
        if (instance == null) {
            Names instance2 = new Names(context);
            context.put(namesKey, instance2);
            return instance2;
        }
        return instance;
    }

    public Names(Context context) {
        Options options = Options.instance(context);
        this.table = createTable(options);
        this.asterisk = fromString(Marker.ANY_MARKER);
        this.comma = fromString(DocLint.TAGS_SEPARATOR);
        this.empty = fromString("");
        this.hyphen = fromString("-");
        this.one = fromString("1");
        this.period = fromString(".");
        this.semicolon = fromString(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        this.slash = fromString(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
        this.slashequals = fromString("/=");
        this._class = fromString("class");
        this._default = fromString("default");
        this._super = fromString("super");
        this._this = fromString("this");
        this._name = fromString("name");
        this.addSuppressed = fromString("addSuppressed");
        this.any = fromString("<any>");
        this.append = fromString("append");
        this.clinit = fromString("<clinit>");
        this.clone = fromString("clone");
        this.close = fromString("close");
        this.compareTo = fromString("compareTo");
        this.deserializeLambda = fromString("$deserializeLambda$");
        this.desiredAssertionStatus = fromString("desiredAssertionStatus");
        this.equals = fromString("equals");
        this.error = fromString("<error>");
        this.family = fromString("family");
        this.finalize = fromString("finalize");
        this.forName = fromString("forName");
        this.getClass = fromString("getClass");
        this.getClassLoader = fromString("getClassLoader");
        this.getComponentType = fromString("getComponentType");
        this.getDeclaringClass = fromString("getDeclaringClass");
        this.getMessage = fromString("getMessage");
        this.hasNext = fromString("hasNext");
        this.hashCode = fromString("hashCode");
        this.init = fromString("<init>");
        this.initCause = fromString("initCause");
        this.iterator = fromString("iterator");
        this.length = fromString("length");
        this.next = fromString("next");
        this.ordinal = fromString("ordinal");
        this.serialVersionUID = fromString("serialVersionUID");
        this.toString = fromString("toString");
        this.value = fromString("value");
        this.valueOf = fromString("valueOf");
        this.values = fromString("values");
        this.dollarThis = fromString("$this");
        this.java_io_Serializable = fromString("java.io.Serializable");
        this.java_lang_AutoCloseable = fromString("java.lang.AutoCloseable");
        this.java_lang_Class = fromString("java.lang.Class");
        this.java_lang_Cloneable = fromString("java.lang.Cloneable");
        this.java_lang_Enum = fromString("java.lang.Enum");
        this.java_lang_Object = fromString("java.lang.Object");
        this.java_lang_invoke_MethodHandle = fromString("java.lang.invoke.MethodHandle");
        this.Array = fromString("Array");
        this.Bound = fromString("Bound");
        this.Method = fromString("Method");
        this.java_lang = fromString("java.lang");
        this.Annotation = fromString("Annotation");
        this.AnnotationDefault = fromString("AnnotationDefault");
        this.BootstrapMethods = fromString("BootstrapMethods");
        this.Bridge = fromString("Bridge");
        this.CharacterRangeTable = fromString("CharacterRangeTable");
        this.Code = fromString("Code");
        this.CompilationID = fromString("CompilationID");
        this.ConstantValue = fromString("ConstantValue");
        this.Deprecated = fromString("Deprecated");
        this.EnclosingMethod = fromString("EnclosingMethod");
        this.Enum = fromString("Enum");
        this.Exceptions = fromString("Exceptions");
        this.InnerClasses = fromString("InnerClasses");
        this.LineNumberTable = fromString("LineNumberTable");
        this.LocalVariableTable = fromString("LocalVariableTable");
        this.LocalVariableTypeTable = fromString("LocalVariableTypeTable");
        this.MethodParameters = fromString("MethodParameters");
        this.RuntimeInvisibleAnnotations = fromString("RuntimeInvisibleAnnotations");
        this.RuntimeInvisibleParameterAnnotations = fromString("RuntimeInvisibleParameterAnnotations");
        this.RuntimeInvisibleTypeAnnotations = fromString("RuntimeInvisibleTypeAnnotations");
        this.RuntimeVisibleAnnotations = fromString("RuntimeVisibleAnnotations");
        this.RuntimeVisibleParameterAnnotations = fromString("RuntimeVisibleParameterAnnotations");
        this.RuntimeVisibleTypeAnnotations = fromString("RuntimeVisibleTypeAnnotations");
        this.Signature = fromString("Signature");
        this.SourceFile = fromString("SourceFile");
        this.SourceID = fromString("SourceID");
        this.StackMap = fromString("StackMap");
        this.StackMapTable = fromString("StackMapTable");
        this.Synthetic = fromString("Synthetic");
        this.Value = fromString("Value");
        this.Varargs = fromString("Varargs");
        this.ANNOTATION_TYPE = fromString("ANNOTATION_TYPE");
        this.CONSTRUCTOR = fromString("CONSTRUCTOR");
        this.FIELD = fromString("FIELD");
        this.LOCAL_VARIABLE = fromString("LOCAL_VARIABLE");
        this.METHOD = fromString("METHOD");
        this.PACKAGE = fromString("PACKAGE");
        this.PARAMETER = fromString("PARAMETER");
        this.TYPE = fromString("TYPE");
        this.TYPE_PARAMETER = fromString("TYPE_PARAMETER");
        this.TYPE_USE = fromString("TYPE_USE");
        this.CLASS = fromString("CLASS");
        this.RUNTIME = fromString("RUNTIME");
        this.SOURCE = fromString("SOURCE");
        this.T = fromString("T");
        this.deprecated = fromString("deprecated");
        this.ex = fromString("ex");
        this.package_info = fromString("package-info");
        this.lambda = fromString("lambda$");
        this.metafactory = fromString("metafactory");
        this.altMetafactory = fromString("altMetafactory");
    }

    protected Name.Table createTable(Options options) {
        boolean useUnsharedTable = options.isSet("useUnsharedTable");
        if (useUnsharedTable) {
            return new UnsharedNameTable(this);
        }
        return new SharedNameTable(this);
    }

    public void dispose() {
        this.table.dispose();
    }

    public Name fromChars(char[] cs, int start, int len) {
        return this.table.fromChars(cs, start, len);
    }

    public Name fromString(String s) {
        return this.table.fromString(s);
    }

    public Name fromUtf(byte[] cs) {
        return this.table.fromUtf(cs);
    }

    public Name fromUtf(byte[] cs, int start, int len) {
        return this.table.fromUtf(cs, start, len);
    }
}
