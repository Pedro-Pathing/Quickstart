package com.sun.tools.javac.jvm;

import com.android.tools.r8.DataResource;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.model.JavacElements;
import com.sun.tools.javac.model.JavacTypes;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Options;
import dk.sgjesse.r8api.DescriptorUtils;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Stack;
import javax.lang.model.element.Element;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.element.Name;
import javax.lang.model.element.TypeElement;
import javax.lang.model.element.VariableElement;
import javax.lang.model.type.ArrayType;
import javax.lang.model.type.DeclaredType;
import javax.lang.model.type.NoType;
import javax.lang.model.type.PrimitiveType;
import javax.lang.model.type.TypeKind;
import javax.lang.model.type.TypeMirror;
import javax.lang.model.type.TypeVariable;
import javax.lang.model.type.TypeVisitor;
import javax.lang.model.util.ElementFilter;
import javax.lang.model.util.Elements;
import javax.lang.model.util.SimpleTypeVisitor8;
import javax.lang.model.util.Types;
import javax.tools.FileObject;
import javax.tools.JavaFileManager;
import javax.tools.StandardLocation;
import org.firstinspires.ftc.onbotjava.RequestConditions;

/* JADX INFO: loaded from: classes.dex */
public class JNIWriter {
    protected static final Context.Key<JNIWriter> jniWriterKey = new Context.Key<>();
    private boolean checkAll;
    private Context context;
    JavacElements elements;
    private final JavaFileManager fileManager;
    private final boolean isWindows = System.getProperty("os.name").startsWith("Windows");
    private String lineSep;
    private final Log log;
    private Mangle mangler;
    private Symtab syms;
    JavacTypes types;
    private boolean verbose;

    public static JNIWriter instance(Context context) {
        JNIWriter instance = (JNIWriter) context.get(jniWriterKey);
        if (instance == null) {
            return new JNIWriter(context);
        }
        return instance;
    }

    private JNIWriter(Context context) {
        context.put(jniWriterKey, this);
        this.fileManager = (JavaFileManager) context.get(JavaFileManager.class);
        this.log = Log.instance(context);
        Options options = Options.instance(context);
        this.verbose = options.isSet(Option.VERBOSE);
        this.checkAll = options.isSet("javah:full");
        this.context = context;
        this.syms = Symtab.instance(context);
        this.lineSep = System.getProperty("line.separator");
    }

    private void lazyInit() {
        if (this.mangler == null) {
            this.elements = JavacElements.instance(this.context);
            this.types = JavacTypes.instance(this.context);
            this.mangler = new Mangle(this.elements, this.types);
        }
    }

    public boolean needsHeader(Symbol.ClassSymbol c) {
        if (c.isLocal() || (c.flags() & 4096) != 0) {
            return false;
        }
        if (this.checkAll) {
            return needsHeader(c.outermostClass(), true);
        }
        return needsHeader(c, false);
    }

    private boolean needsHeader(Symbol.ClassSymbol c, boolean checkNestedClasses) {
        if (c.isLocal() || (c.flags() & 4096) != 0) {
            return false;
        }
        for (Scope.Entry i = c.members_field.elems; i != null; i = i.sibling) {
            if (i.sym.kind == 16 && (i.sym.flags() & 256) != 0) {
                return true;
            }
            for (Attribute.Compound a : i.sym.getDeclarationAttributes()) {
                if (a.type.tsym == this.syms.nativeHeaderType.tsym) {
                    return true;
                }
            }
        }
        if (checkNestedClasses) {
            for (Scope.Entry i2 = c.members_field.elems; i2 != null; i2 = i2.sibling) {
                if (i2.sym.kind == 2 && needsHeader((Symbol.ClassSymbol) i2.sym, true)) {
                    return true;
                }
            }
        }
        return false;
    }

    public FileObject write(Symbol.ClassSymbol c) throws IOException {
        String className = c.flatName().toString();
        FileObject outFile = this.fileManager.getFileForOutput(StandardLocation.NATIVE_HEADER_OUTPUT, "", className.replaceAll("[.$]", "_") + ".h", null);
        Writer out = outFile.openWriter();
        try {
            write(out, c);
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

    public void write(Writer out, Symbol.ClassSymbol sym) throws IOException {
        lazyInit();
        try {
            String cname = this.mangler.mangle(sym.fullname, 1);
            println(out, fileTop());
            println(out, includes());
            println(out, guardBegin(cname));
            println(out, cppGuardBegin());
            writeStatics(out, sym);
            writeMethods(out, sym, cname);
            println(out, cppGuardEnd());
            println(out, guardEnd(cname));
        } catch (TypeSignature.SignatureException e) {
            throw new IOException(e);
        }
    }

    protected void writeStatics(Writer out, Symbol.ClassSymbol sym) throws IOException {
        String s;
        List<VariableElement> classfields = getAllFields(sym);
        for (VariableElement v : classfields) {
            if (v.getModifiers().contains(Modifier.STATIC) && (s = defineForStatic(sym, v)) != null) {
                println(out, s);
            }
        }
    }

    List<VariableElement> getAllFields(TypeElement subclazz) {
        List<VariableElement> fields = new ArrayList<>();
        Stack<TypeElement> s = new Stack<>();
        TypeElement cd = subclazz;
        while (true) {
            s.push(cd);
            TypeElement c = (TypeElement) this.types.asElement(cd.getSuperclass());
            if (c == null) {
                break;
            }
            cd = c;
        }
        while (!s.empty()) {
            TypeElement cd2 = s.pop();
            fields.addAll(ElementFilter.fieldsIn(cd2.getEnclosedElements()));
        }
        return fields;
    }

    protected String defineForStatic(TypeElement c, VariableElement f) {
        Object value;
        CharSequence cnamedoc = c.getQualifiedName();
        CharSequence fnamedoc = f.getSimpleName();
        String cname = this.mangler.mangle(cnamedoc, 1);
        String fname = this.mangler.mangle(fnamedoc, 2);
        Assert.check(f.getModifiers().contains(Modifier.STATIC));
        if (f.getModifiers().contains(Modifier.FINAL) && (value = f.getConstantValue()) != null) {
            String constString = null;
            if ((value instanceof Integer) || (value instanceof Byte) || (value instanceof Short)) {
                constString = value.toString() + "L";
            } else if (value instanceof Boolean) {
                constString = ((Boolean) value).booleanValue() ? "1L" : "0L";
            } else if (value instanceof Character) {
                Character ch = (Character) value;
                constString = String.valueOf(ch.charValue() & 65535) + "L";
            } else if (value instanceof Long) {
                if (this.isWindows) {
                    constString = value.toString() + "i64";
                } else {
                    constString = value.toString() + "LL";
                }
            } else {
                if (value instanceof Float) {
                    float fv = ((Float) value).floatValue();
                    if (Float.isInfinite(fv)) {
                        constString = (fv >= 0.0f ? "" : "-") + "Inff";
                    } else {
                        constString = value.toString() + RequestConditions.REQUEST_KEY_FILE;
                    }
                } else if (value instanceof Double) {
                    double d = ((Double) value).doubleValue();
                    if (Double.isInfinite(d)) {
                        constString = (d >= LynxServoController.apiPositionFirst ? "" : "-") + "InfD";
                    } else {
                        constString = value.toString();
                    }
                }
            }
            if (constString != null) {
                return "#undef " + cname + "_" + fname + this.lineSep + "#define " + cname + "_" + fname + " " + constString;
            }
            return null;
        }
        return null;
    }

    protected void writeMethods(Writer out, Symbol.ClassSymbol sym, String cname) throws IOException, TypeSignature.SignatureException {
        List<ExecutableElement> classmethods;
        Iterator<ExecutableElement> it;
        List<ExecutableElement> classmethods2 = ElementFilter.methodsIn(sym.getEnclosedElements());
        Iterator<ExecutableElement> it2 = classmethods2.iterator();
        while (it2.hasNext()) {
            ExecutableElement md = it2.next();
            if (md.getModifiers().contains(Modifier.NATIVE)) {
                TypeMirror mtr = this.types.erasure(md.getReturnType());
                String sig = signature(md);
                TypeSignature newtypesig = new TypeSignature(this.elements);
                CharSequence methodName = md.getSimpleName();
                boolean longName = false;
                for (ExecutableElement md2 : classmethods2) {
                    if (md2 != md && methodName.equals(md2.getSimpleName()) && md2.getModifiers().contains(Modifier.NATIVE)) {
                        longName = true;
                    }
                }
                println(out, "/*");
                println(out, " * Class:     " + cname);
                println(out, " * Method:    " + this.mangler.mangle(methodName, 2));
                println(out, " * Signature: " + newtypesig.getTypeSignature(sig, mtr));
                println(out, " */");
                println(out, "JNIEXPORT " + jniType(mtr) + " JNICALL " + this.mangler.mangleMethod(md, sym, longName ? 8 : 7));
                print(out, "  (JNIEnv *, ");
                List<? extends VariableElement> paramargs = md.getParameters();
                List<TypeMirror> args = new ArrayList<>();
                for (VariableElement p : paramargs) {
                    args.add(this.types.erasure(p.asType()));
                    classmethods2 = classmethods2;
                    it2 = it2;
                }
                classmethods = classmethods2;
                it = it2;
                if (md.getModifiers().contains(Modifier.STATIC)) {
                    print(out, "jclass");
                } else {
                    print(out, "jobject");
                }
                for (TypeMirror arg : args) {
                    print(out, ", ");
                    print(out, jniType(arg));
                }
                println(out, ");" + this.lineSep);
            } else {
                classmethods = classmethods2;
                it = it2;
            }
            classmethods2 = classmethods;
            it2 = it;
        }
    }

    String signature(ExecutableElement e) {
        StringBuilder sb = new StringBuilder("(");
        String sep = "";
        for (VariableElement p : e.getParameters()) {
            sb.append(sep);
            sb.append(this.types.erasure(p.asType()).toString());
            sep = DocLint.TAGS_SEPARATOR;
        }
        sb.append(")");
        return sb.toString();
    }

    protected final String jniType(TypeMirror t) {
        TypeElement throwable = this.elements.getTypeElement((CharSequence) "java.lang.Throwable");
        TypeElement jClass = this.elements.getTypeElement((CharSequence) "java.lang.Class");
        TypeElement jString = this.elements.getTypeElement((CharSequence) "java.lang.String");
        Element tclassDoc = this.types.asElement(t);
        switch (t.getKind()) {
            case BOOLEAN:
                return "jboolean";
            case BYTE:
                return "jbyte";
            case CHAR:
                return "jchar";
            case SHORT:
                return "jshort";
            case INT:
                return "jint";
            case LONG:
                return "jlong";
            case FLOAT:
                return "jfloat";
            case DOUBLE:
                return "jdouble";
            case ARRAY:
                TypeMirror ct = ((ArrayType) t).getComponentType();
                switch (ct.getKind()) {
                    case BOOLEAN:
                        return "jbooleanArray";
                    case BYTE:
                        return "jbyteArray";
                    case CHAR:
                        return "jcharArray";
                    case SHORT:
                        return "jshortArray";
                    case INT:
                        return "jintArray";
                    case LONG:
                        return "jlongArray";
                    case FLOAT:
                        return "jfloatArray";
                    case DOUBLE:
                        return "jdoubleArray";
                    case ARRAY:
                    case DECLARED:
                        return "jobjectArray";
                    default:
                        throw new Error(ct.toString());
                }
            case DECLARED:
                if (tclassDoc.equals(jString)) {
                    return "jstring";
                }
                if (this.types.isAssignable(t, throwable.asType())) {
                    return "jthrowable";
                }
                if (this.types.isAssignable(t, jClass.asType())) {
                    return "jclass";
                }
                return "jobject";
            case VOID:
                return "void";
            default:
                Assert.check(false, "jni unknown type");
                return null;
        }
    }

    protected String fileTop() {
        return "/* DO NOT EDIT THIS FILE - it is machine generated */";
    }

    protected String includes() {
        return "#include <jni.h>";
    }

    protected String cppGuardBegin() {
        return "#ifdef __cplusplus" + this.lineSep + "extern \"C\" {" + this.lineSep + "#endif";
    }

    protected String cppGuardEnd() {
        return "#ifdef __cplusplus" + this.lineSep + "}" + this.lineSep + "#endif";
    }

    protected String guardBegin(String cname) {
        return "/* Header for class " + cname + " */" + this.lineSep + this.lineSep + "#ifndef _Included_" + cname + this.lineSep + "#define _Included_" + cname;
    }

    protected String guardEnd(String cname) {
        return "#endif";
    }

    protected void print(Writer out, String text) throws IOException {
        out.write(text);
    }

    protected void println(Writer out, String text) throws IOException {
        out.write(text);
        out.write(this.lineSep);
    }

    private static class Mangle {
        private Elements elems;
        private Types types;

        public static class Type {
            public static final int CLASS = 1;
            public static final int FIELD = 3;
            public static final int FIELDSTUB = 2;
            public static final int JNI = 4;
            public static final int METHOD_JDK_1 = 6;
            public static final int METHOD_JNI_LONG = 8;
            public static final int METHOD_JNI_SHORT = 7;
            public static final int SIGNATURE = 5;
        }

        Mangle(Elements elems, Types types) {
            this.elems = elems;
            this.types = types;
        }

        public final String mangle(CharSequence name, int mtype) {
            StringBuilder result = new StringBuilder(100);
            int length = name.length();
            for (int i = 0; i < length; i++) {
                char ch = name.charAt(i);
                if (isalnum(ch)) {
                    result.append(ch);
                } else if (ch == '.' && mtype == 1) {
                    result.append('_');
                } else if (ch == '$' && mtype == 1) {
                    result.append('_');
                    result.append('_');
                } else if (ch == '_' && mtype == 2) {
                    result.append('_');
                } else if (ch == '_' && mtype == 1) {
                    result.append('_');
                } else if (mtype == 4) {
                    String esc = null;
                    if (ch == '_') {
                        esc = "_1";
                    } else if (ch == '.') {
                        esc = "_";
                    } else if (ch == ';') {
                        esc = "_2";
                    } else if (ch == '[') {
                        esc = "_3";
                    }
                    if (esc != null) {
                        result.append(esc);
                    } else {
                        result.append(mangleChar(ch));
                    }
                } else if (mtype == 5) {
                    if (isprint(ch)) {
                        result.append(ch);
                    } else {
                        result.append(mangleChar(ch));
                    }
                } else {
                    result.append(mangleChar(ch));
                }
            }
            return result.toString();
        }

        public String mangleMethod(ExecutableElement method, TypeElement clazz, int mtype) throws TypeSignature.SignatureException {
            StringBuilder result = new StringBuilder(100);
            result.append("Java_");
            if (mtype == 6) {
                result.append(mangle(clazz.getQualifiedName(), 1));
                result.append('_');
                result.append(mangle(method.getSimpleName(), 3));
                result.append("_stub");
                return result.toString();
            }
            result.append(mangle(getInnerQualifiedName(clazz), 4));
            result.append('_');
            result.append(mangle(method.getSimpleName(), 4));
            if (mtype == 8) {
                result.append("__");
                String typesig = signature(method);
                TypeSignature newTypeSig = new TypeSignature(this.elems);
                String sig = newTypeSig.getTypeSignature(typesig, method.getReturnType()).substring(1);
                result.append(mangle(sig.substring(0, sig.lastIndexOf(41)).replace(DataResource.SEPARATOR, DescriptorUtils.JAVA_PACKAGE_SEPARATOR), 4));
            }
            String typesig2 = result.toString();
            return typesig2;
        }

        private String getInnerQualifiedName(TypeElement clazz) {
            return this.elems.getBinaryName(clazz).toString();
        }

        public final String mangleChar(char ch) {
            String s = Integer.toHexString(ch);
            int nzeros = 5 - s.length();
            char[] result = new char[6];
            result[0] = '_';
            for (int i = 1; i <= nzeros; i++) {
                result[i] = '0';
            }
            int i2 = nzeros + 1;
            int j = 0;
            while (i2 < 6) {
                result[i2] = s.charAt(j);
                i2++;
                j++;
            }
            return new String(result);
        }

        private String signature(ExecutableElement e) {
            StringBuilder sb = new StringBuilder();
            String sep = "(";
            for (VariableElement p : e.getParameters()) {
                sb.append(sep);
                sb.append(this.types.erasure(p.asType()).toString());
                sep = DocLint.TAGS_SEPARATOR;
            }
            sb.append(")");
            return sb.toString();
        }

        private static boolean isalnum(char ch) {
            return ch <= 127 && ((ch >= 'A' && ch <= 'Z') || ((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9')));
        }

        private static boolean isprint(char ch) {
            return ch >= ' ' && ch <= '~';
        }
    }

    private static class TypeSignature {
        private static final String SIG_ARRAY = "[";
        private static final String SIG_BOOLEAN = "Z";
        private static final String SIG_BYTE = "B";
        private static final String SIG_CHAR = "C";
        private static final String SIG_CLASS = "L";
        private static final String SIG_DOUBLE = "D";
        private static final String SIG_FLOAT = "F";
        private static final String SIG_INT = "I";
        private static final String SIG_LONG = "J";
        private static final String SIG_SHORT = "S";
        private static final String SIG_VOID = "V";
        Elements elems;

        static class SignatureException extends Exception {
            private static final long serialVersionUID = 1;

            SignatureException(String reason) {
                super(reason);
            }
        }

        public TypeSignature(Elements elems) {
            this.elems = elems;
        }

        public String getTypeSignature(String javasignature) throws SignatureException {
            return getParamJVMSignature(javasignature);
        }

        /* JADX WARN: Removed duplicated region for block: B:17:0x004b  */
        /* JADX WARN: Removed duplicated region for block: B:24:0x006f  */
        /* JADX WARN: Removed duplicated region for block: B:28:0x007e  */
        /* JADX WARN: Removed duplicated region for block: B:34:0x00ba  */
        /* JADX WARN: Removed duplicated region for block: B:37:0x00c2 A[LOOP:2: B:35:0x00be->B:37:0x00c2, LOOP_END] */
        /* JADX WARN: Removed duplicated region for block: B:39:0x00d9  */
        /* JADX WARN: Removed duplicated region for block: B:40:0x00f3  */
        /* JADX WARN: Removed duplicated region for block: B:44:0x00a5 A[EDGE_INSN: B:44:0x00a5->B:32:0x00a5 BREAK  A[LOOP:1: B:26:0x0075->B:47:0x0075], SYNTHETIC] */
        /* JADX WARN: Removed duplicated region for block: B:50:0x00d7 A[EDGE_INSN: B:50:0x00d7->B:38:0x00d7 BREAK  A[LOOP:2: B:35:0x00be->B:37:0x00c2], SYNTHETIC] */
        /*
            Code decompiled incorrectly, please refer to instructions dump.
            To view partially-correct code enable 'Show inconsistent code' option in preferences
        */
        public java.lang.String getTypeSignature(java.lang.String r19, javax.lang.model.type.TypeMirror r20) throws com.sun.tools.javac.jvm.JNIWriter.TypeSignature.SignatureException {
            /*
                Method dump skipped, instruction units count: 268
                To view this dump change 'Code comments level' option to 'DEBUG'
            */
            throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.jvm.JNIWriter.TypeSignature.getTypeSignature(java.lang.String, javax.lang.model.type.TypeMirror):java.lang.String");
        }

        private String getParamJVMSignature(String paramsig) throws SignatureException {
            String componentType;
            String paramJVMSig = "";
            if (paramsig == null) {
                return "";
            }
            if (paramsig.indexOf("[]") != -1) {
                int endindex = paramsig.indexOf("[]");
                componentType = paramsig.substring(0, endindex);
                String dimensionString = paramsig.substring(endindex);
                if (dimensionString != null) {
                    while (dimensionString.indexOf("[]") != -1) {
                        paramJVMSig = paramJVMSig + SIG_ARRAY;
                        int beginindex = dimensionString.indexOf("]") + 1;
                        if (beginindex < dimensionString.length()) {
                            dimensionString = dimensionString.substring(beginindex);
                        } else {
                            dimensionString = "";
                        }
                    }
                }
            } else {
                componentType = paramsig;
            }
            return paramJVMSig + getComponentType(componentType);
        }

        private String getComponentType(String componentType) throws SignatureException {
            if (componentType == null) {
                return "";
            }
            if (componentType.equals("void")) {
                String JVMSig = "" + SIG_VOID;
                return JVMSig;
            }
            if (componentType.equals("boolean")) {
                String JVMSig2 = "" + SIG_BOOLEAN;
                return JVMSig2;
            }
            if (componentType.equals("byte")) {
                String JVMSig3 = "" + SIG_BYTE;
                return JVMSig3;
            }
            if (componentType.equals("char")) {
                String JVMSig4 = "" + SIG_CHAR;
                return JVMSig4;
            }
            if (componentType.equals("short")) {
                String JVMSig5 = "" + SIG_SHORT;
                return JVMSig5;
            }
            if (componentType.equals("int")) {
                String JVMSig6 = "" + SIG_INT;
                return JVMSig6;
            }
            if (componentType.equals("long")) {
                String JVMSig7 = "" + SIG_LONG;
                return JVMSig7;
            }
            if (componentType.equals("float")) {
                String JVMSig8 = "" + SIG_FLOAT;
                return JVMSig8;
            }
            if (componentType.equals("double")) {
                String JVMSig9 = "" + SIG_DOUBLE;
                return JVMSig9;
            }
            if (componentType.equals("")) {
                return "";
            }
            TypeElement classNameDoc = this.elems.getTypeElement(componentType);
            if (classNameDoc == null) {
                throw new SignatureException(componentType);
            }
            String classname = classNameDoc.getQualifiedName().toString();
            String newclassname = classname.replace(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, DataResource.SEPARATOR);
            String JVMSig10 = "" + SIG_CLASS;
            return (JVMSig10 + newclassname) + RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER;
        }

        int dimensions(TypeMirror t) {
            if (t.getKind() != TypeKind.ARRAY) {
                return 0;
            }
            return dimensions(((ArrayType) t).getComponentType()) + 1;
        }

        String qualifiedTypeName(TypeMirror type) {
            TypeVisitor<Name, Void> v = new SimpleTypeVisitor8<Name, Void>() { // from class: com.sun.tools.javac.jvm.JNIWriter.TypeSignature.1
                @Override // javax.lang.model.util.SimpleTypeVisitor6, javax.lang.model.type.TypeVisitor
                public Name visitArray(ArrayType t, Void p) {
                    return (Name) t.getComponentType().accept(this, p);
                }

                @Override // javax.lang.model.util.SimpleTypeVisitor6, javax.lang.model.type.TypeVisitor
                public Name visitDeclared(DeclaredType t, Void p) {
                    return ((TypeElement) t.asElement()).getQualifiedName();
                }

                @Override // javax.lang.model.util.SimpleTypeVisitor6, javax.lang.model.type.TypeVisitor
                public Name visitPrimitive(PrimitiveType t, Void p) {
                    return TypeSignature.this.elems.getName(t.toString());
                }

                @Override // javax.lang.model.util.SimpleTypeVisitor6, javax.lang.model.type.TypeVisitor
                public Name visitNoType(NoType t, Void p) {
                    if (t.getKind() == TypeKind.VOID) {
                        return TypeSignature.this.elems.getName("void");
                    }
                    return defaultAction(t, p);
                }

                @Override // javax.lang.model.util.SimpleTypeVisitor6, javax.lang.model.type.TypeVisitor
                public Name visitTypeVariable(TypeVariable t, Void p) {
                    return (Name) t.getUpperBound().accept(this, p);
                }
            };
            return v.visit(type).toString();
        }
    }
}
