package com.sun.tools.javac.code;

import com.sun.tools.javac.api.Messages;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import dk.sgjesse.r8api.DescriptorUtils;
import java.util.Locale;

/* JADX INFO: loaded from: classes.dex */
public abstract class Printer implements Type.Visitor<String, Locale>, Symbol.Visitor<String, Locale> {
    static final int PRIME = 997;
    List<Type> seenCaptured = List.nil();

    protected abstract String capturedVarId(Type.CapturedType capturedType, Locale locale);

    protected abstract String localize(Locale locale, String str, Object... objArr);

    protected Printer() {
    }

    public static Printer createStandardPrinter(final Messages messages) {
        return new Printer() { // from class: com.sun.tools.javac.code.Printer.1
            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitAnnotatedType(Type.AnnotatedType annotatedType, Locale locale) {
                return super.visitAnnotatedType(annotatedType, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitArrayType(Type.ArrayType arrayType, Locale locale) {
                return super.visitArrayType(arrayType, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitCapturedType(Type.CapturedType capturedType, Locale locale) {
                return super.visitCapturedType(capturedType, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Symbol.Visitor
            public /* bridge */ /* synthetic */ String visitClassSymbol(Symbol.ClassSymbol classSymbol, Locale locale) {
                return super.visitClassSymbol(classSymbol, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitClassType(Type.ClassType classType, Locale locale) {
                return super.visitClassType(classType, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitErrorType(Type.ErrorType errorType, Locale locale) {
                return super.visitErrorType(errorType, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitForAll(Type.ForAll forAll, Locale locale) {
                return super.visitForAll(forAll, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Symbol.Visitor
            public /* bridge */ /* synthetic */ String visitMethodSymbol(Symbol.MethodSymbol methodSymbol, Locale locale) {
                return super.visitMethodSymbol(methodSymbol, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitMethodType(Type.MethodType methodType, Locale locale) {
                return super.visitMethodType(methodType, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Symbol.Visitor
            public /* bridge */ /* synthetic */ String visitOperatorSymbol(Symbol.OperatorSymbol operatorSymbol, Locale locale) {
                return super.visitOperatorSymbol(operatorSymbol, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Symbol.Visitor
            public /* bridge */ /* synthetic */ String visitPackageSymbol(Symbol.PackageSymbol packageSymbol, Locale locale) {
                return super.visitPackageSymbol(packageSymbol, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitPackageType(Type.PackageType packageType, Locale locale) {
                return super.visitPackageType(packageType, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Symbol.Visitor
            public /* bridge */ /* synthetic */ String visitSymbol(Symbol symbol, Locale locale) {
                return super.visitSymbol(symbol, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitType(Type type, Locale locale) {
                return super.visitType(type, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Symbol.Visitor
            public /* bridge */ /* synthetic */ String visitTypeSymbol(Symbol.TypeSymbol typeSymbol, Locale locale) {
                return super.visitTypeSymbol(typeSymbol, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitTypeVar(Type.TypeVar typeVar, Locale locale) {
                return super.visitTypeVar(typeVar, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitUndetVar(Type.UndetVar undetVar, Locale locale) {
                return super.visitUndetVar(undetVar, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Symbol.Visitor
            public /* bridge */ /* synthetic */ String visitVarSymbol(Symbol.VarSymbol varSymbol, Locale locale) {
                return super.visitVarSymbol(varSymbol, locale);
            }

            @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
            public /* bridge */ /* synthetic */ String visitWildcardType(Type.WildcardType wildcardType, Locale locale) {
                return super.visitWildcardType(wildcardType, locale);
            }

            @Override // com.sun.tools.javac.code.Printer
            protected String localize(Locale locale, String key, Object... args) {
                return messages.getLocalizedString(locale, key, args);
            }

            @Override // com.sun.tools.javac.code.Printer
            protected String capturedVarId(Type.CapturedType t, Locale locale) {
                return ((((long) t.hashCode()) & 4294967295L) % 997) + "";
            }
        };
    }

    public String visitTypes(List<Type> ts, Locale locale) {
        ListBuffer<String> sbuf = new ListBuffer<>();
        for (Type t : ts) {
            sbuf.append(visit(t, locale));
        }
        return sbuf.toList().toString();
    }

    public String visitSymbols(List<Symbol> ts, Locale locale) {
        ListBuffer<String> sbuf = new ListBuffer<>();
        for (Symbol t : ts) {
            sbuf.append(visit(t, locale));
        }
        return sbuf.toList().toString();
    }

    public String visit(Type t, Locale locale) {
        return (String) t.accept(this, locale);
    }

    public String visit(Symbol s, Locale locale) {
        return (String) s.accept(this, locale);
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitCapturedType(Type.CapturedType t, Locale locale) {
        if (this.seenCaptured.contains(t)) {
            return localize(locale, "compiler.misc.type.captureof.1", capturedVarId(t, locale));
        }
        try {
            this.seenCaptured = this.seenCaptured.prepend(t);
            return localize(locale, "compiler.misc.type.captureof", capturedVarId(t, locale), visit(t.wildcard, locale));
        } finally {
            this.seenCaptured = this.seenCaptured.tail;
        }
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitForAll(Type.ForAll t, Locale locale) {
        return "<" + visitTypes(t.tvars, locale) + ">" + visit(t.qtype, locale);
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitUndetVar(Type.UndetVar t, Locale locale) {
        if (t.inst != null) {
            return visit(t.inst, locale);
        }
        return visit(t.qtype, locale) + "?";
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitArrayType(Type.ArrayType t, Locale locale) {
        StringBuilder res = new StringBuilder();
        printBaseElementType(t, res, locale);
        printBrackets(t, res, locale);
        return res.toString();
    }

    void printBaseElementType(Type t, StringBuilder sb, Locale locale) {
        Type arrel = t;
        while (arrel.hasTag(TypeTag.ARRAY)) {
            arrel = ((Type.ArrayType) arrel.unannotatedType()).elemtype;
        }
        sb.append(visit(arrel, locale));
    }

    void printBrackets(Type t, StringBuilder sb, Locale locale) {
        for (Type arrel = t; arrel.hasTag(TypeTag.ARRAY); arrel = ((Type.ArrayType) arrel.unannotatedType()).elemtype) {
            if (arrel.isAnnotated()) {
                sb.append(' ');
                sb.append(arrel.getAnnotationMirrors());
                sb.append(' ');
            }
            sb.append("[]");
        }
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitClassType(Type.ClassType t, Locale locale) {
        StringBuilder buf = new StringBuilder();
        if (t.getEnclosingType().hasTag(TypeTag.CLASS) && t.tsym.owner.kind == 2) {
            buf.append(visit(t.getEnclosingType(), locale));
            buf.append(DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
            buf.append(className(t, false, locale));
        } else {
            buf.append(className(t, true, locale));
        }
        if (t.getTypeArguments().nonEmpty()) {
            buf.append('<');
            buf.append(visitTypes(t.getTypeArguments(), locale));
            buf.append('>');
        }
        return buf.toString();
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitMethodType(Type.MethodType t, Locale locale) {
        return "(" + printMethodArgs(t.argtypes, false, locale) + ")" + visit(t.restype, locale);
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitPackageType(Type.PackageType t, Locale locale) {
        return t.tsym.getQualifiedName().toString();
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitWildcardType(Type.WildcardType t, Locale locale) {
        StringBuilder s = new StringBuilder();
        s.append(t.kind);
        if (t.kind != BoundKind.UNBOUND) {
            s.append(visit(t.type, locale));
        }
        return s.toString();
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitErrorType(Type.ErrorType t, Locale locale) {
        return visitType((Type) t, locale);
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitTypeVar(Type.TypeVar t, Locale locale) {
        return visitType((Type) t, locale);
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitAnnotatedType(Type.AnnotatedType t, Locale locale) {
        if (t.getAnnotationMirrors().nonEmpty()) {
            if (t.unannotatedType().hasTag(TypeTag.ARRAY)) {
                StringBuilder res = new StringBuilder();
                printBaseElementType(t, res, locale);
                printBrackets(t, res, locale);
                return res.toString();
            }
            if (t.unannotatedType().hasTag(TypeTag.CLASS) && t.unannotatedType().getEnclosingType() != Type.noType) {
                return visit(t.unannotatedType().getEnclosingType(), locale) + ". " + t.getAnnotationMirrors() + " " + className((Type.ClassType) t.unannotatedType(), false, locale);
            }
            return t.getAnnotationMirrors() + " " + visit(t.unannotatedType(), locale);
        }
        return visit(t.unannotatedType(), locale);
    }

    @Override // com.sun.tools.javac.code.Type.Visitor
    public String visitType(Type t, Locale locale) {
        if (t.tsym == null || t.tsym.name == null) {
            String s = localize(locale, "compiler.misc.type.none", new Object[0]);
            return s;
        }
        String s2 = t.tsym.name.toString();
        return s2;
    }

    /* JADX WARN: Multi-variable type inference failed */
    protected String className(Type.ClassType t, boolean longform, Locale locale) {
        Symbol sym = t.tsym;
        if (sym.name.length() == 0 && (sym.flags() & 16777216) != 0) {
            StringBuilder s = new StringBuilder(visit(t.supertype_field, locale));
            for (List list = t.interfaces_field; list.nonEmpty(); list = list.tail) {
                s.append('&');
                s.append(visit((Type) list.head, locale));
            }
            return s.toString();
        }
        if (sym.name.length() == 0) {
            Type.ClassType norm = (Type.ClassType) t.tsym.type;
            if (norm == null) {
                String s2 = localize(locale, "compiler.misc.anonymous.class", null);
                return s2;
            }
            if (norm.interfaces_field != null && norm.interfaces_field.nonEmpty()) {
                String s3 = localize(locale, "compiler.misc.anonymous.class", visit(norm.interfaces_field.head, locale));
                return s3;
            }
            String s4 = localize(locale, "compiler.misc.anonymous.class", visit(norm.supertype_field, locale));
            return s4;
        }
        if (longform) {
            return sym.getQualifiedName().toString();
        }
        return sym.name.toString();
    }

    /* JADX WARN: Multi-variable type inference failed */
    protected String printMethodArgs(List<Type> list, boolean z, Locale locale) {
        if (!z) {
            return visitTypes(list, locale);
        }
        StringBuilder sb = new StringBuilder();
        List list2 = list;
        while (list2.tail.nonEmpty()) {
            sb.append(visit((Type) list2.head, locale));
            List list3 = list2.tail;
            sb.append(',');
            list2 = list3;
        }
        if (((Type) list2.head).unannotatedType().hasTag(TypeTag.ARRAY)) {
            sb.append(visit(((Type.ArrayType) ((Type) list2.head).unannotatedType()).elemtype, locale));
            if (((Type) list2.head).getAnnotationMirrors().nonEmpty()) {
                sb.append(' ');
                sb.append(((Type) list2.head).getAnnotationMirrors());
                sb.append(' ');
            }
            sb.append("...");
        } else {
            sb.append(visit((Type) list2.head, locale));
        }
        return sb.toString();
    }

    @Override // com.sun.tools.javac.code.Symbol.Visitor
    public String visitClassSymbol(Symbol.ClassSymbol sym, Locale locale) {
        if (sym.name.isEmpty()) {
            return localize(locale, "compiler.misc.anonymous.class", sym.flatname);
        }
        return sym.fullname.toString();
    }

    @Override // com.sun.tools.javac.code.Symbol.Visitor
    public String visitMethodSymbol(Symbol.MethodSymbol s, Locale locale) {
        String ms;
        if (s.isStaticOrInstanceInit()) {
            return s.owner.name.toString();
        }
        if (s.name == s.name.table.names.init) {
            ms = s.owner.name.toString();
        } else {
            ms = s.name.toString();
        }
        if (s.type != null) {
            if (s.type.hasTag(TypeTag.FORALL)) {
                ms = "<" + visitTypes(s.type.getTypeArguments(), locale) + ">" + ms;
            }
            return ms + "(" + printMethodArgs(s.type.mo176getParameterTypes(), (s.flags() & Flags.VARARGS) != 0, locale) + ")";
        }
        return ms;
    }

    @Override // com.sun.tools.javac.code.Symbol.Visitor
    public String visitOperatorSymbol(Symbol.OperatorSymbol s, Locale locale) {
        return visitMethodSymbol((Symbol.MethodSymbol) s, locale);
    }

    @Override // com.sun.tools.javac.code.Symbol.Visitor
    public String visitPackageSymbol(Symbol.PackageSymbol s, Locale locale) {
        if (s.isUnnamed()) {
            return localize(locale, "compiler.misc.unnamed.package", new Object[0]);
        }
        return s.fullname.toString();
    }

    @Override // com.sun.tools.javac.code.Symbol.Visitor
    public String visitTypeSymbol(Symbol.TypeSymbol s, Locale locale) {
        return visitSymbol((Symbol) s, locale);
    }

    @Override // com.sun.tools.javac.code.Symbol.Visitor
    public String visitVarSymbol(Symbol.VarSymbol s, Locale locale) {
        return visitSymbol((Symbol) s, locale);
    }

    @Override // com.sun.tools.javac.code.Symbol.Visitor
    public String visitSymbol(Symbol s, Locale locale) {
        return s.name.toString();
    }
}
