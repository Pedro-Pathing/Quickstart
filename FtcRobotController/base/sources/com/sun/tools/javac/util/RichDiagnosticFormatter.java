package com.sun.tools.javac.util;

import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Kinds;
import com.sun.tools.javac.code.Printer;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Symtab;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.util.ForwardingDiagnosticFormatter;
import com.sun.tools.javac.util.JCDiagnostic;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/* JADX INFO: loaded from: classes.dex */
public class RichDiagnosticFormatter extends ForwardingDiagnosticFormatter<JCDiagnostic, AbstractDiagnosticFormatter> {
    final JCDiagnostic.Factory diags;
    final JavacMessages messages;
    protected ClassNameSimplifier nameSimplifier;
    private RichPrinter printer;
    protected Types.DefaultSymbolVisitor<Void, Void> symbolPreprocessor;
    final Symtab syms;
    protected Types.UnaryVisitor<Void> typePreprocessor;
    final Types types;
    Map<WhereClauseKind, Map<Type, JCDiagnostic>> whereClauses;

    public static RichDiagnosticFormatter instance(Context context) {
        RichDiagnosticFormatter instance = (RichDiagnosticFormatter) context.get(RichDiagnosticFormatter.class);
        if (instance == null) {
            return new RichDiagnosticFormatter(context);
        }
        return instance;
    }

    protected RichDiagnosticFormatter(Context context) {
        super((AbstractDiagnosticFormatter) Log.instance(context).getDiagnosticFormatter());
        this.typePreprocessor = new Types.UnaryVisitor<Void>() { // from class: com.sun.tools.javac.util.RichDiagnosticFormatter.1
            public Void visit(List<Type> ts) {
                for (Type t : ts) {
                    visit(t);
                }
                return null;
            }

            @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
            public Void visitForAll(Type.ForAll t, Void ignored) {
                visit(t.tvars);
                visit(t.qtype);
                return null;
            }

            @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
            public Void visitMethodType(Type.MethodType t, Void ignored) {
                visit(t.argtypes);
                visit(t.restype);
                return null;
            }

            @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
            public Void visitErrorType(Type.ErrorType t, Void ignored) {
                Type ot = t.getOriginalType();
                if (ot != null) {
                    visit(ot);
                    return null;
                }
                return null;
            }

            @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
            public Void visitArrayType(Type.ArrayType t, Void ignored) {
                visit(t.elemtype);
                return null;
            }

            @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
            public Void visitWildcardType(Type.WildcardType t, Void ignored) {
                visit(t.type);
                return null;
            }

            @Override // com.sun.tools.javac.code.Type.Visitor
            public Void visitType(Type t, Void ignored) {
                return null;
            }

            @Override // com.sun.tools.javac.code.Types.SimpleVisitor, com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
            public Void visitCapturedType(Type.CapturedType t, Void ignored) {
                if (RichDiagnosticFormatter.this.indexOf(t, WhereClauseKind.CAPTURED) == -1) {
                    String suffix = t.lower == RichDiagnosticFormatter.this.syms.botType ? ".1" : "";
                    JCDiagnostic d = RichDiagnosticFormatter.this.diags.fragment("where.captured" + suffix, t, t.bound, t.lower, t.wildcard);
                    RichDiagnosticFormatter.this.whereClauses.get(WhereClauseKind.CAPTURED).put(t, d);
                    visit(t.wildcard);
                    visit(t.lower);
                    visit(t.bound);
                    return null;
                }
                return null;
            }

            @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
            public Void visitClassType(Type.ClassType t, Void ignored) {
                Type.ClassType norm;
                if (t.isCompound()) {
                    if (RichDiagnosticFormatter.this.indexOf(t, WhereClauseKind.INTERSECTION) == -1) {
                        Type supertype = RichDiagnosticFormatter.this.types.supertype(t);
                        List<Type> interfaces = RichDiagnosticFormatter.this.types.interfaces(t);
                        JCDiagnostic d = RichDiagnosticFormatter.this.diags.fragment("where.intersection", t, interfaces.prepend(supertype));
                        RichDiagnosticFormatter.this.whereClauses.get(WhereClauseKind.INTERSECTION).put(t, d);
                        visit(supertype);
                        visit(interfaces);
                    }
                } else if (t.tsym.name.isEmpty() && (norm = (Type.ClassType) t.tsym.type) != null) {
                    if (norm.interfaces_field != null && norm.interfaces_field.nonEmpty()) {
                        visit(norm.interfaces_field.head);
                    } else {
                        visit(norm.supertype_field);
                    }
                }
                RichDiagnosticFormatter.this.nameSimplifier.addUsage(t.tsym);
                visit(t.getTypeArguments());
                if (t.getEnclosingType() != Type.noType) {
                    visit(t.getEnclosingType());
                    return null;
                }
                return null;
            }

            @Override // com.sun.tools.javac.code.Types.DefaultTypeVisitor, com.sun.tools.javac.code.Type.Visitor
            public Void visitTypeVar(Type.TypeVar t, Void ignored) {
                List<Type> bounds;
                if (RichDiagnosticFormatter.this.indexOf(t, WhereClauseKind.TYPEVAR) == -1) {
                    Type bound = t.bound;
                    while (bound instanceof Type.ErrorType) {
                        bound = ((Type.ErrorType) bound).getOriginalType();
                    }
                    if (bound != null && (bound.hasTag(TypeTag.CLASS) || bound.hasTag(TypeTag.TYPEVAR))) {
                        bounds = RichDiagnosticFormatter.this.types.getBounds(t);
                    } else {
                        bounds = List.nil();
                    }
                    RichDiagnosticFormatter.this.nameSimplifier.addUsage(t.tsym);
                    boolean boundErroneous = bounds.head == null || bounds.head.hasTag(TypeTag.NONE) || bounds.head.hasTag(TypeTag.ERROR);
                    if ((t.tsym.flags() & 4096) == 0) {
                        JCDiagnostic d = RichDiagnosticFormatter.this.diags.fragment("where.typevar" + (boundErroneous ? ".1" : ""), t, bounds, Kinds.kindName(t.tsym.location()), t.tsym.location());
                        RichDiagnosticFormatter.this.whereClauses.get(WhereClauseKind.TYPEVAR).put(t, d);
                        RichDiagnosticFormatter.this.symbolPreprocessor.visit(t.tsym.location(), null);
                        visit(bounds);
                    } else {
                        Assert.check(boundErroneous ? false : true);
                        JCDiagnostic d2 = RichDiagnosticFormatter.this.diags.fragment("where.fresh.typevar", t, bounds);
                        RichDiagnosticFormatter.this.whereClauses.get(WhereClauseKind.TYPEVAR).put(t, d2);
                        visit(bounds);
                    }
                }
                return null;
            }
        };
        this.symbolPreprocessor = new Types.DefaultSymbolVisitor<Void, Void>() { // from class: com.sun.tools.javac.util.RichDiagnosticFormatter.2
            @Override // com.sun.tools.javac.code.Types.DefaultSymbolVisitor, com.sun.tools.javac.code.Symbol.Visitor
            public Void visitClassSymbol(Symbol.ClassSymbol s, Void ignored) {
                if (s.type.isCompound()) {
                    RichDiagnosticFormatter.this.typePreprocessor.visit(s.type);
                    return null;
                }
                RichDiagnosticFormatter.this.nameSimplifier.addUsage(s);
                return null;
            }

            @Override // com.sun.tools.javac.code.Symbol.Visitor
            public Void visitSymbol(Symbol s, Void ignored) {
                return null;
            }

            @Override // com.sun.tools.javac.code.Types.DefaultSymbolVisitor, com.sun.tools.javac.code.Symbol.Visitor
            public Void visitMethodSymbol(Symbol.MethodSymbol s, Void ignored) {
                visit(s.owner, null);
                if (s.type != null) {
                    RichDiagnosticFormatter.this.typePreprocessor.visit(s.type);
                }
                return null;
            }
        };
        setRichPrinter(new RichPrinter());
        this.syms = Symtab.instance(context);
        this.diags = JCDiagnostic.Factory.instance(context);
        this.types = Types.instance(context);
        this.messages = JavacMessages.instance(context);
        this.whereClauses = new EnumMap(WhereClauseKind.class);
        this.configuration = new RichConfiguration(Options.instance(context), (AbstractDiagnosticFormatter) this.formatter);
        for (WhereClauseKind kind : WhereClauseKind.values()) {
            this.whereClauses.put(kind, new LinkedHashMap());
        }
    }

    @Override // com.sun.tools.javac.util.ForwardingDiagnosticFormatter, com.sun.tools.javac.api.DiagnosticFormatter
    public String format(JCDiagnostic diag, Locale l) {
        StringBuilder sb = new StringBuilder();
        this.nameSimplifier = new ClassNameSimplifier();
        for (WhereClauseKind kind : WhereClauseKind.values()) {
            this.whereClauses.get(kind).clear();
        }
        preprocessDiagnostic(diag);
        sb.append(((AbstractDiagnosticFormatter) this.formatter).format(diag, l));
        if (getConfiguration().isEnabled(RichConfiguration.RichFormatterFeature.WHERE_CLAUSES)) {
            List<JCDiagnostic> clauses = getWhereClauses();
            String indent = ((AbstractDiagnosticFormatter) this.formatter).isRaw() ? "" : ((AbstractDiagnosticFormatter) this.formatter).indentString(2);
            for (JCDiagnostic d : clauses) {
                String whereClause = ((AbstractDiagnosticFormatter) this.formatter).format(d, l);
                if (whereClause.length() > 0) {
                    sb.append('\n' + indent + whereClause);
                }
            }
        }
        return sb.toString();
    }

    @Override // com.sun.tools.javac.util.ForwardingDiagnosticFormatter, com.sun.tools.javac.api.DiagnosticFormatter
    public String formatMessage(JCDiagnostic diag, Locale l) {
        this.nameSimplifier = new ClassNameSimplifier();
        preprocessDiagnostic(diag);
        return super.formatMessage(diag, l);
    }

    protected void setRichPrinter(RichPrinter printer) {
        this.printer = printer;
        ((AbstractDiagnosticFormatter) this.formatter).setPrinter(printer);
    }

    protected RichPrinter getRichPrinter() {
        return this.printer;
    }

    protected void preprocessDiagnostic(JCDiagnostic diag) {
        for (Object o : diag.getArgs()) {
            if (o != null) {
                preprocessArgument(o);
            }
        }
        if (diag.isMultiline()) {
            for (JCDiagnostic d : diag.getSubdiagnostics()) {
                preprocessDiagnostic(d);
            }
        }
    }

    protected void preprocessArgument(Object arg) {
        if (arg instanceof Type) {
            preprocessType((Type) arg);
            return;
        }
        if (arg instanceof Symbol) {
            preprocessSymbol((Symbol) arg);
            return;
        }
        if (arg instanceof JCDiagnostic) {
            preprocessDiagnostic((JCDiagnostic) arg);
        } else if (arg instanceof Iterable) {
            for (Object o : (Iterable) arg) {
                preprocessArgument(o);
            }
        }
    }

    protected List<JCDiagnostic> getWhereClauses() {
        List<JCDiagnostic> clauses = List.nil();
        for (WhereClauseKind kind : WhereClauseKind.values()) {
            List<JCDiagnostic> lines = List.nil();
            for (Map.Entry<Type, JCDiagnostic> entry : this.whereClauses.get(kind).entrySet()) {
                lines = lines.prepend(entry.getValue());
            }
            if (!lines.isEmpty()) {
                String key = kind.key();
                if (lines.size() > 1) {
                    key = key + ".1";
                }
                JCDiagnostic d = this.diags.fragment(key, this.whereClauses.get(kind).keySet());
                clauses = clauses.prepend(new JCDiagnostic.MultilineDiagnostic(d, lines.reverse()));
            }
        }
        return clauses.reverse();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public int indexOf(Type type, WhereClauseKind kind) {
        int index = 1;
        for (Type t : this.whereClauses.get(kind).keySet()) {
            if (t.tsym == type.tsym) {
                return index;
            }
            if (kind != WhereClauseKind.TYPEVAR || t.toString().equals(type.toString())) {
                index++;
            }
        }
        return -1;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public boolean unique(Type.TypeVar typevar) {
        int found = 0;
        for (Type t : this.whereClauses.get(WhereClauseKind.TYPEVAR).keySet()) {
            if (t.toString().equals(typevar.toString())) {
                found++;
            }
        }
        if (found >= 1) {
            return found == 1;
        }
        throw new AssertionError("Missing type variable in where clause " + typevar);
    }

    enum WhereClauseKind {
        TYPEVAR("where.description.typevar"),
        CAPTURED("where.description.captured"),
        INTERSECTION("where.description.intersection");

        private final String key;

        WhereClauseKind(String key) {
            this.key = key;
        }

        String key() {
            return this.key;
        }
    }

    protected class ClassNameSimplifier {
        Map<Name, List<Symbol>> nameClashes = new HashMap();

        protected ClassNameSimplifier() {
        }

        protected void addUsage(Symbol sym) {
            Name n = sym.getSimpleName();
            List<Symbol> conflicts = this.nameClashes.get(n);
            if (conflicts == null) {
                conflicts = List.nil();
            }
            if (!conflicts.contains(sym)) {
                this.nameClashes.put(n, conflicts.append(sym));
            }
        }

        public String simplify(Symbol s) {
            String name = s.getQualifiedName().toString();
            if (!s.type.isCompound() && !s.type.isPrimitive()) {
                List<Symbol> conflicts = this.nameClashes.get(s.getSimpleName());
                if (conflicts == null || (conflicts.size() == 1 && conflicts.contains(s))) {
                    List<Name> l = List.nil();
                    Symbol s2 = s;
                    while (s2.type.hasTag(TypeTag.CLASS) && s2.type.getEnclosingType().hasTag(TypeTag.CLASS) && s2.owner.kind == 2) {
                        l = l.prepend(s2.getSimpleName());
                        s2 = s2.owner;
                    }
                    List<Name> l2 = l.prepend(s2.getSimpleName());
                    StringBuilder buf = new StringBuilder();
                    String sep = "";
                    for (Name n2 : l2) {
                        buf.append(sep);
                        buf.append((CharSequence) n2);
                        sep = ".";
                    }
                    return buf.toString();
                }
                return name;
            }
            return name;
        }
    }

    protected class RichPrinter extends Printer {
        protected RichPrinter() {
        }

        @Override // com.sun.tools.javac.code.Printer
        public String localize(Locale locale, String key, Object... args) {
            return ((AbstractDiagnosticFormatter) RichDiagnosticFormatter.this.formatter).localize(locale, key, args);
        }

        @Override // com.sun.tools.javac.code.Printer
        public String capturedVarId(Type.CapturedType t, Locale locale) {
            return RichDiagnosticFormatter.this.indexOf(t, WhereClauseKind.CAPTURED) + "";
        }

        @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
        public String visitType(Type t, Locale locale) {
            String s = super.visitType(t, locale);
            if (t == RichDiagnosticFormatter.this.syms.botType) {
                String s2 = localize(locale, "compiler.misc.type.null", new Object[0]);
                return s2;
            }
            return s;
        }

        @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
        public String visitCapturedType(Type.CapturedType t, Locale locale) {
            if (RichDiagnosticFormatter.this.getConfiguration().isEnabled(RichConfiguration.RichFormatterFeature.WHERE_CLAUSES)) {
                return localize(locale, "compiler.misc.captured.type", Integer.valueOf(RichDiagnosticFormatter.this.indexOf(t, WhereClauseKind.CAPTURED)));
            }
            return super.visitCapturedType(t, locale);
        }

        @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
        public String visitClassType(Type.ClassType t, Locale locale) {
            if (t.isCompound() && RichDiagnosticFormatter.this.getConfiguration().isEnabled(RichConfiguration.RichFormatterFeature.WHERE_CLAUSES)) {
                return localize(locale, "compiler.misc.intersection.type", Integer.valueOf(RichDiagnosticFormatter.this.indexOf(t, WhereClauseKind.INTERSECTION)));
            }
            return super.visitClassType(t, locale);
        }

        @Override // com.sun.tools.javac.code.Printer
        protected String className(Type.ClassType t, boolean longform, Locale locale) {
            Symbol sym = t.tsym;
            if (sym.name.length() == 0 || !RichDiagnosticFormatter.this.getConfiguration().isEnabled(RichConfiguration.RichFormatterFeature.SIMPLE_NAMES)) {
                return super.className(t, longform, locale);
            }
            if (longform) {
                return RichDiagnosticFormatter.this.nameSimplifier.simplify(sym).toString();
            }
            return sym.name.toString();
        }

        @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Type.Visitor
        public String visitTypeVar(Type.TypeVar t, Locale locale) {
            if (RichDiagnosticFormatter.this.unique(t) || !RichDiagnosticFormatter.this.getConfiguration().isEnabled(RichConfiguration.RichFormatterFeature.UNIQUE_TYPEVAR_NAMES)) {
                return t.toString();
            }
            return localize(locale, "compiler.misc.type.var", t.toString(), Integer.valueOf(RichDiagnosticFormatter.this.indexOf(t, WhereClauseKind.TYPEVAR)));
        }

        @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Symbol.Visitor
        public String visitClassSymbol(Symbol.ClassSymbol s, Locale locale) {
            if (s.type.isCompound()) {
                return visit(s.type, locale);
            }
            String name = RichDiagnosticFormatter.this.nameSimplifier.simplify(s);
            if (name.length() == 0 || !RichDiagnosticFormatter.this.getConfiguration().isEnabled(RichConfiguration.RichFormatterFeature.SIMPLE_NAMES)) {
                return super.visitClassSymbol(s, locale);
            }
            return name;
        }

        @Override // com.sun.tools.javac.code.Printer, com.sun.tools.javac.code.Symbol.Visitor
        public String visitMethodSymbol(Symbol.MethodSymbol s, Locale locale) {
            String ownerName = visit(s.owner, locale);
            if (s.isStaticOrInstanceInit()) {
                return ownerName;
            }
            String ms = s.name == s.name.table.names.init ? ownerName : s.name.toString();
            if (s.type != null) {
                if (s.type.hasTag(TypeTag.FORALL)) {
                    ms = "<" + visitTypes(s.type.getTypeArguments(), locale) + ">" + ms;
                }
                return ms + "(" + printMethodArgs(s.type.mo176getParameterTypes(), (s.flags() & Flags.VARARGS) != 0, locale) + ")";
            }
            return ms;
        }
    }

    protected void preprocessType(Type t) {
        this.typePreprocessor.visit(t);
    }

    protected void preprocessSymbol(Symbol s) {
        this.symbolPreprocessor.visit(s, null);
    }

    @Override // com.sun.tools.javac.util.ForwardingDiagnosticFormatter, com.sun.tools.javac.api.DiagnosticFormatter
    public RichConfiguration getConfiguration() {
        return (RichConfiguration) this.configuration;
    }

    public static class RichConfiguration extends ForwardingDiagnosticFormatter.ForwardingConfiguration {
        protected EnumSet<RichFormatterFeature> features;

        public enum RichFormatterFeature {
            WHERE_CLAUSES,
            SIMPLE_NAMES,
            UNIQUE_TYPEVAR_NAMES
        }

        public RichConfiguration(Options options, AbstractDiagnosticFormatter formatter) {
            super(formatter.getConfiguration());
            this.features = formatter.isRaw() ? EnumSet.noneOf(RichFormatterFeature.class) : EnumSet.of(RichFormatterFeature.SIMPLE_NAMES, RichFormatterFeature.WHERE_CLAUSES, RichFormatterFeature.UNIQUE_TYPEVAR_NAMES);
            String diagOpts = options.get("diags");
            if (diagOpts != null) {
                for (String args : diagOpts.split(DocLint.TAGS_SEPARATOR)) {
                    if (args.equals("-where")) {
                        this.features.remove(RichFormatterFeature.WHERE_CLAUSES);
                    } else if (args.equals("where")) {
                        this.features.add(RichFormatterFeature.WHERE_CLAUSES);
                    }
                    if (args.equals("-simpleNames")) {
                        this.features.remove(RichFormatterFeature.SIMPLE_NAMES);
                    } else if (args.equals("simpleNames")) {
                        this.features.add(RichFormatterFeature.SIMPLE_NAMES);
                    }
                    if (args.equals("-disambiguateTvars")) {
                        this.features.remove(RichFormatterFeature.UNIQUE_TYPEVAR_NAMES);
                    } else if (args.equals("disambiguateTvars")) {
                        this.features.add(RichFormatterFeature.UNIQUE_TYPEVAR_NAMES);
                    }
                }
            }
        }

        public RichFormatterFeature[] getAvailableFeatures() {
            return RichFormatterFeature.values();
        }

        public void enable(RichFormatterFeature feature) {
            this.features.add(feature);
        }

        public void disable(RichFormatterFeature feature) {
            this.features.remove(feature);
        }

        public boolean isEnabled(RichFormatterFeature feature) {
            return this.features.contains(feature);
        }
    }
}
