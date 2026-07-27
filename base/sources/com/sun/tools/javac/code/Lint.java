package com.sun.tools.javac.code;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.Pair;
import java.util.EnumSet;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/* JADX INFO: loaded from: classes.dex */
public class Lint {
    protected static final Context.Key<Lint> lintKey = new Context.Key<>();
    private static final Map<String, LintCategory> map = new ConcurrentHashMap(20);
    private final AugmentVisitor augmentor;
    private final EnumSet<LintCategory> suppressedValues;
    private final EnumSet<LintCategory> values;

    public static Lint instance(Context context) {
        Lint instance = (Lint) context.get(lintKey);
        if (instance == null) {
            return new Lint(context);
        }
        return instance;
    }

    public Lint augment(Attribute.Compound attr) {
        return this.augmentor.augment(this, attr);
    }

    public Lint augment(Symbol sym) {
        Lint l = this.augmentor.augment(this, sym.getDeclarationAttributes());
        if (sym.isDeprecated()) {
            if (l == this) {
                l = new Lint(this);
            }
            l.values.remove(LintCategory.DEPRECATION);
            l.suppressedValues.add(LintCategory.DEPRECATION);
        }
        return l;
    }

    protected Lint(Context context) {
        Options options = Options.instance(context);
        this.values = EnumSet.noneOf(LintCategory.class);
        for (Map.Entry<String, LintCategory> e : map.entrySet()) {
            if (options.lint(e.getKey())) {
                this.values.add(e.getValue());
            }
        }
        this.suppressedValues = EnumSet.noneOf(LintCategory.class);
        context.put(lintKey, this);
        this.augmentor = new AugmentVisitor(context);
    }

    protected Lint(Lint other) {
        this.augmentor = other.augmentor;
        this.values = other.values.clone();
        this.suppressedValues = other.suppressedValues.clone();
    }

    public String toString() {
        return "Lint:[values" + this.values + " suppressedValues" + this.suppressedValues + "]";
    }

    public enum LintCategory {
        AUXILIARYCLASS("auxiliaryclass"),
        CAST("cast"),
        CLASSFILE("classfile"),
        DEPRECATION("deprecation"),
        DEP_ANN("dep-ann"),
        DIVZERO("divzero"),
        EMPTY("empty"),
        FALLTHROUGH("fallthrough"),
        FINALLY("finally"),
        OPTIONS("options"),
        OVERLOADS("overloads"),
        OVERRIDES("overrides"),
        PATH("path"),
        PROCESSING("processing"),
        RAW("rawtypes"),
        SERIAL("serial"),
        STATIC("static"),
        SUNAPI("sunapi", true),
        TRY("try"),
        UNCHECKED("unchecked"),
        VARARGS("varargs");

        public final boolean hidden;
        public final String option;

        LintCategory(String option) {
            this(option, false);
        }

        LintCategory(String option, boolean hidden) {
            this.option = option;
            this.hidden = hidden;
            Lint.map.put(option, this);
        }

        static LintCategory get(String option) {
            return (LintCategory) Lint.map.get(option);
        }
    }

    public boolean isEnabled(LintCategory lc) {
        return this.values.contains(lc);
    }

    public boolean isSuppressed(LintCategory lc) {
        return this.suppressedValues.contains(lc);
    }

    protected static class AugmentVisitor implements Attribute.Visitor {
        private final Context context;
        private Lint lint;
        private Lint parent;
        private Symtab syms;

        AugmentVisitor(Context context) {
            this.context = context;
        }

        Lint augment(Lint parent, Attribute.Compound attr) {
            initSyms();
            this.parent = parent;
            this.lint = null;
            attr.accept(this);
            return this.lint == null ? parent : this.lint;
        }

        Lint augment(Lint parent, List<Attribute.Compound> attrs) {
            initSyms();
            this.parent = parent;
            this.lint = null;
            for (Attribute.Compound a : attrs) {
                a.accept(this);
            }
            return this.lint == null ? parent : this.lint;
        }

        private void initSyms() {
            if (this.syms == null) {
                this.syms = Symtab.instance(this.context);
            }
        }

        private void suppress(LintCategory lc) {
            if (this.lint == null) {
                this.lint = new Lint(this.parent);
            }
            this.lint.suppressedValues.add(lc);
            this.lint.values.remove(lc);
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitConstant(Attribute.Constant value) {
            LintCategory lc;
            if (value.type.tsym == this.syms.stringType.tsym && (lc = LintCategory.get((String) value.value)) != null) {
                suppress(lc);
            }
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitClass(Attribute.Class clazz) {
        }

        /* JADX WARN: Multi-variable type inference failed */
        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitCompound(Attribute.Compound compound) {
            if (compound.type.tsym == this.syms.suppressWarningsType.tsym) {
                for (List list = compound.values; list.nonEmpty(); list = list.tail) {
                    Pair<Symbol.MethodSymbol, Attribute> value = (Pair) list.head;
                    if (value.fst.name.toString().equals("value")) {
                        value.snd.accept(this);
                    }
                }
            }
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitArray(Attribute.Array array) {
            for (Attribute value : array.values) {
                value.accept(this);
            }
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitEnum(Attribute.Enum e) {
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitError(Attribute.Error e) {
        }
    }
}
