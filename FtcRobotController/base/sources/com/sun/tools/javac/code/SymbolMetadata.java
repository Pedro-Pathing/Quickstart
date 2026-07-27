package com.sun.tools.javac.code;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.comp.Annotate;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import java.util.Map;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class SymbolMetadata {
    private final Symbol sym;
    private static final List<Attribute.Compound> DECL_NOT_STARTED = List.of((Object) null);
    private static final List<Attribute.Compound> DECL_IN_PROGRESS = List.of((Object) null);
    private List<Attribute.Compound> attributes = DECL_NOT_STARTED;
    private List<Attribute.TypeCompound> type_attributes = List.nil();
    private List<Attribute.TypeCompound> init_type_attributes = List.nil();
    private List<Attribute.TypeCompound> clinit_type_attributes = List.nil();

    public SymbolMetadata(Symbol sym) {
        this.sym = sym;
    }

    public List<Attribute.Compound> getDeclarationAttributes() {
        return filterDeclSentinels(this.attributes);
    }

    public List<Attribute.TypeCompound> getTypeAttributes() {
        return this.type_attributes;
    }

    public List<Attribute.TypeCompound> getInitTypeAttributes() {
        return this.init_type_attributes;
    }

    public List<Attribute.TypeCompound> getClassInitTypeAttributes() {
        return this.clinit_type_attributes;
    }

    public void setDeclarationAttributes(List<Attribute.Compound> a) {
        Assert.check(pendingCompletion() || !isStarted());
        if (a == null) {
            throw new NullPointerException();
        }
        this.attributes = a;
    }

    public void setTypeAttributes(List<Attribute.TypeCompound> a) {
        if (a == null) {
            throw new NullPointerException();
        }
        this.type_attributes = a;
    }

    public void setInitTypeAttributes(List<Attribute.TypeCompound> a) {
        if (a == null) {
            throw new NullPointerException();
        }
        this.init_type_attributes = a;
    }

    public void setClassInitTypeAttributes(List<Attribute.TypeCompound> a) {
        if (a == null) {
            throw new NullPointerException();
        }
        this.clinit_type_attributes = a;
    }

    public void setAttributes(SymbolMetadata other) {
        if (other == null) {
            throw new NullPointerException();
        }
        setDeclarationAttributes(other.getDeclarationAttributes());
        setTypeAttributes(other.getTypeAttributes());
        setInitTypeAttributes(other.getInitTypeAttributes());
        setClassInitTypeAttributes(other.getClassInitTypeAttributes());
    }

    public void setDeclarationAttributesWithCompletion(Annotate.AnnotateRepeatedContext<Attribute.Compound> ctx) {
        boolean z = true;
        if (!pendingCompletion() && (isStarted() || this.sym.kind != 1)) {
            z = false;
        }
        Assert.check(z);
        setDeclarationAttributes(getAttributesForCompletion(ctx));
    }

    public void appendTypeAttributesWithCompletion(Annotate.AnnotateRepeatedContext<Attribute.TypeCompound> ctx) {
        appendUniqueTypes(getAttributesForCompletion(ctx));
    }

    private <T extends Attribute.Compound> List<T> getAttributesForCompletion(final Annotate.AnnotateRepeatedContext<T> ctx) {
        Map<Symbol.TypeSymbol, ListBuffer<T>> annotated = ctx.annotated;
        boolean atLeastOneRepeated = false;
        List<T> buf = List.nil();
        for (ListBuffer<T> lb : annotated.values()) {
            if (lb.size() == 1) {
                buf = buf.prepend(lb.first());
            } else {
                T ph = new Placeholder(ctx, lb.toList(), this.sym);
                buf = buf.prepend(ph);
                atLeastOneRepeated = true;
            }
        }
        if (atLeastOneRepeated) {
            ctx.annotateRepeated(new Annotate.Worker() { // from class: com.sun.tools.javac.code.SymbolMetadata.1
                @Override // com.sun.tools.javac.comp.Annotate.Worker
                public String toString() {
                    return "repeated annotation pass of: " + SymbolMetadata.this.sym + " in: " + SymbolMetadata.this.sym.owner;
                }

                @Override // com.sun.tools.javac.comp.Annotate.Worker
                public void run() {
                    SymbolMetadata.this.complete(ctx);
                }
            });
        }
        return buf.reverse();
    }

    public SymbolMetadata reset() {
        this.attributes = DECL_IN_PROGRESS;
        return this;
    }

    public boolean isEmpty() {
        return !isStarted() || pendingCompletion() || this.attributes.isEmpty();
    }

    public boolean isTypesEmpty() {
        return this.type_attributes.isEmpty();
    }

    public boolean pendingCompletion() {
        return this.attributes == DECL_IN_PROGRESS;
    }

    public SymbolMetadata append(List<Attribute.Compound> l) {
        this.attributes = filterDeclSentinels(this.attributes);
        if (!l.isEmpty()) {
            if (this.attributes.isEmpty()) {
                this.attributes = l;
            } else {
                this.attributes = this.attributes.appendList(l);
            }
        }
        return this;
    }

    public SymbolMetadata appendUniqueTypes(List<Attribute.TypeCompound> l) {
        if (!l.isEmpty()) {
            if (this.type_attributes.isEmpty()) {
                this.type_attributes = l;
            } else {
                for (Attribute.TypeCompound tc : l) {
                    if (!this.type_attributes.contains(tc)) {
                        this.type_attributes = this.type_attributes.append(tc);
                    }
                }
            }
        }
        return this;
    }

    public SymbolMetadata appendInitTypeAttributes(List<Attribute.TypeCompound> l) {
        if (!l.isEmpty()) {
            if (this.init_type_attributes.isEmpty()) {
                this.init_type_attributes = l;
            } else {
                this.init_type_attributes = this.init_type_attributes.appendList(l);
            }
        }
        return this;
    }

    public SymbolMetadata appendClassInitTypeAttributes(List<Attribute.TypeCompound> l) {
        if (!l.isEmpty()) {
            if (this.clinit_type_attributes.isEmpty()) {
                this.clinit_type_attributes = l;
            } else {
                this.clinit_type_attributes = this.clinit_type_attributes.appendList(l);
            }
        }
        return this;
    }

    public SymbolMetadata prepend(List<Attribute.Compound> l) {
        this.attributes = filterDeclSentinels(this.attributes);
        if (!l.isEmpty()) {
            if (this.attributes.isEmpty()) {
                this.attributes = l;
            } else {
                this.attributes = this.attributes.prependList(l);
            }
        }
        return this;
    }

    private List<Attribute.Compound> filterDeclSentinels(List<Attribute.Compound> a) {
        return (a == DECL_IN_PROGRESS || a == DECL_NOT_STARTED) ? List.nil() : a;
    }

    private boolean isStarted() {
        return this.attributes != DECL_NOT_STARTED;
    }

    private List<Attribute.Compound> getPlaceholders() {
        List<Attribute.Compound> res = List.nil();
        for (Attribute.Compound a : filterDeclSentinels(this.attributes)) {
            if (a instanceof Placeholder) {
                res = res.prepend(a);
            }
        }
        return res.reverse();
    }

    private List<Attribute.TypeCompound> getTypePlaceholders() {
        List<Attribute.TypeCompound> res = List.nil();
        for (Attribute.TypeCompound a : this.type_attributes) {
            if (a instanceof Placeholder) {
                res = res.prepend(a);
            }
        }
        return res.reverse();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public <T extends Attribute.Compound> void complete(Annotate.AnnotateRepeatedContext<T> ctx) {
        Log log = ctx.log;
        Env<AttrContext> env = ctx.env;
        JavaFileObject oldSource = log.useSource(env.toplevel.sourcefile);
        try {
            boolean z = true;
            if (ctx.isTypeCompound) {
                if (isTypesEmpty()) {
                    z = false;
                }
                Assert.check(z);
                if (isTypesEmpty()) {
                    return;
                }
                List<Attribute.TypeCompound> result = List.nil();
                for (Attribute.TypeCompound a : getTypeAttributes()) {
                    if (a instanceof Placeholder) {
                        Placeholder<Attribute.TypeCompound> ph = (Placeholder) a;
                        Attribute.TypeCompound replacement = (Attribute.TypeCompound) replaceOne(ph, ph.getRepeatedContext());
                        if (replacement != null) {
                            result = result.prepend(replacement);
                        }
                    } else {
                        result = result.prepend(a);
                    }
                }
                this.type_attributes = result.reverse();
                Assert.check(getTypePlaceholders().isEmpty());
            } else {
                if (pendingCompletion()) {
                    z = false;
                }
                Assert.check(z);
                if (isEmpty()) {
                    return;
                }
                List<Attribute.Compound> result2 = List.nil();
                for (Attribute.Compound a2 : getDeclarationAttributes()) {
                    if (a2 instanceof Placeholder) {
                        Attribute.Compound replacement2 = replaceOne((Placeholder) a2, ctx);
                        if (replacement2 != null) {
                            result2 = result2.prepend(replacement2);
                        }
                    } else {
                        result2 = result2.prepend(a2);
                    }
                }
                this.attributes = result2.reverse();
                Assert.check(getPlaceholders().isEmpty());
            }
        } finally {
            log.useSource(oldSource);
        }
    }

    private <T extends Attribute.Compound> T replaceOne(Placeholder<T> placeholder, Annotate.AnnotateRepeatedContext<T> annotateRepeatedContext) {
        ListBuffer<T> listBuffer;
        Log log = annotateRepeatedContext.log;
        T t = (T) annotateRepeatedContext.processRepeatedAnnotations(placeholder.getPlaceholderFor(), this.sym);
        if (t != null && (listBuffer = annotateRepeatedContext.annotated.get(t.type.tsym)) != null) {
            log.error(annotateRepeatedContext.pos.get(listBuffer.first()), "invalid.repeatable.annotation.repeated.and.container.present", listBuffer.first().type.tsym);
        }
        return t;
    }

    private static class Placeholder<T extends Attribute.Compound> extends Attribute.TypeCompound {
        private final Annotate.AnnotateRepeatedContext<T> ctx;
        private final Symbol on;
        private final List<T> placeholderFor;

        public Placeholder(Annotate.AnnotateRepeatedContext<T> ctx, List<T> placeholderFor, Symbol on) {
            super(on.type, List.nil(), ctx.isTypeCompound ? ((Attribute.TypeCompound) placeholderFor.head).position : new TypeAnnotationPosition());
            this.ctx = ctx;
            this.placeholderFor = placeholderFor;
            this.on = on;
        }

        @Override // com.sun.tools.javac.code.Attribute.Compound, javax.lang.model.element.AnnotationValue
        public String toString() {
            return "<placeholder: " + this.placeholderFor + " on: " + this.on + ">";
        }

        public List<T> getPlaceholderFor() {
            return this.placeholderFor;
        }

        public Annotate.AnnotateRepeatedContext<T> getRepeatedContext() {
            return this.ctx;
        }
    }
}
