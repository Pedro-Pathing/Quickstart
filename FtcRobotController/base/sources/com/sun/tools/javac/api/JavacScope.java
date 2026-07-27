package com.sun.tools.javac.api;

import com.sun.source.tree.Scope;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Env;
import javax.lang.model.element.Element;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.TypeElement;

/* JADX INFO: loaded from: classes.dex */
public class JavacScope implements Scope {
    protected final Env<AttrContext> env;

    JavacScope(Env<AttrContext> env) {
        env.getClass();
        this.env = env;
    }

    @Override // com.sun.source.tree.Scope
    public JavacScope getEnclosingScope() {
        if (this.env.outer != null && this.env.outer != this.env) {
            return new JavacScope(this.env.outer);
        }
        return new JavacScope(this.env) { // from class: com.sun.tools.javac.api.JavacScope.1
            @Override // com.sun.tools.javac.api.JavacScope
            public boolean isStarImportScope() {
                return true;
            }

            @Override // com.sun.tools.javac.api.JavacScope, com.sun.source.tree.Scope
            public JavacScope getEnclosingScope() {
                return null;
            }

            @Override // com.sun.tools.javac.api.JavacScope, com.sun.source.tree.Scope
            public Iterable<? extends Element> getLocalElements() {
                return this.env.toplevel.starImportScope.getElements();
            }
        };
    }

    @Override // com.sun.source.tree.Scope
    public TypeElement getEnclosingClass() {
        if (this.env.outer == null || this.env.outer == this.env) {
            return null;
        }
        return this.env.enclClass.sym;
    }

    @Override // com.sun.source.tree.Scope
    public ExecutableElement getEnclosingMethod() {
        if (this.env.enclMethod == null) {
            return null;
        }
        return this.env.enclMethod.sym;
    }

    @Override // com.sun.source.tree.Scope
    public Iterable<? extends Element> getLocalElements() {
        return this.env.info.getLocalElements();
    }

    public Env<AttrContext> getEnv() {
        return this.env;
    }

    public boolean isStarImportScope() {
        return false;
    }

    public boolean equals(Object other) {
        if (!(other instanceof JavacScope)) {
            return false;
        }
        JavacScope s = (JavacScope) other;
        return this.env.equals(s.env) && isStarImportScope() == s.isStarImportScope();
    }

    public int hashCode() {
        return this.env.hashCode() + (isStarImportScope() ? 1 : 0);
    }

    public String toString() {
        return "JavacScope[env=" + this.env + ",starImport=" + isStarImportScope() + "]";
    }
}
