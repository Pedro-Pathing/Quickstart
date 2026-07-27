package com.sun.tools.javac.comp;

import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.comp.Resolve;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.List;

/* JADX INFO: loaded from: classes.dex */
public class AttrContext {
    Lint lint;
    JCTree preferredTreeForDiagnostics;
    Scope scope = null;
    int staticLevel = 0;
    boolean isSelfCall = false;
    boolean selectSuper = false;
    boolean isSerializable = false;
    Resolve.MethodResolutionPhase pendingResolutionPhase = null;
    Symbol enclVar = null;
    Attr.ResultInfo returnResult = null;
    Type defaultSuperCallSite = null;

    AttrContext dup(Scope scope) {
        AttrContext info = new AttrContext();
        info.scope = scope;
        info.staticLevel = this.staticLevel;
        info.isSelfCall = this.isSelfCall;
        info.selectSuper = this.selectSuper;
        info.pendingResolutionPhase = this.pendingResolutionPhase;
        info.lint = this.lint;
        info.enclVar = this.enclVar;
        info.returnResult = this.returnResult;
        info.defaultSuperCallSite = this.defaultSuperCallSite;
        info.isSerializable = this.isSerializable;
        info.preferredTreeForDiagnostics = this.preferredTreeForDiagnostics;
        return info;
    }

    AttrContext dup() {
        return dup(this.scope);
    }

    public Iterable<Symbol> getLocalElements() {
        if (this.scope == null) {
            return List.nil();
        }
        return this.scope.getElements();
    }

    boolean lastResolveVarargs() {
        return this.pendingResolutionPhase != null && this.pendingResolutionPhase.isVarargsRequired();
    }

    public String toString() {
        return "AttrContext[" + this.scope.toString() + "]";
    }
}
