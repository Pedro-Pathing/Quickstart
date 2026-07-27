package com.sun.tools.javac.comp;

import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.util.Context;
import java.util.Collection;
import java.util.HashMap;

/* JADX INFO: loaded from: classes.dex */
class TypeEnvs {
    private static final long serialVersionUID = 571524752489954631L;
    protected static final Context.Key<TypeEnvs> typeEnvsKey = new Context.Key<>();
    private HashMap<Symbol.TypeSymbol, Env<AttrContext>> map = new HashMap<>();

    public static TypeEnvs instance(Context context) {
        TypeEnvs instance = (TypeEnvs) context.get(typeEnvsKey);
        if (instance == null) {
            return new TypeEnvs(context);
        }
        return instance;
    }

    protected TypeEnvs(Context context) {
        context.put(typeEnvsKey, this);
    }

    Env<AttrContext> get(Symbol.TypeSymbol sym) {
        return this.map.get(sym);
    }

    Env<AttrContext> put(Symbol.TypeSymbol sym, Env<AttrContext> env) {
        return this.map.put(sym, env);
    }

    Env<AttrContext> remove(Symbol.TypeSymbol sym) {
        return this.map.remove(sym);
    }

    Collection<Env<AttrContext>> values() {
        return this.map.values();
    }

    void clear() {
        this.map.clear();
    }
}
