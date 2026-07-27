package com.sun.tools.javac.jvm;

import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.util.ArrayUtils;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.Name;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes.dex */
public class Pool {
    public static final int MAX_ENTRIES = 65535;
    public static final int MAX_STRING_LENGTH = 65535;
    Map<Object, Integer> indices;
    Object[] pool;
    int pp;
    Types types;

    public Pool(int pp, Object[] pool, Types types) {
        this.pp = pp;
        this.pool = pool;
        this.types = types;
        this.indices = new HashMap(pool.length);
        for (int i = 1; i < pp; i++) {
            if (pool[i] != null) {
                this.indices.put(pool[i], Integer.valueOf(i));
            }
        }
    }

    public Pool(Types types) {
        this(1, new Object[64], types);
    }

    public int numEntries() {
        return this.pp;
    }

    public void reset() {
        this.pp = 1;
        this.indices.clear();
    }

    public int put(Object value) {
        Object value2 = makePoolValue(value);
        Integer index = this.indices.get(value2);
        if (index == null) {
            index = Integer.valueOf(this.pp);
            this.indices.put(value2, index);
            this.pool = ArrayUtils.ensureCapacity(this.pool, this.pp);
            Object[] objArr = this.pool;
            int i = this.pp;
            this.pp = i + 1;
            objArr[i] = value2;
            if ((value2 instanceof Long) || (value2 instanceof Double)) {
                this.pool = ArrayUtils.ensureCapacity(this.pool, this.pp);
                Object[] objArr2 = this.pool;
                int i2 = this.pp;
                this.pp = i2 + 1;
                objArr2[i2] = null;
            }
        }
        return index.intValue();
    }

    Object makePoolValue(Object o) {
        if (o instanceof Symbol.DynamicMethodSymbol) {
            return new DynamicMethod((Symbol.DynamicMethodSymbol) o, this.types);
        }
        if (o instanceof Symbol.MethodSymbol) {
            return new Method((Symbol.MethodSymbol) o, this.types);
        }
        if (o instanceof Symbol.VarSymbol) {
            return new Variable((Symbol.VarSymbol) o, this.types);
        }
        if (o instanceof Type) {
            return new Types.UniqueType((Type) o, this.types);
        }
        return o;
    }

    public int get(Object o) {
        Integer n = this.indices.get(o);
        if (n == null) {
            return -1;
        }
        return n.intValue();
    }

    static class Method extends Symbol.DelegatedSymbol<Symbol.MethodSymbol> {
        Types.UniqueType uniqueType;

        Method(Symbol.MethodSymbol m, Types types) {
            super(m);
            this.uniqueType = new Types.UniqueType(m.type, types);
        }

        public boolean equals(Object any) {
            if (!(any instanceof Method)) {
                return false;
            }
            Symbol.MethodSymbol o = (Symbol.MethodSymbol) ((Method) any).other;
            Symbol.MethodSymbol m = (Symbol.MethodSymbol) this.other;
            return o.name == m.name && o.owner == m.owner && ((Method) any).uniqueType.equals(this.uniqueType);
        }

        public int hashCode() {
            Symbol.MethodSymbol m = (Symbol.MethodSymbol) this.other;
            return (m.name.hashCode() * 33) + (m.owner.hashCode() * 9) + this.uniqueType.hashCode();
        }
    }

    static class DynamicMethod extends Method {
        public Object[] uniqueStaticArgs;

        DynamicMethod(Symbol.DynamicMethodSymbol m, Types types) {
            super(m, types);
            this.uniqueStaticArgs = getUniqueTypeArray(m.staticArgs, types);
        }

        @Override // com.sun.tools.javac.jvm.Pool.Method, javax.lang.model.element.Element
        public boolean equals(Object any) {
            if (!super.equals(any) || !(any instanceof DynamicMethod)) {
                return false;
            }
            Symbol.DynamicMethodSymbol dm1 = (Symbol.DynamicMethodSymbol) this.other;
            Symbol.DynamicMethodSymbol dm2 = (Symbol.DynamicMethodSymbol) ((DynamicMethod) any).other;
            return dm1.bsm == dm2.bsm && dm1.bsmKind == dm2.bsmKind && Arrays.equals(this.uniqueStaticArgs, ((DynamicMethod) any).uniqueStaticArgs);
        }

        @Override // com.sun.tools.javac.jvm.Pool.Method, javax.lang.model.element.Element
        public int hashCode() {
            int hash = super.hashCode();
            Symbol.DynamicMethodSymbol dm = (Symbol.DynamicMethodSymbol) this.other;
            int hash2 = hash + (dm.bsmKind * 7) + (dm.bsm.hashCode() * 11);
            for (int i = 0; i < dm.staticArgs.length; i++) {
                hash2 += this.uniqueStaticArgs[i].hashCode() * 23;
            }
            return hash2;
        }

        private Object[] getUniqueTypeArray(Object[] objects, Types types) {
            Object[] result = new Object[objects.length];
            for (int i = 0; i < objects.length; i++) {
                if (objects[i] instanceof Type) {
                    result[i] = new Types.UniqueType((Type) objects[i], types);
                } else {
                    result[i] = objects[i];
                }
            }
            return result;
        }
    }

    static class Variable extends Symbol.DelegatedSymbol<Symbol.VarSymbol> {
        Types.UniqueType uniqueType;

        Variable(Symbol.VarSymbol v, Types types) {
            super(v);
            this.uniqueType = new Types.UniqueType(v.type, types);
        }

        @Override // javax.lang.model.element.Element
        public boolean equals(Object any) {
            if (!(any instanceof Variable)) {
                return false;
            }
            Symbol.VarSymbol o = (Symbol.VarSymbol) ((Variable) any).other;
            Symbol.VarSymbol v = (Symbol.VarSymbol) this.other;
            return o.name == v.name && o.owner == v.owner && ((Variable) any).uniqueType.equals(this.uniqueType);
        }

        @Override // javax.lang.model.element.Element
        public int hashCode() {
            Symbol.VarSymbol v = (Symbol.VarSymbol) this.other;
            return (v.name.hashCode() * 33) + (v.owner.hashCode() * 9) + this.uniqueType.hashCode();
        }
    }

    public static class MethodHandle {
        int refKind;
        Symbol refSym;
        Types.UniqueType uniqueType;
        Filter<Name> nonInitFilter = new Filter<Name>() { // from class: com.sun.tools.javac.jvm.Pool.MethodHandle.1
            @Override // com.sun.tools.javac.util.Filter
            public boolean accepts(Name n) {
                return (n == n.table.names.init || n == n.table.names.clinit) ? false : true;
            }
        };
        Filter<Name> initFilter = new Filter<Name>() { // from class: com.sun.tools.javac.jvm.Pool.MethodHandle.2
            @Override // com.sun.tools.javac.util.Filter
            public boolean accepts(Name n) {
                return n == n.table.names.init;
            }
        };

        public MethodHandle(int refKind, Symbol refSym, Types types) {
            this.refKind = refKind;
            this.refSym = refSym;
            this.uniqueType = new Types.UniqueType(this.refSym.type, types);
            checkConsistent();
        }

        public boolean equals(Object other) {
            if (!(other instanceof MethodHandle)) {
                return false;
            }
            MethodHandle mr = (MethodHandle) other;
            if (mr.refKind != this.refKind) {
                return false;
            }
            Symbol o = mr.refSym;
            return o.name == this.refSym.name && o.owner == this.refSym.owner && ((MethodHandle) other).uniqueType.equals(this.uniqueType);
        }

        public int hashCode() {
            return (this.refKind * 65) + (this.refSym.name.hashCode() * 33) + (this.refSym.owner.hashCode() * 9) + this.uniqueType.hashCode();
        }

        private void checkConsistent() {
            boolean staticOk = false;
            int expectedKind = -1;
            Filter<Name> nameFilter = this.nonInitFilter;
            boolean interfaceOwner = false;
            switch (this.refKind) {
                case 2:
                case 4:
                    staticOk = true;
                case 1:
                case 3:
                    expectedKind = 4;
                    break;
                case 6:
                    interfaceOwner = true;
                    staticOk = true;
                case 5:
                    expectedKind = 16;
                    break;
                case 7:
                    interfaceOwner = true;
                    expectedKind = 16;
                    break;
                case 8:
                    nameFilter = this.initFilter;
                    expectedKind = 16;
                    break;
                case 9:
                    interfaceOwner = true;
                    expectedKind = 16;
                    break;
            }
            Assert.check(!this.refSym.isStatic() || staticOk);
            Assert.check(this.refSym.kind == expectedKind);
            Assert.check(nameFilter.accepts(this.refSym.name));
            Assert.check(!this.refSym.owner.isInterface() || interfaceOwner);
        }
    }
}
