package com.sun.tools.javac.code;

import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Name;
import java.util.Iterator;

/* JADX INFO: loaded from: classes.dex */
public class Scope {
    private static final int INITIAL_SIZE = 16;
    public Entry elems;
    int hashMask;
    List<ScopeListener> listeners;
    int nelems;
    public Scope next;
    public Symbol owner;
    private int shared;
    Entry[] table;
    private static final Entry sentinel = new Entry(null, null, null, null);
    public static final Scope emptyScope = new Scope(null, null, new Entry[0]);
    static final Filter<Symbol> noFilter = new Filter<Symbol>() { // from class: com.sun.tools.javac.code.Scope.2
        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Symbol s) {
            return true;
        }
    };

    public interface ScopeListener {
        void symbolAdded(Symbol symbol, Scope scope);

        void symbolRemoved(Symbol symbol, Scope scope);
    }

    private Scope(Scope next, Symbol owner, Entry[] table) {
        this.nelems = 0;
        this.listeners = List.nil();
        this.next = next;
        Assert.check(emptyScope == null || owner != null);
        this.owner = owner;
        this.table = table;
        this.hashMask = table.length - 1;
    }

    private Scope(Scope next, Symbol owner, Entry[] table, int nelems) {
        this(next, owner, table);
        this.nelems = nelems;
    }

    public Scope(Symbol owner) {
        this(null, owner, new Entry[16]);
    }

    public Scope dup() {
        return dup(this.owner);
    }

    public Scope dup(Symbol newOwner) {
        Scope result = new Scope(this, newOwner, this.table, this.nelems);
        this.shared++;
        return result;
    }

    public Scope dupUnshared() {
        return new Scope(this, this.owner, (Entry[]) this.table.clone(), this.nelems);
    }

    public Scope leave() {
        Assert.check(this.shared == 0);
        if (this.table != this.next.table) {
            return this.next;
        }
        while (this.elems != null) {
            int hash = getIndex(this.elems.sym.name);
            Entry e = this.table[hash];
            Assert.check(e == this.elems, this.elems.sym);
            this.table[hash] = this.elems.shadowed;
            this.elems = this.elems.sibling;
        }
        Assert.check(this.next.shared > 0);
        this.next.shared--;
        this.next.nelems = this.nelems;
        return this.next;
    }

    private void dble() {
        Assert.check(this.shared == 0);
        Entry[] oldtable = this.table;
        Entry[] newtable = new Entry[oldtable.length * 2];
        Scope s = this;
        while (s != null) {
            if (s.table == oldtable) {
                Assert.check(s == this || s.shared != 0);
                s.table = newtable;
                s.hashMask = newtable.length - 1;
            }
            s = s.next;
        }
        int n = 0;
        int i = oldtable.length;
        while (true) {
            i--;
            if (i >= 0) {
                Entry e = oldtable[i];
                if (e != null && e != sentinel) {
                    this.table[getIndex(e.sym.name)] = e;
                    n++;
                }
            } else {
                this.nelems = n;
                return;
            }
        }
    }

    public void enter(Symbol sym) {
        Assert.check(this.shared == 0);
        enter(sym, this);
    }

    public void enter(Symbol sym, Scope s) {
        enter(sym, s, s, false);
    }

    public void enter(Symbol sym, Scope s, Scope origin, boolean staticallyImported) {
        Assert.check(this.shared == 0);
        if (this.nelems * 3 >= this.hashMask * 2) {
            dble();
        }
        int hash = getIndex(sym.name);
        Entry old = this.table[hash];
        if (old == null) {
            old = sentinel;
            this.nelems++;
        }
        Entry e = makeEntry(sym, old, this.elems, s, origin, staticallyImported);
        this.table[hash] = e;
        this.elems = e;
        for (List list = this.listeners; list.nonEmpty(); list = list.tail) {
            ((ScopeListener) list.head).symbolAdded(sym, this);
        }
    }

    Entry makeEntry(Symbol sym, Entry shadowed, Entry sibling, Scope scope, Scope origin, boolean staticallyImported) {
        return new Entry(sym, shadowed, sibling, scope);
    }

    public void addScopeListener(ScopeListener sl) {
        this.listeners = this.listeners.prepend(sl);
    }

    public void remove(final Symbol sym) {
        Assert.check(this.shared == 0);
        Entry e = lookup(sym.name, new Filter<Symbol>() { // from class: com.sun.tools.javac.code.Scope.1
            @Override // com.sun.tools.javac.util.Filter
            public boolean accepts(Symbol candidate) {
                return candidate == sym;
            }
        });
        if (e.scope == null) {
            return;
        }
        int i = getIndex(sym.name);
        Entry te = this.table[i];
        if (te == e) {
            this.table[i] = e.shadowed;
        } else {
            while (te.shadowed != e) {
                te = te.shadowed;
            }
            te.shadowed = e.shadowed;
        }
        Entry te2 = this.elems;
        if (te2 == e) {
            this.elems = e.sibling;
        } else {
            while (te2.sibling != e) {
                te2 = te2.sibling;
            }
            te2.sibling = e.sibling;
        }
        for (List list = this.listeners; list.nonEmpty(); list = list.tail) {
            ((ScopeListener) list.head).symbolRemoved(sym, this);
        }
    }

    public void enterIfAbsent(Symbol sym) {
        Assert.check(this.shared == 0);
        Entry e = lookup(sym.name);
        while (e.scope == this && e.sym.kind != sym.kind) {
            e = e.next();
        }
        if (e.scope != this) {
            enter(sym);
        }
    }

    public boolean includes(Symbol c) {
        for (Entry e = lookup(c.name); e.scope == this; e = e.next()) {
            if (e.sym == c) {
                return true;
            }
        }
        return false;
    }

    public Entry lookup(Name name) {
        return lookup(name, noFilter);
    }

    public Entry lookup(Name name, Filter<Symbol> sf) {
        Entry e = this.table[getIndex(name)];
        if (e == null || e == sentinel) {
            return sentinel;
        }
        while (e.scope != null && (e.sym.name != name || !sf.accepts(e.sym))) {
            e = e.shadowed;
        }
        return e;
    }

    int getIndex(Name name) {
        int h = name.hashCode();
        int i = this.hashMask & h;
        int x = this.hashMask - (((h >> 16) + h) << 1);
        int d = -1;
        while (true) {
            Entry e = this.table[i];
            if (e == null) {
                return d >= 0 ? d : i;
            }
            if (e == sentinel) {
                if (d < 0) {
                    d = i;
                }
            } else if (e.sym.name == name) {
                return i;
            }
            i = (i + x) & this.hashMask;
        }
    }

    public boolean anyMatch(Filter<Symbol> sf) {
        return getElements(sf).iterator().hasNext();
    }

    public Iterable<Symbol> getElements() {
        return getElements(noFilter);
    }

    public Iterable<Symbol> getElements(final Filter<Symbol> sf) {
        return new Iterable<Symbol>() { // from class: com.sun.tools.javac.code.Scope.3
            @Override // java.lang.Iterable
            public Iterator<Symbol> iterator() {
                return new Iterator<Symbol>() { // from class: com.sun.tools.javac.code.Scope.3.1
                    private Entry currEntry;
                    private Scope currScope;

                    {
                        this.currScope = Scope.this;
                        this.currEntry = Scope.this.elems;
                        update();
                    }

                    @Override // java.util.Iterator
                    public boolean hasNext() {
                        return this.currEntry != null;
                    }

                    /* JADX WARN: Can't rename method to resolve collision */
                    @Override // java.util.Iterator
                    public Symbol next() {
                        Symbol sym = this.currEntry == null ? null : this.currEntry.sym;
                        if (this.currEntry != null) {
                            this.currEntry = this.currEntry.sibling;
                        }
                        update();
                        return sym;
                    }

                    @Override // java.util.Iterator
                    public void remove() {
                        throw new UnsupportedOperationException();
                    }

                    private void update() {
                        skipToNextMatchingEntry();
                        while (this.currEntry == null && this.currScope.next != null) {
                            this.currScope = this.currScope.next;
                            this.currEntry = this.currScope.elems;
                            skipToNextMatchingEntry();
                        }
                    }

                    void skipToNextMatchingEntry() {
                        while (this.currEntry != null && !sf.accepts(this.currEntry.sym)) {
                            this.currEntry = this.currEntry.sibling;
                        }
                    }
                };
            }
        };
    }

    public Iterable<Symbol> getElementsByName(Name name) {
        return getElementsByName(name, noFilter);
    }

    public Iterable<Symbol> getElementsByName(final Name name, final Filter<Symbol> sf) {
        return new Iterable<Symbol>() { // from class: com.sun.tools.javac.code.Scope.4
            @Override // java.lang.Iterable
            public Iterator<Symbol> iterator() {
                return new Iterator<Symbol>() { // from class: com.sun.tools.javac.code.Scope.4.1
                    Entry currentEntry;

                    {
                        this.currentEntry = Scope.this.lookup(name, sf);
                    }

                    @Override // java.util.Iterator
                    public boolean hasNext() {
                        return this.currentEntry.scope != null;
                    }

                    /* JADX WARN: Can't rename method to resolve collision */
                    @Override // java.util.Iterator
                    public Symbol next() {
                        Entry prevEntry = this.currentEntry;
                        this.currentEntry = this.currentEntry.next(sf);
                        return prevEntry.sym;
                    }

                    @Override // java.util.Iterator
                    public void remove() {
                        throw new UnsupportedOperationException();
                    }
                };
            }
        };
    }

    public String toString() {
        StringBuilder result = new StringBuilder();
        result.append("Scope[");
        for (Scope s = this; s != null; s = s.next) {
            if (s != this) {
                result.append(" | ");
            }
            for (Entry e = s.elems; e != null; e = e.sibling) {
                if (e != s.elems) {
                    result.append(", ");
                }
                result.append(e.sym);
            }
        }
        result.append("]");
        return result.toString();
    }

    public static class Entry {
        public Scope scope;
        private Entry shadowed;
        public Entry sibling;
        public Symbol sym;

        public Entry(Symbol sym, Entry shadowed, Entry sibling, Scope scope) {
            this.sym = sym;
            this.shadowed = shadowed;
            this.sibling = sibling;
            this.scope = scope;
        }

        public Entry next() {
            return this.shadowed;
        }

        public Entry next(Filter<Symbol> sf) {
            if (this.shadowed.sym == null || sf.accepts(this.shadowed.sym)) {
                return this.shadowed;
            }
            return this.shadowed.next(sf);
        }

        public boolean isStaticallyImported() {
            return false;
        }

        public Scope getOrigin() {
            return this.scope;
        }
    }

    public static class ImportScope extends Scope {
        public ImportScope(Symbol owner) {
            super(owner);
        }

        @Override // com.sun.tools.javac.code.Scope
        Entry makeEntry(Symbol sym, Entry shadowed, Entry sibling, Scope scope, final Scope origin, final boolean staticallyImported) {
            return new Entry(sym, shadowed, sibling, scope) { // from class: com.sun.tools.javac.code.Scope.ImportScope.1
                @Override // com.sun.tools.javac.code.Scope.Entry
                public Scope getOrigin() {
                    return origin;
                }

                @Override // com.sun.tools.javac.code.Scope.Entry
                public boolean isStaticallyImported() {
                    return staticallyImported;
                }
            };
        }
    }

    public static class StarImportScope extends ImportScope implements ScopeListener {
        public StarImportScope(Symbol owner) {
            super(owner);
        }

        public void importAll(Scope fromScope) {
            for (Entry e = fromScope.elems; e != null; e = e.sibling) {
                if (e.sym.kind == 2 && !includes(e.sym)) {
                    enter(e.sym, fromScope);
                }
            }
            fromScope.addScopeListener(this);
        }

        @Override // com.sun.tools.javac.code.Scope.ScopeListener
        public void symbolRemoved(Symbol sym, Scope s) {
            remove(sym);
        }

        @Override // com.sun.tools.javac.code.Scope.ScopeListener
        public void symbolAdded(Symbol sym, Scope s) {
        }
    }

    public static class DelegatedScope extends Scope {
        public static final Entry[] emptyTable = new Entry[0];
        Scope delegatee;

        public DelegatedScope(Scope outer) {
            super(outer.owner, emptyTable);
            this.delegatee = outer;
        }

        @Override // com.sun.tools.javac.code.Scope
        public Scope dup() {
            return new DelegatedScope(this.next);
        }

        @Override // com.sun.tools.javac.code.Scope
        public Scope dupUnshared() {
            return new DelegatedScope(this.next);
        }

        @Override // com.sun.tools.javac.code.Scope
        public Scope leave() {
            return this.next;
        }

        @Override // com.sun.tools.javac.code.Scope
        public void enter(Symbol sym) {
        }

        @Override // com.sun.tools.javac.code.Scope
        public void enter(Symbol sym, Scope s) {
        }

        @Override // com.sun.tools.javac.code.Scope
        public void remove(Symbol sym) {
            throw new AssertionError(sym);
        }

        @Override // com.sun.tools.javac.code.Scope
        public Entry lookup(Name name) {
            return this.delegatee.lookup(name);
        }
    }

    public static class CompoundScope extends Scope implements ScopeListener {
        public static final Entry[] emptyTable = new Entry[0];
        private int mark;
        private List<Scope> subScopes;

        /* JADX WARN: Multi-variable type inference failed */
        public CompoundScope(Symbol symbol) {
            super(symbol, emptyTable);
            this.subScopes = List.nil();
            this.mark = 0;
        }

        public void addSubScope(Scope that) {
            if (that != null) {
                this.subScopes = this.subScopes.prepend(that);
                that.addScopeListener(this);
                this.mark++;
                for (ScopeListener sl : this.listeners) {
                    sl.symbolAdded(null, this);
                }
            }
        }

        @Override // com.sun.tools.javac.code.Scope.ScopeListener
        public void symbolAdded(Symbol sym, Scope s) {
            this.mark++;
            for (ScopeListener sl : this.listeners) {
                sl.symbolAdded(sym, s);
            }
        }

        @Override // com.sun.tools.javac.code.Scope.ScopeListener
        public void symbolRemoved(Symbol sym, Scope s) {
            this.mark++;
            for (ScopeListener sl : this.listeners) {
                sl.symbolRemoved(sym, s);
            }
        }

        public int getMark() {
            return this.mark;
        }

        @Override // com.sun.tools.javac.code.Scope
        public String toString() {
            StringBuilder buf = new StringBuilder();
            buf.append("CompoundScope{");
            String sep = "";
            for (Scope s : this.subScopes) {
                buf.append(sep);
                buf.append(s);
                sep = DocLint.TAGS_SEPARATOR;
            }
            buf.append("}");
            return buf.toString();
        }

        @Override // com.sun.tools.javac.code.Scope
        public Iterable<Symbol> getElements(final Filter<Symbol> sf) {
            return new Iterable<Symbol>() { // from class: com.sun.tools.javac.code.Scope.CompoundScope.1
                @Override // java.lang.Iterable
                public Iterator<Symbol> iterator() {
                    return new CompoundScopeIterator(CompoundScope.this.subScopes) { // from class: com.sun.tools.javac.code.Scope.CompoundScope.1.1
                        {
                            CompoundScope compoundScope = CompoundScope.this;
                        }

                        @Override // com.sun.tools.javac.code.Scope.CompoundScope.CompoundScopeIterator
                        Iterator<Symbol> nextIterator(Scope s) {
                            return s.getElements(sf).iterator();
                        }
                    };
                }
            };
        }

        @Override // com.sun.tools.javac.code.Scope
        public Iterable<Symbol> getElementsByName(final Name name, final Filter<Symbol> sf) {
            return new Iterable<Symbol>() { // from class: com.sun.tools.javac.code.Scope.CompoundScope.2
                @Override // java.lang.Iterable
                public Iterator<Symbol> iterator() {
                    return new CompoundScopeIterator(CompoundScope.this.subScopes) { // from class: com.sun.tools.javac.code.Scope.CompoundScope.2.1
                        {
                            CompoundScope compoundScope = CompoundScope.this;
                        }

                        @Override // com.sun.tools.javac.code.Scope.CompoundScope.CompoundScopeIterator
                        Iterator<Symbol> nextIterator(Scope s) {
                            return s.getElementsByName(name, sf).iterator();
                        }
                    };
                }
            };
        }

        abstract class CompoundScopeIterator implements Iterator<Symbol> {
            private Iterator<Symbol> currentIterator;
            private List<Scope> scopesToScan;

            abstract Iterator<Symbol> nextIterator(Scope scope);

            public CompoundScopeIterator(List<Scope> scopesToScan) {
                this.scopesToScan = scopesToScan;
                update();
            }

            @Override // java.util.Iterator
            public boolean hasNext() {
                return this.currentIterator != null;
            }

            /* JADX WARN: Can't rename method to resolve collision */
            @Override // java.util.Iterator
            public Symbol next() {
                Symbol sym = this.currentIterator.next();
                if (!this.currentIterator.hasNext()) {
                    update();
                }
                return sym;
            }

            @Override // java.util.Iterator
            public void remove() {
                throw new UnsupportedOperationException();
            }

            private void update() {
                while (this.scopesToScan.nonEmpty()) {
                    this.currentIterator = nextIterator(this.scopesToScan.head);
                    this.scopesToScan = this.scopesToScan.tail;
                    if (this.currentIterator.hasNext()) {
                        return;
                    }
                }
                this.currentIterator = null;
            }
        }

        @Override // com.sun.tools.javac.code.Scope
        public Entry lookup(Name name, Filter<Symbol> sf) {
            throw new UnsupportedOperationException();
        }

        @Override // com.sun.tools.javac.code.Scope
        public Scope dup(Symbol newOwner) {
            throw new UnsupportedOperationException();
        }

        @Override // com.sun.tools.javac.code.Scope
        public void enter(Symbol sym, Scope s, Scope origin, boolean staticallyImported) {
            throw new UnsupportedOperationException();
        }

        @Override // com.sun.tools.javac.code.Scope
        public void remove(Symbol sym) {
            throw new UnsupportedOperationException();
        }
    }

    public static class ErrorScope extends Scope {
        ErrorScope(Scope next, Symbol errSymbol, Entry[] table) {
            super(errSymbol, table);
        }

        public ErrorScope(Symbol errSymbol) {
            super(errSymbol);
        }

        @Override // com.sun.tools.javac.code.Scope
        public Scope dup() {
            return new ErrorScope(this, this.owner, this.table);
        }

        @Override // com.sun.tools.javac.code.Scope
        public Scope dupUnshared() {
            return new ErrorScope(this, this.owner, (Entry[]) this.table.clone());
        }

        @Override // com.sun.tools.javac.code.Scope
        public Entry lookup(Name name) {
            Entry e = super.lookup(name);
            if (e.scope == null) {
                return new Entry(this.owner, null, null, null);
            }
            return e;
        }
    }
}
