package com.sun.tools.javac.model;

import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import java.util.AbstractList;
import java.util.Iterator;
import java.util.NoSuchElementException;

/* JADX INFO: loaded from: classes.dex */
public class FilteredMemberList extends AbstractList<Symbol> {
    private final Scope scope;

    public FilteredMemberList(Scope scope) {
        this.scope = scope;
    }

    @Override // java.util.AbstractCollection, java.util.Collection, java.util.List
    public int size() {
        int cnt = 0;
        for (Scope.Entry e = this.scope.elems; e != null; e = e.sibling) {
            if (!unwanted(e.sym)) {
                cnt++;
            }
        }
        return cnt;
    }

    @Override // java.util.AbstractList, java.util.List
    public Symbol get(int index) {
        for (Scope.Entry e = this.scope.elems; e != null; e = e.sibling) {
            if (!unwanted(e.sym)) {
                int index2 = index - 1;
                if (index != 0) {
                    index = index2;
                } else {
                    return e.sym;
                }
            }
        }
        throw new IndexOutOfBoundsException();
    }

    @Override // java.util.AbstractList, java.util.AbstractCollection, java.util.Collection, java.lang.Iterable, java.util.List
    public Iterator<Symbol> iterator() {
        return new Iterator<Symbol>() { // from class: com.sun.tools.javac.model.FilteredMemberList.1
            private boolean hasNextForSure = false;
            private Scope.Entry nextEntry;

            {
                this.nextEntry = FilteredMemberList.this.scope.elems;
            }

            @Override // java.util.Iterator
            public boolean hasNext() {
                if (this.hasNextForSure) {
                    return true;
                }
                while (this.nextEntry != null && FilteredMemberList.unwanted(this.nextEntry.sym)) {
                    this.nextEntry = this.nextEntry.sibling;
                }
                this.hasNextForSure = this.nextEntry != null;
                return this.hasNextForSure;
            }

            /* JADX WARN: Can't rename method to resolve collision */
            @Override // java.util.Iterator
            public Symbol next() {
                if (hasNext()) {
                    Symbol result = this.nextEntry.sym;
                    this.nextEntry = this.nextEntry.sibling;
                    this.hasNextForSure = false;
                    return result;
                }
                throw new NoSuchElementException();
            }

            @Override // java.util.Iterator
            public void remove() {
                throw new UnsupportedOperationException();
            }
        };
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static boolean unwanted(Symbol s) {
        return s == null || (s.flags() & 4096) != 0;
    }
}
