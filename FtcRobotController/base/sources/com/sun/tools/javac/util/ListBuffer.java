package com.sun.tools.javac.util;

import java.util.AbstractQueue;
import java.util.Collection;
import java.util.Iterator;
import java.util.NoSuchElementException;

/* JADX INFO: loaded from: classes.dex */
public class ListBuffer<A> extends AbstractQueue<A> {
    private int count;
    private List<A> elems;
    private List<A> last;
    private boolean shared;

    public static <T> ListBuffer<T> of(T x) {
        ListBuffer<T> lb = new ListBuffer<>();
        lb.add(x);
        return lb;
    }

    public ListBuffer() {
        clear();
    }

    @Override // java.util.AbstractQueue, java.util.AbstractCollection, java.util.Collection
    public final void clear() {
        this.elems = List.nil();
        this.last = null;
        this.count = 0;
        this.shared = false;
    }

    public int length() {
        return this.count;
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public int size() {
        return this.count;
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public boolean isEmpty() {
        return this.count == 0;
    }

    public boolean nonEmpty() {
        return this.count != 0;
    }

    private void copy() {
        if (this.elems.nonEmpty()) {
            List<A> orig = this.elems;
            List<A> listOf = List.of((Object) orig.head);
            this.last = listOf;
            this.elems = listOf;
            while (true) {
                List<A> list = orig.tail;
                orig = list;
                if (list.nonEmpty()) {
                    this.last.tail = List.of((Object) orig.head);
                    this.last = this.last.tail;
                } else {
                    return;
                }
            }
        }
    }

    public ListBuffer<A> prepend(A x) {
        this.elems = this.elems.prepend(x);
        if (this.last == null) {
            this.last = this.elems;
        }
        this.count++;
        return this;
    }

    public ListBuffer<A> append(A x) {
        x.getClass();
        if (this.shared) {
            copy();
        }
        List<A> newLast = List.of((Object) x);
        if (this.last != null) {
            this.last.tail = newLast;
            this.last = newLast;
        } else {
            this.last = newLast;
            this.elems = newLast;
        }
        this.count++;
        return this;
    }

    public ListBuffer<A> appendList(List<A> xs) {
        while (xs.nonEmpty()) {
            append(xs.head);
            xs = xs.tail;
        }
        return this;
    }

    public ListBuffer<A> appendList(ListBuffer<A> xs) {
        return appendList(xs.toList());
    }

    public ListBuffer<A> appendArray(A[] xs) {
        for (A a : xs) {
            append(a);
        }
        return this;
    }

    public List<A> toList() {
        this.shared = true;
        return this.elems;
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public boolean contains(Object x) {
        return this.elems.contains(x);
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public <T> T[] toArray(T[] tArr) {
        return (T[]) this.elems.toArray(tArr);
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public Object[] toArray() {
        return toArray(new Object[size()]);
    }

    public A first() {
        return this.elems.head;
    }

    public A next() {
        A x = this.elems.head;
        if (!this.elems.isEmpty()) {
            this.elems = this.elems.tail;
            if (this.elems.isEmpty()) {
                this.last = null;
            }
            this.count--;
        }
        return x;
    }

    @Override // java.util.AbstractCollection, java.util.Collection, java.lang.Iterable
    public Iterator<A> iterator() {
        return new Iterator<A>() { // from class: com.sun.tools.javac.util.ListBuffer.1
            List<A> elems;

            {
                this.elems = ListBuffer.this.elems;
            }

            @Override // java.util.Iterator
            public boolean hasNext() {
                return !this.elems.isEmpty();
            }

            @Override // java.util.Iterator
            public A next() {
                if (this.elems.isEmpty()) {
                    throw new NoSuchElementException();
                }
                A elem = this.elems.head;
                this.elems = this.elems.tail;
                return elem;
            }

            @Override // java.util.Iterator
            public void remove() {
                throw new UnsupportedOperationException();
            }
        };
    }

    @Override // java.util.AbstractQueue, java.util.AbstractCollection, java.util.Collection, java.util.Queue
    public boolean add(A a) {
        append(a);
        return true;
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public boolean remove(Object o) {
        throw new UnsupportedOperationException();
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public boolean containsAll(Collection<?> c) {
        for (Object x : c) {
            if (!contains(x)) {
                return false;
            }
        }
        return true;
    }

    @Override // java.util.AbstractQueue, java.util.AbstractCollection, java.util.Collection
    public boolean addAll(Collection<? extends A> c) {
        for (A a : c) {
            append(a);
        }
        return true;
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public boolean removeAll(Collection<?> c) {
        throw new UnsupportedOperationException();
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public boolean retainAll(Collection<?> c) {
        throw new UnsupportedOperationException();
    }

    @Override // java.util.Queue
    public boolean offer(A a) {
        append(a);
        return true;
    }

    @Override // java.util.Queue
    public A poll() {
        return next();
    }

    @Override // java.util.Queue
    public A peek() {
        return first();
    }

    public A last() {
        if (this.last != null) {
            return this.last.head;
        }
        return null;
    }
}
