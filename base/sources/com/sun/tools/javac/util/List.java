package com.sun.tools.javac.util;

import com.sun.tools.doclint.DocLint;
import java.lang.reflect.Array;
import java.util.AbstractCollection;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.NoSuchElementException;

/* JADX INFO: loaded from: classes.dex */
public class List<A> extends AbstractCollection<A> implements java.util.List<A> {
    private static final Iterator<?> EMPTYITERATOR = new Iterator<Object>() { // from class: com.sun.tools.javac.util.List.2
        @Override // java.util.Iterator
        public boolean hasNext() {
            return false;
        }

        @Override // java.util.Iterator
        public Object next() {
            throw new NoSuchElementException();
        }

        @Override // java.util.Iterator
        public void remove() {
            throw new UnsupportedOperationException();
        }
    };
    private static final List<?> EMPTY_LIST;
    public A head;
    public List<A> tail;

    List(A head, List<A> tail) {
        this.tail = tail;
        this.head = head;
    }

    public static <A> List<A> nil() {
        return (List<A>) EMPTY_LIST;
    }

    static {
        List list = null;
        EMPTY_LIST = new List<Object>(list, list) { // from class: com.sun.tools.javac.util.List.1
            @Override // com.sun.tools.javac.util.List
            public List<Object> setTail(List<Object> tail) {
                throw new UnsupportedOperationException();
            }

            @Override // com.sun.tools.javac.util.List, java.util.AbstractCollection, java.util.Collection, java.util.List
            public boolean isEmpty() {
                return true;
            }
        };
    }

    public static <A> List<A> filter(List<A> l, A elem) {
        Assert.checkNonNull(elem);
        List<A> res = nil();
        for (A a : l) {
            if (a != null && !a.equals(elem)) {
                res = res.prepend(a);
            }
        }
        return res.reverse();
    }

    public List<A> intersect(List<A> that) {
        ListBuffer<A> buf = new ListBuffer<>();
        for (A el : this) {
            if (that.contains(el)) {
                buf.append(el);
            }
        }
        return buf.toList();
    }

    public List<A> diff(List<A> that) {
        ListBuffer<A> buf = new ListBuffer<>();
        for (A el : this) {
            if (!that.contains(el)) {
                buf.append(el);
            }
        }
        return buf.toList();
    }

    public List<A> take(int n) {
        ListBuffer<A> buf = new ListBuffer<>();
        int count = 0;
        for (A el : this) {
            int count2 = count + 1;
            if (count == n) {
                break;
            }
            buf.append(el);
            count = count2;
        }
        return buf.toList();
    }

    public static <A> List<A> of(A x1) {
        return new List<>(x1, nil());
    }

    public static <A> List<A> of(A x1, A x2) {
        return new List<>(x1, of((Object) x2));
    }

    public static <A> List<A> of(A x1, A x2, A x3) {
        return new List<>(x1, of((Object) x2, (Object) x3));
    }

    public static <A> List<A> of(A x1, A x2, A x3, A... rest) {
        return new List<>(x1, new List(x2, new List(x3, from(rest))));
    }

    public static <A> List<A> from(A[] array) {
        List<A> xs = nil();
        if (array != null) {
            for (int i = array.length - 1; i >= 0; i--) {
                xs = new List<>(array[i], xs);
            }
        }
        return xs;
    }

    public static <A> List<A> from(Iterable<? extends A> coll) {
        ListBuffer<A> xs = new ListBuffer<>();
        for (A a : coll) {
            xs.append(a);
        }
        return xs.toList();
    }

    @Deprecated
    public static <A> List<A> fill(int len, A init) {
        List<A> l = nil();
        for (int i = 0; i < len; i++) {
            l = new List<>(init, l);
        }
        return l;
    }

    @Override // java.util.AbstractCollection, java.util.Collection, java.util.List
    public boolean isEmpty() {
        return this.tail == null;
    }

    public boolean nonEmpty() {
        return this.tail != null;
    }

    public int length() {
        List<A> l = this;
        int len = 0;
        while (l.tail != null) {
            l = l.tail;
            len++;
        }
        return len;
    }

    @Override // java.util.AbstractCollection, java.util.Collection, java.util.List
    public int size() {
        return length();
    }

    public List<A> setTail(List<A> tail) {
        this.tail = tail;
        return tail;
    }

    public List<A> prepend(A x) {
        return new List<>(x, this);
    }

    public List<A> prependList(List<A> xs) {
        if (isEmpty()) {
            return xs;
        }
        if (xs.isEmpty()) {
            return this;
        }
        if (xs.tail.isEmpty()) {
            return prepend(xs.head);
        }
        List<A> result = this;
        List<A> rev = xs.reverse();
        Assert.check(rev != xs);
        while (rev.nonEmpty()) {
            List<A> h = rev;
            rev = rev.tail;
            h.setTail(result);
            result = h;
        }
        return result;
    }

    public List<A> reverse() {
        if (isEmpty() || this.tail.isEmpty()) {
            return this;
        }
        List<A> rev = nil();
        for (List<A> l = this; l.nonEmpty(); l = l.tail) {
            rev = new List<>(l.head, rev);
        }
        return rev;
    }

    public List<A> append(A x) {
        return of((Object) x).prependList(this);
    }

    public List<A> appendList(List<A> x) {
        return x.prependList(this);
    }

    public List<A> appendList(ListBuffer<A> x) {
        return appendList(x.toList());
    }

    @Override // java.util.AbstractCollection, java.util.Collection, java.util.List
    public <T> T[] toArray(T[] tArr) {
        int i = 0;
        List<A> list = this;
        while (list.nonEmpty() && i < tArr.length) {
            tArr[i] = list.head;
            list = list.tail;
            i++;
        }
        if (list.isEmpty()) {
            if (i < tArr.length) {
                tArr[i] = 0;
            }
            return tArr;
        }
        return (T[]) toArray((Object[]) Array.newInstance(tArr.getClass().getComponentType(), size()));
    }

    @Override // java.util.AbstractCollection, java.util.Collection, java.util.List
    public Object[] toArray() {
        return toArray(new Object[size()]);
    }

    public String toString(String sep) {
        if (isEmpty()) {
            return "";
        }
        StringBuilder buf = new StringBuilder();
        buf.append(this.head);
        for (List<A> l = this.tail; l.nonEmpty(); l = l.tail) {
            buf.append(sep);
            buf.append(l.head);
        }
        return buf.toString();
    }

    @Override // java.util.AbstractCollection
    public String toString() {
        return toString(DocLint.TAGS_SEPARATOR);
    }

    @Override // java.util.Collection, java.util.List
    public int hashCode() {
        int h = 1;
        for (List<A> l = this; l.tail != null; l = l.tail) {
            h = (h * 31) + (l.head == null ? 0 : l.head.hashCode());
        }
        return h;
    }

    @Override // java.util.Collection, java.util.List
    public boolean equals(Object other) {
        if (other instanceof List) {
            return equals(this, (List) other);
        }
        if (!(other instanceof java.util.List)) {
            return false;
        }
        List<A> t = this;
        Iterator<?> oIter = ((java.util.List) other).iterator();
        while (t.tail != null && oIter.hasNext()) {
            Object o = oIter.next();
            if (t.head == null) {
                if (o != null) {
                    return false;
                }
                t = t.tail;
            } else {
                if (!t.head.equals(o)) {
                    return false;
                }
                t = t.tail;
            }
        }
        return t.isEmpty() && !oIter.hasNext();
    }

    public static boolean equals(List<?> list, List<?> list2) {
        while (list.tail != null && list2.tail != null) {
            if (list.head == null) {
                if (list2.head != null) {
                    return false;
                }
            } else if (!list.head.equals(list2.head)) {
                return false;
            }
            list = (List<A>) list.tail;
            list2 = (List<A>) list2.tail;
        }
        return list.tail == null && list2.tail == null;
    }

    @Override // java.util.AbstractCollection, java.util.Collection, java.util.List
    public boolean contains(Object x) {
        for (List<A> l = this; l.tail != null; l = l.tail) {
            if (x == null) {
                if (l.head == null) {
                    return true;
                }
            } else if (l.head.equals(x)) {
                return true;
            }
        }
        return false;
    }

    public A last() {
        A last = null;
        for (List<A> t = this; t.tail != null; t = t.tail) {
            last = t.head;
        }
        return last;
    }

    /* JADX WARN: Multi-variable type inference failed */
    public static <T> List<T> convert(Class<T> klass, List<?> list) {
        if (list == 0) {
            return null;
        }
        for (Object o : list) {
            klass.cast(o);
        }
        return list;
    }

    private static <A> Iterator<A> emptyIterator() {
        return (Iterator<A>) EMPTYITERATOR;
    }

    @Override // java.util.AbstractCollection, java.util.Collection, java.lang.Iterable, java.util.List
    public Iterator<A> iterator() {
        if (this.tail == null) {
            return emptyIterator();
        }
        return new Iterator<A>() { // from class: com.sun.tools.javac.util.List.3
            List<A> elems;

            {
                this.elems = List.this;
            }

            @Override // java.util.Iterator
            public boolean hasNext() {
                return this.elems.tail != null;
            }

            @Override // java.util.Iterator
            public A next() {
                if (this.elems.tail == null) {
                    throw new NoSuchElementException();
                }
                A result = this.elems.head;
                this.elems = this.elems.tail;
                return result;
            }

            @Override // java.util.Iterator
            public void remove() {
                throw new UnsupportedOperationException();
            }
        };
    }

    @Override // java.util.List
    public A get(int index) {
        if (index < 0) {
            throw new IndexOutOfBoundsException(String.valueOf(index));
        }
        List<A> l = this;
        int i = index;
        while (true) {
            int i2 = i - 1;
            if (i <= 0 || l.isEmpty()) {
                break;
            }
            l = l.tail;
            i = i2;
        }
        if (l.isEmpty()) {
            throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size());
        }
        return l.head;
    }

    @Override // java.util.List
    public boolean addAll(int index, Collection<? extends A> c) {
        if (c.isEmpty()) {
            return false;
        }
        throw new UnsupportedOperationException();
    }

    @Override // java.util.List
    public A set(int index, A element) {
        throw new UnsupportedOperationException();
    }

    @Override // java.util.List
    public void add(int index, A element) {
        throw new UnsupportedOperationException();
    }

    @Override // java.util.List
    public A remove(int index) {
        throw new UnsupportedOperationException();
    }

    @Override // java.util.List
    public int indexOf(Object o) {
        int i = 0;
        List<A> l = this;
        while (l.tail != null) {
            if (l.head == null) {
                if (o != null) {
                    l = l.tail;
                    i++;
                } else {
                    return i;
                }
            } else if (!l.head.equals(o)) {
                l = l.tail;
                i++;
            } else {
                return i;
            }
        }
        return -1;
    }

    /* JADX WARN: Removed duplicated region for block: B:11:0x0016  */
    @Override // java.util.List
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public int lastIndexOf(java.lang.Object r5) {
        /*
            r4 = this;
            r0 = -1
            r1 = 0
            r2 = r4
        L3:
            com.sun.tools.javac.util.List<A> r3 = r2.tail
            if (r3 == 0) goto L1c
            A r3 = r2.head
            if (r3 != 0) goto Le
            if (r5 != 0) goto L17
            goto L16
        Le:
            A r3 = r2.head
            boolean r3 = r3.equals(r5)
            if (r3 == 0) goto L17
        L16:
            r0 = r1
        L17:
            com.sun.tools.javac.util.List<A> r2 = r2.tail
            int r1 = r1 + 1
            goto L3
        L1c:
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.util.List.lastIndexOf(java.lang.Object):int");
    }

    @Override // java.util.List
    public ListIterator<A> listIterator() {
        return Collections.unmodifiableList(new ArrayList(this)).listIterator();
    }

    @Override // java.util.List
    public ListIterator<A> listIterator(int index) {
        return Collections.unmodifiableList(new ArrayList(this)).listIterator(index);
    }

    @Override // java.util.List
    public java.util.List<A> subList(int fromIndex, int toIndex) {
        if (fromIndex < 0 || toIndex > size() || fromIndex > toIndex) {
            throw new IllegalArgumentException();
        }
        ArrayList<A> a = new ArrayList<>(toIndex - fromIndex);
        List<A> l = this;
        for (int i = 0; l.tail != null && i != toIndex; i++) {
            if (i >= fromIndex) {
                a.add(l.head);
            }
            l = l.tail;
        }
        return Collections.unmodifiableList(a);
    }
}
