package com.sun.tools.javac.util;

import com.sun.tools.javac.util.Name;
import java.lang.ref.SoftReference;

/* JADX INFO: loaded from: classes.dex */
public class SharedNameTable extends Name.Table {
    private static List<SoftReference<SharedNameTable>> freelist = List.nil();
    public byte[] bytes;
    private int hashMask;
    private NameImpl[] hashes;
    private int nc;

    public static synchronized SharedNameTable create(Names names) {
        while (freelist.nonEmpty()) {
            SharedNameTable t = freelist.head.get();
            freelist = freelist.tail;
            if (t != null) {
                return t;
            }
        }
        return new SharedNameTable(names);
    }

    private static synchronized void dispose(SharedNameTable t) {
        freelist = freelist.prepend(new SoftReference<>(t));
    }

    public SharedNameTable(Names names, int hashSize, int nameSize) {
        super(names);
        this.nc = 0;
        this.hashMask = hashSize - 1;
        this.hashes = new NameImpl[hashSize];
        this.bytes = new byte[nameSize];
    }

    public SharedNameTable(Names names) {
        this(names, 32768, 131072);
    }

    @Override // com.sun.tools.javac.util.Name.Table
    public Name fromChars(char[] cs, int start, int len) {
        int nc = this.nc;
        byte[] bytes = ArrayUtils.ensureCapacity(this.bytes, (len * 3) + nc);
        this.bytes = bytes;
        int nbytes = Convert.chars2utf(cs, start, bytes, nc, len) - nc;
        int h = hashValue(bytes, nc, nbytes) & this.hashMask;
        NameImpl n = this.hashes[h];
        while (n != null && (n.getByteLength() != nbytes || !equals(bytes, n.index, bytes, nc, nbytes))) {
            n = n.next;
        }
        if (n == null) {
            n = new NameImpl(this);
            n.index = nc;
            n.length = nbytes;
            n.next = this.hashes[h];
            this.hashes[h] = n;
            this.nc = nc + nbytes;
            if (nbytes == 0) {
                this.nc++;
            }
        }
        return n;
    }

    @Override // com.sun.tools.javac.util.Name.Table
    public Name fromUtf(byte[] cs, int start, int len) {
        int h = hashValue(cs, start, len) & this.hashMask;
        NameImpl n = this.hashes[h];
        byte[] names = this.bytes;
        while (n != null && (n.getByteLength() != len || !equals(names, n.index, cs, start, len))) {
            n = n.next;
        }
        if (n == null) {
            int nc = this.nc;
            byte[] names2 = ArrayUtils.ensureCapacity(names, nc + len);
            this.bytes = names2;
            System.arraycopy(cs, start, names2, nc, len);
            n = new NameImpl(this);
            n.index = nc;
            n.length = len;
            n.next = this.hashes[h];
            this.hashes[h] = n;
            this.nc = nc + len;
            if (len == 0) {
                this.nc++;
            }
        }
        return n;
    }

    @Override // com.sun.tools.javac.util.Name.Table
    public void dispose() {
        dispose(this);
    }

    static class NameImpl extends Name {
        int index;
        int length;
        NameImpl next;

        NameImpl(SharedNameTable table) {
            super(table);
        }

        @Override // com.sun.tools.javac.util.Name
        public int getIndex() {
            return this.index;
        }

        @Override // com.sun.tools.javac.util.Name
        public int getByteLength() {
            return this.length;
        }

        @Override // com.sun.tools.javac.util.Name
        public byte getByteAt(int i) {
            return getByteArray()[this.index + i];
        }

        @Override // com.sun.tools.javac.util.Name
        public byte[] getByteArray() {
            return ((SharedNameTable) this.table).bytes;
        }

        @Override // com.sun.tools.javac.util.Name
        public int getByteOffset() {
            return this.index;
        }

        @Override // javax.lang.model.element.Name
        public int hashCode() {
            return this.index;
        }

        @Override // javax.lang.model.element.Name
        public boolean equals(Object other) {
            return (other instanceof Name) && this.table == ((Name) other).table && this.index == ((Name) other).getIndex();
        }
    }
}
