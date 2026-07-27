package com.sun.tools.javac.util;

import com.sun.tools.javac.util.Name;
import java.lang.ref.WeakReference;

/* JADX INFO: loaded from: classes.dex */
public class UnsharedNameTable extends Name.Table {
    private int hashMask;
    private HashEntry[] hashes;
    public int index;

    public static Name.Table create(Names names) {
        return new UnsharedNameTable(names);
    }

    static class HashEntry extends WeakReference<NameImpl> {
        HashEntry next;

        HashEntry(NameImpl referent) {
            super(referent);
        }
    }

    public UnsharedNameTable(Names names, int hashSize) {
        super(names);
        this.hashes = null;
        this.hashMask = hashSize - 1;
        this.hashes = new HashEntry[hashSize];
    }

    public UnsharedNameTable(Names names) {
        this(names, 32768);
    }

    @Override // com.sun.tools.javac.util.Name.Table
    public Name fromChars(char[] cs, int start, int len) {
        byte[] name = new byte[len * 3];
        int nbytes = Convert.chars2utf(cs, start, name, 0, len);
        return fromUtf(name, 0, nbytes);
    }

    @Override // com.sun.tools.javac.util.Name.Table
    public Name fromUtf(byte[] cs, int start, int len) {
        int h = hashValue(cs, start, len) & this.hashMask;
        HashEntry element = this.hashes[h];
        HashEntry previousNonNullTableEntry = null;
        HashEntry firstTableEntry = element;
        while (element != null && element != null) {
            NameImpl n = (NameImpl) element.get();
            if (n == null) {
                if (firstTableEntry == element) {
                    HashEntry[] hashEntryArr = this.hashes;
                    HashEntry hashEntry = element.next;
                    firstTableEntry = hashEntry;
                    hashEntryArr[h] = hashEntry;
                } else {
                    Assert.checkNonNull(previousNonNullTableEntry, "previousNonNullTableEntry cannot be null here.");
                    previousNonNullTableEntry.next = element.next;
                }
            } else {
                if (n.getByteLength() == len && equals(n.bytes, 0, cs, start, len)) {
                    return n;
                }
                previousNonNullTableEntry = element;
            }
            element = element.next;
        }
        byte[] bytes = new byte[len];
        System.arraycopy(cs, start, bytes, 0, len);
        int i = this.index;
        this.index = i + 1;
        NameImpl n2 = new NameImpl(this, bytes, i);
        HashEntry newEntry = new HashEntry(n2);
        if (previousNonNullTableEntry == null) {
            this.hashes[h] = newEntry;
        } else {
            Assert.checkNull((Object) previousNonNullTableEntry.next, "previousNonNullTableEntry.next must be null.");
            previousNonNullTableEntry.next = newEntry;
        }
        return n2;
    }

    @Override // com.sun.tools.javac.util.Name.Table
    public void dispose() {
        this.hashes = null;
    }

    static class NameImpl extends Name {
        final byte[] bytes;
        final int index;

        NameImpl(UnsharedNameTable table, byte[] bytes, int index) {
            super(table);
            this.bytes = bytes;
            this.index = index;
        }

        @Override // com.sun.tools.javac.util.Name
        public int getIndex() {
            return this.index;
        }

        @Override // com.sun.tools.javac.util.Name
        public int getByteLength() {
            return this.bytes.length;
        }

        @Override // com.sun.tools.javac.util.Name
        public byte getByteAt(int i) {
            return this.bytes[i];
        }

        @Override // com.sun.tools.javac.util.Name
        public byte[] getByteArray() {
            return this.bytes;
        }

        @Override // com.sun.tools.javac.util.Name
        public int getByteOffset() {
            return 0;
        }
    }
}
