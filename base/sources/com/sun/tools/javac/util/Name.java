package com.sun.tools.javac.util;

/* JADX INFO: loaded from: classes.dex */
public abstract class Name implements javax.lang.model.element.Name {
    public final Table table;

    public abstract byte[] getByteArray();

    public abstract byte getByteAt(int i);

    public abstract int getByteLength();

    public abstract int getByteOffset();

    public abstract int getIndex();

    protected Name(Table table) {
        this.table = table;
    }

    @Override // javax.lang.model.element.Name
    public boolean contentEquals(CharSequence cs) {
        return toString().equals(cs.toString());
    }

    @Override // java.lang.CharSequence
    public int length() {
        return toString().length();
    }

    @Override // java.lang.CharSequence
    public char charAt(int index) {
        return toString().charAt(index);
    }

    @Override // java.lang.CharSequence
    public CharSequence subSequence(int start, int end) {
        return toString().subSequence(start, end);
    }

    public Name append(Name n) {
        int len = getByteLength();
        byte[] bs = new byte[n.getByteLength() + len];
        getBytes(bs, 0);
        n.getBytes(bs, len);
        return this.table.fromUtf(bs, 0, bs.length);
    }

    public Name append(char c, Name n) {
        int len = getByteLength();
        byte[] bs = new byte[len + 1 + n.getByteLength()];
        getBytes(bs, 0);
        bs[len] = (byte) c;
        n.getBytes(bs, len + 1);
        return this.table.fromUtf(bs, 0, bs.length);
    }

    public int compareTo(Name other) {
        return other.getIndex() - getIndex();
    }

    public boolean isEmpty() {
        return getByteLength() == 0;
    }

    public int lastIndexOf(byte b) {
        byte[] bytes = getByteArray();
        int offset = getByteOffset();
        int i = getByteLength() - 1;
        while (i >= 0 && bytes[offset + i] != b) {
            i--;
        }
        return i;
    }

    public boolean startsWith(Name prefix) {
        byte[] thisBytes = getByteArray();
        int thisOffset = getByteOffset();
        int thisLength = getByteLength();
        byte[] prefixBytes = prefix.getByteArray();
        int prefixOffset = prefix.getByteOffset();
        int prefixLength = prefix.getByteLength();
        int i = 0;
        while (i < prefixLength && i < thisLength && thisBytes[thisOffset + i] == prefixBytes[prefixOffset + i]) {
            i++;
        }
        return i == prefixLength;
    }

    public Name subName(int start, int end) {
        if (end < start) {
            end = start;
        }
        return this.table.fromUtf(getByteArray(), getByteOffset() + start, end - start);
    }

    @Override // java.lang.CharSequence
    public String toString() {
        return Convert.utf2string(getByteArray(), getByteOffset(), getByteLength());
    }

    public byte[] toUtf() {
        byte[] bs = new byte[getByteLength()];
        getBytes(bs, 0);
        return bs;
    }

    public void getBytes(byte[] cs, int start) {
        System.arraycopy(getByteArray(), getByteOffset(), cs, start, getByteLength());
    }

    public static abstract class Table {
        public final Names names;

        public abstract void dispose();

        public abstract Name fromChars(char[] cArr, int i, int i2);

        public abstract Name fromUtf(byte[] bArr, int i, int i2);

        Table(Names names) {
            this.names = names;
        }

        public Name fromString(String s) {
            char[] cs = s.toCharArray();
            return fromChars(cs, 0, cs.length);
        }

        public Name fromUtf(byte[] cs) {
            return fromUtf(cs, 0, cs.length);
        }

        protected static int hashValue(byte[] bytes, int offset, int length) {
            int h = 0;
            int off = offset;
            int i = 0;
            while (i < length) {
                h = ((h << 5) - h) + bytes[off];
                i++;
                off++;
            }
            return h;
        }

        protected static boolean equals(byte[] bytes1, int offset1, byte[] bytes2, int offset2, int length) {
            int i = 0;
            while (i < length && bytes1[offset1 + i] == bytes2[offset2 + i]) {
                i++;
            }
            return i == length;
        }
    }
}
