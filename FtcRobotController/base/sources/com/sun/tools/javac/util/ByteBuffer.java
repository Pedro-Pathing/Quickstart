package com.sun.tools.javac.util;

import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;

/* JADX INFO: loaded from: classes.dex */
public class ByteBuffer {
    public byte[] elems;
    public int length;

    public ByteBuffer() {
        this(64);
    }

    public ByteBuffer(int initialSize) {
        this.elems = new byte[initialSize];
        this.length = 0;
    }

    public void appendByte(int b) {
        this.elems = ArrayUtils.ensureCapacity(this.elems, this.length);
        byte[] bArr = this.elems;
        int i = this.length;
        this.length = i + 1;
        bArr[i] = (byte) b;
    }

    public void appendBytes(byte[] bs, int start, int len) {
        this.elems = ArrayUtils.ensureCapacity(this.elems, this.length + len);
        System.arraycopy(bs, start, this.elems, this.length, len);
        this.length += len;
    }

    public void appendBytes(byte[] bs) {
        appendBytes(bs, 0, bs.length);
    }

    public void appendChar(int x) {
        this.elems = ArrayUtils.ensureCapacity(this.elems, this.length + 1);
        this.elems[this.length] = (byte) ((x >> 8) & 255);
        this.elems[this.length + 1] = (byte) (x & 255);
        this.length += 2;
    }

    public void appendInt(int x) {
        this.elems = ArrayUtils.ensureCapacity(this.elems, this.length + 3);
        this.elems[this.length] = (byte) ((x >> 24) & 255);
        this.elems[this.length + 1] = (byte) ((x >> 16) & 255);
        this.elems[this.length + 2] = (byte) ((x >> 8) & 255);
        this.elems[this.length + 3] = (byte) (x & 255);
        this.length += 4;
    }

    public void appendLong(long x) {
        ByteArrayOutputStream buffer = new ByteArrayOutputStream(8);
        DataOutputStream bufout = new DataOutputStream(buffer);
        try {
            bufout.writeLong(x);
            appendBytes(buffer.toByteArray(), 0, 8);
        } catch (IOException e) {
            throw new AssertionError("write");
        }
    }

    public void appendFloat(float x) {
        ByteArrayOutputStream buffer = new ByteArrayOutputStream(4);
        DataOutputStream bufout = new DataOutputStream(buffer);
        try {
            bufout.writeFloat(x);
            appendBytes(buffer.toByteArray(), 0, 4);
        } catch (IOException e) {
            throw new AssertionError("write");
        }
    }

    public void appendDouble(double x) {
        ByteArrayOutputStream buffer = new ByteArrayOutputStream(8);
        DataOutputStream bufout = new DataOutputStream(buffer);
        try {
            bufout.writeDouble(x);
            appendBytes(buffer.toByteArray(), 0, 8);
        } catch (IOException e) {
            throw new AssertionError("write");
        }
    }

    public void appendName(Name name) {
        appendBytes(name.getByteArray(), name.getByteOffset(), name.getByteLength());
    }

    public void reset() {
        this.length = 0;
    }

    public Name toName(Names names) {
        return names.fromUtf(this.elems, 0, this.length);
    }
}
