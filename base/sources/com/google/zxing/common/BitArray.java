package com.google.zxing.common;

import dk.sgjesse.r8api.DescriptorUtils;
import java.util.Arrays;

/* JADX INFO: loaded from: classes8.dex */
public final class BitArray implements Cloneable {
    private int[] bits;
    private int size;

    public BitArray() {
        this.size = 0;
        this.bits = new int[1];
    }

    public BitArray(int size) {
        this.size = size;
        this.bits = makeArray(size);
    }

    BitArray(int[] bits, int size) {
        this.bits = bits;
        this.size = size;
    }

    public int getSize() {
        return this.size;
    }

    public int getSizeInBytes() {
        return (this.size + 7) / 8;
    }

    private void ensureCapacity(int size) {
        if (size > (this.bits.length << 5)) {
            int[] newBits = makeArray(size);
            System.arraycopy(this.bits, 0, newBits, 0, this.bits.length);
            this.bits = newBits;
        }
    }

    public boolean get(int i) {
        return (this.bits[i / 32] & (1 << (i & 31))) != 0;
    }

    public void set(int i) {
        int[] iArr = this.bits;
        int i2 = i / 32;
        iArr[i2] = iArr[i2] | (1 << (i & 31));
    }

    public void flip(int i) {
        int[] iArr = this.bits;
        int i2 = i / 32;
        iArr[i2] = iArr[i2] ^ (1 << (i & 31));
    }

    public int getNextSet(int from) {
        if (from >= this.size) {
            return this.size;
        }
        int bitsOffset = from / 32;
        int currentBits = this.bits[bitsOffset] & (-(1 << (from & 31)));
        while (currentBits == 0) {
            bitsOffset++;
            if (bitsOffset == this.bits.length) {
                return this.size;
            }
            currentBits = this.bits[bitsOffset];
        }
        return Math.min((bitsOffset << 5) + Integer.numberOfTrailingZeros(currentBits), this.size);
    }

    public int getNextUnset(int from) {
        if (from >= this.size) {
            return this.size;
        }
        int bitsOffset = from / 32;
        int currentBits = (~this.bits[bitsOffset]) & (-(1 << (from & 31)));
        while (currentBits == 0) {
            bitsOffset++;
            if (bitsOffset == this.bits.length) {
                return this.size;
            }
            currentBits = ~this.bits[bitsOffset];
        }
        return Math.min((bitsOffset << 5) + Integer.numberOfTrailingZeros(currentBits), this.size);
    }

    public void setBulk(int i, int newBits) {
        this.bits[i / 32] = newBits;
    }

    public void setRange(int start, int end) {
        if (end < start || start < 0 || end > this.size) {
            throw new IllegalArgumentException();
        }
        if (end == start) {
            return;
        }
        int end2 = end - 1;
        int firstInt = start / 32;
        int lastInt = end2 / 32;
        int i = firstInt;
        while (i <= lastInt) {
            int firstBit = i > firstInt ? 0 : start & 31;
            int lastBit = i < lastInt ? 31 : end2 & 31;
            int mask = (2 << lastBit) - (1 << firstBit);
            int[] iArr = this.bits;
            iArr[i] = iArr[i] | mask;
            i++;
        }
    }

    public void clear() {
        int max = this.bits.length;
        for (int i = 0; i < max; i++) {
            this.bits[i] = 0;
        }
    }

    public boolean isRange(int start, int end, boolean value) {
        if (end < start || start < 0 || end > this.size) {
            throw new IllegalArgumentException();
        }
        if (end == start) {
            return true;
        }
        int end2 = end - 1;
        int firstInt = start / 32;
        int lastInt = end2 / 32;
        int i = firstInt;
        while (i <= lastInt) {
            int firstBit = i > firstInt ? 0 : start & 31;
            int lastBit = i < lastInt ? 31 : end2 & 31;
            int mask = (2 << lastBit) - (1 << firstBit);
            if ((this.bits[i] & mask) == (value ? mask : 0)) {
                i++;
            } else {
                return false;
            }
        }
        return true;
    }

    public void appendBit(boolean bit) {
        ensureCapacity(this.size + 1);
        if (bit) {
            int[] iArr = this.bits;
            int i = this.size / 32;
            iArr[i] = iArr[i] | (1 << (this.size & 31));
        }
        this.size++;
    }

    public void appendBits(int value, int numBits) {
        if (numBits < 0 || numBits > 32) {
            throw new IllegalArgumentException("Num bits must be between 0 and 32");
        }
        ensureCapacity(this.size + numBits);
        for (int numBitsLeft = numBits; numBitsLeft > 0; numBitsLeft--) {
            boolean z = true;
            if (((value >> (numBitsLeft - 1)) & 1) != 1) {
                z = false;
            }
            appendBit(z);
        }
    }

    public void appendBitArray(BitArray other) {
        int otherSize = other.size;
        ensureCapacity(this.size + otherSize);
        for (int i = 0; i < otherSize; i++) {
            appendBit(other.get(i));
        }
    }

    public void xor(BitArray other) {
        if (this.size != other.size) {
            throw new IllegalArgumentException("Sizes don't match");
        }
        for (int i = 0; i < this.bits.length; i++) {
            int[] iArr = this.bits;
            iArr[i] = iArr[i] ^ other.bits[i];
        }
    }

    public void toBytes(int bitOffset, byte[] array, int offset, int numBytes) {
        for (int i = 0; i < numBytes; i++) {
            int theByte = 0;
            for (int j = 0; j < 8; j++) {
                if (get(bitOffset)) {
                    theByte |= 1 << (7 - j);
                }
                bitOffset++;
            }
            int j2 = offset + i;
            array[j2] = (byte) theByte;
        }
    }

    public int[] getBitArray() {
        return this.bits;
    }

    public void reverse() {
        int[] iArr = new int[this.bits.length];
        int i = (this.size - 1) / 32;
        int i2 = i + 1;
        for (int i3 = 0; i3 < i2; i3++) {
            long j = this.bits[i3];
            long j2 = ((j & 1431655765) << 1) | ((j >> 1) & 1431655765);
            long j3 = ((j2 & 858993459) << 2) | ((j2 >> 2) & 858993459);
            long j4 = ((j3 & 252645135) << 4) | ((j3 >> 4) & 252645135);
            long j5 = ((j4 & 16711935) << 8) | ((j4 >> 8) & 16711935);
            iArr[i - i3] = (int) (((j5 & 65535) << 16) | ((j5 >> 16) & 65535));
        }
        int i4 = i2 << 5;
        if (this.size != i4) {
            int i5 = i4 - this.size;
            int i6 = iArr[0] >>> i5;
            for (int i7 = 1; i7 < i2; i7++) {
                int i8 = iArr[i7];
                iArr[i7 - 1] = i6 | (i8 << (32 - i5));
                i6 = i8 >>> i5;
            }
            iArr[i2 - 1] = i6;
        }
        this.bits = iArr;
    }

    private static int[] makeArray(int size) {
        return new int[(size + 31) / 32];
    }

    public boolean equals(Object o) {
        if (!(o instanceof BitArray)) {
            return false;
        }
        BitArray other = (BitArray) o;
        return this.size == other.size && Arrays.equals(this.bits, other.bits);
    }

    public int hashCode() {
        return (this.size * 31) + Arrays.hashCode(this.bits);
    }

    public String toString() {
        StringBuilder result = new StringBuilder(this.size + (this.size / 8) + 1);
        for (int i = 0; i < this.size; i++) {
            if ((i & 7) == 0) {
                result.append(' ');
            }
            result.append(get(i) ? 'X' : DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
        }
        return result.toString();
    }

    /* JADX INFO: renamed from: clone, reason: merged with bridge method [inline-methods] */
    public BitArray m138clone() {
        return new BitArray((int[]) this.bits.clone(), this.size);
    }
}
