package com.sun.tools.javac.util;

import com.qualcomm.robotcore.eventloop.SyncdDevice;
import java.util.Arrays;
import java.util.Random;

/* JADX INFO: loaded from: classes.dex */
public class Bits {
    private static final int[] unassignedBits = new int[0];
    private static final int wordlen = 32;
    private static final int wordmask = 31;
    private static final int wordshift = 5;
    public int[] bits;
    protected BitsState currentState;

    protected enum BitsState {
        UNKNOWN,
        UNINIT,
        NORMAL;

        static BitsState getState(int[] someBits, boolean reset) {
            if (!reset) {
                if (someBits != Bits.unassignedBits) {
                    return NORMAL;
                }
                return UNINIT;
            }
            return UNKNOWN;
        }
    }

    public Bits() {
        this(false);
    }

    public Bits(Bits someBits) {
        this(someBits.dup().bits, BitsState.getState(someBits.bits, false));
    }

    public Bits(boolean reset) {
        this(unassignedBits, BitsState.getState(unassignedBits, reset));
    }

    protected Bits(int[] bits, BitsState initState) {
        this.bits = null;
        this.bits = bits;
        this.currentState = initState;
        switch (initState) {
            case UNKNOWN:
                this.bits = null;
                break;
            case NORMAL:
                Assert.check(bits != unassignedBits);
                break;
        }
    }

    protected void sizeTo(int len) {
        if (this.bits.length < len) {
            this.bits = Arrays.copyOf(this.bits, len);
        }
    }

    public void clear() {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        for (int i = 0; i < this.bits.length; i++) {
            this.bits[i] = 0;
        }
        this.currentState = BitsState.NORMAL;
    }

    public void reset() {
        internalReset();
    }

    protected void internalReset() {
        this.bits = null;
        this.currentState = BitsState.UNKNOWN;
    }

    public boolean isReset() {
        return this.currentState == BitsState.UNKNOWN;
    }

    public Bits assign(Bits someBits) {
        this.bits = someBits.dup().bits;
        this.currentState = BitsState.NORMAL;
        return this;
    }

    public Bits dup() {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        Bits tmp = new Bits();
        tmp.bits = dupBits();
        this.currentState = BitsState.NORMAL;
        return tmp;
    }

    protected int[] dupBits() {
        if (this.currentState != BitsState.NORMAL) {
            return this.bits;
        }
        int[] result = new int[this.bits.length];
        System.arraycopy(this.bits, 0, result, 0, this.bits.length);
        return result;
    }

    public void incl(int x) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        Assert.check(x >= 0, "Value of x " + x);
        sizeTo((x >>> 5) + 1);
        this.bits[x >>> 5] = this.bits[x >>> 5] | (1 << (x & 31));
        this.currentState = BitsState.NORMAL;
    }

    public void inclRange(int start, int limit) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        sizeTo((limit >>> 5) + 1);
        for (int x = start; x < limit; x++) {
            this.bits[x >>> 5] = this.bits[x >>> 5] | (1 << (x & 31));
        }
        this.currentState = BitsState.NORMAL;
    }

    public void excludeFrom(int start) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        Bits temp = new Bits();
        temp.sizeTo(this.bits.length);
        temp.inclRange(0, start);
        internalAndSet(temp);
        this.currentState = BitsState.NORMAL;
    }

    public void excl(int x) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        Assert.check(x >= 0);
        sizeTo((x >>> 5) + 1);
        this.bits[x >>> 5] = this.bits[x >>> 5] & (~(1 << (x & 31)));
        this.currentState = BitsState.NORMAL;
    }

    public boolean isMember(int x) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        return x >= 0 && x < (this.bits.length << 5) && (this.bits[x >>> 5] & (1 << (x & 31))) != 0;
    }

    public Bits andSet(Bits xs) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        internalAndSet(xs);
        this.currentState = BitsState.NORMAL;
        return this;
    }

    protected void internalAndSet(Bits xs) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        sizeTo(xs.bits.length);
        for (int i = 0; i < xs.bits.length; i++) {
            this.bits[i] = this.bits[i] & xs.bits[i];
        }
    }

    public Bits orSet(Bits xs) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        sizeTo(xs.bits.length);
        for (int i = 0; i < xs.bits.length; i++) {
            this.bits[i] = this.bits[i] | xs.bits[i];
        }
        this.currentState = BitsState.NORMAL;
        return this;
    }

    public Bits diffSet(Bits xs) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        for (int i = 0; i < this.bits.length; i++) {
            if (i < xs.bits.length) {
                this.bits[i] = this.bits[i] & (~xs.bits[i]);
            }
        }
        this.currentState = BitsState.NORMAL;
        return this;
    }

    public Bits xorSet(Bits xs) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        sizeTo(xs.bits.length);
        for (int i = 0; i < xs.bits.length; i++) {
            this.bits[i] = this.bits[i] ^ xs.bits[i];
        }
        this.currentState = BitsState.NORMAL;
        return this;
    }

    private static int trailingZeroBits(int x) {
        Assert.check(true);
        if (x == 0) {
            return 32;
        }
        int n = 1;
        if ((65535 & x) == 0) {
            n = 1 + 16;
            x >>>= 16;
        }
        if ((x & 255) == 0) {
            n += 8;
            x >>>= 8;
        }
        if ((x & 15) == 0) {
            n += 4;
            x >>>= 4;
        }
        if ((x & 3) == 0) {
            n += 2;
            x >>>= 2;
        }
        return n - (x & 1);
    }

    public int nextBit(int x) {
        Assert.check(this.currentState != BitsState.UNKNOWN);
        int windex = x >>> 5;
        if (windex >= this.bits.length) {
            return -1;
        }
        int word = this.bits[windex] & (~((1 << (x & 31)) - 1));
        while (word == 0) {
            windex++;
            if (windex >= this.bits.length) {
                return -1;
            }
            word = this.bits[windex];
        }
        return (windex << 5) + trailingZeroBits(word);
    }

    public String toString() {
        if (this.bits != null && this.bits.length > 0) {
            char[] digits = new char[this.bits.length * 32];
            for (int i = 0; i < this.bits.length * 32; i++) {
                digits[i] = isMember(i) ? '1' : '0';
            }
            return new String(digits);
        }
        return "[]";
    }

    public static void main(String[] args) {
        int k;
        Random r = new Random();
        Bits bits = new Bits();
        for (int i = 0; i < 125; i++) {
            do {
                k = r.nextInt(SyncdDevice.msAbnormalReopenInterval);
            } while (bits.isMember(k));
            System.out.println("adding " + k);
            bits.incl(k);
        }
        int count = 0;
        for (int i2 = bits.nextBit(0); i2 >= 0; i2 = bits.nextBit(i2 + 1)) {
            System.out.println("found " + i2);
            count++;
        }
        if (count != 125) {
            throw new Error();
        }
    }
}
