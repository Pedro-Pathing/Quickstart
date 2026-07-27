package com.google.zxing.aztec.encoder;

import com.google.zxing.common.BitArray;
import java.util.Deque;
import java.util.Iterator;
import java.util.LinkedList;

/* JADX INFO: loaded from: classes8.dex */
final class State {
    static final State INITIAL_STATE = new State(Token.EMPTY, 0, 0, 0);
    private final int binaryShiftByteCount;
    private final int bitCount;
    private final int mode;
    private final Token token;

    private State(Token token, int mode, int binaryBytes, int bitCount) {
        this.token = token;
        this.mode = mode;
        this.binaryShiftByteCount = binaryBytes;
        this.bitCount = bitCount;
    }

    int getMode() {
        return this.mode;
    }

    Token getToken() {
        return this.token;
    }

    int getBinaryShiftByteCount() {
        return this.binaryShiftByteCount;
    }

    int getBitCount() {
        return this.bitCount;
    }

    State latchAndAppend(int mode, int value) {
        int bitCount = this.bitCount;
        Token token = this.token;
        if (mode != this.mode) {
            int latch = HighLevelEncoder.LATCH_TABLE[this.mode][mode];
            token = token.add(65535 & latch, latch >> 16);
            bitCount += latch >> 16;
        }
        int latchModeBitCount = mode == 2 ? 4 : 5;
        return new State(token.add(value, latchModeBitCount), mode, 0, bitCount + latchModeBitCount);
    }

    State shiftAndAppend(int mode, int value) {
        Token token = this.token;
        int thisModeBitCount = this.mode == 2 ? 4 : 5;
        return new State(token.add(HighLevelEncoder.SHIFT_TABLE[this.mode][mode], thisModeBitCount).add(value, 5), this.mode, 0, this.bitCount + thisModeBitCount + 5);
    }

    State addBinaryShiftChar(int index) {
        Token token = this.token;
        int mode = this.mode;
        int bitCount = this.bitCount;
        if (this.mode == 4 || this.mode == 2) {
            int latch = HighLevelEncoder.LATCH_TABLE[mode][0];
            token = token.add(65535 & latch, latch >> 16);
            bitCount += latch >> 16;
            mode = 0;
        }
        int deltaBitCount = (this.binaryShiftByteCount == 0 || this.binaryShiftByteCount == 31) ? 18 : this.binaryShiftByteCount == 62 ? 9 : 8;
        State result = new State(token, mode, this.binaryShiftByteCount + 1, bitCount + deltaBitCount);
        return result.binaryShiftByteCount == 2078 ? result.endBinaryShift(index + 1) : result;
    }

    State endBinaryShift(int index) {
        if (this.binaryShiftByteCount == 0) {
            return this;
        }
        Token token = this.token.addBinaryShift(index - this.binaryShiftByteCount, this.binaryShiftByteCount);
        return new State(token, this.mode, 0, this.bitCount);
    }

    boolean isBetterThanOrEqualTo(State other) {
        int newModeBitCount = this.bitCount + (HighLevelEncoder.LATCH_TABLE[this.mode][other.mode] >> 16);
        if (this.binaryShiftByteCount < other.binaryShiftByteCount) {
            newModeBitCount += calculateBinaryShiftCost(other) - calculateBinaryShiftCost(this);
        } else if (this.binaryShiftByteCount > other.binaryShiftByteCount && other.binaryShiftByteCount > 0) {
            newModeBitCount += 10;
        }
        return newModeBitCount <= other.bitCount;
    }

    BitArray toBitArray(byte[] text) {
        Deque<Token> symbols = new LinkedList<>();
        for (Token token = endBinaryShift(text.length).token; token != null; token = token.getPrevious()) {
            symbols.addFirst(token);
        }
        BitArray bitArray = new BitArray();
        Iterator<Token> it = symbols.iterator();
        while (it.hasNext()) {
            it.next().appendTo(bitArray, text);
        }
        return bitArray;
    }

    public String toString() {
        return String.format("%s bits=%d bytes=%d", HighLevelEncoder.MODE_NAMES[this.mode], Integer.valueOf(this.bitCount), Integer.valueOf(this.binaryShiftByteCount));
    }

    private static int calculateBinaryShiftCost(State state) {
        if (state.binaryShiftByteCount > 62) {
            return 21;
        }
        if (state.binaryShiftByteCount > 31) {
            return 20;
        }
        if (state.binaryShiftByteCount > 0) {
            return 10;
        }
        return 0;
    }
}
