package com.google.zxing.aztec.encoder;

import com.google.zxing.common.BitArray;

/* JADX INFO: loaded from: classes8.dex */
final class BinaryShiftToken extends Token {
    private final short binaryShiftByteCount;
    private final short binaryShiftStart;

    BinaryShiftToken(Token previous, int binaryShiftStart, int binaryShiftByteCount) {
        super(previous);
        this.binaryShiftStart = (short) binaryShiftStart;
        this.binaryShiftByteCount = (short) binaryShiftByteCount;
    }

    @Override // com.google.zxing.aztec.encoder.Token
    public void appendTo(BitArray bitArray, byte[] text) {
        for (int i = 0; i < this.binaryShiftByteCount; i++) {
            if (i == 0 || (i == 31 && this.binaryShiftByteCount <= 62)) {
                bitArray.appendBits(31, 5);
                if (this.binaryShiftByteCount > 62) {
                    bitArray.appendBits(this.binaryShiftByteCount - 31, 16);
                } else if (i == 0) {
                    bitArray.appendBits(Math.min((int) this.binaryShiftByteCount, 31), 5);
                } else {
                    bitArray.appendBits(this.binaryShiftByteCount - 31, 5);
                }
            }
            bitArray.appendBits(text[this.binaryShiftStart + i], 8);
        }
    }

    public String toString() {
        return "<" + ((int) this.binaryShiftStart) + "::" + ((this.binaryShiftStart + this.binaryShiftByteCount) - 1) + '>';
    }
}
