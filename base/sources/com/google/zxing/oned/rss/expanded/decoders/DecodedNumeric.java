package com.google.zxing.oned.rss.expanded.decoders;

import com.google.zxing.FormatException;

/* JADX INFO: loaded from: classes8.dex */
final class DecodedNumeric extends DecodedObject {
    static final int FNC1 = 10;
    private final int firstDigit;
    private final int secondDigit;

    DecodedNumeric(int newPosition, int firstDigit, int secondDigit) throws FormatException {
        super(newPosition);
        if (firstDigit < 0 || firstDigit > 10 || secondDigit < 0 || secondDigit > 10) {
            throw FormatException.getFormatInstance();
        }
        this.firstDigit = firstDigit;
        this.secondDigit = secondDigit;
    }

    int getFirstDigit() {
        return this.firstDigit;
    }

    int getSecondDigit() {
        return this.secondDigit;
    }

    int getValue() {
        return (this.firstDigit * 10) + this.secondDigit;
    }

    boolean isFirstDigitFNC1() {
        return this.firstDigit == 10;
    }

    boolean isSecondDigitFNC1() {
        return this.secondDigit == 10;
    }
}
