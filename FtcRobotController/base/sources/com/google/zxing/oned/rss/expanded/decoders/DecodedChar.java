package com.google.zxing.oned.rss.expanded.decoders;

/* JADX INFO: loaded from: classes8.dex */
final class DecodedChar extends DecodedObject {
    static final char FNC1 = '$';
    private final char value;

    DecodedChar(int newPosition, char value) {
        super(newPosition);
        this.value = value;
    }

    char getValue() {
        return this.value;
    }

    boolean isFNC1() {
        return this.value == '$';
    }
}
