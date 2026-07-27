package com.google.zxing.aztec.encoder;

import com.google.zxing.common.BitMatrix;

/* JADX INFO: loaded from: classes8.dex */
public final class AztecCode {
    private int codeWords;
    private boolean compact;
    private int layers;
    private BitMatrix matrix;
    private int size;

    public boolean isCompact() {
        return this.compact;
    }

    public void setCompact(boolean compact) {
        this.compact = compact;
    }

    public int getSize() {
        return this.size;
    }

    public void setSize(int size) {
        this.size = size;
    }

    public int getLayers() {
        return this.layers;
    }

    public void setLayers(int layers) {
        this.layers = layers;
    }

    public int getCodeWords() {
        return this.codeWords;
    }

    public void setCodeWords(int codeWords) {
        this.codeWords = codeWords;
    }

    public BitMatrix getMatrix() {
        return this.matrix;
    }

    public void setMatrix(BitMatrix matrix) {
        this.matrix = matrix;
    }
}
