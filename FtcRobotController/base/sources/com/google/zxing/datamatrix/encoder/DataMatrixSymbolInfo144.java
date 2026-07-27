package com.google.zxing.datamatrix.encoder;

import com.sun.tools.javac.jvm.ByteCodes;

/* JADX INFO: loaded from: classes8.dex */
final class DataMatrixSymbolInfo144 extends SymbolInfo {
    DataMatrixSymbolInfo144() {
        super(false, 1558, 620, 22, 22, 36, -1, 62);
    }

    @Override // com.google.zxing.datamatrix.encoder.SymbolInfo
    public int getInterleavedBlockCount() {
        return 10;
    }

    @Override // com.google.zxing.datamatrix.encoder.SymbolInfo
    public int getDataLengthForInterleavedBlock(int index) {
        if (index <= 8) {
            return ByteCodes.ifge;
        }
        return 155;
    }
}
