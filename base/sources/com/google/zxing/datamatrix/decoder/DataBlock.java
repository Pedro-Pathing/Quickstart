package com.google.zxing.datamatrix.decoder;

import com.google.zxing.datamatrix.decoder.Version;

/* JADX INFO: loaded from: classes8.dex */
final class DataBlock {
    private final byte[] codewords;
    private final int numDataCodewords;

    private DataBlock(int numDataCodewords, byte[] codewords) {
        this.numDataCodewords = numDataCodewords;
        this.codewords = codewords;
    }

    static DataBlock[] getDataBlocks(byte[] bArr, Version version) {
        Version.ECBlocks eCBlocks = version.getECBlocks();
        Version.ECB[] eCBlocks2 = eCBlocks.getECBlocks();
        int count = 0;
        for (Version.ECB ecb : eCBlocks2) {
            count += ecb.getCount();
        }
        DataBlock[] dataBlockArr = new DataBlock[count];
        int i = 0;
        for (Version.ECB ecb2 : eCBlocks2) {
            int i2 = 0;
            while (i2 < ecb2.getCount()) {
                int dataCodewords = ecb2.getDataCodewords();
                dataBlockArr[i] = new DataBlock(dataCodewords, new byte[eCBlocks.getECCodewords() + dataCodewords]);
                i2++;
                i++;
            }
        }
        int length = dataBlockArr[0].codewords.length - eCBlocks.getECCodewords();
        int i3 = length - 1;
        int i4 = 0;
        for (int i5 = 0; i5 < i3; i5++) {
            int i6 = 0;
            while (i6 < i) {
                dataBlockArr[i6].codewords[i5] = bArr[i4];
                i6++;
                i4++;
            }
        }
        boolean z = version.getVersionNumber() == 24;
        int i7 = z ? 8 : i;
        int i8 = 0;
        while (i8 < i7) {
            dataBlockArr[i8].codewords[i3] = bArr[i4];
            i8++;
            i4++;
        }
        int length2 = dataBlockArr[0].codewords.length;
        while (length < length2) {
            int i9 = 0;
            while (i9 < i) {
                int i10 = z ? (i9 + 8) % i : i9;
                dataBlockArr[i10].codewords[(!z || i10 <= 7) ? length : length - 1] = bArr[i4];
                i9++;
                i4++;
            }
            length++;
        }
        if (i4 != bArr.length) {
            throw new IllegalArgumentException();
        }
        return dataBlockArr;
    }

    int getNumDataCodewords() {
        return this.numDataCodewords;
    }

    byte[] getCodewords() {
        return this.codewords;
    }
}
