package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.DecodeHintType;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class MultiFormatUPCEANReader extends OneDReader {
    private static final UPCEANReader[] EMPTY_READER_ARRAY = new UPCEANReader[0];
    private final UPCEANReader[] readers;

    public MultiFormatUPCEANReader(Map<DecodeHintType, ?> hints) {
        Collection<BarcodeFormat> possibleFormats = hints == null ? null : (Collection) hints.get(DecodeHintType.POSSIBLE_FORMATS);
        Collection<UPCEANReader> readers = new ArrayList<>();
        if (possibleFormats != null) {
            if (possibleFormats.contains(BarcodeFormat.EAN_13)) {
                readers.add(new EAN13Reader());
            } else if (possibleFormats.contains(BarcodeFormat.UPC_A)) {
                readers.add(new UPCAReader());
            }
            if (possibleFormats.contains(BarcodeFormat.EAN_8)) {
                readers.add(new EAN8Reader());
            }
            if (possibleFormats.contains(BarcodeFormat.UPC_E)) {
                readers.add(new UPCEReader());
            }
        }
        if (readers.isEmpty()) {
            readers.add(new EAN13Reader());
            readers.add(new EAN8Reader());
            readers.add(new UPCEReader());
        }
        this.readers = (UPCEANReader[]) readers.toArray(EMPTY_READER_ARRAY);
    }

    /* JADX WARN: Removed duplicated region for block: B:11:0x0034  */
    @Override // com.google.zxing.oned.OneDReader
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public com.google.zxing.Result decodeRow(int r19, com.google.zxing.common.BitArray r20, java.util.Map<com.google.zxing.DecodeHintType, ?> r21) throws com.google.zxing.NotFoundException {
        /*
            r18 = this;
            r1 = r21
            int[] r2 = com.google.zxing.oned.UPCEANReader.findStartGuardPattern(r20)
            r3 = r18
            com.google.zxing.oned.UPCEANReader[] r4 = r3.readers
            int r5 = r4.length
            r0 = 0
            r6 = 0
            r7 = r0
            r8 = r7
            r9 = r8
            r10 = r6
        L11:
            if (r10 >= r5) goto L95
            r11 = r4[r10]
            r12 = r19
            r13 = r20
            com.google.zxing.Result r0 = r11.decodeRow(r12, r13, r2, r1)     // Catch: com.google.zxing.ReaderException -> L89
            r7 = r0
            com.google.zxing.BarcodeFormat r0 = r0.getBarcodeFormat()     // Catch: com.google.zxing.ReaderException -> L89
            com.google.zxing.BarcodeFormat r14 = com.google.zxing.BarcodeFormat.EAN_13     // Catch: com.google.zxing.ReaderException -> L89
            if (r0 != r14) goto L34
            java.lang.String r0 = r7.getText()     // Catch: com.google.zxing.ReaderException -> L50
            char r0 = r0.charAt(r6)     // Catch: com.google.zxing.ReaderException -> L50
            r14 = 48
            if (r0 != r14) goto L34
            r0 = 1
            goto L35
        L34:
            r0 = r6
        L35:
            if (r1 != 0) goto L39
            r14 = 0
            goto L41
        L39:
            com.google.zxing.DecodeHintType r14 = com.google.zxing.DecodeHintType.POSSIBLE_FORMATS     // Catch: com.google.zxing.ReaderException -> L89
            java.lang.Object r14 = r1.get(r14)     // Catch: com.google.zxing.ReaderException -> L89
            java.util.Collection r14 = (java.util.Collection) r14     // Catch: com.google.zxing.ReaderException -> L89
        L41:
            r8 = r14
            if (r14 == 0) goto L54
            com.google.zxing.BarcodeFormat r14 = com.google.zxing.BarcodeFormat.UPC_A     // Catch: com.google.zxing.ReaderException -> L50
            boolean r14 = r8.contains(r14)     // Catch: com.google.zxing.ReaderException -> L50
            if (r14 == 0) goto L4e
            goto L54
        L4e:
            r14 = r6
            goto L55
        L50:
            r0 = move-exception
            r16 = r2
            goto L8c
        L54:
            r14 = 1
        L55:
            if (r0 == 0) goto L84
            if (r14 == 0) goto L84
            com.google.zxing.Result r6 = new com.google.zxing.Result     // Catch: com.google.zxing.ReaderException -> L89
            java.lang.String r15 = r7.getText()     // Catch: com.google.zxing.ReaderException -> L89
            r17 = r0
            r0 = 1
            java.lang.String r0 = r15.substring(r0)     // Catch: com.google.zxing.ReaderException -> L89
            byte[] r15 = r7.getRawBytes()     // Catch: com.google.zxing.ReaderException -> L89
            com.google.zxing.ResultPoint[] r1 = r7.getResultPoints()     // Catch: com.google.zxing.ReaderException -> L89
            r16 = r2
            com.google.zxing.BarcodeFormat r2 = com.google.zxing.BarcodeFormat.UPC_A     // Catch: com.google.zxing.ReaderException -> L82
            r6.<init>(r0, r15, r1, r2)     // Catch: com.google.zxing.ReaderException -> L82
            r0 = r9
            r1 = r6
            java.util.Map r0 = r7.getResultMetadata()     // Catch: com.google.zxing.ReaderException -> L7f
            r6.putAllMetadata(r0)     // Catch: com.google.zxing.ReaderException -> L7f
            return r1
        L7f:
            r0 = move-exception
            r9 = r1
            goto L8c
        L82:
            r0 = move-exception
            goto L8c
        L84:
            r17 = r0
            r16 = r2
            return r7
        L89:
            r0 = move-exception
            r16 = r2
        L8c:
            int r10 = r10 + 1
            r1 = r21
            r2 = r16
            r6 = 0
            goto L11
        L95:
            com.google.zxing.NotFoundException r0 = com.google.zxing.NotFoundException.getNotFoundInstance()
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.oned.MultiFormatUPCEANReader.decodeRow(int, com.google.zxing.common.BitArray, java.util.Map):com.google.zxing.Result");
    }

    @Override // com.google.zxing.oned.OneDReader, com.google.zxing.Reader
    public void reset() {
        for (UPCEANReader uPCEANReader : this.readers) {
            uPCEANReader.reset();
        }
    }
}
