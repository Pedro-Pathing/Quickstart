package com.google.zxing;

import com.google.zxing.aztec.AztecReader;
import com.google.zxing.datamatrix.DataMatrixReader;
import com.google.zxing.maxicode.MaxiCodeReader;
import com.google.zxing.oned.MultiFormatOneDReader;
import com.google.zxing.pdf417.PDF417Reader;
import com.google.zxing.qrcode.QRCodeReader;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class MultiFormatReader implements Reader {
    private static final Reader[] EMPTY_READER_ARRAY = new Reader[0];
    private Map<DecodeHintType, ?> hints;
    private Reader[] readers;

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap image) throws NotFoundException {
        setHints(null);
        return decodeInternal(image);
    }

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap image, Map<DecodeHintType, ?> hints) throws NotFoundException {
        setHints(hints);
        return decodeInternal(image);
    }

    public Result decodeWithState(BinaryBitmap image) throws NotFoundException {
        if (this.readers == null) {
            setHints(null);
        }
        return decodeInternal(image);
    }

    public void setHints(Map<DecodeHintType, ?> hints) {
        this.hints = hints;
        boolean z = true;
        boolean tryHarder = hints != null && hints.containsKey(DecodeHintType.TRY_HARDER);
        Collection<BarcodeFormat> formats = hints == null ? null : (Collection) hints.get(DecodeHintType.POSSIBLE_FORMATS);
        Collection<Reader> readers = new ArrayList<>();
        if (formats != null) {
            if (!formats.contains(BarcodeFormat.UPC_A) && !formats.contains(BarcodeFormat.UPC_E) && !formats.contains(BarcodeFormat.EAN_13) && !formats.contains(BarcodeFormat.EAN_8) && !formats.contains(BarcodeFormat.CODABAR) && !formats.contains(BarcodeFormat.CODE_39) && !formats.contains(BarcodeFormat.CODE_93) && !formats.contains(BarcodeFormat.CODE_128) && !formats.contains(BarcodeFormat.ITF) && !formats.contains(BarcodeFormat.RSS_14) && !formats.contains(BarcodeFormat.RSS_EXPANDED)) {
                z = false;
            }
            boolean addOneDReader = z;
            if (z && !tryHarder) {
                readers.add(new MultiFormatOneDReader(hints));
            }
            if (formats.contains(BarcodeFormat.QR_CODE)) {
                readers.add(new QRCodeReader());
            }
            if (formats.contains(BarcodeFormat.DATA_MATRIX)) {
                readers.add(new DataMatrixReader());
            }
            if (formats.contains(BarcodeFormat.AZTEC)) {
                readers.add(new AztecReader());
            }
            if (formats.contains(BarcodeFormat.PDF_417)) {
                readers.add(new PDF417Reader());
            }
            if (formats.contains(BarcodeFormat.MAXICODE)) {
                readers.add(new MaxiCodeReader());
            }
            if (addOneDReader && tryHarder) {
                readers.add(new MultiFormatOneDReader(hints));
            }
        }
        if (readers.isEmpty()) {
            if (!tryHarder) {
                readers.add(new MultiFormatOneDReader(hints));
            }
            readers.add(new QRCodeReader());
            readers.add(new DataMatrixReader());
            readers.add(new AztecReader());
            readers.add(new PDF417Reader());
            readers.add(new MaxiCodeReader());
            if (tryHarder) {
                readers.add(new MultiFormatOneDReader(hints));
            }
        }
        this.readers = (Reader[]) readers.toArray(EMPTY_READER_ARRAY);
    }

    @Override // com.google.zxing.Reader
    public void reset() {
        if (this.readers != null) {
            for (Reader reader : this.readers) {
                reader.reset();
            }
        }
    }

    private Result decodeInternal(BinaryBitmap image) throws NotFoundException {
        if (this.readers != null) {
            Reader[] readerArr = this.readers;
            int length = readerArr.length;
            for (int i = 0; i < length; i++) {
                Reader reader = readerArr[i];
                try {
                    return reader.decode(image, this.hints);
                } catch (ReaderException e) {
                }
            }
        }
        throw NotFoundException.getNotFoundInstance();
    }
}
