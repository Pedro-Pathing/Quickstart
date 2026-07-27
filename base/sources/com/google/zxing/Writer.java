package com.google.zxing;

import com.google.zxing.common.BitMatrix;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public interface Writer {
    BitMatrix encode(String str, BarcodeFormat barcodeFormat, int i, int i2) throws WriterException;

    BitMatrix encode(String str, BarcodeFormat barcodeFormat, int i, int i2, Map<EncodeHintType, ?> map) throws WriterException;
}
