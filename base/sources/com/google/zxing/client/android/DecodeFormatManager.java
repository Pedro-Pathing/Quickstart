package com.google.zxing.client.android;

import android.content.Intent;
import com.google.zxing.BarcodeFormat;
import com.google.zxing.client.android.Intents;
import com.sun.tools.doclint.DocLint;
import java.util.Arrays;
import java.util.Collection;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public final class DecodeFormatManager {
    private static final Map<String, Set<BarcodeFormat>> FORMATS_FOR_MODE;
    private static final Pattern COMMA_PATTERN = Pattern.compile(DocLint.TAGS_SEPARATOR);
    static final Set<BarcodeFormat> QR_CODE_FORMATS = EnumSet.of(BarcodeFormat.QR_CODE);
    static final Set<BarcodeFormat> DATA_MATRIX_FORMATS = EnumSet.of(BarcodeFormat.DATA_MATRIX);
    static final Set<BarcodeFormat> AZTEC_FORMATS = EnumSet.of(BarcodeFormat.AZTEC);
    static final Set<BarcodeFormat> PDF417_FORMATS = EnumSet.of(BarcodeFormat.PDF_417);
    static final Set<BarcodeFormat> PRODUCT_FORMATS = EnumSet.of(BarcodeFormat.UPC_A, BarcodeFormat.UPC_E, BarcodeFormat.EAN_13, BarcodeFormat.EAN_8, BarcodeFormat.RSS_14, BarcodeFormat.RSS_EXPANDED);
    static final Set<BarcodeFormat> INDUSTRIAL_FORMATS = EnumSet.of(BarcodeFormat.CODE_39, BarcodeFormat.CODE_93, BarcodeFormat.CODE_128, BarcodeFormat.ITF, BarcodeFormat.CODABAR);
    private static final Set<BarcodeFormat> ONE_D_FORMATS = EnumSet.copyOf((Collection) PRODUCT_FORMATS);

    static {
        ONE_D_FORMATS.addAll(INDUSTRIAL_FORMATS);
        FORMATS_FOR_MODE = new HashMap();
        FORMATS_FOR_MODE.put(Intents.Scan.ONE_D_MODE, ONE_D_FORMATS);
        FORMATS_FOR_MODE.put(Intents.Scan.PRODUCT_MODE, PRODUCT_FORMATS);
        FORMATS_FOR_MODE.put(Intents.Scan.QR_CODE_MODE, QR_CODE_FORMATS);
        FORMATS_FOR_MODE.put(Intents.Scan.DATA_MATRIX_MODE, DATA_MATRIX_FORMATS);
        FORMATS_FOR_MODE.put(Intents.Scan.AZTEC_MODE, AZTEC_FORMATS);
        FORMATS_FOR_MODE.put(Intents.Scan.PDF417_MODE, PDF417_FORMATS);
    }

    private DecodeFormatManager() {
    }

    public static Set<BarcodeFormat> parseDecodeFormats(Intent intent) {
        Iterable<String> scanFormats = null;
        CharSequence scanFormatsString = intent.getStringExtra(Intents.Scan.FORMATS);
        if (scanFormatsString != null) {
            scanFormats = Arrays.asList(COMMA_PATTERN.split(scanFormatsString));
        }
        return parseDecodeFormats(scanFormats, intent.getStringExtra(Intents.Scan.MODE));
    }

    private static Set<BarcodeFormat> parseDecodeFormats(Iterable<String> scanFormats, String decodeMode) {
        if (scanFormats != null) {
            Set<BarcodeFormat> formats = EnumSet.noneOf(BarcodeFormat.class);
            try {
                for (String format : scanFormats) {
                    formats.add(BarcodeFormat.valueOf(format));
                }
                return formats;
            } catch (IllegalArgumentException e) {
            }
        }
        if (decodeMode != null) {
            return FORMATS_FOR_MODE.get(decodeMode);
        }
        return null;
    }
}
