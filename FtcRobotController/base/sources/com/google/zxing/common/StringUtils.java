package com.google.zxing.common;

import java.nio.charset.Charset;

/* JADX INFO: loaded from: classes8.dex */
public final class StringUtils {
    private static final boolean ASSUME_SHIFT_JIS;
    private static final String EUC_JP = "EUC_JP";
    public static final String GB2312 = "GB2312";
    private static final String ISO88591 = "ISO8859_1";
    private static final String PLATFORM_DEFAULT_ENCODING = Charset.defaultCharset().name();
    public static final String SHIFT_JIS = "SJIS";
    private static final String UTF8 = "UTF8";

    static {
        ASSUME_SHIFT_JIS = SHIFT_JIS.equalsIgnoreCase(PLATFORM_DEFAULT_ENCODING) || EUC_JP.equalsIgnoreCase(PLATFORM_DEFAULT_ENCODING);
    }

    private StringUtils() {
    }

    /* JADX WARN: Removed duplicated region for block: B:43:0x008b A[PHI: r6
      0x008b: PHI (r6v6 'utf8BytesLeft' int) = (r6v1 'utf8BytesLeft' int), (r6v5 'utf8BytesLeft' int), (r6v1 'utf8BytesLeft' int) binds: [B:33:0x006e, B:41:0x0086, B:28:0x0063] A[DONT_GENERATE, DONT_INLINE]] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public static java.lang.String guessEncoding(byte[] r20, java.util.Map<com.google.zxing.DecodeHintType, ?> r21) {
        /*
            Method dump skipped, instruction units count: 343
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.common.StringUtils.guessEncoding(byte[], java.util.Map):java.lang.String");
    }
}
