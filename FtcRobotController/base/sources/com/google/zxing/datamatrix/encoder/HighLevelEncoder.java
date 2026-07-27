package com.google.zxing.datamatrix.encoder;

import com.google.zxing.Dimension;
import java.util.Arrays;

/* JADX INFO: loaded from: classes8.dex */
public final class HighLevelEncoder {
    static final int ASCII_ENCODATION = 0;
    static final int BASE256_ENCODATION = 5;
    static final int C40_ENCODATION = 1;
    static final char C40_UNLATCH = 254;
    static final int EDIFACT_ENCODATION = 4;
    static final char LATCH_TO_ANSIX12 = 238;
    static final char LATCH_TO_BASE256 = 231;
    static final char LATCH_TO_C40 = 230;
    static final char LATCH_TO_EDIFACT = 240;
    static final char LATCH_TO_TEXT = 239;
    private static final char MACRO_05 = 236;
    private static final String MACRO_05_HEADER = "[)>\u001e05\u001d";
    private static final char MACRO_06 = 237;
    private static final String MACRO_06_HEADER = "[)>\u001e06\u001d";
    private static final String MACRO_TRAILER = "\u001e\u0004";
    private static final char PAD = 129;
    static final int TEXT_ENCODATION = 2;
    static final char UPPER_SHIFT = 235;
    static final int X12_ENCODATION = 3;
    static final char X12_UNLATCH = 254;

    private HighLevelEncoder() {
    }

    private static char randomize253State(int codewordPosition) {
        int tempVariable = ((codewordPosition * 149) % 253) + 1 + 129;
        return (char) (tempVariable <= 254 ? tempVariable : tempVariable - 254);
    }

    public static String encodeHighLevel(String msg) {
        return encodeHighLevel(msg, SymbolShapeHint.FORCE_NONE, null, null);
    }

    public static String encodeHighLevel(String msg, SymbolShapeHint shape, Dimension minSize, Dimension maxSize) {
        Encoder[] encoders = {new ASCIIEncoder(), new C40Encoder(), new TextEncoder(), new X12Encoder(), new EdifactEncoder(), new Base256Encoder()};
        EncoderContext context = new EncoderContext(msg);
        context.setSymbolShape(shape);
        context.setSizeConstraints(minSize, maxSize);
        if (msg.startsWith(MACRO_05_HEADER) && msg.endsWith(MACRO_TRAILER)) {
            context.writeCodeword(MACRO_05);
            context.setSkipAtEnd(2);
            context.pos += 7;
        } else if (msg.startsWith(MACRO_06_HEADER) && msg.endsWith(MACRO_TRAILER)) {
            context.writeCodeword(MACRO_06);
            context.setSkipAtEnd(2);
            context.pos += 7;
        }
        int encodingMode = 0;
        while (context.hasMoreCharacters()) {
            encoders[encodingMode].encode(context);
            if (context.getNewEncoding() >= 0) {
                encodingMode = context.getNewEncoding();
                context.resetEncoderSignal();
            }
        }
        int len = context.getCodewordCount();
        context.updateSymbolInfo();
        int capacity = context.getSymbolInfo().getDataCapacity();
        if (len < capacity && encodingMode != 0 && encodingMode != 5 && encodingMode != 4) {
            context.writeCodeword((char) 254);
        }
        StringBuilder codewords = context.getCodewords();
        if (codewords.length() < capacity) {
            codewords.append(PAD);
        }
        while (codewords.length() < capacity) {
            codewords.append(randomize253State(codewords.length() + 1));
        }
        return context.getCodewords().toString();
    }

    /* JADX WARN: Code restructure failed: missing block: B:128:0x0200, code lost:
    
        return 5;
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    static int lookAheadTest(java.lang.CharSequence r18, int r19, int r20) {
        /*
            Method dump skipped, instruction units count: 521
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.datamatrix.encoder.HighLevelEncoder.lookAheadTest(java.lang.CharSequence, int, int):int");
    }

    private static int findMinimums(float[] charCounts, int[] intCharCounts, int min, byte[] mins) {
        Arrays.fill(mins, (byte) 0);
        for (int i = 0; i < 6; i++) {
            intCharCounts[i] = (int) Math.ceil(charCounts[i]);
            int current = intCharCounts[i];
            if (min > current) {
                min = current;
                Arrays.fill(mins, (byte) 0);
            }
            if (min == current) {
                mins[i] = (byte) (mins[i] + 1);
            }
        }
        return min;
    }

    private static int getMinimumCount(byte[] mins) {
        int minCount = 0;
        for (int i = 0; i < 6; i++) {
            minCount += mins[i];
        }
        return minCount;
    }

    static boolean isDigit(char ch) {
        return ch >= '0' && ch <= '9';
    }

    static boolean isExtendedASCII(char ch) {
        return ch >= 128 && ch <= 255;
    }

    private static boolean isNativeC40(char ch) {
        if (ch == ' ') {
            return true;
        }
        if (ch < '0' || ch > '9') {
            return ch >= 'A' && ch <= 'Z';
        }
        return true;
    }

    private static boolean isNativeText(char ch) {
        if (ch == ' ') {
            return true;
        }
        if (ch < '0' || ch > '9') {
            return ch >= 'a' && ch <= 'z';
        }
        return true;
    }

    private static boolean isNativeX12(char ch) {
        if (isX12TermSep(ch) || ch == ' ') {
            return true;
        }
        if (ch < '0' || ch > '9') {
            return ch >= 'A' && ch <= 'Z';
        }
        return true;
    }

    private static boolean isX12TermSep(char ch) {
        return ch == '\r' || ch == '*' || ch == '>';
    }

    private static boolean isNativeEDIFACT(char ch) {
        return ch >= ' ' && ch <= '^';
    }

    private static boolean isSpecialB256(char ch) {
        return false;
    }

    public static int determineConsecutiveDigitCount(CharSequence msg, int startpos) {
        int count = 0;
        int len = msg.length();
        int idx = startpos;
        if (startpos < len) {
            char ch = msg.charAt(startpos);
            while (isDigit(ch) && idx < len) {
                count++;
                idx++;
                if (idx < len) {
                    ch = msg.charAt(idx);
                }
            }
        }
        return count;
    }

    static void illegalCharacter(char c) {
        String hex = Integer.toHexString(c);
        throw new IllegalArgumentException("Illegal character: " + c + " (0x" + ("0000".substring(0, 4 - hex.length()) + hex) + ')');
    }
}
