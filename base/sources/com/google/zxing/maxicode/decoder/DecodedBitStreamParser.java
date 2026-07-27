package com.google.zxing.maxicode.decoder;

import com.google.zxing.common.DecoderResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.sun.tools.javac.util.LayoutCharacters;
import java.text.DecimalFormat;
import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderDatagram;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

/* JADX INFO: loaded from: classes8.dex */
final class DecodedBitStreamParser {
    private static final char ECI = 65530;
    private static final char FS = 28;
    private static final char GS = 29;
    private static final char LATCHA = 65527;
    private static final char LATCHB = 65528;
    private static final char LOCK = 65529;
    private static final char NS = 65531;
    private static final char PAD = 65532;
    private static final char RS = 30;
    private static final String[] SETS = {"\nABCDEFGHIJKLMNOPQRSTUVWXYZ\ufffa\u001c\u001d\u001e\ufffb ￼\"#$%&'()*+,-./0123456789:\ufff1\ufff2\ufff3\ufff4\ufff8", "`abcdefghijklmnopqrstuvwxyz\ufffa\u001c\u001d\u001e\ufffb{￼}~\u007f;<=>?[\\]^_ ,./:@!|￼\ufff5\ufff6￼\ufff0\ufff2\ufff3\ufff4\ufff7", "ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚ\ufffa\u001c\u001d\u001eÛÜÝÞßª¬±²³µ¹º¼½¾\u0080\u0081\u0082\u0083\u0084\u0085\u0086\u0087\u0088\u0089\ufff7 \ufff9\ufff3\ufff4\ufff8", "àáâãäåæçèéêëìíîïðñòóôõö÷øùú\ufffa\u001c\u001d\u001e\ufffbûüýþÿ¡¨«¯°´·¸»¿\u008a\u008b\u008c\u008d\u008e\u008f\u0090\u0091\u0092\u0093\u0094\ufff7 \ufff2\ufff9\ufff4\ufff8", "\u0000\u0001\u0002\u0003\u0004\u0005\u0006\u0007\b\t\n\u000b\f\r\u000e\u000f\u0010\u0011\u0012\u0013\u0014\u0015\u0016\u0017\u0018\u0019\u001a\ufffa￼￼\u001b\ufffb\u001c\u001d\u001e\u001f\u009f ¢£¤¥¦§©\u00ad®¶\u0095\u0096\u0097\u0098\u0099\u009a\u009b\u009c\u009d\u009e\ufff7 \ufff2\ufff3\ufff9\ufff8", "\u0000\u0001\u0002\u0003\u0004\u0005\u0006\u0007\b\t\n\u000b\f\r\u000e\u000f\u0010\u0011\u0012\u0013\u0014\u0015\u0016\u0017\u0018\u0019\u001a\u001b\u001c\u001d\u001e\u001f !\"#$%&'()*+,-./0123456789:;<=>?"};
    private static final char SHIFTA = 65520;
    private static final char SHIFTB = 65521;
    private static final char SHIFTC = 65522;
    private static final char SHIFTD = 65523;
    private static final char SHIFTE = 65524;
    private static final char THREESHIFTA = 65526;
    private static final char TWOSHIFTA = 65525;

    private DecodedBitStreamParser() {
    }

    static DecoderResult decode(byte[] bArr, int i) {
        String postCode3;
        StringBuilder sb = new StringBuilder(144);
        switch (i) {
            case 2:
            case 3:
                if (i == 2) {
                    postCode3 = new DecimalFormat("0000000000".substring(0, getPostCode2Length(bArr))).format(getPostCode2(bArr));
                } else {
                    postCode3 = getPostCode3(bArr);
                }
                DecimalFormat decimalFormat = new DecimalFormat("000");
                String str = decimalFormat.format(getCountry(bArr));
                String str2 = decimalFormat.format(getServiceClass(bArr));
                sb.append(getMessage(bArr, 10, 84));
                if (sb.toString().startsWith("[)>\u001e01\u001d")) {
                    sb.insert(9, postCode3 + GS + str + GS + str2 + GS);
                } else {
                    sb.insert(0, postCode3 + GS + str + GS + str2 + GS);
                }
                break;
            case 4:
                sb.append(getMessage(bArr, 1, 93));
                break;
            case 5:
                sb.append(getMessage(bArr, 1, 77));
                break;
        }
        return new DecoderResult(bArr, sb.toString(), null, String.valueOf(i));
    }

    private static int getBit(int bit, byte[] bytes) {
        int bit2 = bit - 1;
        return (bytes[bit2 / 6] & (1 << (5 - (bit2 % 6)))) == 0 ? 0 : 1;
    }

    private static int getInt(byte[] bytes, byte[] x) {
        if (x.length == 0) {
            throw new IllegalArgumentException();
        }
        int val = 0;
        for (int i = 0; i < x.length; i++) {
            val += getBit(x[i], bytes) << ((x.length - i) - 1);
        }
        return val;
    }

    private static int getCountry(byte[] bytes) {
        return getInt(bytes, new byte[]{53, 54, 43, 44, 45, 46, 47, 48, 37, 38});
    }

    private static int getServiceClass(byte[] bytes) {
        return getInt(bytes, new byte[]{55, 56, 57, 58, 59, 60, GoBildaPinpointDriver.DEFAULT_ADDRESS, 50, FlashLoaderDatagram.NAK, 52});
    }

    private static int getPostCode2Length(byte[] bytes) {
        return getInt(bytes, new byte[]{39, 40, 41, 42, 31, 32});
    }

    private static int getPostCode2(byte[] bytes) {
        return getInt(bytes, new byte[]{33, 34, 35, 36, 25, LayoutCharacters.EOI, 27, 28, 29, 30, 19, 20, 21, 22, SparkFunOTOS.DEFAULT_ADDRESS, 24, LayoutCharacters.CR, 14, 15, 16, 17, 18, 7, 8, 9, 10, 11, LayoutCharacters.FF, 1, 2});
    }

    private static String getPostCode3(byte[] bytes) {
        return String.valueOf(new char[]{SETS[0].charAt(getInt(bytes, new byte[]{39, 40, 41, 42, 31, 32})), SETS[0].charAt(getInt(bytes, new byte[]{33, 34, 35, 36, 25, LayoutCharacters.EOI})), SETS[0].charAt(getInt(bytes, new byte[]{27, 28, 29, 30, 19, 20})), SETS[0].charAt(getInt(bytes, new byte[]{21, 22, SparkFunOTOS.DEFAULT_ADDRESS, 24, LayoutCharacters.CR, 14})), SETS[0].charAt(getInt(bytes, new byte[]{15, 16, 17, 18, 7, 8})), SETS[0].charAt(getInt(bytes, new byte[]{9, 10, 11, LayoutCharacters.FF, 1, 2}))});
    }

    /* JADX WARN: Incorrect condition in loop: B:20:0x0076 */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private static java.lang.String getMessage(byte[] r10, int r11, int r12) {
        /*
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            r1 = -1
            r2 = 0
            r3 = 0
            r4 = r11
            r5 = 0
        La:
            int r6 = r11 + r12
            if (r4 >= r6) goto L72
            java.lang.String[] r6 = com.google.zxing.maxicode.decoder.DecodedBitStreamParser.SETS
            r6 = r6[r2]
            r7 = r10[r4]
            char r6 = r6.charAt(r7)
            r5 = r6
            switch(r6) {
                case 65520: goto L60;
                case 65521: goto L60;
                case 65522: goto L60;
                case 65523: goto L60;
                case 65524: goto L60;
                case 65525: goto L5c;
                case 65526: goto L58;
                case 65527: goto L55;
                case 65528: goto L52;
                case 65529: goto L50;
                case 65530: goto L1c;
                case 65531: goto L20;
                default: goto L1c;
            }
        L1c:
            r0.append(r5)
            goto L68
        L20:
            int r4 = r4 + 1
            r6 = r10[r4]
            int r6 = r6 << 24
            int r4 = r4 + 1
            r7 = r10[r4]
            int r7 = r7 << 18
            int r6 = r6 + r7
            int r4 = r4 + 1
            r7 = r10[r4]
            int r7 = r7 << 12
            int r6 = r6 + r7
            int r4 = r4 + 1
            r7 = r10[r4]
            int r7 = r7 << 6
            int r6 = r6 + r7
            int r4 = r4 + 1
            r7 = r10[r4]
            int r6 = r6 + r7
            java.text.DecimalFormat r7 = new java.text.DecimalFormat
            java.lang.String r8 = "000000000"
            r7.<init>(r8)
            long r8 = (long) r6
            java.lang.String r7 = r7.format(r8)
            r0.append(r7)
            goto L68
        L50:
            r1 = -1
            goto L68
        L52:
            r2 = 1
            r1 = -1
            goto L68
        L55:
            r2 = 0
            r1 = -1
            goto L68
        L58:
            r3 = r2
            r2 = 0
            r1 = 3
            goto L68
        L5c:
            r3 = r2
            r2 = 0
            r1 = 2
            goto L68
        L60:
            r3 = r2
            r6 = 65520(0xfff0, float:9.1813E-41)
            int r2 = r5 - r6
            r1 = 1
        L68:
            int r6 = r1 + (-1)
            if (r1 != 0) goto L6e
            r1 = r3
            r2 = r1
        L6e:
            int r4 = r4 + 1
            r1 = r6
            goto La
        L72:
            int r4 = r0.length()
            if (r4 <= 0) goto L91
            int r4 = r0.length()
            int r4 = r4 + (-1)
            char r4 = r0.charAt(r4)
            r5 = 65532(0xfffc, float:9.183E-41)
            if (r4 != r5) goto L91
            int r4 = r0.length()
            int r4 = r4 + (-1)
            r0.setLength(r4)
            goto L72
        L91:
            java.lang.String r4 = r0.toString()
            return r4
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.maxicode.decoder.DecodedBitStreamParser.getMessage(byte[], int, int):java.lang.String");
    }
}
