package com.google.zxing.pdf417.encoder;

import com.google.zxing.WriterException;
import com.google.zxing.common.CharacterSetECI;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.sun.tools.javac.util.LayoutCharacters;
import java.math.BigInteger;
import java.nio.charset.Charset;
import java.nio.charset.CharsetEncoder;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderDatagram;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

/* JADX INFO: loaded from: classes8.dex */
final class PDF417HighLevelEncoder {
    private static final int BYTE_COMPACTION = 1;
    private static final int ECI_CHARSET = 927;
    private static final int ECI_GENERAL_PURPOSE = 926;
    private static final int ECI_USER_DEFINED = 925;
    private static final int LATCH_TO_BYTE = 924;
    private static final int LATCH_TO_BYTE_PADDED = 901;
    private static final int LATCH_TO_NUMERIC = 902;
    private static final int LATCH_TO_TEXT = 900;
    private static final int NUMERIC_COMPACTION = 2;
    private static final int SHIFT_TO_BYTE = 913;
    private static final int SUBMODE_ALPHA = 0;
    private static final int SUBMODE_LOWER = 1;
    private static final int SUBMODE_MIXED = 2;
    private static final int SUBMODE_PUNCTUATION = 3;
    private static final int TEXT_COMPACTION = 0;
    private static final byte[] TEXT_MIXED_RAW = {48, GoBildaPinpointDriver.DEFAULT_ADDRESS, 50, FlashLoaderDatagram.NAK, 52, 53, 54, 55, 56, 57, 38, LayoutCharacters.CR, 9, 44, 58, 35, 45, 46, 36, 47, 43, 37, 42, 61, 94, 0, 32, 0, 0, 0};
    private static final byte[] TEXT_PUNCTUATION_RAW = {59, 60, 62, 64, 91, 92, 93, 95, AMSColorSensor.AMS_TMD37821_ID, 126, 33, LayoutCharacters.CR, 9, 44, 58, 10, 45, 46, 36, 47, 34, 124, 42, 40, 41, 63, 123, 125, 39, 0};
    private static final byte[] MIXED = new byte[128];
    private static final byte[] PUNCTUATION = new byte[128];
    private static final Charset DEFAULT_ENCODING = StandardCharsets.ISO_8859_1;

    static {
        Arrays.fill(MIXED, (byte) -1);
        for (int i = 0; i < TEXT_MIXED_RAW.length; i++) {
            byte b = TEXT_MIXED_RAW[i];
            if (b > 0) {
                MIXED[b] = (byte) i;
            }
        }
        Arrays.fill(PUNCTUATION, (byte) -1);
        for (int i2 = 0; i2 < TEXT_PUNCTUATION_RAW.length; i2++) {
            byte b2 = TEXT_PUNCTUATION_RAW[i2];
            if (b2 > 0) {
                PUNCTUATION[b2] = (byte) i2;
            }
        }
    }

    private PDF417HighLevelEncoder() {
    }

    static String encodeHighLevel(String msg, Compaction compaction, Charset encoding) throws WriterException {
        CharacterSetECI eci;
        Charset encoding2 = encoding;
        StringBuilder sb = new StringBuilder(msg.length());
        if (encoding2 == null) {
            encoding2 = DEFAULT_ENCODING;
        } else if (!DEFAULT_ENCODING.equals(encoding2) && (eci = CharacterSetECI.getCharacterSetECIByName(encoding.name())) != null) {
            encodingECI(eci.getValue(), sb);
        }
        int len = msg.length();
        int p = 0;
        int textSubMode = 0;
        switch (compaction) {
            case TEXT:
                encodeText(msg, 0, len, sb, 0);
                break;
            case BYTE:
                byte[] msgBytes = msg.getBytes(encoding2);
                encodeBinary(msgBytes, 0, msgBytes.length, 1, sb);
                break;
            case NUMERIC:
                sb.append((char) 902);
                encodeNumeric(msg, 0, len, sb);
                break;
            default:
                int encodingMode = 0;
                while (p < len) {
                    int n = determineConsecutiveDigitCount(msg, p);
                    if (n >= 13) {
                        sb.append((char) 902);
                        encodingMode = 2;
                        textSubMode = 0;
                        encodeNumeric(msg, p, n, sb);
                        p += n;
                    } else {
                        int t = determineConsecutiveTextCount(msg, p);
                        if (t >= 5 || n == len) {
                            if (encodingMode != 0) {
                                sb.append((char) 900);
                                encodingMode = 0;
                                textSubMode = 0;
                            }
                            textSubMode = encodeText(msg, p, t, sb, textSubMode);
                            p += t;
                        } else {
                            int iDetermineConsecutiveBinaryCount = determineConsecutiveBinaryCount(msg, p, encoding2);
                            int b = iDetermineConsecutiveBinaryCount;
                            if (iDetermineConsecutiveBinaryCount == 0) {
                                b = 1;
                            }
                            byte[] bytes = msg.substring(p, p + b).getBytes(encoding2);
                            if (bytes.length != 1 || encodingMode != 0) {
                                encodeBinary(bytes, 0, bytes.length, encodingMode, sb);
                                encodingMode = 1;
                                textSubMode = 0;
                            } else {
                                encodeBinary(bytes, 0, 1, 0, sb);
                            }
                            p += b;
                        }
                    }
                }
                break;
        }
        return sb.toString();
    }

    private static int encodeText(CharSequence charSequence, int i, int i2, StringBuilder sb, int i3) {
        StringBuilder sb2 = new StringBuilder(i2);
        int i4 = i3;
        int i5 = 0;
        while (true) {
            int i6 = i + i5;
            char cCharAt = charSequence.charAt(i6);
            switch (i4) {
                case 0:
                    if (isAlphaUpper(cCharAt)) {
                        if (cCharAt == ' ') {
                            sb2.append((char) 26);
                        } else {
                            sb2.append((char) (cCharAt - 'A'));
                        }
                    } else {
                        if (isAlphaLower(cCharAt)) {
                            sb2.append((char) 27);
                            i4 = 1;
                        } else if (isMixed(cCharAt)) {
                            sb2.append((char) 28);
                            i4 = 2;
                        } else {
                            sb2.append((char) 29);
                            sb2.append((char) PUNCTUATION[cCharAt]);
                        }
                        break;
                    }
                    break;
                case 1:
                    if (isAlphaLower(cCharAt)) {
                        if (cCharAt == ' ') {
                            sb2.append((char) 26);
                        } else {
                            sb2.append((char) (cCharAt - 'a'));
                        }
                    } else if (isAlphaUpper(cCharAt)) {
                        sb2.append((char) 27);
                        sb2.append((char) (cCharAt - 'A'));
                    } else if (isMixed(cCharAt)) {
                        sb2.append((char) 28);
                        i4 = 2;
                    } else {
                        sb2.append((char) 29);
                        sb2.append((char) PUNCTUATION[cCharAt]);
                    }
                    break;
                case 2:
                    if (isMixed(cCharAt)) {
                        sb2.append((char) MIXED[cCharAt]);
                    } else if (isAlphaUpper(cCharAt)) {
                        sb2.append((char) 28);
                        i4 = 0;
                    } else if (isAlphaLower(cCharAt)) {
                        sb2.append((char) 27);
                        i4 = 1;
                    } else {
                        int i7 = i6 + 1;
                        if (i7 < i2 && isPunctuation(charSequence.charAt(i7))) {
                            sb2.append((char) 25);
                            i4 = 3;
                        } else {
                            sb2.append((char) 29);
                            sb2.append((char) PUNCTUATION[cCharAt]);
                        }
                    }
                    break;
                default:
                    if (isPunctuation(cCharAt)) {
                        sb2.append((char) PUNCTUATION[cCharAt]);
                    } else {
                        sb2.append((char) 29);
                        i4 = 0;
                    }
                    break;
            }
            i5++;
            if (i5 >= i2) {
                int length = sb2.length();
                char cCharAt2 = 0;
                for (int i8 = 0; i8 < length; i8++) {
                    if (i8 % 2 != 0) {
                        cCharAt2 = (char) ((cCharAt2 * 30) + sb2.charAt(i8));
                        sb.append(cCharAt2);
                    } else {
                        cCharAt2 = sb2.charAt(i8);
                    }
                }
                if (length % 2 != 0) {
                    sb.append((char) ((cCharAt2 * 30) + 29));
                }
                return i4;
            }
        }
    }

    private static void encodeBinary(byte[] bytes, int startpos, int count, int startmode, StringBuilder sb) {
        if (count == 1 && startmode == 0) {
            sb.append((char) 913);
        } else if (count % 6 == 0) {
            sb.append((char) 924);
        } else {
            sb.append((char) 901);
        }
        int idx = startpos;
        if (count >= 6) {
            char[] chars = new char[5];
            while ((startpos + count) - idx >= 6) {
                long t = 0;
                for (int i = 0; i < 6; i++) {
                    t = (t << 8) + ((long) (bytes[idx + i] & 255));
                }
                for (int i2 = 0; i2 < 5; i2++) {
                    chars[i2] = (char) (t % 900);
                    t /= 900;
                }
                for (int i3 = 4; i3 >= 0; i3--) {
                    sb.append(chars[i3]);
                }
                idx += 6;
            }
        }
        for (int i4 = idx; i4 < startpos + count; i4++) {
            int ch = bytes[i4] & 255;
            sb.append((char) ch);
        }
    }

    private static void encodeNumeric(String msg, int startpos, int count, StringBuilder sb) {
        BigInteger bigIntegerDivide;
        int idx = 0;
        StringBuilder tmp = new StringBuilder((count / 3) + 1);
        BigInteger num900 = BigInteger.valueOf(900L);
        BigInteger num0 = BigInteger.valueOf(0L);
        while (idx < count) {
            tmp.setLength(0);
            int len = Math.min(44, count - idx);
            String part = "1" + msg.substring(startpos + idx, startpos + idx + len);
            BigInteger bigint = new BigInteger(part);
            do {
                tmp.append((char) bigint.mod(num900).intValue());
                bigIntegerDivide = bigint.divide(num900);
                bigint = bigIntegerDivide;
            } while (!bigIntegerDivide.equals(num0));
            for (int i = tmp.length() - 1; i >= 0; i--) {
                sb.append(tmp.charAt(i));
            }
            idx += len;
        }
    }

    private static boolean isDigit(char ch) {
        return ch >= '0' && ch <= '9';
    }

    private static boolean isAlphaUpper(char ch) {
        if (ch != ' ') {
            return ch >= 'A' && ch <= 'Z';
        }
        return true;
    }

    private static boolean isAlphaLower(char ch) {
        if (ch != ' ') {
            return ch >= 'a' && ch <= 'z';
        }
        return true;
    }

    private static boolean isMixed(char ch) {
        return MIXED[ch] != -1;
    }

    private static boolean isPunctuation(char ch) {
        return PUNCTUATION[ch] != -1;
    }

    private static boolean isText(char ch) {
        if (ch == '\t' || ch == '\n' || ch == '\r') {
            return true;
        }
        return ch >= ' ' && ch <= '~';
    }

    private static int determineConsecutiveDigitCount(CharSequence msg, int startpos) {
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

    private static int determineConsecutiveTextCount(CharSequence msg, int startpos) {
        int len = msg.length();
        int idx = startpos;
        while (idx < len) {
            char ch = msg.charAt(idx);
            int numericCount = 0;
            while (numericCount < 13 && isDigit(ch) && idx < len) {
                numericCount++;
                idx++;
                if (idx < len) {
                    ch = msg.charAt(idx);
                }
            }
            if (numericCount >= 13) {
                return (idx - startpos) - numericCount;
            }
            if (numericCount <= 0) {
                if (!isText(msg.charAt(idx))) {
                    break;
                }
                idx++;
            }
        }
        return idx - startpos;
    }

    private static int determineConsecutiveBinaryCount(String msg, int startpos, Charset encoding) throws WriterException {
        int i;
        CharsetEncoder encoder = encoding.newEncoder();
        int len = msg.length();
        int idx = startpos;
        while (idx < len) {
            char ch = msg.charAt(idx);
            int numericCount = 0;
            while (numericCount < 13 && isDigit(ch) && (i = idx + (numericCount = numericCount + 1)) < len) {
                ch = msg.charAt(i);
            }
            if (numericCount >= 13) {
                return idx - startpos;
            }
            char ch2 = msg.charAt(idx);
            if (!encoder.canEncode(ch2)) {
                throw new WriterException("Non-encodable character detected: " + ch2 + " (Unicode: " + ((int) ch2) + ')');
            }
            idx++;
        }
        return idx - startpos;
    }

    private static void encodingECI(int eci, StringBuilder sb) throws WriterException {
        if (eci >= 0 && eci < 900) {
            sb.append((char) 927);
            sb.append((char) eci);
        } else if (eci < 810900) {
            sb.append((char) 926);
            sb.append((char) ((eci / 900) - 1));
            sb.append((char) (eci % 900));
        } else {
            if (eci < 811800) {
                sb.append((char) 925);
                sb.append((char) (810900 - eci));
                return;
            }
            throw new WriterException("ECI number not in valid range from 0..811799, but was ".concat(String.valueOf(eci)));
        }
    }
}
