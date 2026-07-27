package com.google.zxing.datamatrix.decoder;

import com.android.tools.r8.DataResource;
import com.google.zxing.FormatException;
import com.google.zxing.common.BitSource;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import dk.sgjesse.r8api.DescriptorUtils;
import java.io.UnsupportedEncodingException;
import java.util.Collection;
import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;
import org.opencv.videoio.Videoio;

/* JADX INFO: loaded from: classes8.dex */
final class DecodedBitStreamParser {
    private static final char[] C40_BASIC_SET_CHARS = {'*', '*', '*', ' ', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'};
    private static final char[] C40_SHIFT2_SET_CHARS = {'!', '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', DescriptorUtils.JAVA_PACKAGE_SEPARATOR, DataResource.SEPARATOR, ':', ';', '<', '=', '>', '?', '@', '[', '\\', ']', '^', '_'};
    private static final char[] TEXT_BASIC_SET_CHARS = {'*', '*', '*', ' ', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z'};
    private static final char[] TEXT_SHIFT2_SET_CHARS = C40_SHIFT2_SET_CHARS;
    private static final char[] TEXT_SHIFT3_SET_CHARS = {'`', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '{', '|', '}', '~', 127};

    private enum Mode {
        PAD_ENCODE,
        ASCII_ENCODE,
        C40_ENCODE,
        TEXT_ENCODE,
        ANSIX12_ENCODE,
        EDIFACT_ENCODE,
        BASE256_ENCODE
    }

    private DecodedBitStreamParser() {
    }

    /* JADX WARN: Removed duplicated region for block: B:22:0x0059  */
    /* JADX WARN: Removed duplicated region for block: B:25:0x0069  */
    /* JADX WARN: Removed duplicated region for block: B:26:0x006b  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    static com.google.zxing.common.DecoderResult decode(byte[] r9) throws com.google.zxing.FormatException {
        /*
            com.google.zxing.common.BitSource r0 = new com.google.zxing.common.BitSource
            r0.<init>(r9)
            java.lang.StringBuilder r1 = new java.lang.StringBuilder
            r2 = 100
            r1.<init>(r2)
            java.lang.StringBuilder r2 = new java.lang.StringBuilder
            r3 = 0
            r2.<init>(r3)
            java.util.ArrayList r3 = new java.util.ArrayList
            r4 = 1
            r3.<init>(r4)
            com.google.zxing.datamatrix.decoder.DecodedBitStreamParser$Mode r4 = com.google.zxing.datamatrix.decoder.DecodedBitStreamParser.Mode.ASCII_ENCODE
        L1a:
            com.google.zxing.datamatrix.decoder.DecodedBitStreamParser$Mode r5 = com.google.zxing.datamatrix.decoder.DecodedBitStreamParser.Mode.ASCII_ENCODE
            if (r4 != r5) goto L23
            com.google.zxing.datamatrix.decoder.DecodedBitStreamParser$Mode r4 = decodeAsciiSegment(r0, r1, r2)
            goto L49
        L23:
            int[] r5 = com.google.zxing.datamatrix.decoder.DecodedBitStreamParser.AnonymousClass1.$SwitchMap$com$google$zxing$datamatrix$decoder$DecodedBitStreamParser$Mode
            int r6 = r4.ordinal()
            r5 = r5[r6]
            switch(r5) {
                case 1: goto L43;
                case 2: goto L3f;
                case 3: goto L3b;
                case 4: goto L37;
                case 5: goto L33;
                default: goto L2e;
            }
        L2e:
            com.google.zxing.FormatException r5 = com.google.zxing.FormatException.getFormatInstance()
            throw r5
        L33:
            decodeBase256Segment(r0, r1, r3)
            goto L47
        L37:
            decodeEdifactSegment(r0, r1)
            goto L47
        L3b:
            decodeAnsiX12Segment(r0, r1)
            goto L47
        L3f:
            decodeTextSegment(r0, r1)
            goto L47
        L43:
            decodeC40Segment(r0, r1)
        L47:
            com.google.zxing.datamatrix.decoder.DecodedBitStreamParser$Mode r4 = com.google.zxing.datamatrix.decoder.DecodedBitStreamParser.Mode.ASCII_ENCODE
        L49:
            com.google.zxing.datamatrix.decoder.DecodedBitStreamParser$Mode r5 = com.google.zxing.datamatrix.decoder.DecodedBitStreamParser.Mode.PAD_ENCODE
            if (r4 == r5) goto L53
            int r5 = r0.available()
            if (r5 > 0) goto L1a
        L53:
            int r5 = r2.length()
            if (r5 <= 0) goto L5c
            r1.append(r2)
        L5c:
            com.google.zxing.common.DecoderResult r5 = new com.google.zxing.common.DecoderResult
            java.lang.String r6 = r1.toString()
            boolean r7 = r3.isEmpty()
            r8 = 0
            if (r7 == 0) goto L6b
            r7 = r8
            goto L6c
        L6b:
            r7 = r3
        L6c:
            r5.<init>(r9, r6, r7, r8)
            return r5
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.datamatrix.decoder.DecodedBitStreamParser.decode(byte[]):com.google.zxing.common.DecoderResult");
    }

    private static Mode decodeAsciiSegment(BitSource bits, StringBuilder result, StringBuilder resultTrailer) throws FormatException {
        boolean upperShift = false;
        do {
            int bits2 = bits.readBits(8);
            int oneByte = bits2;
            if (bits2 == 0) {
                throw FormatException.getFormatInstance();
            }
            if (oneByte <= 128) {
                if (upperShift) {
                    oneByte += 128;
                }
                result.append((char) (oneByte - 1));
                return Mode.ASCII_ENCODE;
            }
            if (oneByte == 129) {
                return Mode.PAD_ENCODE;
            }
            if (oneByte <= 229) {
                int value = oneByte - 130;
                if (value < 10) {
                    result.append('0');
                }
                result.append(value);
            } else {
                switch (oneByte) {
                    case 230:
                        return Mode.C40_ENCODE;
                    case 231:
                        return Mode.BASE256_ENCODE;
                    case 232:
                        result.append((char) 29);
                        break;
                    case 233:
                    case 234:
                    case 241:
                        break;
                    case 235:
                        upperShift = true;
                        break;
                    case 236:
                        result.append("[)>\u001e05\u001d");
                        resultTrailer.insert(0, "\u001e\u0004");
                        break;
                    case 237:
                        result.append("[)>\u001e06\u001d");
                        resultTrailer.insert(0, "\u001e\u0004");
                        break;
                    case 238:
                        return Mode.ANSIX12_ENCODE;
                    case UsbConstants.USB_CLASS_MISC /* 239 */:
                        return Mode.TEXT_ENCODE;
                    case 240:
                        return Mode.EDIFACT_ENCODE;
                    default:
                        if (oneByte != 254 || bits.available() != 0) {
                            throw FormatException.getFormatInstance();
                        }
                        break;
                }
            }
        } while (bits.available() > 0);
        return Mode.ASCII_ENCODE;
    }

    private static void decodeC40Segment(BitSource bits, StringBuilder result) throws FormatException {
        int firstByte;
        boolean upperShift = false;
        int[] cValues = new int[3];
        int shift = 0;
        while (bits.available() != 8 && (firstByte = bits.readBits(8)) != 254) {
            parseTwoBytes(firstByte, bits.readBits(8), cValues);
            for (int i = 0; i < 3; i++) {
                int cValue = cValues[i];
                switch (shift) {
                    case 0:
                        if (cValue < 3) {
                            shift = cValue + 1;
                        } else if (cValue < C40_BASIC_SET_CHARS.length) {
                            char c40char = C40_BASIC_SET_CHARS[cValue];
                            if (upperShift) {
                                result.append((char) (c40char + 128));
                                upperShift = false;
                            } else {
                                result.append(c40char);
                            }
                        } else {
                            throw FormatException.getFormatInstance();
                        }
                        break;
                    case 1:
                        if (upperShift) {
                            result.append((char) (cValue + 128));
                            upperShift = false;
                        } else {
                            result.append((char) cValue);
                        }
                        shift = 0;
                        break;
                    case 2:
                        if (cValue < C40_SHIFT2_SET_CHARS.length) {
                            char c40char2 = C40_SHIFT2_SET_CHARS[cValue];
                            if (upperShift) {
                                result.append((char) (c40char2 + 128));
                                upperShift = false;
                            } else {
                                result.append(c40char2);
                            }
                        } else {
                            switch (cValue) {
                                case 27:
                                    result.append((char) 29);
                                    break;
                                case 30:
                                    upperShift = true;
                                    break;
                                default:
                                    throw FormatException.getFormatInstance();
                            }
                        }
                        shift = 0;
                        break;
                    case 3:
                        if (upperShift) {
                            result.append((char) (cValue + UsbConstants.USB_CLASS_WIRELESS_CONTROLLER));
                            upperShift = false;
                        } else {
                            result.append((char) (cValue + 96));
                        }
                        shift = 0;
                        break;
                    default:
                        throw FormatException.getFormatInstance();
                }
            }
            int i2 = bits.available();
            if (i2 <= 0) {
                return;
            }
        }
    }

    private static void decodeTextSegment(BitSource bits, StringBuilder result) throws FormatException {
        int firstByte;
        boolean upperShift = false;
        int[] cValues = new int[3];
        int shift = 0;
        while (bits.available() != 8 && (firstByte = bits.readBits(8)) != 254) {
            parseTwoBytes(firstByte, bits.readBits(8), cValues);
            for (int i = 0; i < 3; i++) {
                int cValue = cValues[i];
                switch (shift) {
                    case 0:
                        if (cValue < 3) {
                            shift = cValue + 1;
                        } else if (cValue < TEXT_BASIC_SET_CHARS.length) {
                            char textChar = TEXT_BASIC_SET_CHARS[cValue];
                            if (upperShift) {
                                result.append((char) (textChar + 128));
                                upperShift = false;
                            } else {
                                result.append(textChar);
                            }
                        } else {
                            throw FormatException.getFormatInstance();
                        }
                        break;
                    case 1:
                        if (upperShift) {
                            result.append((char) (cValue + 128));
                            upperShift = false;
                        } else {
                            result.append((char) cValue);
                        }
                        shift = 0;
                        break;
                    case 2:
                        if (cValue < TEXT_SHIFT2_SET_CHARS.length) {
                            char textChar2 = TEXT_SHIFT2_SET_CHARS[cValue];
                            if (upperShift) {
                                result.append((char) (textChar2 + 128));
                                upperShift = false;
                            } else {
                                result.append(textChar2);
                            }
                        } else {
                            switch (cValue) {
                                case 27:
                                    result.append((char) 29);
                                    break;
                                case 30:
                                    upperShift = true;
                                    break;
                                default:
                                    throw FormatException.getFormatInstance();
                            }
                        }
                        shift = 0;
                        break;
                    case 3:
                        if (cValue < TEXT_SHIFT3_SET_CHARS.length) {
                            char textChar3 = TEXT_SHIFT3_SET_CHARS[cValue];
                            if (upperShift) {
                                result.append((char) (textChar3 + 128));
                                upperShift = false;
                            } else {
                                result.append(textChar3);
                            }
                            shift = 0;
                        } else {
                            throw FormatException.getFormatInstance();
                        }
                        break;
                    default:
                        throw FormatException.getFormatInstance();
                }
            }
            int i2 = bits.available();
            if (i2 <= 0) {
                return;
            }
        }
    }

    private static void decodeAnsiX12Segment(BitSource bits, StringBuilder result) throws FormatException {
        int firstByte;
        int[] cValues = new int[3];
        while (bits.available() != 8 && (firstByte = bits.readBits(8)) != 254) {
            parseTwoBytes(firstByte, bits.readBits(8), cValues);
            for (int i = 0; i < 3; i++) {
                int cValue = cValues[i];
                switch (cValue) {
                    case 0:
                        result.append('\r');
                        break;
                    case 1:
                        result.append('*');
                        break;
                    case 2:
                        result.append('>');
                        break;
                    case 3:
                        result.append(' ');
                        break;
                    default:
                        if (cValue >= 14) {
                            if (cValue < 40) {
                                result.append((char) (cValue + 51));
                            } else {
                                throw FormatException.getFormatInstance();
                            }
                        } else {
                            result.append((char) (cValue + 44));
                        }
                        break;
                }
            }
            int i2 = bits.available();
            if (i2 <= 0) {
                return;
            }
        }
    }

    private static void parseTwoBytes(int firstByte, int secondByte, int[] result) {
        int fullBitValue = ((firstByte << 8) + secondByte) - 1;
        int temp = fullBitValue / Videoio.CAP_OPENNI2;
        result[0] = temp;
        int fullBitValue2 = fullBitValue - (temp * Videoio.CAP_OPENNI2);
        int temp2 = fullBitValue2 / 40;
        result[1] = temp2;
        result[2] = fullBitValue2 - (temp2 * 40);
    }

    private static void decodeEdifactSegment(BitSource bits, StringBuilder result) {
        while (bits.available() > 16) {
            for (int i = 0; i < 4; i++) {
                int bits2 = bits.readBits(6);
                int edifactValue = bits2;
                if (bits2 == 31) {
                    int bitsLeft = 8 - bits.getBitOffset();
                    if (bitsLeft != 8) {
                        bits.readBits(bitsLeft);
                        return;
                    }
                    return;
                }
                if ((edifactValue & 32) == 0) {
                    edifactValue |= 64;
                }
                result.append((char) edifactValue);
            }
            int i2 = bits.available();
            if (i2 <= 0) {
                return;
            }
        }
    }

    private static void decodeBase256Segment(BitSource bits, StringBuilder result, Collection<byte[]> byteSegments) throws FormatException {
        int count;
        int codewordPosition = bits.getByteOffset() + 1;
        int codewordPosition2 = codewordPosition + 1;
        int d1 = unrandomize255State(bits.readBits(8), codewordPosition);
        if (d1 == 0) {
            count = bits.available() / 8;
        } else if (d1 >= 250) {
            count = unrandomize255State(bits.readBits(8), codewordPosition2) + ((d1 - 249) * SyncdDevice.msAbnormalReopenInterval);
            codewordPosition2++;
        } else {
            count = d1;
        }
        if (count < 0) {
            throw FormatException.getFormatInstance();
        }
        byte[] bytes = new byte[count];
        int i = 0;
        while (i < count) {
            if (bits.available() < 8) {
                throw FormatException.getFormatInstance();
            }
            bytes[i] = (byte) unrandomize255State(bits.readBits(8), codewordPosition2);
            i++;
            codewordPosition2++;
        }
        byteSegments.add(bytes);
        try {
            result.append(new String(bytes, "ISO8859_1"));
        } catch (UnsupportedEncodingException uee) {
            throw new IllegalStateException("Platform does not support required encoding: ".concat(String.valueOf(uee)));
        }
    }

    private static int unrandomize255State(int randomizedBase256Codeword, int base256CodewordPosition) {
        int pseudoRandomNumber = ((base256CodewordPosition * 149) % 255) + 1;
        int tempVariable = randomizedBase256Codeword - pseudoRandomNumber;
        return tempVariable >= 0 ? tempVariable : tempVariable + 256;
    }
}
