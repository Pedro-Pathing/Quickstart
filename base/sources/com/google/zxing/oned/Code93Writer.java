package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import java.util.Collection;
import java.util.Collections;

/* JADX INFO: loaded from: classes8.dex */
public class Code93Writer extends OneDimensionalCodeWriter {
    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    protected Collection<BarcodeFormat> getSupportedWriteFormats() {
        return Collections.singleton(BarcodeFormat.CODE_93);
    }

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    public boolean[] encode(String contents) {
        String contents2 = convertToExtended(contents);
        int length = contents2.length();
        if (length > 80) {
            throw new IllegalArgumentException("Requested contents should be less than 80 digits long after converting to extended encoding, but got ".concat(String.valueOf(length)));
        }
        boolean[] result = new boolean[((contents2.length() + 2 + 2) * 9) + 1];
        int pos = appendPattern(result, 0, Code93Reader.ASTERISK_ENCODING);
        for (int i = 0; i < length; i++) {
            int indexInString = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%abcd*".indexOf(contents2.charAt(i));
            pos += appendPattern(result, pos, Code93Reader.CHARACTER_ENCODINGS[indexInString]);
        }
        int check1 = computeChecksumIndex(contents2, 20);
        int pos2 = pos + appendPattern(result, pos, Code93Reader.CHARACTER_ENCODINGS[check1]);
        int check2 = computeChecksumIndex(contents2 + "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%abcd*".charAt(check1), 15);
        int pos3 = appendPattern(result, pos2, Code93Reader.CHARACTER_ENCODINGS[check2]) + pos2;
        result[pos3 + appendPattern(result, pos3, Code93Reader.ASTERISK_ENCODING)] = true;
        return result;
    }

    @Deprecated
    protected static int appendPattern(boolean[] target, int pos, int[] pattern, boolean startColor) {
        int length = pattern.length;
        int i = 0;
        while (i < length) {
            int bit = pattern[i];
            int pos2 = pos + 1;
            target[pos] = bit != 0;
            i++;
            pos = pos2;
        }
        return 9;
    }

    private static int appendPattern(boolean[] target, int pos, int a) {
        for (int i = 0; i < 9; i++) {
            boolean z = true;
            int temp = (1 << (8 - i)) & a;
            int i2 = pos + i;
            if (temp == 0) {
                z = false;
            }
            target[i2] = z;
        }
        return 9;
    }

    private static int computeChecksumIndex(String contents, int maxWeight) {
        int weight = 1;
        int total = 0;
        for (int i = contents.length() - 1; i >= 0; i--) {
            int indexInString = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%abcd*".indexOf(contents.charAt(i));
            total += indexInString * weight;
            weight++;
            if (weight > maxWeight) {
                weight = 1;
            }
        }
        int i2 = total % 47;
        return i2;
    }

    static String convertToExtended(String contents) {
        int length = contents.length();
        StringBuilder extendedContent = new StringBuilder(length << 1);
        for (int i = 0; i < length; i++) {
            char character = contents.charAt(i);
            if (character != 0) {
                if (character > 26) {
                    if (character > 31) {
                        if (character != ' ' && character != '$' && character != '%' && character != '+') {
                            if (character > ',') {
                                if (character > '9') {
                                    if (character != ':') {
                                        if (character > '?') {
                                            if (character != '@') {
                                                if (character > 'Z') {
                                                    if (character > '_') {
                                                        if (character != '`') {
                                                            if (character > 'z') {
                                                                if (character <= 127) {
                                                                    extendedContent.append('b');
                                                                    extendedContent.append((char) ((character + 'P') - 123));
                                                                } else {
                                                                    throw new IllegalArgumentException("Requested content contains a non-encodable character: '" + character + "'");
                                                                }
                                                            } else {
                                                                extendedContent.append('d');
                                                                extendedContent.append((char) ((character + 'A') - 97));
                                                            }
                                                        } else {
                                                            extendedContent.append("bW");
                                                        }
                                                    } else {
                                                        extendedContent.append('b');
                                                        extendedContent.append((char) ((character + 'K') - 91));
                                                    }
                                                } else {
                                                    extendedContent.append(character);
                                                }
                                            } else {
                                                extendedContent.append("bV");
                                            }
                                        } else {
                                            extendedContent.append('b');
                                            extendedContent.append((char) ((character + 'F') - 59));
                                        }
                                    } else {
                                        extendedContent.append("cZ");
                                    }
                                } else {
                                    extendedContent.append(character);
                                }
                            } else {
                                extendedContent.append('c');
                                extendedContent.append((char) ((character + 'A') - 33));
                            }
                        } else {
                            extendedContent.append(character);
                        }
                    } else {
                        extendedContent.append('b');
                        extendedContent.append((char) ((character + 'A') - 27));
                    }
                } else {
                    extendedContent.append('a');
                    extendedContent.append((char) ((character + 'A') - 1));
                }
            } else {
                extendedContent.append("bU");
            }
        }
        return extendedContent.toString();
    }
}
