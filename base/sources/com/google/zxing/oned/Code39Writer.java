package com.google.zxing.oned;

import com.android.tools.r8.DataResource;
import com.google.zxing.BarcodeFormat;
import java.util.Collection;
import java.util.Collections;

/* JADX INFO: loaded from: classes8.dex */
public final class Code39Writer extends OneDimensionalCodeWriter {
    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    protected Collection<BarcodeFormat> getSupportedWriteFormats() {
        return Collections.singleton(BarcodeFormat.CODE_39);
    }

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    public boolean[] encode(String contents) {
        int length = contents.length();
        int length2 = length;
        if (length > 80) {
            throw new IllegalArgumentException("Requested contents should be less than 80 digits long, but got ".concat(String.valueOf(length2)));
        }
        int i = 0;
        while (true) {
            if (i >= length2) {
                break;
            }
            if ("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%".indexOf(contents.charAt(i)) >= 0) {
                i++;
            } else {
                String strTryToConvertToExtendedMode = tryToConvertToExtendedMode(contents);
                contents = strTryToConvertToExtendedMode;
                int length3 = strTryToConvertToExtendedMode.length();
                length2 = length3;
                if (length3 > 80) {
                    throw new IllegalArgumentException("Requested contents should be less than 80 digits long, but got " + length2 + " (extended full ASCII mode)");
                }
            }
        }
        int[] widths = new int[9];
        boolean[] result = new boolean[(length2 * 13) + 25];
        toIntArray(148, widths);
        int pos = appendPattern(result, 0, widths, true);
        int[] narrowWhite = {1};
        int pos2 = pos + appendPattern(result, pos, narrowWhite, false);
        for (int i2 = 0; i2 < length2; i2++) {
            int indexInString = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%".indexOf(contents.charAt(i2));
            toIntArray(Code39Reader.CHARACTER_ENCODINGS[indexInString], widths);
            int pos3 = appendPattern(result, pos2, widths, true) + pos2;
            pos2 = pos3 + appendPattern(result, pos3, narrowWhite, false);
        }
        toIntArray(148, widths);
        appendPattern(result, pos2, widths, true);
        return result;
    }

    private static void toIntArray(int a, int[] toReturn) {
        for (int i = 0; i < 9; i++) {
            int i2 = 1;
            int temp = (1 << (8 - i)) & a;
            if (temp != 0) {
                i2 = 2;
            }
            toReturn[i] = i2;
        }
    }

    private static String tryToConvertToExtendedMode(String contents) {
        int length = contents.length();
        StringBuilder extendedContent = new StringBuilder();
        for (int i = 0; i < length; i++) {
            char character = contents.charAt(i);
            switch (character) {
                case 0:
                    extendedContent.append("%U");
                    break;
                case ' ':
                case '-':
                case '.':
                    extendedContent.append(character);
                    break;
                case '@':
                    extendedContent.append("%V");
                    break;
                case '`':
                    extendedContent.append("%W");
                    break;
                default:
                    if (character > 26) {
                        if (character >= ' ') {
                            if (character > ',' && character != '/' && character != ':') {
                                if (character > '9') {
                                    if (character > '?') {
                                        if (character > 'Z') {
                                            if (character > '_') {
                                                if (character > 'z') {
                                                    if (character <= 127) {
                                                        extendedContent.append('%');
                                                        extendedContent.append((char) ((character - '{') + 80));
                                                    } else {
                                                        throw new IllegalArgumentException("Requested content contains a non-encodable character: '" + contents.charAt(i) + "'");
                                                    }
                                                } else {
                                                    extendedContent.append('+');
                                                    extendedContent.append((char) ((character - 'a') + 65));
                                                }
                                            } else {
                                                extendedContent.append('%');
                                                extendedContent.append((char) ((character - '[') + 75));
                                            }
                                        } else {
                                            extendedContent.append((char) ((character - 'A') + 65));
                                        }
                                    } else {
                                        extendedContent.append('%');
                                        extendedContent.append((char) ((character - ';') + 70));
                                    }
                                } else {
                                    extendedContent.append((char) ((character - '0') + 48));
                                }
                            } else {
                                extendedContent.append(DataResource.SEPARATOR);
                                extendedContent.append((char) ((character - '!') + 65));
                            }
                        } else {
                            extendedContent.append('%');
                            extendedContent.append((char) ((character - 27) + 65));
                        }
                    } else {
                        extendedContent.append('$');
                        extendedContent.append((char) ((character - 1) + 65));
                    }
                    break;
            }
        }
        return extendedContent.toString();
    }
}
