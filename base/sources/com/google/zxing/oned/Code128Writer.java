package com.google.zxing.oned;

import com.google.zxing.BarcodeFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;

/* JADX INFO: loaded from: classes8.dex */
public final class Code128Writer extends OneDimensionalCodeWriter {
    private static final int CODE_CODE_A = 101;
    private static final int CODE_CODE_B = 100;
    private static final int CODE_CODE_C = 99;
    private static final int CODE_FNC_1 = 102;
    private static final int CODE_FNC_2 = 97;
    private static final int CODE_FNC_3 = 96;
    private static final int CODE_FNC_4_A = 101;
    private static final int CODE_FNC_4_B = 100;
    private static final int CODE_START_A = 103;
    private static final int CODE_START_B = 104;
    private static final int CODE_START_C = 105;
    private static final int CODE_STOP = 106;
    private static final char ESCAPE_FNC_1 = 241;
    private static final char ESCAPE_FNC_2 = 242;
    private static final char ESCAPE_FNC_3 = 243;
    private static final char ESCAPE_FNC_4 = 244;

    private enum CType {
        UNCODABLE,
        ONE_DIGIT,
        TWO_DIGITS,
        FNC_1
    }

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    protected Collection<BarcodeFormat> getSupportedWriteFormats() {
        return Collections.singleton(BarcodeFormat.CODE_128);
    }

    @Override // com.google.zxing.oned.OneDimensionalCodeWriter
    public boolean[] encode(String str) {
        int length = str.length();
        if (length <= 0 || length > 80) {
            throw new IllegalArgumentException("Contents length should be between 1 and 80 characters, but got ".concat(String.valueOf(length)));
        }
        int iAppendPattern = 0;
        for (int i = 0; i < length; i++) {
            char cCharAt = str.charAt(i);
            switch (cCharAt) {
                case 241:
                case 242:
                case 243:
                case 244:
                    break;
                default:
                    if (cCharAt > 127) {
                        throw new IllegalArgumentException("Bad character in input: ".concat(String.valueOf(cCharAt)));
                    }
                    break;
                    break;
            }
        }
        ArrayList<int[]> arrayList = new ArrayList();
        int i2 = 0;
        int i3 = 0;
        int i4 = 0;
        int i5 = 1;
        while (true) {
            int iCharAt = 103;
            if (i2 < length) {
                int iChooseCode = chooseCode(str, i2, i4);
                if (iChooseCode == i4) {
                    switch (str.charAt(i2)) {
                        case 241:
                            iCharAt = 102;
                            break;
                        case 242:
                            iCharAt = 97;
                            break;
                        case 243:
                            iCharAt = 96;
                            break;
                        case 244:
                            iCharAt = 101;
                            if (i4 != 101) {
                                iCharAt = 100;
                            }
                            break;
                        default:
                            switch (i4) {
                                case 100:
                                    iCharAt = str.charAt(i2) - ' ';
                                    break;
                                case 101:
                                    iCharAt = str.charAt(i2) - ' ';
                                    if (iCharAt < 0) {
                                        iCharAt += 96;
                                    }
                                    break;
                                default:
                                    iCharAt = Integer.parseInt(str.substring(i2, i2 + 2));
                                    i2++;
                                    break;
                            }
                            break;
                    }
                    i2++;
                } else {
                    if (i4 == 0) {
                        switch (iChooseCode) {
                            case 100:
                                iCharAt = 104;
                                break;
                            case 101:
                                break;
                            default:
                                iCharAt = 105;
                                break;
                        }
                    } else {
                        iCharAt = iChooseCode;
                    }
                    i4 = iChooseCode;
                }
                arrayList.add(Code128Reader.CODE_PATTERNS[iCharAt]);
                i3 += iCharAt * i5;
                if (i2 != 0) {
                    i5++;
                }
            } else {
                arrayList.add(Code128Reader.CODE_PATTERNS[i3 % 103]);
                arrayList.add(Code128Reader.CODE_PATTERNS[106]);
                int i6 = 0;
                for (int[] iArr : arrayList) {
                    for (int i7 : iArr) {
                        i6 += i7;
                    }
                }
                boolean[] zArr = new boolean[i6];
                Iterator it = arrayList.iterator();
                while (it.hasNext()) {
                    iAppendPattern += appendPattern(zArr, iAppendPattern, (int[]) it.next(), true);
                }
                return zArr;
            }
        }
    }

    private static CType findCType(CharSequence value, int start) {
        int last = value.length();
        if (start >= last) {
            return CType.UNCODABLE;
        }
        char c = value.charAt(start);
        if (c != 241) {
            if (c < '0' || c > '9') {
                return CType.UNCODABLE;
            }
            if (start + 1 >= last) {
                return CType.ONE_DIGIT;
            }
            char c2 = value.charAt(start + 1);
            if (c2 < '0' || c2 > '9') {
                return CType.ONE_DIGIT;
            }
            return CType.TWO_DIGITS;
        }
        return CType.FNC_1;
    }

    private static int chooseCode(CharSequence value, int start, int oldCode) {
        CType lookahead;
        CType lookahead2;
        char c;
        CType cTypeFindCType = findCType(value, start);
        CType lookahead3 = cTypeFindCType;
        if (cTypeFindCType == CType.ONE_DIGIT) {
            return oldCode == 101 ? 101 : 100;
        }
        if (lookahead3 == CType.UNCODABLE) {
            return (start >= value.length() || ((c = value.charAt(start)) >= ' ' && (oldCode != 101 || (c >= '`' && (c < 241 || c > 244))))) ? 100 : 101;
        }
        if (oldCode == 101 && lookahead3 == CType.FNC_1) {
            return 101;
        }
        if (oldCode == 99) {
            return 99;
        }
        if (oldCode == 100) {
            if (lookahead3 == CType.FNC_1 || (lookahead = findCType(value, start + 2)) == CType.UNCODABLE || lookahead == CType.ONE_DIGIT) {
                return 100;
            }
            if (lookahead == CType.FNC_1) {
                return findCType(value, start + 3) == CType.TWO_DIGITS ? 99 : 100;
            }
            int index = start + 4;
            while (true) {
                lookahead2 = findCType(value, index);
                if (lookahead2 != CType.TWO_DIGITS) {
                    break;
                }
                index += 2;
            }
            return lookahead2 == CType.ONE_DIGIT ? 100 : 99;
        }
        if (lookahead3 == CType.FNC_1) {
            lookahead3 = findCType(value, start + 1);
        }
        return lookahead3 == CType.TWO_DIGITS ? 99 : 100;
    }
}
