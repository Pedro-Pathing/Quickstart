package com.google.zxing.oned.rss.expanded.decoders;

import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.common.BitArray;

/* JADX INFO: loaded from: classes8.dex */
final class AI01393xDecoder extends AI01decoder {
    private static final int FIRST_THREE_DIGITS_SIZE = 10;
    private static final int HEADER_SIZE = 8;
    private static final int LAST_DIGIT_SIZE = 2;

    AI01393xDecoder(BitArray information) {
        super(information);
    }

    @Override // com.google.zxing.oned.rss.expanded.decoders.AbstractExpandedDecoder
    public String parseInformation() throws NotFoundException, FormatException {
        if (getInformation().getSize() < 48) {
            throw NotFoundException.getNotFoundInstance();
        }
        StringBuilder buf = new StringBuilder();
        encodeCompressedGtin(buf, 8);
        int lastAIdigit = getGeneralDecoder().extractNumericValueFromBitArray(48, 2);
        buf.append("(393");
        buf.append(lastAIdigit);
        buf.append(')');
        int firstThreeDigits = getGeneralDecoder().extractNumericValueFromBitArray(50, 10);
        if (firstThreeDigits / 100 == 0) {
            buf.append('0');
        }
        if (firstThreeDigits / 10 == 0) {
            buf.append('0');
        }
        buf.append(firstThreeDigits);
        DecodedInformation generalInformation = getGeneralDecoder().decodeGeneralPurposeField(60, null);
        buf.append(generalInformation.getNewString());
        return buf.toString();
    }
}
