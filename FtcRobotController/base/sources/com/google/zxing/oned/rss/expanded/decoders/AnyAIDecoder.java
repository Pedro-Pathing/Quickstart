package com.google.zxing.oned.rss.expanded.decoders;

import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.common.BitArray;

/* JADX INFO: loaded from: classes8.dex */
final class AnyAIDecoder extends AbstractExpandedDecoder {
    private static final int HEADER_SIZE = 5;

    AnyAIDecoder(BitArray information) {
        super(information);
    }

    @Override // com.google.zxing.oned.rss.expanded.decoders.AbstractExpandedDecoder
    public String parseInformation() throws NotFoundException, FormatException {
        StringBuilder buf = new StringBuilder();
        return getGeneralDecoder().decodeAllCodes(buf, 5);
    }
}
