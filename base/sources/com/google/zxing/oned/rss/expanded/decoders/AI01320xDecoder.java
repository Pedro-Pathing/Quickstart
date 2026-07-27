package com.google.zxing.oned.rss.expanded.decoders;

import com.google.zxing.common.BitArray;

/* JADX INFO: loaded from: classes8.dex */
final class AI01320xDecoder extends AI013x0xDecoder {
    AI01320xDecoder(BitArray information) {
        super(information);
    }

    @Override // com.google.zxing.oned.rss.expanded.decoders.AI01weightDecoder
    protected void addWeightCode(StringBuilder buf, int weight) {
        if (weight < 10000) {
            buf.append("(3202)");
        } else {
            buf.append("(3203)");
        }
    }

    @Override // com.google.zxing.oned.rss.expanded.decoders.AI01weightDecoder
    protected int checkWeight(int weight) {
        if (weight < 10000) {
            return weight;
        }
        return weight - 10000;
    }
}
