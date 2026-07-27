package com.google.zxing.oned.rss.expanded.decoders;

import com.google.zxing.common.BitArray;

/* JADX INFO: loaded from: classes8.dex */
final class AI013103decoder extends AI013x0xDecoder {
    AI013103decoder(BitArray information) {
        super(information);
    }

    @Override // com.google.zxing.oned.rss.expanded.decoders.AI01weightDecoder
    protected void addWeightCode(StringBuilder buf, int weight) {
        buf.append("(3103)");
    }

    @Override // com.google.zxing.oned.rss.expanded.decoders.AI01weightDecoder
    protected int checkWeight(int weight) {
        return weight;
    }
}
