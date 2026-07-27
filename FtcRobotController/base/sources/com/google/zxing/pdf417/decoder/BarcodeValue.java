package com.google.zxing.pdf417.decoder;

import com.google.zxing.pdf417.PDF417Common;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
final class BarcodeValue {
    private final Map<Integer, Integer> values = new HashMap();

    BarcodeValue() {
    }

    void setValue(int value) {
        Integer num = this.values.get(Integer.valueOf(value));
        Integer confidence = num;
        if (num == null) {
            confidence = 0;
        }
        this.values.put(Integer.valueOf(value), Integer.valueOf(confidence.intValue() + 1));
    }

    int[] getValue() {
        int maxConfidence = -1;
        Collection<Integer> result = new ArrayList<>();
        for (Map.Entry<Integer, Integer> entry : this.values.entrySet()) {
            if (entry.getValue().intValue() > maxConfidence) {
                maxConfidence = entry.getValue().intValue();
                result.clear();
                result.add(entry.getKey());
            } else if (entry.getValue().intValue() == maxConfidence) {
                result.add(entry.getKey());
            }
        }
        return PDF417Common.toIntArray(result);
    }

    Integer getConfidence(int value) {
        return this.values.get(Integer.valueOf(value));
    }
}
