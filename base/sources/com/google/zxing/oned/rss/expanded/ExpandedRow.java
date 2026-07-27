package com.google.zxing.oned.rss.expanded;

import java.util.ArrayList;
import java.util.List;

/* JADX INFO: loaded from: classes8.dex */
final class ExpandedRow {
    private final List<ExpandedPair> pairs;
    private final int rowNumber;
    private final boolean wasReversed;

    ExpandedRow(List<ExpandedPair> pairs, int rowNumber, boolean wasReversed) {
        this.pairs = new ArrayList(pairs);
        this.rowNumber = rowNumber;
        this.wasReversed = wasReversed;
    }

    List<ExpandedPair> getPairs() {
        return this.pairs;
    }

    int getRowNumber() {
        return this.rowNumber;
    }

    boolean isEquivalent(List<ExpandedPair> otherPairs) {
        return this.pairs.equals(otherPairs);
    }

    public String toString() {
        return "{ " + this.pairs + " }";
    }

    public boolean equals(Object o) {
        if (!(o instanceof ExpandedRow)) {
            return false;
        }
        ExpandedRow that = (ExpandedRow) o;
        return this.pairs.equals(that.pairs) && this.wasReversed == that.wasReversed;
    }

    public int hashCode() {
        return this.pairs.hashCode() ^ Boolean.valueOf(this.wasReversed).hashCode();
    }
}
