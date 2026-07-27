package com.google.zxing.oned.rss.expanded;

import com.google.zxing.oned.rss.DataCharacter;
import com.google.zxing.oned.rss.FinderPattern;
import java.util.Objects;

/* JADX INFO: loaded from: classes8.dex */
final class ExpandedPair {
    private final FinderPattern finderPattern;
    private final DataCharacter leftChar;
    private final DataCharacter rightChar;

    ExpandedPair(DataCharacter leftChar, DataCharacter rightChar, FinderPattern finderPattern) {
        this.leftChar = leftChar;
        this.rightChar = rightChar;
        this.finderPattern = finderPattern;
    }

    DataCharacter getLeftChar() {
        return this.leftChar;
    }

    DataCharacter getRightChar() {
        return this.rightChar;
    }

    FinderPattern getFinderPattern() {
        return this.finderPattern;
    }

    boolean mustBeLast() {
        return this.rightChar == null;
    }

    public String toString() {
        return "[ " + this.leftChar + " , " + this.rightChar + " : " + (this.finderPattern == null ? "null" : Integer.valueOf(this.finderPattern.getValue())) + " ]";
    }

    public boolean equals(Object o) {
        if (!(o instanceof ExpandedPair)) {
            return false;
        }
        ExpandedPair that = (ExpandedPair) o;
        return Objects.equals(this.leftChar, that.leftChar) && Objects.equals(this.rightChar, that.rightChar) && Objects.equals(this.finderPattern, that.finderPattern);
    }

    public int hashCode() {
        return (Objects.hashCode(this.leftChar) ^ Objects.hashCode(this.rightChar)) ^ Objects.hashCode(this.finderPattern);
    }
}
