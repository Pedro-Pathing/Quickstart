package com.google.zxing.common;

import java.util.Arrays;

/* JADX INFO: loaded from: classes8.dex */
public final class BitMatrix implements Cloneable {
    private final int[] bits;
    private final int height;
    private final int rowSize;
    private final int width;

    public BitMatrix(int dimension) {
        this(dimension, dimension);
    }

    public BitMatrix(int width, int height) {
        if (width <= 0 || height <= 0) {
            throw new IllegalArgumentException("Both dimensions must be greater than 0");
        }
        this.width = width;
        this.height = height;
        this.rowSize = (width + 31) / 32;
        this.bits = new int[this.rowSize * height];
    }

    private BitMatrix(int width, int height, int rowSize, int[] bits) {
        this.width = width;
        this.height = height;
        this.rowSize = rowSize;
        this.bits = bits;
    }

    public static BitMatrix parse(boolean[][] image) {
        int height = image.length;
        int width = image[0].length;
        BitMatrix bits = new BitMatrix(width, height);
        for (int i = 0; i < height; i++) {
            boolean[] imageI = image[i];
            for (int j = 0; j < width; j++) {
                if (imageI[j]) {
                    bits.set(j, i);
                }
            }
        }
        return bits;
    }

    public static BitMatrix parse(String stringRepresentation, String setString, String unsetString) {
        if (stringRepresentation == null) {
            throw new IllegalArgumentException();
        }
        boolean[] bits = new boolean[stringRepresentation.length()];
        int bitsPos = 0;
        int rowStartPos = 0;
        int rowLength = -1;
        int nRows = 0;
        int pos = 0;
        while (pos < stringRepresentation.length()) {
            if (stringRepresentation.charAt(pos) == '\n' || stringRepresentation.charAt(pos) == '\r') {
                if (bitsPos > rowStartPos) {
                    if (rowLength == -1) {
                        rowLength = bitsPos - rowStartPos;
                    } else if (bitsPos - rowStartPos != rowLength) {
                        throw new IllegalArgumentException("row lengths do not match");
                    }
                    rowStartPos = bitsPos;
                    nRows++;
                }
                pos++;
            } else if (stringRepresentation.startsWith(setString, pos)) {
                pos += setString.length();
                bits[bitsPos] = true;
                bitsPos++;
            } else if (stringRepresentation.startsWith(unsetString, pos)) {
                pos += unsetString.length();
                bits[bitsPos] = false;
                bitsPos++;
            } else {
                throw new IllegalArgumentException("illegal character encountered: " + stringRepresentation.substring(pos));
            }
        }
        if (bitsPos > rowStartPos) {
            if (rowLength == -1) {
                rowLength = bitsPos - rowStartPos;
            } else if (bitsPos - rowStartPos != rowLength) {
                throw new IllegalArgumentException("row lengths do not match");
            }
            nRows++;
        }
        BitMatrix matrix = new BitMatrix(rowLength, nRows);
        for (int i = 0; i < bitsPos; i++) {
            if (bits[i]) {
                matrix.set(i % rowLength, i / rowLength);
            }
        }
        return matrix;
    }

    public boolean get(int x, int y) {
        int offset = (this.rowSize * y) + (x / 32);
        return ((this.bits[offset] >>> (x & 31)) & 1) != 0;
    }

    public void set(int x, int y) {
        int offset = (this.rowSize * y) + (x / 32);
        int[] iArr = this.bits;
        iArr[offset] = iArr[offset] | (1 << (x & 31));
    }

    public void unset(int x, int y) {
        int offset = (this.rowSize * y) + (x / 32);
        int[] iArr = this.bits;
        iArr[offset] = iArr[offset] & (~(1 << (x & 31)));
    }

    public void flip(int x, int y) {
        int offset = (this.rowSize * y) + (x / 32);
        int[] iArr = this.bits;
        iArr[offset] = iArr[offset] ^ (1 << (x & 31));
    }

    public void xor(BitMatrix mask) {
        if (this.width != mask.width || this.height != mask.height || this.rowSize != mask.rowSize) {
            throw new IllegalArgumentException("input matrix dimensions do not match");
        }
        BitArray rowArray = new BitArray(this.width);
        for (int y = 0; y < this.height; y++) {
            int offset = this.rowSize * y;
            int[] row = mask.getRow(y, rowArray).getBitArray();
            for (int x = 0; x < this.rowSize; x++) {
                int[] iArr = this.bits;
                int i = offset + x;
                iArr[i] = iArr[i] ^ row[x];
            }
        }
    }

    public void clear() {
        int max = this.bits.length;
        for (int i = 0; i < max; i++) {
            this.bits[i] = 0;
        }
    }

    public void setRegion(int left, int top, int width, int height) {
        if (top < 0 || left < 0) {
            throw new IllegalArgumentException("Left and top must be nonnegative");
        }
        if (height <= 0 || width <= 0) {
            throw new IllegalArgumentException("Height and width must be at least 1");
        }
        int right = left + width;
        int bottom = top + height;
        if (bottom > this.height || right > this.width) {
            throw new IllegalArgumentException("The region must fit inside the matrix");
        }
        for (int y = top; y < bottom; y++) {
            int offset = this.rowSize * y;
            for (int x = left; x < right; x++) {
                int[] iArr = this.bits;
                int i = (x / 32) + offset;
                iArr[i] = iArr[i] | (1 << (x & 31));
            }
        }
    }

    public BitArray getRow(int y, BitArray row) {
        if (row == null || row.getSize() < this.width) {
            row = new BitArray(this.width);
        } else {
            row.clear();
        }
        int offset = this.rowSize * y;
        for (int x = 0; x < this.rowSize; x++) {
            row.setBulk(x << 5, this.bits[offset + x]);
        }
        return row;
    }

    public void setRow(int y, BitArray row) {
        System.arraycopy(row.getBitArray(), 0, this.bits, this.rowSize * y, this.rowSize);
    }

    public void rotate180() {
        BitArray topRow = new BitArray(this.width);
        BitArray bottomRow = new BitArray(this.width);
        int maxHeight = (this.height + 1) / 2;
        for (int i = 0; i < maxHeight; i++) {
            topRow = getRow(i, topRow);
            int bottomRowIndex = (this.height - 1) - i;
            bottomRow = getRow(bottomRowIndex, bottomRow);
            topRow.reverse();
            bottomRow.reverse();
            setRow(i, bottomRow);
            setRow(bottomRowIndex, topRow);
        }
    }

    public int[] getEnclosingRectangle() {
        int left = this.width;
        int top = this.height;
        int right = -1;
        int bottom = -1;
        for (int y = 0; y < this.height; y++) {
            for (int x32 = 0; x32 < this.rowSize; x32++) {
                int theBits = this.bits[(this.rowSize * y) + x32];
                if (theBits != 0) {
                    if (y < top) {
                        top = y;
                    }
                    if (y > bottom) {
                        bottom = y;
                    }
                    if ((x32 << 5) < left) {
                        int bit = 0;
                        while ((theBits << (31 - bit)) == 0) {
                            bit++;
                        }
                        if ((x32 << 5) + bit < left) {
                            left = (x32 << 5) + bit;
                        }
                    }
                    int bit2 = x32 << 5;
                    if (bit2 + 31 > right) {
                        int bit3 = 31;
                        while ((theBits >>> bit3) == 0) {
                            bit3--;
                        }
                        if ((x32 << 5) + bit3 > right) {
                            right = (x32 << 5) + bit3;
                        }
                    }
                }
            }
        }
        if (right < left || bottom < top) {
            return null;
        }
        return new int[]{left, top, (right - left) + 1, (bottom - top) + 1};
    }

    public int[] getTopLeftOnBit() {
        int bitsOffset = 0;
        while (bitsOffset < this.bits.length && this.bits[bitsOffset] == 0) {
            bitsOffset++;
        }
        if (bitsOffset == this.bits.length) {
            return null;
        }
        int y = bitsOffset / this.rowSize;
        int x = (bitsOffset % this.rowSize) << 5;
        int theBits = this.bits[bitsOffset];
        int bit = 0;
        while ((theBits << (31 - bit)) == 0) {
            bit++;
        }
        return new int[]{x + bit, y};
    }

    public int[] getBottomRightOnBit() {
        int bitsOffset = this.bits.length - 1;
        while (bitsOffset >= 0 && this.bits[bitsOffset] == 0) {
            bitsOffset--;
        }
        if (bitsOffset < 0) {
            return null;
        }
        int y = bitsOffset / this.rowSize;
        int x = (bitsOffset % this.rowSize) << 5;
        int theBits = this.bits[bitsOffset];
        int bit = 31;
        while ((theBits >>> bit) == 0) {
            bit--;
        }
        return new int[]{x + bit, y};
    }

    public int getWidth() {
        return this.width;
    }

    public int getHeight() {
        return this.height;
    }

    public int getRowSize() {
        return this.rowSize;
    }

    public boolean equals(Object o) {
        if (!(o instanceof BitMatrix)) {
            return false;
        }
        BitMatrix other = (BitMatrix) o;
        return this.width == other.width && this.height == other.height && this.rowSize == other.rowSize && Arrays.equals(this.bits, other.bits);
    }

    public int hashCode() {
        return (((((((this.width * 31) + this.width) * 31) + this.height) * 31) + this.rowSize) * 31) + Arrays.hashCode(this.bits);
    }

    public String toString() {
        return toString("X ", "  ");
    }

    public String toString(String setString, String unsetString) {
        return buildToString(setString, unsetString, "\n");
    }

    @Deprecated
    public String toString(String setString, String unsetString, String lineSeparator) {
        return buildToString(setString, unsetString, lineSeparator);
    }

    private String buildToString(String setString, String unsetString, String lineSeparator) {
        StringBuilder result = new StringBuilder(this.height * (this.width + 1));
        for (int y = 0; y < this.height; y++) {
            for (int x = 0; x < this.width; x++) {
                result.append(get(x, y) ? setString : unsetString);
            }
            result.append(lineSeparator);
        }
        return result.toString();
    }

    /* JADX INFO: renamed from: clone, reason: merged with bridge method [inline-methods] */
    public BitMatrix m139clone() {
        return new BitMatrix(this.width, this.height, this.rowSize, (int[]) this.bits.clone());
    }
}
