package com.google.zxing.common;

import com.google.zxing.Binarizer;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import java.lang.reflect.Array;

/* JADX INFO: loaded from: classes8.dex */
public final class HybridBinarizer extends GlobalHistogramBinarizer {
    private static final int BLOCK_SIZE = 8;
    private static final int BLOCK_SIZE_MASK = 7;
    private static final int BLOCK_SIZE_POWER = 3;
    private static final int MINIMUM_DIMENSION = 40;
    private static final int MIN_DYNAMIC_RANGE = 24;
    private BitMatrix matrix;

    public HybridBinarizer(LuminanceSource source) {
        super(source);
    }

    @Override // com.google.zxing.common.GlobalHistogramBinarizer, com.google.zxing.Binarizer
    public BitMatrix getBlackMatrix() throws NotFoundException {
        int subWidth;
        int subHeight;
        if (this.matrix != null) {
            return this.matrix;
        }
        LuminanceSource source = getLuminanceSource();
        int width = source.getWidth();
        int height = source.getHeight();
        if (width >= 40 && height >= 40) {
            byte[] luminances = source.getMatrix();
            int subWidth2 = width >> 3;
            if ((width & 7) == 0) {
                subWidth = subWidth2;
            } else {
                subWidth = subWidth2 + 1;
            }
            int subWidth3 = height >> 3;
            if ((height & 7) == 0) {
                subHeight = subWidth3;
            } else {
                int subHeight2 = subWidth3 + 1;
                subHeight = subHeight2;
            }
            int[][] blackPoints = calculateBlackPoints(luminances, subWidth, subHeight, width, height);
            BitMatrix newMatrix = new BitMatrix(width, height);
            calculateThresholdForBlock(luminances, subWidth, subHeight, width, height, blackPoints, newMatrix);
            this.matrix = newMatrix;
        } else {
            this.matrix = super.getBlackMatrix();
        }
        return this.matrix;
    }

    @Override // com.google.zxing.common.GlobalHistogramBinarizer, com.google.zxing.Binarizer
    public Binarizer createBinarizer(LuminanceSource source) {
        return new HybridBinarizer(source);
    }

    private static void calculateThresholdForBlock(byte[] luminances, int subWidth, int subHeight, int width, int height, int[][] blackPoints, BitMatrix matrix) {
        int yoffset;
        int xoffset;
        int maxYOffset = height - 8;
        int maxXOffset = width - 8;
        for (int y = 0; y < subHeight; y++) {
            int yoffset2 = y << 3;
            if (yoffset2 <= maxYOffset) {
                yoffset = yoffset2;
            } else {
                yoffset = maxYOffset;
            }
            int top = cap(y, subHeight - 3);
            for (int x = 0; x < subWidth; x++) {
                int xoffset2 = x << 3;
                if (xoffset2 <= maxXOffset) {
                    xoffset = xoffset2;
                } else {
                    xoffset = maxXOffset;
                }
                int left = cap(x, subWidth - 3);
                int sum = 0;
                for (int z = -2; z <= 2; z++) {
                    int[] blackRow = blackPoints[top + z];
                    sum += blackRow[left - 2] + blackRow[left - 1] + blackRow[left] + blackRow[left + 1] + blackRow[left + 2];
                }
                int average = sum / 25;
                thresholdBlock(luminances, xoffset, yoffset, average, width, matrix);
            }
        }
    }

    private static int cap(int value, int max) {
        if (value < 2) {
            return 2;
        }
        return Math.min(value, max);
    }

    private static void thresholdBlock(byte[] luminances, int xoffset, int yoffset, int threshold, int stride, BitMatrix matrix) {
        int y = 0;
        int offset = (yoffset * stride) + xoffset;
        while (y < 8) {
            for (int x = 0; x < 8; x++) {
                if ((luminances[offset + x] & 255) <= threshold) {
                    matrix.set(xoffset + x, yoffset + y);
                }
            }
            y++;
            offset += stride;
        }
    }

    private static int[][] calculateBlackPoints(byte[] luminances, int subWidth, int subHeight, int width, int height) {
        char c;
        int maxYOffset = height - 8;
        int maxXOffset = width - 8;
        char c2 = 2;
        boolean z = true;
        int[][] blackPoints = (int[][]) Array.newInstance((Class<?>) Integer.TYPE, subHeight, subWidth);
        int y = 0;
        while (y < subHeight) {
            int i = y << 3;
            int yoffset = i;
            if (i > maxYOffset) {
                yoffset = maxYOffset;
            }
            int x = 0;
            while (x < subWidth) {
                int i2 = x << 3;
                int xoffset = i2;
                if (i2 > maxXOffset) {
                    xoffset = maxXOffset;
                }
                int sum = 0;
                int min = 255;
                int max = 0;
                int yy = 0;
                int offset = (yoffset * width) + xoffset;
                while (true) {
                    if (yy >= 8) {
                        break;
                    }
                    int xx = 0;
                    for (int i3 = 8; xx < i3; i3 = 8) {
                        int pixel = luminances[offset + xx] & 255;
                        sum += pixel;
                        if (pixel < min) {
                            min = pixel;
                        }
                        if (pixel > max) {
                            max = pixel;
                        }
                        xx++;
                    }
                    int xx2 = max - min;
                    if (xx2 > 24) {
                        while (true) {
                            yy++;
                            offset += width;
                            if (yy < 8) {
                                int xx3 = 0;
                                for (int i4 = 8; xx3 < i4; i4 = 8) {
                                    sum += luminances[offset + xx3] & 255;
                                    xx3++;
                                }
                            }
                        }
                    }
                    yy++;
                    offset += width;
                }
                int average = sum >> 6;
                if (max - min > 24) {
                    c = 2;
                } else {
                    average = min / 2;
                    if (y <= 0 || x <= 0) {
                        c = 2;
                    } else {
                        c = 2;
                        int averageNeighborBlackPoint = ((blackPoints[y - 1][x] + (blackPoints[y][x - 1] * 2)) + blackPoints[y - 1][x - 1]) / 4;
                        if (min < averageNeighborBlackPoint) {
                            average = averageNeighborBlackPoint;
                        }
                    }
                }
                blackPoints[y][x] = average;
                x++;
                z = true;
                c2 = c;
            }
            y++;
            c2 = c2;
        }
        return blackPoints;
    }
}
