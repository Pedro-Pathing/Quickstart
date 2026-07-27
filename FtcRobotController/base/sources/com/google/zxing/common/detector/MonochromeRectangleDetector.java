package com.google.zxing.common.detector;

import com.google.zxing.NotFoundException;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.BitMatrix;

/* JADX INFO: loaded from: classes8.dex */
@Deprecated
public final class MonochromeRectangleDetector {
    private static final int MAX_MODULES = 32;
    private final BitMatrix image;

    public MonochromeRectangleDetector(BitMatrix image) {
        this.image = image;
    }

    public ResultPoint[] detect() throws NotFoundException {
        int height = this.image.getHeight();
        int width = this.image.getWidth();
        int halfHeight = height / 2;
        int halfWidth = width / 2;
        int deltaY = Math.max(1, height / 256);
        int deltaX = Math.max(1, width / 256);
        int top = ((int) findCornerFromCenter(halfWidth, 0, 0, width, halfHeight, -deltaY, 0, height, halfWidth / 2).getY()) - 1;
        ResultPoint pointB = findCornerFromCenter(halfWidth, -deltaX, 0, width, halfHeight, 0, top, height, halfHeight / 2);
        int left = ((int) pointB.getX()) - 1;
        ResultPoint pointC = findCornerFromCenter(halfWidth, deltaX, left, width, halfHeight, 0, top, height, halfHeight / 2);
        int right = ((int) pointC.getX()) + 1;
        ResultPoint pointD = findCornerFromCenter(halfWidth, 0, left, right, halfHeight, deltaY, top, height, halfWidth / 2);
        int bottom = ((int) pointD.getY()) + 1;
        ResultPoint pointA = findCornerFromCenter(halfWidth, 0, left, right, halfHeight, -deltaY, top, bottom, halfWidth / 4);
        return new ResultPoint[]{pointA, pointB, pointC, pointD};
    }

    private ResultPoint findCornerFromCenter(int centerX, int deltaX, int left, int right, int centerY, int deltaY, int top, int bottom, int maxWhiteRun) throws NotFoundException {
        int[] range;
        int[] lastRange = null;
        int y = centerY;
        int x = centerX;
        while (y < bottom && y >= top && x < right && x >= left) {
            if (deltaX == 0) {
                range = blackWhiteRange(y, maxWhiteRun, left, right, true);
            } else {
                range = blackWhiteRange(x, maxWhiteRun, top, bottom, false);
            }
            if (range == null) {
                if (lastRange == null) {
                    throw NotFoundException.getNotFoundInstance();
                }
                if (deltaX == 0) {
                    int lastY = y - deltaY;
                    if (lastRange[0] < centerX) {
                        if (lastRange[1] > centerX) {
                            return new ResultPoint(lastRange[deltaY > 0 ? (char) 0 : (char) 1], lastY);
                        }
                        return new ResultPoint(lastRange[0], lastY);
                    }
                    return new ResultPoint(lastRange[1], lastY);
                }
                int lastY2 = x - deltaX;
                if (lastRange[0] < centerY) {
                    if (lastRange[1] > centerY) {
                        return new ResultPoint(lastY2, lastRange[deltaX < 0 ? (char) 0 : (char) 1]);
                    }
                    return new ResultPoint(lastY2, lastRange[0]);
                }
                return new ResultPoint(lastY2, lastRange[1]);
            }
            lastRange = range;
            y += deltaY;
            x += deltaX;
        }
        throw NotFoundException.getNotFoundInstance();
    }

    /* JADX WARN: Removed duplicated region for block: B:15:0x0021  */
    /* JADX WARN: Removed duplicated region for block: B:41:0x005a  */
    /* JADX WARN: Removed duplicated region for block: B:72:0x0032 A[EDGE_INSN: B:72:0x0032->B:22:0x0032 BREAK  A[LOOP:1: B:13:0x001d->B:75:0x001d], SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:88:0x006b A[EDGE_INSN: B:88:0x006b->B:48:0x006b BREAK  A[LOOP:3: B:39:0x0056->B:93:0x0056], SYNTHETIC] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private int[] blackWhiteRange(int r6, int r7, int r8, int r9, boolean r10) {
        /*
            r5 = this;
            int r0 = r8 + r9
            int r0 = r0 / 2
            r1 = 0
            r1 = r0
        L6:
            if (r0 < r8) goto L3c
            com.google.zxing.common.BitMatrix r2 = r5.image
            if (r10 == 0) goto L13
            boolean r2 = r2.get(r0, r6)
            if (r2 == 0) goto L1c
            goto L19
        L13:
            boolean r2 = r2.get(r6, r0)
            if (r2 == 0) goto L1c
        L19:
            int r0 = r0 + (-1)
            goto L6
        L1c:
            r2 = r0
        L1d:
            int r0 = r0 + (-1)
            if (r0 < r8) goto L32
            com.google.zxing.common.BitMatrix r3 = r5.image
            if (r10 == 0) goto L2c
            boolean r3 = r3.get(r0, r6)
            if (r3 == 0) goto L1d
            goto L32
        L2c:
            boolean r3 = r3.get(r6, r0)
            if (r3 == 0) goto L1d
        L32:
            int r3 = r2 - r0
            if (r0 < r8) goto L3a
            if (r3 <= r7) goto L39
            goto L3a
        L39:
            goto L6
        L3a:
            r0 = r2
        L3c:
            int r0 = r0 + 1
            r2 = r1
        L3f:
            if (r2 >= r9) goto L75
            com.google.zxing.common.BitMatrix r3 = r5.image
            if (r10 == 0) goto L4c
            boolean r3 = r3.get(r2, r6)
            if (r3 == 0) goto L55
            goto L52
        L4c:
            boolean r3 = r3.get(r6, r2)
            if (r3 == 0) goto L55
        L52:
            int r2 = r2 + 1
            goto L3f
        L55:
            r3 = r2
        L56:
            int r2 = r2 + 1
            if (r2 >= r9) goto L6b
            com.google.zxing.common.BitMatrix r4 = r5.image
            if (r10 == 0) goto L65
            boolean r4 = r4.get(r2, r6)
            if (r4 == 0) goto L56
            goto L6b
        L65:
            boolean r4 = r4.get(r6, r2)
            if (r4 == 0) goto L56
        L6b:
            int r4 = r2 - r3
            if (r2 >= r9) goto L73
            if (r4 <= r7) goto L72
            goto L73
        L72:
            goto L3f
        L73:
            r2 = r3
        L75:
            int r2 = r2 + (-1)
            if (r2 <= r0) goto L7e
            int[] r3 = new int[]{r0, r2}
            return r3
        L7e:
            r3 = 0
            return r3
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.zxing.common.detector.MonochromeRectangleDetector.blackWhiteRange(int, int, int, int, boolean):int[]");
    }
}
