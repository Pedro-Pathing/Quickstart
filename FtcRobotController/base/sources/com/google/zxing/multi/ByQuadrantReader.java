package com.google.zxing.multi;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.ResultPoint;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class ByQuadrantReader implements Reader {
    private final Reader delegate;

    public ByQuadrantReader(Reader delegate) {
        this.delegate = delegate;
    }

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap image) throws NotFoundException, ChecksumException, FormatException {
        return decode(image, null);
    }

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap binaryBitmap, Map<DecodeHintType, ?> map) throws NotFoundException, ChecksumException, FormatException {
        int width = binaryBitmap.getWidth() / 2;
        int height = binaryBitmap.getHeight() / 2;
        try {
            return this.delegate.decode(binaryBitmap.crop(0, 0, width, height), map);
        } catch (NotFoundException e) {
            try {
                Result resultDecode = this.delegate.decode(binaryBitmap.crop(width, 0, width, height), map);
                makeAbsolute(resultDecode.getResultPoints(), width, 0);
                return resultDecode;
            } catch (NotFoundException e2) {
                try {
                    Result resultDecode2 = this.delegate.decode(binaryBitmap.crop(0, height, width, height), map);
                    makeAbsolute(resultDecode2.getResultPoints(), 0, height);
                    return resultDecode2;
                } catch (NotFoundException e3) {
                    try {
                        Result resultDecode3 = this.delegate.decode(binaryBitmap.crop(width, height, width, height), map);
                        makeAbsolute(resultDecode3.getResultPoints(), width, height);
                        return resultDecode3;
                    } catch (NotFoundException e4) {
                        int i = width / 2;
                        int i2 = height / 2;
                        Result resultDecode4 = this.delegate.decode(binaryBitmap.crop(i, i2, width, height), map);
                        makeAbsolute(resultDecode4.getResultPoints(), i, i2);
                        return resultDecode4;
                    }
                }
            }
        }
    }

    @Override // com.google.zxing.Reader
    public void reset() {
        this.delegate.reset();
    }

    private static void makeAbsolute(ResultPoint[] points, int leftOffset, int topOffset) {
        if (points != null) {
            for (int i = 0; i < points.length; i++) {
                ResultPoint relative = points[i];
                if (relative != null) {
                    points[i] = new ResultPoint(relative.getX() + leftOffset, relative.getY() + topOffset);
                }
            }
        }
    }
}
