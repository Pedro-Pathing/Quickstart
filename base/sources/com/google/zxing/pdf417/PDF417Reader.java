package com.google.zxing.pdf417;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.ResultMetadataType;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.DecoderResult;
import com.google.zxing.multi.MultipleBarcodeReader;
import com.google.zxing.pdf417.decoder.PDF417ScanningDecoder;
import com.google.zxing.pdf417.detector.Detector;
import com.google.zxing.pdf417.detector.PDF417DetectorResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class PDF417Reader implements Reader, MultipleBarcodeReader {
    private static final Result[] EMPTY_RESULT_ARRAY = new Result[0];

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap image) throws NotFoundException, ChecksumException, FormatException {
        return decode(image, null);
    }

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap image, Map<DecodeHintType, ?> hints) throws NotFoundException, ChecksumException, FormatException {
        Result[] result = decode(image, hints, false);
        if (result.length == 0 || result[0] == null) {
            throw NotFoundException.getNotFoundInstance();
        }
        return result[0];
    }

    @Override // com.google.zxing.multi.MultipleBarcodeReader
    public Result[] decodeMultiple(BinaryBitmap image) throws NotFoundException {
        return decodeMultiple(image, null);
    }

    @Override // com.google.zxing.multi.MultipleBarcodeReader
    public Result[] decodeMultiple(BinaryBitmap image, Map<DecodeHintType, ?> hints) throws NotFoundException {
        try {
            return decode(image, hints, true);
        } catch (ChecksumException | FormatException e) {
            throw NotFoundException.getNotFoundInstance();
        }
    }

    private static Result[] decode(BinaryBitmap image, Map<DecodeHintType, ?> hints, boolean multiple) throws NotFoundException, ChecksumException, FormatException {
        List<Result> results = new ArrayList<>();
        PDF417DetectorResult detectorResult = Detector.detect(image, hints, multiple);
        for (ResultPoint[] points : detectorResult.getPoints()) {
            DecoderResult decoderResult = PDF417ScanningDecoder.decode(detectorResult.getBits(), points[4], points[5], points[6], points[7], getMinCodewordWidth(points), getMaxCodewordWidth(points));
            Result result = new Result(decoderResult.getText(), decoderResult.getRawBytes(), points, BarcodeFormat.PDF_417);
            result.putMetadata(ResultMetadataType.ERROR_CORRECTION_LEVEL, decoderResult.getECLevel());
            PDF417ResultMetadata pdf417ResultMetadata = (PDF417ResultMetadata) decoderResult.getOther();
            if (pdf417ResultMetadata != null) {
                result.putMetadata(ResultMetadataType.PDF417_EXTRA_METADATA, pdf417ResultMetadata);
            }
            results.add(result);
        }
        return (Result[]) results.toArray(EMPTY_RESULT_ARRAY);
    }

    private static int getMaxWidth(ResultPoint p1, ResultPoint p2) {
        if (p1 == null || p2 == null) {
            return 0;
        }
        return (int) Math.abs(p1.getX() - p2.getX());
    }

    private static int getMinWidth(ResultPoint p1, ResultPoint p2) {
        if (p1 == null || p2 == null) {
            return Integer.MAX_VALUE;
        }
        return (int) Math.abs(p1.getX() - p2.getX());
    }

    private static int getMaxCodewordWidth(ResultPoint[] p) {
        return Math.max(Math.max(getMaxWidth(p[0], p[4]), (getMaxWidth(p[6], p[2]) * 17) / 18), Math.max(getMaxWidth(p[1], p[5]), (getMaxWidth(p[7], p[3]) * 17) / 18));
    }

    private static int getMinCodewordWidth(ResultPoint[] p) {
        return Math.min(Math.min(getMinWidth(p[0], p[4]), (getMinWidth(p[6], p[2]) * 17) / 18), Math.min(getMinWidth(p[1], p[5]), (getMinWidth(p[7], p[3]) * 17) / 18));
    }

    @Override // com.google.zxing.Reader
    public void reset() {
    }
}
