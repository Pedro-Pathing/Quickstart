package com.google.zxing.aztec;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.ResultMetadataType;
import com.google.zxing.ResultPoint;
import com.google.zxing.ResultPointCallback;
import com.google.zxing.aztec.decoder.Decoder;
import com.google.zxing.aztec.detector.Detector;
import com.google.zxing.common.DecoderResult;
import java.util.List;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class AztecReader implements Reader {
    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap image) throws NotFoundException, FormatException {
        return decode(image, null);
    }

    @Override // com.google.zxing.Reader
    public Result decode(BinaryBitmap binaryBitmap, Map<DecodeHintType, ?> map) throws NotFoundException, FormatException {
        ResultPoint[] points;
        ResultPoint[] resultPointArr;
        FormatException formatException;
        ResultPoint[] resultPointArr2;
        ResultPointCallback resultPointCallback;
        AztecDetectorResult aztecDetectorResultDetect;
        Detector detector = new Detector(binaryBitmap.getBlackMatrix());
        DecoderResult decoderResultDecode = null;
        try {
            aztecDetectorResultDetect = detector.detect(false);
            points = aztecDetectorResultDetect.getPoints();
        } catch (FormatException e) {
            e = e;
            points = null;
        } catch (NotFoundException e2) {
            e = e2;
            points = null;
        }
        try {
            resultPointArr = points;
            formatException = null;
            decoderResultDecode = new Decoder().decode(aztecDetectorResultDetect);
            e = null;
        } catch (FormatException e3) {
            e = e3;
            resultPointArr = points;
            formatException = e;
            e = null;
        } catch (NotFoundException e4) {
            e = e4;
            resultPointArr = points;
            formatException = null;
        }
        if (decoderResultDecode != null) {
            resultPointArr2 = resultPointArr;
        } else {
            try {
                AztecDetectorResult aztecDetectorResultDetect2 = detector.detect(true);
                ResultPoint[] points2 = aztecDetectorResultDetect2.getPoints();
                decoderResultDecode = new Decoder().decode(aztecDetectorResultDetect2);
                resultPointArr2 = points2;
            } catch (FormatException | NotFoundException e5) {
                if (e != null) {
                    throw e;
                }
                if (formatException != null) {
                    throw formatException;
                }
                throw e5;
            }
        }
        if (map != null && (resultPointCallback = (ResultPointCallback) map.get(DecodeHintType.NEED_RESULT_POINT_CALLBACK)) != null) {
            for (ResultPoint resultPoint : resultPointArr2) {
                resultPointCallback.foundPossibleResultPoint(resultPoint);
            }
        }
        Result result = new Result(decoderResultDecode.getText(), decoderResultDecode.getRawBytes(), decoderResultDecode.getNumBits(), resultPointArr2, BarcodeFormat.AZTEC, System.currentTimeMillis());
        List<byte[]> byteSegments = decoderResultDecode.getByteSegments();
        if (byteSegments != null) {
            result.putMetadata(ResultMetadataType.BYTE_SEGMENTS, byteSegments);
        }
        String eCLevel = decoderResultDecode.getECLevel();
        if (eCLevel != null) {
            result.putMetadata(ResultMetadataType.ERROR_CORRECTION_LEVEL, eCLevel);
        }
        return result;
    }

    @Override // com.google.zxing.Reader
    public void reset() {
    }
}
