package com.google.zxing;

import java.util.EnumMap;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class Result {
    private final BarcodeFormat format;
    private final int numBits;
    private final byte[] rawBytes;
    private Map<ResultMetadataType, Object> resultMetadata;
    private ResultPoint[] resultPoints;
    private final String text;
    private final long timestamp;

    public Result(String text, byte[] rawBytes, ResultPoint[] resultPoints, BarcodeFormat format) {
        this(text, rawBytes, resultPoints, format, System.currentTimeMillis());
    }

    public Result(String text, byte[] rawBytes, ResultPoint[] resultPoints, BarcodeFormat format, long timestamp) {
        this(text, rawBytes, rawBytes == null ? 0 : rawBytes.length * 8, resultPoints, format, timestamp);
    }

    public Result(String text, byte[] rawBytes, int numBits, ResultPoint[] resultPoints, BarcodeFormat format, long timestamp) {
        this.text = text;
        this.rawBytes = rawBytes;
        this.numBits = numBits;
        this.resultPoints = resultPoints;
        this.format = format;
        this.resultMetadata = null;
        this.timestamp = timestamp;
    }

    public String getText() {
        return this.text;
    }

    public byte[] getRawBytes() {
        return this.rawBytes;
    }

    public int getNumBits() {
        return this.numBits;
    }

    public ResultPoint[] getResultPoints() {
        return this.resultPoints;
    }

    public BarcodeFormat getBarcodeFormat() {
        return this.format;
    }

    public Map<ResultMetadataType, Object> getResultMetadata() {
        return this.resultMetadata;
    }

    public void putMetadata(ResultMetadataType type, Object value) {
        if (this.resultMetadata == null) {
            this.resultMetadata = new EnumMap(ResultMetadataType.class);
        }
        this.resultMetadata.put(type, value);
    }

    public void putAllMetadata(Map<ResultMetadataType, Object> metadata) {
        if (metadata != null) {
            if (this.resultMetadata == null) {
                this.resultMetadata = metadata;
            } else {
                this.resultMetadata.putAll(metadata);
            }
        }
    }

    public void addResultPoints(ResultPoint[] newPoints) {
        ResultPoint[] oldPoints = this.resultPoints;
        if (oldPoints == null) {
            this.resultPoints = newPoints;
            return;
        }
        if (newPoints != null && newPoints.length > 0) {
            ResultPoint[] allPoints = new ResultPoint[oldPoints.length + newPoints.length];
            System.arraycopy(oldPoints, 0, allPoints, 0, oldPoints.length);
            System.arraycopy(newPoints, 0, allPoints, oldPoints.length, newPoints.length);
            this.resultPoints = allPoints;
        }
    }

    public long getTimestamp() {
        return this.timestamp;
    }

    public String toString() {
        return this.text;
    }
}
