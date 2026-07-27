package com.google.zxing.pdf417;

/* JADX INFO: loaded from: classes8.dex */
public final class PDF417ResultMetadata {
    private String addressee;
    private String fileId;
    private String fileName;
    private boolean lastSegment;
    private int[] optionalData;
    private int segmentIndex;
    private String sender;
    private int segmentCount = -1;
    private long fileSize = -1;
    private long timestamp = -1;
    private int checksum = -1;

    public int getSegmentIndex() {
        return this.segmentIndex;
    }

    public void setSegmentIndex(int segmentIndex) {
        this.segmentIndex = segmentIndex;
    }

    public String getFileId() {
        return this.fileId;
    }

    public void setFileId(String fileId) {
        this.fileId = fileId;
    }

    @Deprecated
    public int[] getOptionalData() {
        return this.optionalData;
    }

    @Deprecated
    public void setOptionalData(int[] optionalData) {
        this.optionalData = optionalData;
    }

    public boolean isLastSegment() {
        return this.lastSegment;
    }

    public void setLastSegment(boolean lastSegment) {
        this.lastSegment = lastSegment;
    }

    public int getSegmentCount() {
        return this.segmentCount;
    }

    public void setSegmentCount(int segmentCount) {
        this.segmentCount = segmentCount;
    }

    public String getSender() {
        return this.sender;
    }

    public void setSender(String sender) {
        this.sender = sender;
    }

    public String getAddressee() {
        return this.addressee;
    }

    public void setAddressee(String addressee) {
        this.addressee = addressee;
    }

    public String getFileName() {
        return this.fileName;
    }

    public void setFileName(String fileName) {
        this.fileName = fileName;
    }

    public long getFileSize() {
        return this.fileSize;
    }

    public void setFileSize(long fileSize) {
        this.fileSize = fileSize;
    }

    public int getChecksum() {
        return this.checksum;
    }

    public void setChecksum(int checksum) {
        this.checksum = checksum;
    }

    public long getTimestamp() {
        return this.timestamp;
    }

    public void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }
}
