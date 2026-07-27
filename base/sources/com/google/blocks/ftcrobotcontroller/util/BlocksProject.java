package com.google.blocks.ftcrobotcontroller.util;

/* JADX INFO: loaded from: classes8.dex */
class BlocksProject {
    final String content;
    final long dateModifiedMillis;
    final String fileName;

    BlocksProject(String fileName, String content, long dateModifiedMillis) {
        this.fileName = fileName;
        this.content = content;
        this.dateModifiedMillis = dateModifiedMillis;
    }

    public boolean equals(Object o) {
        if (!(o instanceof BlocksProject)) {
            return false;
        }
        BlocksProject that = (BlocksProject) o;
        return this.fileName.equals(that.fileName) && this.content.equals(that.content) && this.dateModifiedMillis == that.dateModifiedMillis;
    }

    public int hashCode() {
        return this.fileName.hashCode() + this.content.hashCode();
    }
}
