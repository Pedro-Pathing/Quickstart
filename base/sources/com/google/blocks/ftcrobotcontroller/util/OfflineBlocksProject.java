package com.google.blocks.ftcrobotcontroller.util;

/* JADX INFO: loaded from: classes8.dex */
class OfflineBlocksProject extends BlocksProject {
    final boolean enabled;
    final String name;

    OfflineBlocksProject(String fileName, String content, String name, long dateModifiedMillis, boolean enabled) {
        super(fileName, content.replace("\n", " ").replaceAll("\\> +\\<", "><"), dateModifiedMillis);
        this.name = name;
        this.enabled = enabled;
    }

    @Override // com.google.blocks.ftcrobotcontroller.util.BlocksProject
    public boolean equals(Object o) {
        if (!(o instanceof OfflineBlocksProject)) {
            return false;
        }
        OfflineBlocksProject that = (OfflineBlocksProject) o;
        return this.fileName.equals(that.fileName) && this.content.equals(that.content) && this.name.equals(that.name) && this.dateModifiedMillis == that.dateModifiedMillis && this.enabled == that.enabled;
    }

    @Override // com.google.blocks.ftcrobotcontroller.util.BlocksProject
    public int hashCode() {
        return this.fileName.hashCode() + this.content.hashCode() + this.name.hashCode();
    }
}
