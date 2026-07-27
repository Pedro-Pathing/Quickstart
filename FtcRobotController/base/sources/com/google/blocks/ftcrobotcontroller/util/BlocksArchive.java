package com.google.blocks.ftcrobotcontroller.util;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

/* JADX INFO: loaded from: classes8.dex */
public class BlocksArchive {
    public static InputStream fetchBlocksArchive() throws Throwable {
        List<BlocksProject> projects = new ArrayList<>();
        ProjectsUtil.fetchProjects(projects);
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        ZipOutputStream zos = new ZipOutputStream(baos);
        try {
            for (BlocksProject project : projects) {
                ZipEntry zipEntry = new ZipEntry(project.fileName);
                zipEntry.setTime(project.dateModifiedMillis);
                zos.putNextEntry(zipEntry);
                zos.write(project.content.getBytes());
                zos.closeEntry();
            }
            zos.close();
            return new ByteArrayInputStream(baos.toByteArray());
        } catch (Throwable th) {
            try {
                zos.close();
            } catch (Throwable th2) {
                th.addSuppressed(th2);
            }
            throw th;
        }
    }
}
