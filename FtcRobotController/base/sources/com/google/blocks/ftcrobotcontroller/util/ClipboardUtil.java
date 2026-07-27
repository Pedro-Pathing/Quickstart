package com.google.blocks.ftcrobotcontroller.util;

import java.io.File;
import java.io.IOException;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/* JADX INFO: loaded from: classes8.dex */
public class ClipboardUtil {
    private static final File CLIPBOARD_FILE = new File(AppUtil.BLOCK_OPMODES_DIR, "clipboard.xml");

    private ClipboardUtil() {
    }

    public static void saveClipboardContent(String clipboardContent) throws IOException {
        AppUtil.getInstance().ensureDirectoryExists(AppUtil.BLOCK_OPMODES_DIR);
        FileUtil.writeFile(CLIPBOARD_FILE, clipboardContent);
    }

    public static String fetchClipboardContent() throws IOException {
        return FileUtil.readFile(CLIPBOARD_FILE);
    }
}
