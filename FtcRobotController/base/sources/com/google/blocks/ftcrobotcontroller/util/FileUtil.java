package com.google.blocks.ftcrobotcontroller.util;

import android.content.res.AssetManager;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.channels.FileChannel;

/* JADX INFO: loaded from: classes8.dex */
public class FileUtil {
    private FileUtil() {
    }

    public static void writeFile(File file, String content) throws IOException {
        BufferedWriter writer = new BufferedWriter(new FileWriter(file));
        try {
            writer.write(content);
        } finally {
            writer.close();
        }
    }

    public static void writeBinaryFile(File file, byte[] content) throws IOException {
        BufferedOutputStream outputStream = new BufferedOutputStream(new FileOutputStream(file));
        try {
            outputStream.write(content, 0, content.length);
        } finally {
            outputStream.close();
        }
    }

    public static String readFile(File file) throws IOException {
        StringBuilder sb = new StringBuilder();
        BufferedReader reader = new BufferedReader(new FileReader(file));
        while (true) {
            try {
                String line = reader.readLine();
                if (line != null) {
                    sb.append(line).append("\n");
                } else {
                    reader.close();
                    return sb.toString();
                }
            } catch (Throwable th) {
                reader.close();
                throw th;
            }
        }
    }

    public static byte[] readBinaryFile(File file) throws IOException {
        byte[] content = new byte[(int) file.length()];
        new StringBuilder();
        BufferedInputStream inputStream = new BufferedInputStream(new FileInputStream(file));
        try {
            inputStream.read(content, 0, content.length);
            return content;
        } finally {
            inputStream.close();
        }
    }

    public static void copyFile(File source, File dest) throws IOException {
        FileChannel sourceChannel = null;
        FileChannel destChannel = null;
        try {
            sourceChannel = new FileInputStream(source).getChannel();
            destChannel = new FileOutputStream(dest).getChannel();
            destChannel.transferFrom(sourceChannel, 0L, sourceChannel.size());
        } finally {
            if (sourceChannel != null) {
                sourceChannel.close();
            }
            if (destChannel != null) {
                destChannel.close();
            }
        }
    }

    public static void readAsset(StringBuilder sb, AssetManager assetManager, String assetName) throws IOException {
        BufferedReader reader = new BufferedReader(new InputStreamReader(assetManager.open(assetName)));
        while (true) {
            try {
                String line = reader.readLine();
                if (line != null) {
                    sb.append(line).append("\n");
                } else {
                    reader.close();
                    return;
                }
            } catch (Throwable th) {
                try {
                    reader.close();
                } catch (Throwable th2) {
                    th.addSuppressed(th2);
                }
                throw th;
            }
        }
    }
}
