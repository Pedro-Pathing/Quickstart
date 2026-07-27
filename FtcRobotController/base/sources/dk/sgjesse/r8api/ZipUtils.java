package dk.sgjesse.r8api;

import com.android.tools.r8.ByteDataView;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.zip.CRC32;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/* JADX INFO: loaded from: classes.dex */
class ZipUtils {
    ZipUtils() {
    }

    public static boolean isClassFile(String entry) {
        String name = entry.toLowerCase();
        if (name.endsWith(FileUtils.MODULE_INFO_CLASS) || name.startsWith("meta-inf") || name.startsWith("/meta-inf")) {
            return false;
        }
        return name.endsWith(FileUtils.CLASS_EXTENSION);
    }

    public static void writeToZipStream(ZipOutputStream stream, String entry, ByteDataView content, int compressionMethod) throws IOException {
        byte[] buffer = content.getBuffer();
        int offset = content.getOffset();
        int length = content.getLength();
        CRC32 crc = new CRC32();
        crc.update(buffer, offset, length);
        ZipEntry zipEntry = new ZipEntry(entry);
        zipEntry.setMethod(compressionMethod);
        zipEntry.setSize(length);
        zipEntry.setCrc(crc.getValue());
        zipEntry.setTime(0L);
        stream.putNextEntry(zipEntry);
        stream.write(buffer, offset, length);
        stream.closeEntry();
    }

    public static byte[] toByteArray(InputStream is) throws IOException {
        ByteArrayOutputStream os = new ByteArrayOutputStream();
        AppUtil.getInstance().copyStream(is, os);
        return os.toByteArray();
    }
}
