package com.google.blocks.ftcrobotcontroller.util;

import android.text.Html;
import android.util.Base64;
import com.sun.tools.doclint.DocLint;
import java.io.File;
import java.io.IOException;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/* JADX INFO: loaded from: classes8.dex */
public enum FileManager {
    SOUNDS(AppUtil.BLOCKS_SOUNDS_DIR);

    private final String VALID_NAME_REGEX = ProjectsUtil.VALID_PROJECT_REGEX;
    private final File dir;

    FileManager(File dir) {
        this.dir = dir;
    }

    public String fetchJavaScript() throws IOException {
        return "\nfunction isValidName(name) {\n  if (name) {\n    return /" + ProjectsUtil.VALID_PROJECT_REGEX + "/.test(name);\n  }\n  return false;\n}\n\n";
    }

    public String fetchFiles() throws IOException {
        File[] files = this.dir.listFiles();
        if (files != null) {
            StringBuilder json = new StringBuilder();
            json.append("[");
            String delimiter = "";
            for (int i = 0; i < files.length; i++) {
                String name = files[i].getName();
                json.append(delimiter).append("{").append("\"name\":\"").append(ProjectsUtil.escapeDoubleQuotes(name)).append("\", ").append("\"escapedName\":\"").append(ProjectsUtil.escapeDoubleQuotes(Html.escapeHtml(name))).append("\", ").append("\"dateModifiedMillis\":").append(files[i].lastModified()).append("}");
                delimiter = DocLint.TAGS_SEPARATOR;
            }
            json.append("]");
            return json.toString();
        }
        return "[]";
    }

    public boolean isValidName(String name) {
        if (name != null) {
            return name.matches(ProjectsUtil.VALID_PROJECT_REGEX);
        }
        return false;
    }

    public String fetchFileContent(String name) throws IOException {
        if (!isValidName(name)) {
            throw new IllegalArgumentException();
        }
        byte[] content = FileUtil.readBinaryFile(new File(this.dir, name));
        return Base64.encodeToString(content, 0);
    }

    public void saveFile(String name, String base64Content) throws IOException {
        if (!isValidName(name)) {
            throw new IllegalArgumentException();
        }
        if (!this.dir.exists()) {
            this.dir.mkdirs();
        }
        byte[] content = Base64.decode(base64Content, 0);
        File file = new File(this.dir, name);
        File tempBackupFile = null;
        if (file.exists()) {
            long timestamp = System.currentTimeMillis();
            tempBackupFile = new File(this.dir, "backup_" + timestamp + "_" + name);
            FileUtil.copyFile(file, tempBackupFile);
        }
        FileUtil.writeBinaryFile(file, content);
        if (tempBackupFile != null) {
            tempBackupFile.delete();
        }
    }

    public void renameFile(String oldName, String newName) throws IOException {
        if (!isValidName(oldName) || !isValidName(newName)) {
            throw new IllegalArgumentException();
        }
        if (!this.dir.exists()) {
            this.dir.mkdirs();
        }
        File oldFile = new File(this.dir, oldName);
        File newFile = new File(this.dir, newName);
        oldFile.renameTo(newFile);
    }

    public void copyFile(String oldName, String newName) throws IOException {
        if (!isValidName(oldName) || !isValidName(newName)) {
            throw new IllegalArgumentException();
        }
        if (!this.dir.exists()) {
            this.dir.mkdirs();
        }
        File oldFile = new File(this.dir, oldName);
        File newFile = new File(this.dir, newName);
        FileUtil.copyFile(oldFile, newFile);
    }

    public boolean deleteFiles(String[] names) {
        for (String name : names) {
            if (!isValidName(name)) {
                throw new IllegalArgumentException();
            }
        }
        boolean success = true;
        for (String name2 : names) {
            File file = new File(this.dir, name2);
            if (file.exists() && !file.delete()) {
                success = false;
            }
        }
        return success;
    }

    public String getPathForFile(String name) {
        return new File(this.dir, name).getAbsolutePath();
    }
}
