package com.sun.tools.javac.file;

import com.android.tools.r8.DataResource;
import com.sun.tools.javac.file.RelativePath;
import com.sun.tools.javac.file.ZipArchive;
import com.sun.tools.javac.util.List;
import dk.sgjesse.r8api.DescriptorUtils;
import java.io.File;
import java.io.IOException;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class SymbolArchive extends ZipArchive {
    final File origFile;
    final RelativePath.RelativeDirectory prefix;

    public SymbolArchive(JavacFileManager fileManager, File orig, ZipFile zdir, RelativePath.RelativeDirectory prefix) throws IOException {
        super(fileManager, zdir, false);
        this.origFile = orig;
        this.prefix = prefix;
        initMap();
    }

    @Override // com.sun.tools.javac.file.ZipArchive
    void addZipEntry(ZipEntry entry) {
        String name = entry.getName();
        if (!name.startsWith(this.prefix.path)) {
            return;
        }
        String name2 = name.substring(this.prefix.path.length());
        int i = name2.lastIndexOf(47);
        RelativePath.RelativeDirectory dirname = new RelativePath.RelativeDirectory(name2.substring(0, i + 1));
        String basename = name2.substring(i + 1);
        if (basename.length() == 0) {
            return;
        }
        List<String> list = this.map.get(dirname);
        if (list == null) {
            list = List.nil();
        }
        this.map.put(dirname, list.prepend(basename));
    }

    @Override // com.sun.tools.javac.file.ZipArchive, com.sun.tools.javac.file.JavacFileManager.Archive
    public JavaFileObject getFileObject(RelativePath.RelativeDirectory subdirectory, String file) {
        RelativePath.RelativeDirectory prefix_subdir = new RelativePath.RelativeDirectory(this.prefix, subdirectory.path);
        ZipEntry ze = new RelativePath.RelativeFile(prefix_subdir, file).getZipEntry(this.zfile);
        return new SymbolFileObject(this, file, ze);
    }

    @Override // com.sun.tools.javac.file.ZipArchive
    public String toString() {
        return "SymbolArchive[" + this.zfile.getName() + "]";
    }

    public static class SymbolFileObject extends ZipArchive.ZipFileObject {
        protected SymbolFileObject(SymbolArchive zarch, String name, ZipEntry entry) {
            super(zarch, name, entry);
        }

        @Override // com.sun.tools.javac.file.ZipArchive.ZipFileObject, com.sun.tools.javac.file.BaseFileObject
        protected String inferBinaryName(Iterable<? extends File> path) {
            String entryName = this.entry.getName();
            String prefix = ((SymbolArchive) this.zarch).prefix.path;
            if (entryName.startsWith(prefix)) {
                entryName = entryName.substring(prefix.length());
            }
            return removeExtension(entryName).replace(DataResource.SEPARATOR, DescriptorUtils.JAVA_PACKAGE_SEPARATOR);
        }
    }
}
