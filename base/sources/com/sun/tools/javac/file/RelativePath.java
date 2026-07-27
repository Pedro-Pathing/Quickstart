package com.sun.tools.javac.file;

import com.android.tools.r8.DataResource;
import dk.sgjesse.r8api.DescriptorUtils;
import java.io.File;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;
import javax.tools.JavaFileObject;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public abstract class RelativePath implements Comparable<RelativePath> {
    protected final String path;

    public abstract String basename();

    public abstract RelativeDirectory dirname();

    protected RelativePath(String p) {
        this.path = p;
    }

    public File getFile(File directory) {
        if (this.path.length() == 0) {
            return directory;
        }
        return new File(directory, this.path.replace(DataResource.SEPARATOR, File.separatorChar));
    }

    @Override // java.lang.Comparable
    public int compareTo(RelativePath other) {
        return this.path.compareTo(other.path);
    }

    public boolean equals(Object other) {
        if (!(other instanceof RelativePath)) {
            return false;
        }
        return this.path.equals(((RelativePath) other).path);
    }

    public int hashCode() {
        return this.path.hashCode();
    }

    public String toString() {
        return "RelPath[" + this.path + "]";
    }

    public String getPath() {
        return this.path;
    }

    public static class RelativeDirectory extends RelativePath {
        @Override // com.sun.tools.javac.file.RelativePath, java.lang.Comparable
        public /* bridge */ /* synthetic */ int compareTo(RelativePath relativePath) {
            return super.compareTo(relativePath);
        }

        static RelativeDirectory forPackage(CharSequence packageName) {
            return new RelativeDirectory(packageName.toString().replace(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, DataResource.SEPARATOR));
        }

        public RelativeDirectory(String p) {
            super((p.length() == 0 || p.endsWith(OnBotJavaFileSystemUtils.PATH_SEPARATOR)) ? p : p + OnBotJavaFileSystemUtils.PATH_SEPARATOR);
        }

        public RelativeDirectory(RelativeDirectory d, String p) {
            this(d.path + p);
        }

        @Override // com.sun.tools.javac.file.RelativePath
        public RelativeDirectory dirname() {
            int l = this.path.length();
            if (l == 0) {
                return this;
            }
            int sep = this.path.lastIndexOf(47, l - 2);
            return new RelativeDirectory(this.path.substring(0, sep + 1));
        }

        @Override // com.sun.tools.javac.file.RelativePath
        public String basename() {
            int l = this.path.length();
            if (l == 0) {
                return this.path;
            }
            int sep = this.path.lastIndexOf(47, l - 2);
            return this.path.substring(sep + 1, l - 1);
        }

        boolean contains(RelativePath other) {
            return other.path.length() > this.path.length() && other.path.startsWith(this.path);
        }

        @Override // com.sun.tools.javac.file.RelativePath
        public String toString() {
            return "RelativeDirectory[" + this.path + "]";
        }
    }

    public static class RelativeFile extends RelativePath {
        @Override // com.sun.tools.javac.file.RelativePath, java.lang.Comparable
        public /* bridge */ /* synthetic */ int compareTo(RelativePath relativePath) {
            return super.compareTo(relativePath);
        }

        static RelativeFile forClass(CharSequence className, JavaFileObject.Kind kind) {
            return new RelativeFile(className.toString().replace(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, DataResource.SEPARATOR) + kind.extension);
        }

        public RelativeFile(String p) {
            super(p);
            if (p.endsWith(OnBotJavaFileSystemUtils.PATH_SEPARATOR)) {
                throw new IllegalArgumentException(p);
            }
        }

        public RelativeFile(RelativeDirectory d, String p) {
            this(d.path + p);
        }

        RelativeFile(RelativeDirectory d, RelativePath p) {
            this(d, p.path);
        }

        @Override // com.sun.tools.javac.file.RelativePath
        public RelativeDirectory dirname() {
            int sep = this.path.lastIndexOf(47);
            return new RelativeDirectory(this.path.substring(0, sep + 1));
        }

        @Override // com.sun.tools.javac.file.RelativePath
        public String basename() {
            int sep = this.path.lastIndexOf(47);
            return this.path.substring(sep + 1);
        }

        ZipEntry getZipEntry(ZipFile zip) {
            return zip.getEntry(this.path);
        }

        @Override // com.sun.tools.javac.file.RelativePath
        public String toString() {
            return "RelativeFile[" + this.path + "]";
        }
    }
}
