package com.sun.tools.javac.nio;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import javax.tools.FileObject;
import javax.tools.JavaFileManager;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public interface PathFileManager extends JavaFileManager {
    FileSystem getDefaultFileSystem();

    Iterable<? extends JavaFileObject> getJavaFileObjects(Path... pathArr);

    Iterable<? extends JavaFileObject> getJavaFileObjectsFromPaths(Iterable<? extends Path> iterable);

    Iterable<? extends Path> getLocation(JavaFileManager.Location location);

    Path getPath(FileObject fileObject);

    void setDefaultFileSystem(FileSystem fileSystem);

    void setLocation(JavaFileManager.Location location, Iterable<? extends Path> iterable) throws IOException;
}
