package com.sun.tools.javac.file;

import com.sun.tools.javac.file.RelativePath;
import com.sun.tools.javac.util.List;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.lang.ref.Reference;
import java.lang.ref.SoftReference;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.zip.DataFormatException;
import java.util.zip.Inflater;
import java.util.zip.ZipException;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class ZipFileIndex {
    private static final boolean NON_BATCH_MODE;
    public static final long NOT_MODIFIED = Long.MIN_VALUE;
    private Reference<File> absFileRef;
    private Entry[] entries;
    private SoftReference<Inflater> inflaterRef;
    private final String preindexedCacheLocation;
    final RelativePath.RelativeDirectory symbolFilePrefix;
    private final int symbolFilePrefixLength;
    private final boolean usePreindexedCache;
    private boolean writeIndex;
    final File zipFile;
    long zipFileLastModified;
    private RandomAccessFile zipRandomFile;
    private static final String MIN_CHAR = String.valueOf((char) 0);
    private static final String MAX_CHAR = String.valueOf((char) 65535);
    private Map<RelativePath.RelativeDirectory, DirectoryEntry> directories = Collections.emptyMap();
    private Set<RelativePath.RelativeDirectory> allDirs = Collections.emptySet();
    private boolean readFromIndex = false;
    private File zipIndexFile = null;
    private boolean triedToReadIndex = false;
    private boolean hasPopulatedData = false;
    long lastReferenceTimeStamp = Long.MIN_VALUE;
    private Map<String, SoftReference<RelativePath.RelativeDirectory>> relativeDirectoryCache = new HashMap();

    static {
        NON_BATCH_MODE = System.getProperty("nonBatchMode") != null;
    }

    public synchronized boolean isOpen() {
        return this.zipRandomFile != null;
    }

    ZipFileIndex(File zipFile, RelativePath.RelativeDirectory symbolFilePrefix, boolean writeIndex, boolean useCache, String cacheLocation) throws IOException {
        this.zipFileLastModified = Long.MIN_VALUE;
        this.writeIndex = false;
        this.zipFile = zipFile;
        this.symbolFilePrefix = symbolFilePrefix;
        this.symbolFilePrefixLength = symbolFilePrefix != null ? symbolFilePrefix.getPath().getBytes("UTF-8").length : 0;
        this.writeIndex = writeIndex;
        this.usePreindexedCache = useCache;
        this.preindexedCacheLocation = cacheLocation;
        if (zipFile != null) {
            this.zipFileLastModified = zipFile.lastModified();
        }
        checkIndex();
    }

    public String toString() {
        return "ZipFileIndex[" + this.zipFile + "]";
    }

    protected void finalize() throws Throwable {
        closeFile();
        super.finalize();
    }

    private boolean isUpToDate() {
        if (this.zipFile == null) {
            return false;
        }
        if ((!NON_BATCH_MODE || this.zipFileLastModified == this.zipFile.lastModified()) && this.hasPopulatedData) {
            return true;
        }
        return false;
    }

    private void checkIndex() throws IOException {
        boolean isUpToDate = true;
        if (!isUpToDate()) {
            closeFile();
            isUpToDate = false;
        }
        if (this.zipRandomFile != null || isUpToDate) {
            this.lastReferenceTimeStamp = System.currentTimeMillis();
            return;
        }
        this.hasPopulatedData = true;
        if (readIndex()) {
            this.lastReferenceTimeStamp = System.currentTimeMillis();
            return;
        }
        this.directories = Collections.emptyMap();
        this.allDirs = Collections.emptySet();
        try {
            openFile();
            long totalLength = this.zipRandomFile.length();
            ZipDirectory directory = new ZipDirectory(this.zipRandomFile, 0L, totalLength, this);
            directory.buildIndex();
            this.lastReferenceTimeStamp = System.currentTimeMillis();
        } finally {
            if (this.zipRandomFile != null) {
                closeFile();
            }
        }
    }

    private void openFile() throws FileNotFoundException {
        if (this.zipRandomFile == null && this.zipFile != null) {
            this.zipRandomFile = new RandomAccessFile(this.zipFile, "r");
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void cleanupState() {
        this.entries = Entry.EMPTY_ARRAY;
        this.directories = Collections.emptyMap();
        this.zipFileLastModified = Long.MIN_VALUE;
        this.allDirs = Collections.emptySet();
    }

    public synchronized void close() {
        writeIndex();
        closeFile();
    }

    private void closeFile() {
        if (this.zipRandomFile != null) {
            try {
                this.zipRandomFile.close();
            } catch (IOException e) {
            }
            this.zipRandomFile = null;
        }
    }

    synchronized Entry getZipIndexEntry(RelativePath path) {
        Entry entry;
        entry = null;
        try {
            checkIndex();
            DirectoryEntry de = this.directories.get(path.dirname());
            String lookFor = path.basename();
            if (de != null) {
                entry = de.getEntry(lookFor);
            }
        } catch (IOException e) {
            return null;
        }
        return entry;
    }

    public synchronized List<String> getFiles(RelativePath.RelativeDirectory path) {
        try {
            checkIndex();
            DirectoryEntry de = this.directories.get(path);
            List<String> ret = de == null ? null : de.getFiles();
            if (ret == null) {
                return List.nil();
            }
            return ret;
        } catch (IOException e) {
            return List.nil();
        }
    }

    public synchronized java.util.List<String> getDirectories(RelativePath.RelativeDirectory path) {
        try {
            checkIndex();
            DirectoryEntry de = this.directories.get(path);
            List<String> ret = de == null ? null : de.getDirectories();
            if (ret == null) {
                return List.nil();
            }
            return ret;
        } catch (IOException e) {
            return List.nil();
        }
    }

    public synchronized Set<RelativePath.RelativeDirectory> getAllDirectories() {
        try {
            checkIndex();
            if (this.allDirs == Collections.EMPTY_SET) {
                this.allDirs = new LinkedHashSet(this.directories.keySet());
            }
        } catch (IOException e) {
            return Collections.emptySet();
        }
        return this.allDirs;
    }

    public synchronized boolean contains(RelativePath path) {
        try {
            checkIndex();
        } catch (IOException e) {
            return false;
        }
        return getZipIndexEntry(path) != null;
    }

    public synchronized boolean isDirectory(RelativePath path) throws IOException {
        if (path.getPath().length() == 0) {
            this.lastReferenceTimeStamp = System.currentTimeMillis();
            return true;
        }
        checkIndex();
        return this.directories.get(path) != null;
    }

    public synchronized long getLastModified(RelativePath.RelativeFile path) throws IOException {
        Entry entry;
        entry = getZipIndexEntry(path);
        if (entry == null) {
            throw new FileNotFoundException();
        }
        return entry.getLastModified();
    }

    public synchronized int length(RelativePath.RelativeFile path) throws IOException {
        Entry entry = getZipIndexEntry(path);
        if (entry == null) {
            throw new FileNotFoundException();
        }
        if (entry.isDir) {
            return 0;
        }
        byte[] header = getHeader(entry);
        if (get2ByteLittleEndian(header, 8) == 0) {
            return entry.compressedSize;
        }
        return entry.size;
    }

    public synchronized byte[] read(RelativePath.RelativeFile path) throws IOException {
        Entry entry;
        entry = getZipIndexEntry(path);
        if (entry == null) {
            throw new FileNotFoundException("Path not found in ZIP: " + path.path);
        }
        return read(entry);
    }

    synchronized byte[] read(Entry entry) throws IOException {
        byte[] result;
        openFile();
        result = readBytes(entry);
        closeFile();
        return result;
    }

    public synchronized int read(RelativePath.RelativeFile path, byte[] buffer) throws IOException {
        Entry entry;
        entry = getZipIndexEntry(path);
        if (entry == null) {
            throw new FileNotFoundException();
        }
        return read(entry, buffer);
    }

    synchronized int read(Entry entry, byte[] buffer) throws IOException {
        int result;
        result = readBytes(entry, buffer);
        return result;
    }

    private byte[] readBytes(Entry entry) throws IOException {
        byte[] header = getHeader(entry);
        int csize = entry.compressedSize;
        byte[] cbuf = new byte[csize];
        this.zipRandomFile.skipBytes(get2ByteLittleEndian(header, 26) + get2ByteLittleEndian(header, 28));
        this.zipRandomFile.readFully(cbuf, 0, csize);
        if (get2ByteLittleEndian(header, 8) == 0) {
            return cbuf;
        }
        int size = entry.size;
        byte[] buf = new byte[size];
        if (inflate(cbuf, buf) != size) {
            throw new ZipException("corrupted zip file");
        }
        return buf;
    }

    private int readBytes(Entry entry, byte[] buffer) throws IOException {
        byte[] header = getHeader(entry);
        if (get2ByteLittleEndian(header, 8) == 0) {
            this.zipRandomFile.skipBytes(get2ByteLittleEndian(header, 26) + get2ByteLittleEndian(header, 28));
            int offset = 0;
            int size = buffer.length;
            while (offset < size) {
                int count = this.zipRandomFile.read(buffer, offset, size - offset);
                if (count == -1) {
                    break;
                }
                offset += count;
            }
            return entry.size;
        }
        int offset2 = entry.compressedSize;
        byte[] cbuf = new byte[offset2];
        this.zipRandomFile.skipBytes(get2ByteLittleEndian(header, 26) + get2ByteLittleEndian(header, 28));
        this.zipRandomFile.readFully(cbuf, 0, offset2);
        if (inflate(cbuf, buffer) == -1) {
            throw new ZipException("corrupted zip file");
        }
        return entry.size;
    }

    private byte[] getHeader(Entry entry) throws IOException {
        this.zipRandomFile.seek(entry.offset);
        byte[] header = new byte[30];
        this.zipRandomFile.readFully(header);
        if (get4ByteLittleEndian(header, 0) != 67324752) {
            throw new ZipException("corrupted zip file");
        }
        if ((get2ByteLittleEndian(header, 6) & 1) != 0) {
            throw new ZipException("encrypted zip file");
        }
        return header;
    }

    private int inflate(byte[] src, byte[] dest) {
        Inflater inflater = this.inflaterRef == null ? null : this.inflaterRef.get();
        if (inflater == null) {
            Inflater inflater2 = new Inflater(true);
            inflater = inflater2;
            this.inflaterRef = new SoftReference<>(inflater2);
        }
        inflater.reset();
        inflater.setInput(src);
        try {
            return inflater.inflate(dest);
        } catch (DataFormatException e) {
            return -1;
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static int get2ByteLittleEndian(byte[] buf, int pos) {
        return (buf[pos] & 255) + ((buf[pos + 1] & 255) << 8);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static int get4ByteLittleEndian(byte[] buf, int pos) {
        return (buf[pos] & 255) + ((buf[pos + 1] & 255) << 8) + ((buf[pos + 2] & 255) << 16) + ((buf[pos + 3] & 255) << 24);
    }

    private class ZipDirectory {
        private RelativePath.RelativeDirectory lastDir;
        private int lastLen;
        private int lastStart;
        byte[] zipDir;
        ZipFileIndex zipFileIndex;
        RandomAccessFile zipRandomFile;

        public ZipDirectory(RandomAccessFile zipRandomFile, long start, long end, ZipFileIndex index) throws IOException {
            this.zipRandomFile = null;
            this.zipFileIndex = null;
            this.zipRandomFile = zipRandomFile;
            this.zipFileIndex = index;
            hasValidHeader();
            findCENRecord(start, end);
        }

        private boolean hasValidHeader() throws IOException {
            long pos = this.zipRandomFile.getFilePointer();
            try {
                if (this.zipRandomFile.read() == 80 && this.zipRandomFile.read() == 75 && this.zipRandomFile.read() == 3) {
                    if (this.zipRandomFile.read() == 4) {
                        this.zipRandomFile.seek(pos);
                        return true;
                    }
                }
                this.zipRandomFile.seek(pos);
                throw new ZipFormatException("invalid zip magic");
            } catch (Throwable th) {
                this.zipRandomFile.seek(pos);
                throw th;
            }
        }

        private void findCENRecord(long start, long end) throws IOException {
            long totalLength = end - start;
            int endbuflen = 1024;
            byte[] endbuf = new byte[1024];
            long endbufend = end - start;
            while (true) {
                if (endbufend >= 22) {
                    if (endbufend < endbuflen) {
                        endbuflen = (int) endbufend;
                    }
                    long endbufpos = endbufend - ((long) endbuflen);
                    this.zipRandomFile.seek(start + endbufpos);
                    this.zipRandomFile.readFully(endbuf, 0, endbuflen);
                    int i = endbuflen - 22;
                    for (long j = 22; i >= 0 && (endbuf[i] != 80 || endbuf[i + 1] != 75 || endbuf[i + 2] != 5 || endbuf[i + 3] != 6 || ((long) i) + endbufpos + j + ((long) ZipFileIndex.get2ByteLittleEndian(endbuf, i + 20)) != totalLength); j = 22) {
                        i--;
                    }
                    if (i >= 0) {
                        this.zipDir = new byte[ZipFileIndex.get4ByteLittleEndian(endbuf, i + 12)];
                        int sz = ZipFileIndex.get4ByteLittleEndian(endbuf, i + 16);
                        if (sz < 0 || ZipFileIndex.get2ByteLittleEndian(endbuf, i + 10) == 65535) {
                            throw new ZipFormatException("detected a zip64 archive");
                        }
                        this.zipRandomFile.seek(start + ((long) sz));
                        this.zipRandomFile.readFully(this.zipDir, 0, this.zipDir.length);
                        return;
                    }
                    endbufend = endbufpos + 21;
                } else {
                    throw new ZipException("cannot read zip file");
                }
            }
        }

        /* JADX INFO: Access modifiers changed from: private */
        public void buildIndex() throws IOException {
            int len = this.zipDir.length;
            if (len <= 0) {
                ZipFileIndex.this.cleanupState();
                return;
            }
            ZipFileIndex.this.directories = new LinkedHashMap();
            ArrayList<Entry> entryList = new ArrayList<>();
            int pos = 0;
            while (pos < len) {
                pos = readEntry(pos, entryList, ZipFileIndex.this.directories);
            }
            for (RelativePath.RelativeDirectory d : ZipFileIndex.this.directories.keySet()) {
                RelativePath.RelativeDirectory parent = ZipFileIndex.this.getRelativeDirectory(d.dirname().getPath());
                String file = d.basename();
                Entry zipFileIndexEntry = new Entry(parent, file);
                zipFileIndexEntry.isDir = true;
                entryList.add(zipFileIndexEntry);
            }
            ZipFileIndex.this.entries = (Entry[]) entryList.toArray(new Entry[entryList.size()]);
            Arrays.sort(ZipFileIndex.this.entries);
        }

        private int readEntry(int pos, java.util.List<Entry> entryList, Map<RelativePath.RelativeDirectory, DirectoryEntry> directories) throws IOException {
            if (ZipFileIndex.get4ByteLittleEndian(this.zipDir, pos) != 33639248) {
                throw new ZipException("cannot read zip file entry");
            }
            int dirStart = pos + 46;
            int fileStart = dirStart;
            int fileEnd = ZipFileIndex.get2ByteLittleEndian(this.zipDir, pos + 28) + fileStart;
            if (this.zipFileIndex.symbolFilePrefixLength != 0 && fileEnd - fileStart >= ZipFileIndex.this.symbolFilePrefixLength) {
                dirStart += this.zipFileIndex.symbolFilePrefixLength;
                fileStart += this.zipFileIndex.symbolFilePrefixLength;
            }
            for (int index = fileStart; index < fileEnd; index++) {
                byte nextByte = this.zipDir[index];
                if (nextByte == 92) {
                    this.zipDir[index] = 47;
                    fileStart = index + 1;
                } else if (nextByte == 47) {
                    fileStart = index + 1;
                }
            }
            RelativePath.RelativeDirectory directory = null;
            if (fileStart == dirStart) {
                directory = ZipFileIndex.this.getRelativeDirectory("");
            } else if (this.lastDir != null && this.lastLen == (fileStart - dirStart) - 1) {
                int index2 = this.lastLen;
                while (true) {
                    index2--;
                    if (this.zipDir[this.lastStart + index2] != this.zipDir[dirStart + index2]) {
                        break;
                    }
                    if (index2 == 0) {
                        directory = this.lastDir;
                        break;
                    }
                }
            }
            if (directory == null) {
                this.lastStart = dirStart;
                this.lastLen = (fileStart - dirStart) - 1;
                directory = ZipFileIndex.this.getRelativeDirectory(new String(this.zipDir, dirStart, this.lastLen, "UTF-8"));
                this.lastDir = directory;
                RelativePath.RelativeDirectory tempDirectory = directory;
                while (directories.get(tempDirectory) == null) {
                    directories.put(tempDirectory, new DirectoryEntry(tempDirectory, this.zipFileIndex));
                    if (tempDirectory.path.indexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR) == tempDirectory.path.length() - 1) {
                        break;
                    }
                    tempDirectory = ZipFileIndex.this.getRelativeDirectory(tempDirectory.dirname().getPath());
                }
            } else if (directories.get(directory) == null) {
                directories.put(directory, new DirectoryEntry(directory, this.zipFileIndex));
            }
            if (fileStart != fileEnd) {
                Entry entry = new Entry(directory, new String(this.zipDir, fileStart, fileEnd - fileStart, "UTF-8"));
                entry.setNativeTime(ZipFileIndex.get4ByteLittleEndian(this.zipDir, pos + 12));
                entry.compressedSize = ZipFileIndex.get4ByteLittleEndian(this.zipDir, pos + 20);
                entry.size = ZipFileIndex.get4ByteLittleEndian(this.zipDir, pos + 24);
                entry.offset = ZipFileIndex.get4ByteLittleEndian(this.zipDir, pos + 42);
                entryList.add(entry);
            }
            return pos + 46 + ZipFileIndex.get2ByteLittleEndian(this.zipDir, pos + 28) + ZipFileIndex.get2ByteLittleEndian(this.zipDir, pos + 30) + ZipFileIndex.get2ByteLittleEndian(this.zipDir, pos + 32);
        }
    }

    public long getZipFileLastModified() throws IOException {
        long j;
        synchronized (this) {
            checkIndex();
            j = this.zipFileLastModified;
        }
        return j;
    }

    static class DirectoryEntry {
        private RelativePath.RelativeDirectory dirName;
        private int numEntries;
        private boolean zipFileEntriesInited;
        private ZipFileIndex zipFileIndex;
        private long writtenOffsetOffset = 0;
        private List<String> zipFileEntriesFiles = List.nil();
        private List<String> zipFileEntriesDirectories = List.nil();
        private List<Entry> zipFileEntries = List.nil();
        private java.util.List<Entry> entries = new ArrayList();
        private boolean filesInited = false;
        private boolean directoriesInited = false;
        private boolean entriesInited = false;

        DirectoryEntry(RelativePath.RelativeDirectory dirName, ZipFileIndex index) {
            this.dirName = dirName;
            this.zipFileIndex = index;
        }

        /* JADX INFO: Access modifiers changed from: private */
        public List<String> getFiles() {
            if (!this.filesInited) {
                initEntries();
                for (Entry e : this.entries) {
                    if (!e.isDir) {
                        this.zipFileEntriesFiles = this.zipFileEntriesFiles.append(e.name);
                    }
                }
                this.filesInited = true;
            }
            return this.zipFileEntriesFiles;
        }

        /* JADX INFO: Access modifiers changed from: private */
        public List<String> getDirectories() {
            if (!this.directoriesInited) {
                initEntries();
                for (Entry e : this.entries) {
                    if (e.isDir) {
                        this.zipFileEntriesDirectories = this.zipFileEntriesDirectories.append(e.name);
                    }
                }
                this.directoriesInited = true;
            }
            return this.zipFileEntriesDirectories;
        }

        private List<Entry> getEntries() {
            if (!this.zipFileEntriesInited) {
                initEntries();
                this.zipFileEntries = List.nil();
                for (Entry zfie : this.entries) {
                    this.zipFileEntries = this.zipFileEntries.append(zfie);
                }
                this.zipFileEntriesInited = true;
            }
            return this.zipFileEntries;
        }

        /* JADX INFO: Access modifiers changed from: private */
        public Entry getEntry(String rootName) {
            initEntries();
            int index = Collections.binarySearch(this.entries, new Entry(this.dirName, rootName));
            if (index < 0) {
                return null;
            }
            return this.entries.get(index);
        }

        private void initEntries() {
            if (!this.entriesInited) {
                if (this.zipFileIndex.readFromIndex) {
                    File indexFile = this.zipFileIndex.getIndexFile();
                    if (indexFile != null) {
                        RandomAccessFile raf = null;
                        try {
                            try {
                                raf = new RandomAccessFile(indexFile, "r");
                                raf.seek(this.writtenOffsetOffset);
                                for (int nFiles = 0; nFiles < this.numEntries; nFiles++) {
                                    int zfieNameBytesLen = raf.readInt();
                                    byte[] zfieNameBytes = new byte[zfieNameBytesLen];
                                    raf.read(zfieNameBytes);
                                    String eName = new String(zfieNameBytes, "UTF-8");
                                    boolean eIsDir = raf.readByte() != 0;
                                    int eOffset = raf.readInt();
                                    int eSize = raf.readInt();
                                    int eCsize = raf.readInt();
                                    long eJavaTimestamp = raf.readLong();
                                    Entry rfie = new Entry(this.dirName, eName);
                                    rfie.isDir = eIsDir;
                                    rfie.offset = eOffset;
                                    rfie.size = eSize;
                                    rfie.compressedSize = eCsize;
                                    rfie.javatime = eJavaTimestamp;
                                    this.entries.add(rfie);
                                }
                                raf.close();
                            } catch (Throwable th) {
                                if (raf != null) {
                                    raf.close();
                                }
                            }
                        } catch (Throwable th2) {
                        }
                    }
                } else {
                    int from = (-Arrays.binarySearch(this.zipFileIndex.entries, new Entry(this.dirName, ZipFileIndex.MIN_CHAR))) - 1;
                    int to = (-Arrays.binarySearch(this.zipFileIndex.entries, new Entry(this.dirName, ZipFileIndex.MAX_CHAR))) - 1;
                    for (int i = from; i < to; i++) {
                        this.entries.add(this.zipFileIndex.entries[i]);
                    }
                }
                this.entriesInited = true;
            }
        }

        java.util.List<Entry> getEntriesAsCollection() {
            initEntries();
            return this.entries;
        }
    }

    /* JADX WARN: Removed duplicated region for block: B:29:0x007d A[Catch: all -> 0x0081, TRY_ENTER, TryCatch #0 {, blocks: (B:10:0x000d, B:29:0x007d, B:30:0x007f), top: B:37:0x000d }] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    private boolean readIndex() {
        /*
            r14 = this;
            boolean r0 = r14.triedToReadIndex
            if (r0 != 0) goto L84
            boolean r0 = r14.usePreindexedCache
            if (r0 != 0) goto La
            goto L84
        La:
            r0 = 0
            monitor-enter(r14)
            r1 = 1
            r14.triedToReadIndex = r1     // Catch: java.lang.Throwable -> L81
            r2 = 0
            java.io.File r3 = r14.getIndexFile()     // Catch: java.lang.Throwable -> L74
            java.io.RandomAccessFile r4 = new java.io.RandomAccessFile     // Catch: java.lang.Throwable -> L74
            java.lang.String r5 = "r"
            r4.<init>(r3, r5)     // Catch: java.lang.Throwable -> L74
            r2 = r4
            long r4 = r2.readLong()     // Catch: java.lang.Throwable -> L74
            java.io.File r6 = r14.zipFile     // Catch: java.lang.Throwable -> L74
            long r6 = r6.lastModified()     // Catch: java.lang.Throwable -> L74
            int r6 = (r6 > r4 ? 1 : (r6 == r4 ? 0 : -1))
            if (r6 == 0) goto L2c
            r0 = 0
            goto L6d
        L2c:
            java.util.LinkedHashMap r6 = new java.util.LinkedHashMap     // Catch: java.lang.Throwable -> L74
            r6.<init>()     // Catch: java.lang.Throwable -> L74
            r14.directories = r6     // Catch: java.lang.Throwable -> L74
            int r6 = r2.readInt()     // Catch: java.lang.Throwable -> L74
            r7 = 0
        L38:
            if (r7 >= r6) goto L6a
            int r8 = r2.readInt()     // Catch: java.lang.Throwable -> L74
            byte[] r9 = new byte[r8]     // Catch: java.lang.Throwable -> L74
            r2.read(r9)     // Catch: java.lang.Throwable -> L74
            java.lang.String r10 = new java.lang.String     // Catch: java.lang.Throwable -> L74
            java.lang.String r11 = "UTF-8"
            r10.<init>(r9, r11)     // Catch: java.lang.Throwable -> L74
            com.sun.tools.javac.file.RelativePath$RelativeDirectory r10 = r14.getRelativeDirectory(r10)     // Catch: java.lang.Throwable -> L74
            com.sun.tools.javac.file.ZipFileIndex$DirectoryEntry r11 = new com.sun.tools.javac.file.ZipFileIndex$DirectoryEntry     // Catch: java.lang.Throwable -> L74
            r11.<init>(r10, r14)     // Catch: java.lang.Throwable -> L74
            int r12 = r2.readInt()     // Catch: java.lang.Throwable -> L74
            com.sun.tools.javac.file.ZipFileIndex.DirectoryEntry.access$1502(r11, r12)     // Catch: java.lang.Throwable -> L74
            long r12 = r2.readLong()     // Catch: java.lang.Throwable -> L74
            com.sun.tools.javac.file.ZipFileIndex.DirectoryEntry.access$1602(r11, r12)     // Catch: java.lang.Throwable -> L74
            java.util.Map<com.sun.tools.javac.file.RelativePath$RelativeDirectory, com.sun.tools.javac.file.ZipFileIndex$DirectoryEntry> r12 = r14.directories     // Catch: java.lang.Throwable -> L74
            r12.put(r10, r11)     // Catch: java.lang.Throwable -> L74
            int r7 = r7 + 1
            goto L38
        L6a:
            r0 = 1
            r14.zipFileLastModified = r4     // Catch: java.lang.Throwable -> L74
        L6d:
            r2.close()     // Catch: java.lang.Throwable -> L72
        L71:
            goto L7b
        L72:
            r3 = move-exception
            goto L71
        L74:
            r3 = move-exception
            if (r2 == 0) goto L7b
            r2.close()     // Catch: java.lang.Throwable -> L72
            goto L71
        L7b:
            if (r0 != r1) goto L7f
            r14.readFromIndex = r1     // Catch: java.lang.Throwable -> L81
        L7f:
            monitor-exit(r14)     // Catch: java.lang.Throwable -> L81
            return r0
        L81:
            r1 = move-exception
            monitor-exit(r14)     // Catch: java.lang.Throwable -> L81
            throw r1
        L84:
            r0 = 0
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.file.ZipFileIndex.readIndex():boolean");
    }

    private boolean writeIndex() {
        java.util.List<DirectoryEntry> directoriesToWrite;
        Map<RelativePath.RelativeDirectory, Long> offsets;
        long j;
        long writtenSoFar;
        Iterator<RelativePath.RelativeDirectory> it;
        String str;
        if (this.readFromIndex || !this.usePreindexedCache || !this.writeIndex) {
            return true;
        }
        File indexFile = getIndexFile();
        if (indexFile == null) {
            return false;
        }
        RandomAccessFile raf = null;
        try {
            try {
                raf = new RandomAccessFile(indexFile, "rw");
                raf.writeLong(this.zipFileLastModified);
                long writtenSoFar2 = 0 + 8;
                directoriesToWrite = new ArrayList<>();
                offsets = new HashMap<>();
                raf.writeInt(this.directories.keySet().size());
                j = 4;
                writtenSoFar = writtenSoFar2 + 4;
                it = this.directories.keySet().iterator();
            } catch (IOException e) {
            }
        } catch (Throwable th) {
        }
        while (true) {
            str = "UTF-8";
            if (!it.hasNext()) {
                break;
            }
            try {
                RelativePath.RelativeDirectory dirName = it.next();
                DirectoryEntry dirEntry = this.directories.get(dirName);
                directoriesToWrite.add(dirEntry);
                byte[] dirNameBytes = dirName.getPath().getBytes("UTF-8");
                int dirNameBytesLen = dirNameBytes.length;
                raf.writeInt(dirNameBytesLen);
                raf.write(dirNameBytes);
                java.util.List<Entry> dirEntries = dirEntry.getEntriesAsCollection();
                raf.writeInt(dirEntries.size());
                long writtenSoFar3 = writtenSoFar + j + ((long) dirNameBytesLen) + 4;
                offsets.put(dirName, new Long(writtenSoFar3));
                dirEntry.writtenOffsetOffset = 0L;
                raf.writeLong(0L);
                writtenSoFar = writtenSoFar3 + 8;
                j = 4;
            } catch (Throwable th2) {
            }
            if (raf != null) {
                raf.close();
            }
            return false;
        }
        Iterator<DirectoryEntry> it2 = directoriesToWrite.iterator();
        while (it2.hasNext()) {
            DirectoryEntry de = it2.next();
            long currFP = raf.getFilePointer();
            long offsetOffset = offsets.get(de.dirName).longValue();
            raf.seek(offsetOffset);
            raf.writeLong(writtenSoFar);
            raf.seek(currFP);
            java.util.List<Entry> list = de.getEntriesAsCollection();
            for (Entry zfie : list) {
                Iterator<DirectoryEntry> it3 = it2;
                byte[] zfieNameBytes = zfie.name.getBytes(str);
                String str2 = str;
                int zfieNameBytesLen = zfieNameBytes.length;
                raf.writeInt(zfieNameBytesLen);
                long writtenSoFar4 = writtenSoFar + 4;
                raf.write(zfieNameBytes);
                File indexFile2 = indexFile;
                long writtenSoFar5 = writtenSoFar4 + ((long) zfieNameBytesLen);
                try {
                    raf.writeByte(zfie.isDir ? 1 : 0);
                    raf.writeInt(zfie.offset);
                    raf.writeInt(zfie.size);
                    raf.writeInt(zfie.compressedSize);
                    long writtenSoFar6 = writtenSoFar5 + 1 + 4 + 4 + 4;
                    raf.writeLong(zfie.getLastModified());
                    writtenSoFar = writtenSoFar6 + 8;
                    it2 = it3;
                    str = str2;
                    indexFile = indexFile2;
                } catch (Throwable th3) {
                }
            }
            it2 = it2;
            indexFile = indexFile;
        }
        raf.close();
        return false;
    }

    public boolean writeZipIndex() {
        boolean zWriteIndex;
        synchronized (this) {
            zWriteIndex = writeIndex();
        }
        return zWriteIndex;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public File getIndexFile() {
        if (this.zipIndexFile == null) {
            if (this.zipFile == null) {
                return null;
            }
            this.zipIndexFile = new File((this.preindexedCacheLocation == null ? "" : this.preindexedCacheLocation) + this.zipFile.getName() + ".index");
        }
        return this.zipIndexFile;
    }

    public File getZipFile() {
        return this.zipFile;
    }

    File getAbsoluteFile() {
        File absFile = this.absFileRef == null ? null : this.absFileRef.get();
        if (absFile == null) {
            File absFile2 = this.zipFile.getAbsoluteFile();
            this.absFileRef = new SoftReference(absFile2);
            return absFile2;
        }
        return absFile;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public RelativePath.RelativeDirectory getRelativeDirectory(String path) {
        RelativePath.RelativeDirectory rd;
        SoftReference<RelativePath.RelativeDirectory> ref = this.relativeDirectoryCache.get(path);
        if (ref != null && (rd = ref.get()) != null) {
            return rd;
        }
        RelativePath.RelativeDirectory rd2 = new RelativePath.RelativeDirectory(path);
        this.relativeDirectoryCache.put(path, new SoftReference<>(rd2));
        return rd2;
    }

    static class Entry implements Comparable<Entry> {
        public static final Entry[] EMPTY_ARRAY = new Entry[0];
        int compressedSize;
        RelativePath.RelativeDirectory dir;
        boolean isDir;
        long javatime;
        String name;
        private int nativetime;
        int offset;
        int size;

        public Entry(RelativePath path) {
            this(path.dirname(), path.basename());
        }

        public Entry(RelativePath.RelativeDirectory directory, String name) {
            this.dir = directory;
            this.name = name;
        }

        public String getName() {
            return new RelativePath.RelativeFile(this.dir, this.name).getPath();
        }

        public String getFileName() {
            return this.name;
        }

        public long getLastModified() {
            if (this.javatime == 0) {
                this.javatime = dosToJavaTime(this.nativetime);
            }
            return this.javatime;
        }

        private static long dosToJavaTime(int dtime) {
            Calendar c = Calendar.getInstance();
            c.set(1, ((dtime >> 25) & 127) + 1980);
            c.set(2, ((dtime >> 21) & 15) - 1);
            c.set(5, (dtime >> 16) & 31);
            c.set(11, (dtime >> 11) & 31);
            c.set(12, (dtime >> 5) & 63);
            c.set(13, (dtime << 1) & 62);
            c.set(14, 0);
            return c.getTimeInMillis();
        }

        void setNativeTime(int natTime) {
            this.nativetime = natTime;
        }

        public boolean isDirectory() {
            return this.isDir;
        }

        @Override // java.lang.Comparable
        public int compareTo(Entry other) {
            int c;
            RelativePath.RelativeDirectory otherD = other.dir;
            if (this.dir != otherD && (c = this.dir.compareTo((RelativePath) otherD)) != 0) {
                return c;
            }
            return this.name.compareTo(other.name);
        }

        public boolean equals(Object o) {
            if (!(o instanceof Entry)) {
                return false;
            }
            Entry other = (Entry) o;
            return this.dir.equals(other.dir) && this.name.equals(other.name);
        }

        public int hashCode() {
            int hash = (7 * 97) + (this.dir != null ? this.dir.hashCode() : 0);
            return (hash * 97) + (this.name != null ? this.name.hashCode() : 0);
        }

        public String toString() {
            StringBuilder sbAppend;
            String str;
            if (this.isDir) {
                sbAppend = new StringBuilder().append("Dir:").append(this.dir);
                str = " : ";
            } else {
                sbAppend = new StringBuilder().append(this.dir);
                str = ":";
            }
            return sbAppend.append(str).append(this.name).toString();
        }
    }

    static final class ZipFormatException extends IOException {
        private static final long serialVersionUID = 8000196834066748623L;

        protected ZipFormatException(String message) {
            super(message);
        }

        protected ZipFormatException(String message, Throwable cause) {
            super(message, cause);
        }
    }
}
