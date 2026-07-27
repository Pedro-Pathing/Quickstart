package com.sun.tools.javac.util;

import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.file.FSInfo;
import com.sun.tools.javac.file.Locations;
import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.main.OptionHelper;
import com.sun.tools.javac.util.JCDiagnostic;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.lang.ref.SoftReference;
import java.lang.reflect.Constructor;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.CharBuffer;
import java.nio.charset.Charset;
import java.nio.charset.CharsetDecoder;
import java.nio.charset.CoderResult;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.IllegalCharsetNameException;
import java.nio.charset.UnsupportedCharsetException;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public abstract class BaseFileManager {
    private static final Set<Option> javacFileManagerOptions = Option.getJavacFileManagerOptions();
    protected Charset charset;
    protected String classLoaderClass;
    private String defaultEncodingName;
    public Log log;
    protected Options options;
    protected final Map<JavaFileObject, ContentCacheEntry> contentCache = new HashMap();
    private final ByteBufferCache byteBufferCache = new ByteBufferCache();
    protected Locations locations = createLocations();

    public abstract boolean isDefaultBootClassPath();

    protected BaseFileManager(Charset charset) {
        this.charset = charset;
    }

    public void setContext(Context context) {
        this.log = Log.instance(context);
        this.options = Options.instance(context);
        this.classLoaderClass = this.options.get("procloader");
        this.locations.update(this.log, this.options, Lint.instance(context), FSInfo.instance(context));
    }

    protected Locations createLocations() {
        return new Locations();
    }

    protected Source getSource() {
        String sourceName = this.options.get(Option.SOURCE);
        Source source = null;
        if (sourceName != null) {
            source = Source.lookup(sourceName);
        }
        return source != null ? source : Source.DEFAULT;
    }

    protected ClassLoader getClassLoader(URL[] urls) {
        ClassLoader thisClassLoader = getClass().getClassLoader();
        if (this.classLoaderClass != null) {
            try {
                Class<?>[] constrArgTypes = {URL[].class, ClassLoader.class};
                Constructor<? extends ClassLoader> constr = Class.forName(this.classLoaderClass).asSubclass(ClassLoader.class).getConstructor(constrArgTypes);
                return (ClassLoader) constr.newInstance(urls, thisClassLoader);
            } catch (Throwable th) {
            }
        }
        return new URLClassLoader(urls, thisClassLoader);
    }

    public boolean handleOption(String current, Iterator<String> remaining) {
        OptionHelper helper = new OptionHelper.GrumpyHelper(this.log) { // from class: com.sun.tools.javac.util.BaseFileManager.1
            @Override // com.sun.tools.javac.main.OptionHelper.GrumpyHelper, com.sun.tools.javac.main.OptionHelper
            public String get(Option option) {
                return BaseFileManager.this.options.get(option.getText());
            }

            @Override // com.sun.tools.javac.main.OptionHelper.GrumpyHelper, com.sun.tools.javac.main.OptionHelper
            public void put(String name, String value) {
                BaseFileManager.this.options.put(name, value);
            }

            @Override // com.sun.tools.javac.main.OptionHelper.GrumpyHelper, com.sun.tools.javac.main.OptionHelper
            public void remove(String name) {
                BaseFileManager.this.options.remove(name);
            }
        };
        for (Option o : javacFileManagerOptions) {
            if (o.matches(current)) {
                if (o.hasArg()) {
                    if (remaining.hasNext() && !o.process(helper, current, remaining.next())) {
                        return true;
                    }
                } else if (!o.process(helper, current)) {
                    return true;
                }
                throw new IllegalArgumentException(current);
            }
        }
        return false;
    }

    public int isSupportedOption(String str) {
        for (Option option : javacFileManagerOptions) {
            if (option.matches(str)) {
                return option.hasArg() ? 1 : 0;
            }
        }
        return -1;
    }

    private String getDefaultEncodingName() {
        if (this.defaultEncodingName == null) {
            this.defaultEncodingName = new OutputStreamWriter(new ByteArrayOutputStream()).getEncoding();
        }
        return this.defaultEncodingName;
    }

    public String getEncodingName() {
        String encName = this.options.get(Option.ENCODING);
        if (encName == null) {
            return getDefaultEncodingName();
        }
        return encName;
    }

    public CharBuffer decode(java.nio.ByteBuffer inbuf, boolean ignoreEncodingErrors) {
        String encodingName = getEncodingName();
        try {
            CharsetDecoder decoder = getDecoder(encodingName, ignoreEncodingErrors);
            float factor = (decoder.maxCharsPerByte() * 0.2f) + (decoder.averageCharsPerByte() * 0.8f);
            CharBuffer dest = CharBuffer.allocate(((int) (inbuf.remaining() * factor)) + 10);
            while (true) {
                CoderResult result = decoder.decode(inbuf, dest, true);
                dest.flip();
                if (result.isUnderflow()) {
                    if (dest.limit() == dest.capacity()) {
                        CharBuffer dest2 = CharBuffer.allocate(dest.capacity() + 1).put(dest);
                        dest2.flip();
                        return dest2;
                    }
                    return dest;
                }
                if (result.isOverflow()) {
                    int newCapacity = dest.capacity() + 10 + ((int) (inbuf.remaining() * decoder.maxCharsPerByte()));
                    dest = CharBuffer.allocate(newCapacity).put(dest);
                } else if (result.isMalformed() || result.isUnmappable()) {
                    if (!getSource().allowEncodingErrors()) {
                        this.log.error(new JCDiagnostic.SimpleDiagnosticPosition(dest.limit()), "illegal.char.for.encoding", this.charset == null ? encodingName : this.charset.name());
                    } else {
                        this.log.warning(new JCDiagnostic.SimpleDiagnosticPosition(dest.limit()), "illegal.char.for.encoding", this.charset == null ? encodingName : this.charset.name());
                    }
                    inbuf.position(inbuf.position() + result.length());
                    dest.position(dest.limit());
                    dest.limit(dest.capacity());
                    dest.put((char) 65533);
                } else {
                    throw new AssertionError(result);
                }
            }
        } catch (IllegalCharsetNameException e) {
            this.log.error("unsupported.encoding", encodingName);
            return (CharBuffer) CharBuffer.allocate(1).flip();
        } catch (UnsupportedCharsetException e2) {
            this.log.error("unsupported.encoding", encodingName);
            return (CharBuffer) CharBuffer.allocate(1).flip();
        }
    }

    public CharsetDecoder getDecoder(String encodingName, boolean ignoreEncodingErrors) {
        CodingErrorAction action;
        Charset cs = this.charset == null ? Charset.forName(encodingName) : this.charset;
        CharsetDecoder decoder = cs.newDecoder();
        if (ignoreEncodingErrors) {
            action = CodingErrorAction.REPLACE;
        } else {
            action = CodingErrorAction.REPORT;
        }
        return decoder.onMalformedInput(action).onUnmappableCharacter(action);
    }

    public java.nio.ByteBuffer makeByteBuffer(InputStream in) throws IOException {
        int limit = in.available();
        if (limit < 1024) {
            limit = 1024;
        }
        java.nio.ByteBuffer result = this.byteBufferCache.get(limit);
        int position = 0;
        while (in.available() != 0) {
            if (position >= limit) {
                int i = limit << 1;
                limit = i;
                result = java.nio.ByteBuffer.allocate(i).put((java.nio.ByteBuffer) result.flip());
            }
            int count = in.read(result.array(), position, limit - position);
            if (count < 0) {
                break;
            }
            int i2 = position + count;
            position = i2;
            result.position(i2);
        }
        return (java.nio.ByteBuffer) result.flip();
    }

    public void recycleByteBuffer(java.nio.ByteBuffer bb) {
        this.byteBufferCache.put(bb);
    }

    private static class ByteBufferCache {
        private java.nio.ByteBuffer cached;

        private ByteBufferCache() {
        }

        java.nio.ByteBuffer get(int capacity) {
            java.nio.ByteBuffer result;
            if (capacity < 20480) {
                capacity = 20480;
            }
            if (this.cached != null && this.cached.capacity() >= capacity) {
                result = (java.nio.ByteBuffer) this.cached.clear();
            } else {
                result = java.nio.ByteBuffer.allocate((capacity + capacity) >> 1);
            }
            this.cached = null;
            return result;
        }

        void put(java.nio.ByteBuffer x) {
            this.cached = x;
        }
    }

    public CharBuffer getCachedContent(JavaFileObject file) {
        ContentCacheEntry e = this.contentCache.get(file);
        if (e == null) {
            return null;
        }
        if (!e.isValid(file)) {
            this.contentCache.remove(file);
            return null;
        }
        return e.getValue();
    }

    public void cache(JavaFileObject file, CharBuffer cb) {
        this.contentCache.put(file, new ContentCacheEntry(file, cb));
    }

    public void flushCache(JavaFileObject file) {
        this.contentCache.remove(file);
    }

    protected static class ContentCacheEntry {
        final SoftReference<CharBuffer> ref;
        final long timestamp;

        ContentCacheEntry(JavaFileObject file, CharBuffer cb) {
            this.timestamp = file.getLastModified();
            this.ref = new SoftReference<>(cb);
        }

        boolean isValid(JavaFileObject file) {
            return this.timestamp == file.getLastModified();
        }

        CharBuffer getValue() {
            return this.ref.get();
        }
    }

    public static JavaFileObject.Kind getKind(String name) {
        if (name.endsWith(JavaFileObject.Kind.CLASS.extension)) {
            return JavaFileObject.Kind.CLASS;
        }
        if (name.endsWith(JavaFileObject.Kind.SOURCE.extension)) {
            return JavaFileObject.Kind.SOURCE;
        }
        if (name.endsWith(JavaFileObject.Kind.HTML.extension)) {
            return JavaFileObject.Kind.HTML;
        }
        return JavaFileObject.Kind.OTHER;
    }

    protected static <T> T nullCheck(T o) {
        o.getClass();
        return o;
    }

    protected static <T> Collection<T> nullCheck(Collection<T> it) {
        for (T t : it) {
            t.getClass();
        }
        return it;
    }
}
