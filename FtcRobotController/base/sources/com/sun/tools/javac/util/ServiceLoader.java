package com.sun.tools.javac.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.net.URL;
import java.util.Enumeration;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.ServiceConfigurationError;

/* JADX INFO: loaded from: classes.dex */
public final class ServiceLoader<S> implements Iterable<S> {
    private static final String PREFIX = "META-INF/services/";
    private ClassLoader loader;
    private ServiceLoader<S>.LazyIterator lookupIterator;
    private LinkedHashMap<String, S> providers = new LinkedHashMap<>();
    private Class<S> service;

    public void reload() {
        this.providers.clear();
        this.lookupIterator = new LazyIterator(this.service, this.loader);
    }

    private ServiceLoader(Class<S> svc, ClassLoader cl) {
        this.service = (Class) Objects.requireNonNull(svc, "Service interface cannot be null");
        this.loader = cl == null ? ClassLoader.getSystemClassLoader() : cl;
        reload();
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static void fail(Class<?> service, String msg, Throwable cause) throws ServiceConfigurationError {
        throw new ServiceConfigurationError(service.getName() + ": " + msg, cause);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static void fail(Class<?> service, String msg) throws ServiceConfigurationError {
        throw new ServiceConfigurationError(service.getName() + ": " + msg);
    }

    private static void fail(Class<?> service, URL u, int line, String msg) throws ServiceConfigurationError {
        fail(service, u + ":" + line + ": " + msg);
    }

    private int parseLine(Class<?> service, URL u, BufferedReader r, int lc, java.util.List<String> names) throws ServiceConfigurationError, IOException {
        String ln = r.readLine();
        if (ln == null) {
            return -1;
        }
        int ci = ln.indexOf(35);
        if (ci >= 0) {
            ln = ln.substring(0, ci);
        }
        String ln2 = ln.trim();
        int n = ln2.length();
        if (n != 0) {
            if (ln2.indexOf(32) >= 0 || ln2.indexOf(9) >= 0) {
                fail(service, u, lc, "Illegal configuration-file syntax");
            }
            int cp = ln2.codePointAt(0);
            if (!Character.isJavaIdentifierStart(cp)) {
                fail(service, u, lc, "Illegal provider-class name: " + ln2);
            }
            int i = Character.charCount(cp);
            while (i < n) {
                int cp2 = ln2.codePointAt(i);
                if (!Character.isJavaIdentifierPart(cp2) && cp2 != 46) {
                    fail(service, u, lc, "Illegal provider-class name: " + ln2);
                }
                i += Character.charCount(cp2);
            }
            if (!this.providers.containsKey(ln2) && !names.contains(ln2)) {
                names.add(ln2);
            }
        }
        return lc + 1;
    }

    /* JADX INFO: Access modifiers changed from: private */
    /* JADX WARN: Removed duplicated region for block: B:36:0x0068 A[Catch: IOException -> 0x0064, TRY_LEAVE, TryCatch #3 {IOException -> 0x0064, blocks: (B:32:0x0060, B:36:0x0068), top: B:42:0x0060 }] */
    /* JADX WARN: Removed duplicated region for block: B:42:0x0060 A[EXC_TOP_SPLITTER, SYNTHETIC] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public java.util.Iterator<java.lang.String> parse(java.lang.Class<?> r11, java.net.URL r12) throws java.lang.Throwable {
        /*
            r10 = this;
            java.lang.String r0 = "Error closing configuration file"
            r1 = 0
            r2 = 0
            java.util.ArrayList r8 = new java.util.ArrayList
            r8.<init>()
            java.net.URLConnection r3 = r12.openConnection()     // Catch: java.lang.Throwable -> L41 java.io.IOException -> L45
            r9 = r3
            r3 = 0
            r9.setUseCaches(r3)     // Catch: java.lang.Throwable -> L41 java.io.IOException -> L45
            java.io.InputStream r3 = r9.getInputStream()     // Catch: java.lang.Throwable -> L41 java.io.IOException -> L45
            r1 = r3
            java.io.BufferedReader r6 = new java.io.BufferedReader     // Catch: java.lang.Throwable -> L41 java.io.IOException -> L45
            java.io.InputStreamReader r3 = new java.io.InputStreamReader     // Catch: java.lang.Throwable -> L41 java.io.IOException -> L45
            java.lang.String r4 = "utf-8"
            r3.<init>(r1, r4)     // Catch: java.lang.Throwable -> L41 java.io.IOException -> L45
            r6.<init>(r3)     // Catch: java.lang.Throwable -> L41 java.io.IOException -> L45
            r2 = 1
        L25:
            r3 = r10
            r4 = r11
            r5 = r12
            r7 = r2
            int r3 = r3.parseLine(r4, r5, r6, r7, r8)     // Catch: java.io.IOException -> L3f java.lang.Throwable -> L5d
            r2 = r3
            if (r3 < 0) goto L31
            goto L25
        L31:
            r6.close()     // Catch: java.io.IOException -> L3a
            if (r1 == 0) goto L39
            r1.close()     // Catch: java.io.IOException -> L3a
        L39:
            goto L58
        L3a:
            r2 = move-exception
            fail(r11, r0, r2)
            goto L58
        L3f:
            r2 = move-exception
            goto L48
        L41:
            r3 = move-exception
            r6 = r2
            r2 = r3
            goto L5e
        L45:
            r3 = move-exception
            r6 = r2
            r2 = r3
        L48:
            java.lang.String r3 = "Error reading configuration file"
            fail(r11, r3, r2)     // Catch: java.lang.Throwable -> L5d
            if (r6 == 0) goto L52
            r6.close()     // Catch: java.io.IOException -> L3a
        L52:
            if (r1 == 0) goto L39
            r1.close()     // Catch: java.io.IOException -> L3a
            goto L39
        L58:
            java.util.Iterator r0 = r8.iterator()
            return r0
        L5d:
            r2 = move-exception
        L5e:
            if (r6 == 0) goto L66
            r6.close()     // Catch: java.io.IOException -> L64
            goto L66
        L64:
            r3 = move-exception
            goto L6c
        L66:
            if (r1 == 0) goto L70
            r1.close()     // Catch: java.io.IOException -> L64
            goto L70
        L6c:
            fail(r11, r0, r3)
            goto L71
        L70:
        L71:
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.util.ServiceLoader.parse(java.lang.Class, java.net.URL):java.util.Iterator");
    }

    private class LazyIterator implements Iterator<S> {
        Enumeration<URL> configs;
        ClassLoader loader;
        String nextName;
        Iterator<String> pending;
        Class<S> service;

        private LazyIterator(Class<S> service, ClassLoader loader) {
            this.configs = null;
            this.pending = null;
            this.nextName = null;
            this.service = service;
            this.loader = loader;
        }

        /* JADX WARN: Removed duplicated region for block: B:16:0x0043  */
        /* JADX WARN: Removed duplicated region for block: B:23:0x005f A[RETURN] */
        /* JADX WARN: Removed duplicated region for block: B:25:0x0061  */
        /* JADX WARN: Unsupported multi-entry loop pattern (BACK_EDGE: B:13:0x0038 -> B:14:0x003f). Please report as a decompilation issue!!! */
        /* JADX WARN: Unsupported multi-entry loop pattern (BACK_EDGE: B:26:0x003f -> B:14:0x003f). Please report as a decompilation issue!!! */
        @Override // java.util.Iterator
        /*
            Code decompiled incorrectly, please refer to instructions dump.
            To view partially-correct code enable 'Show inconsistent code' option in preferences
        */
        public boolean hasNext() {
            /*
                r4 = this;
                java.lang.String r0 = r4.nextName
                r1 = 1
                if (r0 == 0) goto L6
                return r1
            L6:
                java.util.Enumeration<java.net.URL> r0 = r4.configs
                if (r0 != 0) goto L3f
                java.lang.StringBuilder r0 = new java.lang.StringBuilder     // Catch: java.io.IOException -> L37
                r0.<init>()     // Catch: java.io.IOException -> L37
                java.lang.String r2 = "META-INF/services/"
                java.lang.StringBuilder r0 = r0.append(r2)     // Catch: java.io.IOException -> L37
                java.lang.Class<S> r2 = r4.service     // Catch: java.io.IOException -> L37
                java.lang.String r2 = r2.getName()     // Catch: java.io.IOException -> L37
                java.lang.StringBuilder r0 = r0.append(r2)     // Catch: java.io.IOException -> L37
                java.lang.String r0 = r0.toString()     // Catch: java.io.IOException -> L37
                java.lang.ClassLoader r2 = r4.loader     // Catch: java.io.IOException -> L37
                if (r2 != 0) goto L2e
                java.util.Enumeration r2 = java.lang.ClassLoader.getSystemResources(r0)     // Catch: java.io.IOException -> L37
                r4.configs = r2     // Catch: java.io.IOException -> L37
                goto L36
            L2e:
                java.lang.ClassLoader r2 = r4.loader     // Catch: java.io.IOException -> L37
                java.util.Enumeration r2 = r2.getResources(r0)     // Catch: java.io.IOException -> L37
                r4.configs = r2     // Catch: java.io.IOException -> L37
            L36:
                goto L3f
            L37:
                r0 = move-exception
                java.lang.Class<S> r2 = r4.service
                java.lang.String r3 = "Error locating configuration files"
                com.sun.tools.javac.util.ServiceLoader.access$100(r2, r3, r0)
            L3f:
                java.util.Iterator<java.lang.String> r0 = r4.pending
                if (r0 == 0) goto L57
                java.util.Iterator<java.lang.String> r0 = r4.pending
                boolean r0 = r0.hasNext()
                if (r0 != 0) goto L4c
                goto L57
            L4c:
                java.util.Iterator<java.lang.String> r0 = r4.pending
                java.lang.Object r0 = r0.next()
                java.lang.String r0 = (java.lang.String) r0
                r4.nextName = r0
                return r1
            L57:
                java.util.Enumeration<java.net.URL> r0 = r4.configs
                boolean r0 = r0.hasMoreElements()
                if (r0 != 0) goto L61
                r0 = 0
                return r0
            L61:
                com.sun.tools.javac.util.ServiceLoader r0 = com.sun.tools.javac.util.ServiceLoader.this
                java.lang.Class<S> r2 = r4.service
                java.util.Enumeration<java.net.URL> r3 = r4.configs
                java.lang.Object r3 = r3.nextElement()
                java.net.URL r3 = (java.net.URL) r3
                java.util.Iterator r0 = com.sun.tools.javac.util.ServiceLoader.access$200(r0, r2, r3)
                r4.pending = r0
                goto L3f
            */
            throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.util.ServiceLoader.LazyIterator.hasNext():boolean");
        }

        @Override // java.util.Iterator
        public S next() throws ClassNotFoundException {
            if (!hasNext()) {
                throw new NoSuchElementException();
            }
            String cn = this.nextName;
            this.nextName = null;
            Class<?> c = null;
            try {
                c = Class.forName(cn, false, this.loader);
            } catch (ClassNotFoundException e) {
                ServiceLoader.fail(this.service, "Provider " + cn + " not found");
            }
            if (!this.service.isAssignableFrom(c)) {
                ServiceLoader.fail(this.service, "Provider " + cn + " not a subtype");
            }
            try {
                S p = this.service.cast(c.newInstance());
                ServiceLoader.this.providers.put(cn, p);
                return p;
            } catch (Throwable x) {
                ServiceLoader.fail(this.service, "Provider " + cn + " could not be instantiated: " + x, x);
                throw new Error();
            }
        }

        @Override // java.util.Iterator
        public void remove() {
            throw new UnsupportedOperationException();
        }
    }

    @Override // java.lang.Iterable
    public Iterator<S> iterator() {
        return new Iterator<S>() { // from class: com.sun.tools.javac.util.ServiceLoader.1
            Iterator<Map.Entry<String, S>> knownProviders;

            {
                this.knownProviders = ServiceLoader.this.providers.entrySet().iterator();
            }

            @Override // java.util.Iterator
            public boolean hasNext() {
                if (!this.knownProviders.hasNext()) {
                    return ServiceLoader.this.lookupIterator.hasNext();
                }
                return true;
            }

            @Override // java.util.Iterator
            public S next() {
                if (!this.knownProviders.hasNext()) {
                    return (S) ServiceLoader.this.lookupIterator.next();
                }
                return this.knownProviders.next().getValue();
            }

            @Override // java.util.Iterator
            public void remove() {
                throw new UnsupportedOperationException();
            }
        };
    }

    public static <S> ServiceLoader<S> load(Class<S> service, ClassLoader loader) {
        return new ServiceLoader<>(service, loader);
    }

    public static <S> ServiceLoader<S> load(Class<S> service) {
        ClassLoader cl = Thread.currentThread().getContextClassLoader();
        return load(service, cl);
    }

    public static <S> ServiceLoader<S> loadInstalled(Class<S> service) {
        ClassLoader prev = null;
        for (ClassLoader cl = ClassLoader.getSystemClassLoader(); cl != null; cl = cl.getParent()) {
            prev = cl;
        }
        return load(service, prev);
    }

    public String toString() {
        return "java.util.ServiceLoader[" + this.service.getName() + "]";
    }
}
