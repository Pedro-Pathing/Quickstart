package com.sun.tools.javac.main;

import com.sun.tools.javac.util.Log;
import java.io.File;

/* JADX INFO: loaded from: classes.dex */
public abstract class OptionHelper {
    abstract void addClassName(String str);

    abstract void addFile(File file);

    abstract void error(String str, Object... objArr);

    public abstract String get(Option option);

    public abstract Log getLog();

    public abstract String getOwnName();

    public abstract void put(String str, String str2);

    public abstract void remove(String str);

    public static class GrumpyHelper extends OptionHelper {
        private final Log log;

        public GrumpyHelper(Log log) {
            this.log = log;
        }

        @Override // com.sun.tools.javac.main.OptionHelper
        public Log getLog() {
            return this.log;
        }

        @Override // com.sun.tools.javac.main.OptionHelper
        public String getOwnName() {
            throw new IllegalStateException();
        }

        @Override // com.sun.tools.javac.main.OptionHelper
        public String get(Option option) {
            throw new IllegalArgumentException();
        }

        @Override // com.sun.tools.javac.main.OptionHelper
        public void put(String name, String value) {
            throw new IllegalArgumentException();
        }

        @Override // com.sun.tools.javac.main.OptionHelper
        public void remove(String name) {
            throw new IllegalArgumentException();
        }

        @Override // com.sun.tools.javac.main.OptionHelper
        void error(String key, Object... args) {
            throw new IllegalArgumentException(this.log.localize(Log.PrefixKind.JAVAC, key, args));
        }

        @Override // com.sun.tools.javac.main.OptionHelper
        public void addFile(File f) {
            throw new IllegalArgumentException(f.getPath());
        }

        @Override // com.sun.tools.javac.main.OptionHelper
        public void addClassName(String s) {
            throw new IllegalArgumentException(s);
        }
    }
}
