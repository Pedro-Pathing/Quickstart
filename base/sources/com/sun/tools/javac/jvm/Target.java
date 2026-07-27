package com.sun.tools.javac.jvm;

import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Options;
import java.util.HashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes.dex */
public enum Target {
    JDK1_1("1.1", 45, 3),
    JDK1_2("1.2", 46, 0),
    JDK1_3("1.3", 47, 0),
    JDK1_4("1.4", 48, 0),
    JDK1_5("1.5", 49, 0),
    JDK1_6("1.6", 50, 0),
    JDK1_7("1.7", 51, 0),
    JDK1_8("1.8", 52, 0);

    public static final Target DEFAULT;
    public final int majorVersion;
    public final int minorVersion;
    public final String name;
    private static final Context.Key<Target> targetKey = new Context.Key<>();
    private static final Target MIN = values()[0];
    private static final Target MAX = values()[values().length - 1];
    private static final Map<String, Target> tab = new HashMap();

    static {
        for (Target t : values()) {
            tab.put(t.name, t);
        }
        tab.put("5", JDK1_5);
        tab.put("6", JDK1_6);
        tab.put("7", JDK1_7);
        tab.put("8", JDK1_8);
        DEFAULT = JDK1_8;
    }

    public static Target instance(Context context) {
        Target instance = (Target) context.get(targetKey);
        if (instance == null) {
            Options options = Options.instance(context);
            String targetString = options.get(Option.TARGET);
            if (targetString != null) {
                instance = lookup(targetString);
            }
            if (instance == null) {
                instance = DEFAULT;
            }
            context.put(targetKey, instance);
        }
        return instance;
    }

    public static Target MIN() {
        return MIN;
    }

    public static Target MAX() {
        return MAX;
    }

    Target(String name, int majorVersion, int minorVersion) {
        this.name = name;
        this.majorVersion = majorVersion;
        this.minorVersion = minorVersion;
    }

    public static Target lookup(String name) {
        return tab.get(name);
    }

    public boolean requiresIproxy() {
        return compareTo(JDK1_1) <= 0;
    }

    public boolean initializeFieldsBeforeSuper() {
        return compareTo(JDK1_4) >= 0;
    }

    public boolean obeyBinaryCompatibility() {
        return compareTo(JDK1_2) >= 0;
    }

    public boolean arrayBinaryCompatibility() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean interfaceFieldsBinaryCompatibility() {
        return compareTo(JDK1_2) > 0;
    }

    public boolean interfaceObjectOverridesBinaryCompatibility() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean usePrivateSyntheticFields() {
        return compareTo(JDK1_5) < 0;
    }

    public boolean useInnerCacheClass() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean generateCLDCStackmap() {
        return false;
    }

    public boolean generateStackMapTable() {
        return compareTo(JDK1_6) >= 0;
    }

    public boolean isPackageInfoSynthetic() {
        return compareTo(JDK1_6) >= 0;
    }

    public boolean generateEmptyAfterBig() {
        return false;
    }

    public boolean useStringBuilder() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean useSyntheticFlag() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean useEnumFlag() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean useAnnotationFlag() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean useVarargsFlag() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean useBridgeFlag() {
        return compareTo(JDK1_5) >= 0;
    }

    public char syntheticNameChar() {
        return '$';
    }

    public boolean hasClassLiterals() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean hasInvokedynamic() {
        return compareTo(JDK1_7) >= 0;
    }

    public boolean hasMethodHandles() {
        return hasInvokedynamic();
    }

    public boolean classLiteralsNoInit() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean hasInitCause() {
        return compareTo(JDK1_4) >= 0;
    }

    public boolean boxWithConstructors() {
        return compareTo(JDK1_5) < 0;
    }

    public boolean hasIterable() {
        return compareTo(JDK1_5) >= 0;
    }

    public boolean hasEnclosingMethodAttribute() {
        return compareTo(JDK1_5) >= 0;
    }
}
