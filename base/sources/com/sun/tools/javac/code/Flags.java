package com.sun.tools.javac.code;

import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.StringUtils;
import java.util.Collections;
import java.util.EnumSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import javax.lang.model.element.Modifier;

/* JADX INFO: loaded from: classes.dex */
public class Flags {
    public static final int ABSTRACT = 1024;
    public static final int ACC_BRIDGE = 64;
    public static final int ACC_SUPER = 32;
    public static final int ACC_VARARGS = 128;
    public static final int ACYCLIC = 1073741824;
    public static final long ACYCLIC_ANN = 34359738368L;
    public static final int ANNOTATION = 8192;
    public static final int ANONCONSTR = 536870912;
    public static final long AUXILIARY = 17592186044416L;
    public static final int AccessFlags = 7;
    public static final long AnnotationTypeElementMask = 1025;
    public static final long BAD_OVERRIDE = 35184372088832L;
    public static final int BLOCK = 1048576;
    public static final long BRIDGE = 2147483648L;
    public static final long CLASH = 4398046511104L;
    public static final int CLASS_SEEN = 33554432;
    public static final int COMPOUND = 16777216;
    public static final int ClassFlags = 32273;
    public static final int ConstructorFlags = 7;
    public static final long DEFAULT = 8796093022208L;
    public static final int DEPRECATED = 131072;
    public static final long EFFECTIVELY_FINAL = 2199023255552L;
    public static final int ENUM = 16384;
    public static final int EXISTS = 8388608;
    public static final long ExtendedStandardFlags = 8796093026303L;
    public static final int FINAL = 16;
    public static final long GENERATEDCONSTR = 68719476736L;
    public static final int HASINIT = 262144;
    public static final long HYPOTHETICAL = 137438953472L;
    public static final int INTERFACE = 512;
    public static final int IPROXY = 2097152;
    public static final int InterfaceMethodFlags = 1025;
    public static final long InterfaceMethodMask = 8796093025289L;
    public static final int InterfaceVarFlags = 25;
    public static final long LAMBDA_METHOD = 562949953421312L;
    public static final int LOCKED = 134217728;
    public static final int LocalClassFlags = 23568;
    public static final long LocalVarFlags = 8589934608L;
    public static final int MANDATED = 32768;
    public static final int MemberClassFlags = 24087;
    public static final int MethodFlags = 3391;
    public static final long ModifierFlags = 8796093025791L;
    public static final int NATIVE = 256;
    public static final int NOOUTERTHIS = 4194304;
    public static final long NOT_IN_PROFILE = 35184372088832L;
    public static final long OVERRIDE_BRIDGE = 1099511627776L;
    public static final long PARAMETER = 8589934592L;
    public static final long POTENTIALLY_AMBIGUOUS = 281474976710656L;
    public static final int PRIVATE = 2;
    public static final long PROPRIETARY = 274877906944L;
    public static final int PROTECTED = 4;
    public static final int PUBLIC = 1;
    public static final long ReceiverParamFlags = 8589934592L;
    public static final long SIGNATURE_POLYMORPHIC = 70368744177664L;
    public static final int SOURCE_SEEN = 67108864;
    public static final int STATIC = 8;
    public static final int STRICTFP = 2048;
    public static final int SYNCHRONIZED = 32;
    public static final int SYNTHETIC = 4096;
    public static final int StandardFlags = 4095;
    public static final long THROWS = 140737488355328L;
    public static final int TRANSIENT = 128;
    public static final long TYPE_TRANSLATED = 1125899906842624L;
    public static final int UNATTRIBUTED = 268435456;
    public static final long UNION = 549755813888L;
    public static final long VARARGS = 17179869184L;
    public static final int VOLATILE = 64;
    public static final int VarFlags = 16607;
    private static final Map<Long, Set<Modifier>> modifierSets = new ConcurrentHashMap(64);

    private Flags() {
    }

    public static String toString(long flags) {
        StringBuilder buf = new StringBuilder();
        String sep = "";
        for (Flag flag : asFlagSet(flags)) {
            buf.append(sep);
            buf.append(flag);
            sep = " ";
        }
        return buf.toString();
    }

    public static EnumSet<Flag> asFlagSet(long flags) {
        EnumSet<Flag> flagSet = EnumSet.noneOf(Flag.class);
        for (Flag flag : Flag.values()) {
            if ((flag.value & flags) != 0) {
                flagSet.add(flag);
                flags &= ~flag.value;
            }
        }
        Assert.check(flags == 0, "Flags parameter contains unknown flags " + flags);
        return flagSet;
    }

    public static Set<Modifier> asModifierSet(long flags) {
        Set<Modifier> modifiers = modifierSets.get(Long.valueOf(flags));
        if (modifiers == null) {
            Set<Modifier> modifiers2 = EnumSet.noneOf(Modifier.class);
            if (0 != (1 & flags)) {
                modifiers2.add(Modifier.PUBLIC);
            }
            if (0 != (4 & flags)) {
                modifiers2.add(Modifier.PROTECTED);
            }
            if (0 != (2 & flags)) {
                modifiers2.add(Modifier.PRIVATE);
            }
            if (0 != (1024 & flags)) {
                modifiers2.add(Modifier.ABSTRACT);
            }
            if (0 != (8 & flags)) {
                modifiers2.add(Modifier.STATIC);
            }
            if (0 != (16 & flags)) {
                modifiers2.add(Modifier.FINAL);
            }
            if (0 != (128 & flags)) {
                modifiers2.add(Modifier.TRANSIENT);
            }
            if (0 != (64 & flags)) {
                modifiers2.add(Modifier.VOLATILE);
            }
            if (0 != (32 & flags)) {
                modifiers2.add(Modifier.SYNCHRONIZED);
            }
            if (0 != (256 & flags)) {
                modifiers2.add(Modifier.NATIVE);
            }
            if (0 != (2048 & flags)) {
                modifiers2.add(Modifier.STRICTFP);
            }
            if (0 != (DEFAULT & flags)) {
                modifiers2.add(Modifier.DEFAULT);
            }
            Set<Modifier> modifiers3 = Collections.unmodifiableSet(modifiers2);
            modifierSets.put(Long.valueOf(flags), modifiers3);
            return modifiers3;
        }
        return modifiers;
    }

    public static boolean isStatic(Symbol symbol) {
        return (symbol.flags() & 8) != 0;
    }

    public static boolean isEnum(Symbol symbol) {
        return (symbol.flags() & 16384) != 0;
    }

    public static boolean isConstant(Symbol.VarSymbol symbol) {
        return symbol.getConstValue() != null;
    }

    public enum Flag {
        PUBLIC(1),
        PRIVATE(2),
        PROTECTED(4),
        STATIC(8),
        FINAL(16),
        SYNCHRONIZED(32),
        VOLATILE(64),
        TRANSIENT(128),
        NATIVE(256),
        INTERFACE(512),
        ABSTRACT(1024),
        DEFAULT(Flags.DEFAULT),
        STRICTFP(2048),
        BRIDGE(Flags.BRIDGE),
        SYNTHETIC(4096),
        ANNOTATION(8192),
        DEPRECATED(131072),
        HASINIT(262144),
        BLOCK(1048576),
        ENUM(16384),
        MANDATED(32768),
        IPROXY(2097152),
        NOOUTERTHIS(4194304),
        EXISTS(8388608),
        COMPOUND(16777216),
        CLASS_SEEN(33554432),
        SOURCE_SEEN(67108864),
        LOCKED(134217728),
        UNATTRIBUTED(268435456),
        ANONCONSTR(536870912),
        ACYCLIC(1073741824),
        PARAMETER(8589934592L),
        VARARGS(Flags.VARARGS),
        ACYCLIC_ANN(Flags.ACYCLIC_ANN),
        GENERATEDCONSTR(Flags.GENERATEDCONSTR),
        HYPOTHETICAL(Flags.HYPOTHETICAL),
        PROPRIETARY(Flags.PROPRIETARY),
        UNION(Flags.UNION),
        OVERRIDE_BRIDGE(Flags.OVERRIDE_BRIDGE),
        EFFECTIVELY_FINAL(Flags.EFFECTIVELY_FINAL),
        CLASH(Flags.CLASH),
        AUXILIARY(Flags.AUXILIARY),
        NOT_IN_PROFILE(35184372088832L),
        BAD_OVERRIDE(35184372088832L),
        SIGNATURE_POLYMORPHIC(Flags.SIGNATURE_POLYMORPHIC),
        THROWS(Flags.THROWS),
        LAMBDA_METHOD(Flags.LAMBDA_METHOD),
        TYPE_TRANSLATED(Flags.TYPE_TRANSLATED);

        final String lowercaseName = StringUtils.toLowerCase(name());
        final long value;

        Flag(long flag) {
            this.value = flag;
        }

        @Override // java.lang.Enum
        public String toString() {
            return this.lowercaseName;
        }
    }
}
