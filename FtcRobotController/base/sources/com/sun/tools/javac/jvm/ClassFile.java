package com.sun.tools.javac.jvm;

import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.Types;
import com.sun.tools.javac.util.Name;

/* JADX INFO: loaded from: classes.dex */
public class ClassFile {
    public static final int CONSTANT_Class = 7;
    public static final int CONSTANT_Double = 6;
    public static final int CONSTANT_Fieldref = 9;
    public static final int CONSTANT_Float = 4;
    public static final int CONSTANT_Integer = 3;
    public static final int CONSTANT_InterfaceMethodref = 11;
    public static final int CONSTANT_InvokeDynamic = 18;
    public static final int CONSTANT_Long = 5;
    public static final int CONSTANT_MethodHandle = 15;
    public static final int CONSTANT_MethodType = 16;
    public static final int CONSTANT_Methodref = 10;
    public static final int CONSTANT_NameandType = 12;
    public static final int CONSTANT_String = 8;
    public static final int CONSTANT_Unicode = 2;
    public static final int CONSTANT_Utf8 = 1;
    public static final int JAVA_MAGIC = -889275714;
    public static final int MAX_CODE = 65535;
    public static final int MAX_DIMENSIONS = 255;
    public static final int MAX_LOCALS = 65535;
    public static final int MAX_PARAMETERS = 255;
    public static final int MAX_STACK = 65535;
    public static final int REF_getField = 1;
    public static final int REF_getStatic = 2;
    public static final int REF_invokeInterface = 9;
    public static final int REF_invokeSpecial = 7;
    public static final int REF_invokeStatic = 6;
    public static final int REF_invokeVirtual = 5;
    public static final int REF_newInvokeSpecial = 8;
    public static final int REF_putField = 3;
    public static final int REF_putStatic = 4;

    public enum Version {
        V45_3(45, 3),
        V49(49, 0),
        V50(50, 0),
        V51(51, 0),
        V52(52, 0);

        public final int major;
        public final int minor;

        Version(int major, int minor) {
            this.major = major;
            this.minor = minor;
        }
    }

    public static byte[] internalize(byte[] buf, int offset, int len) {
        byte[] translated = new byte[len];
        for (int j = 0; j < len; j++) {
            byte b = buf[offset + j];
            if (b == 47) {
                translated[j] = 46;
            } else {
                translated[j] = b;
            }
        }
        return translated;
    }

    public static byte[] internalize(Name name) {
        return internalize(name.getByteArray(), name.getByteOffset(), name.getByteLength());
    }

    public static byte[] externalize(byte[] buf, int offset, int len) {
        byte[] translated = new byte[len];
        for (int j = 0; j < len; j++) {
            byte b = buf[offset + j];
            if (b == 46) {
                translated[j] = 47;
            } else {
                translated[j] = b;
            }
        }
        return translated;
    }

    public static byte[] externalize(Name name) {
        return externalize(name.getByteArray(), name.getByteOffset(), name.getByteLength());
    }

    public static class NameAndType {
        Name name;
        Types types;
        Types.UniqueType uniqueType;

        NameAndType(Name name, Type type, Types types) {
            this.name = name;
            this.uniqueType = new Types.UniqueType(type, types);
            this.types = types;
        }

        void setType(Type type) {
            this.uniqueType = new Types.UniqueType(type, this.types);
        }

        public boolean equals(Object other) {
            return (other instanceof NameAndType) && this.name == ((NameAndType) other).name && this.uniqueType.equals(((NameAndType) other).uniqueType);
        }

        public int hashCode() {
            return this.name.hashCode() * this.uniqueType.hashCode();
        }
    }
}
