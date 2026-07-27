package com.sun.tools.javac.util;

import com.sun.tools.javac.jvm.ByteCodes;
import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;

/* JADX INFO: loaded from: classes.dex */
public class Convert {
    public static int string2int(String s, int radix) throws NumberFormatException {
        if (radix == 10) {
            return Integer.parseInt(s, radix);
        }
        char[] cs = s.toCharArray();
        int limit = Integer.MAX_VALUE / (radix / 2);
        int n = 0;
        for (char c : cs) {
            int d = Character.digit(c, radix);
            if (n < 0 || n > limit || n * radix > Integer.MAX_VALUE - d) {
                throw new NumberFormatException();
            }
            n = (n * radix) + d;
        }
        return n;
    }

    public static long string2long(String s, int radix) throws NumberFormatException {
        if (radix == 10) {
            return Long.parseLong(s, radix);
        }
        char[] cs = s.toCharArray();
        long limit = Long.MAX_VALUE / ((long) (radix / 2));
        long n = 0;
        for (char c : cs) {
            int d = Character.digit(c, radix);
            if (n < 0 || n > limit || ((long) radix) * n > Long.MAX_VALUE - ((long) d)) {
                throw new NumberFormatException();
            }
            n = (((long) radix) * n) + ((long) d);
        }
        return n;
    }

    public static int utf2chars(byte[] src, int sindex, char[] dst, int dindex, int len) {
        int b = sindex;
        int j = dindex;
        int limit = sindex + len;
        while (b < limit) {
            int i = b + 1;
            int b2 = src[b] & 255;
            if (b2 >= 224) {
                int i2 = i + 1;
                int b3 = ((b2 & 15) << 12) | ((src[i] & 63) << 6);
                i = i2 + 1;
                b2 = b3 | (src[i2] & 63);
            } else if (b2 >= 192) {
                b2 = ((b2 & 31) << 6) | (src[i] & 63);
                i++;
            }
            dst[j] = (char) b2;
            b = i;
            j++;
        }
        return j;
    }

    public static char[] utf2chars(byte[] src, int sindex, int len) {
        char[] dst = new char[len];
        int len1 = utf2chars(src, sindex, dst, 0, len);
        char[] result = new char[len1];
        System.arraycopy(dst, 0, result, 0, len1);
        return result;
    }

    public static char[] utf2chars(byte[] src) {
        return utf2chars(src, 0, src.length);
    }

    public static String utf2string(byte[] src, int sindex, int len) {
        char[] dst = new char[len];
        int len1 = utf2chars(src, sindex, dst, 0, len);
        return new String(dst, 0, len1);
    }

    public static String utf2string(byte[] src) {
        return utf2string(src, 0, src.length);
    }

    public static int chars2utf(char[] src, int sindex, byte[] dst, int dindex, int len) {
        int j = dindex;
        int limit = sindex + len;
        for (int i = sindex; i < limit; i++) {
            char ch = src[i];
            if (1 <= ch && ch <= 127) {
                dst[j] = (byte) ch;
                j++;
            } else if (ch <= 2047) {
                int j2 = j + 1;
                dst[j] = (byte) ((ch >> 6) | ByteCodes.checkcast);
                j = j2 + 1;
                dst[j2] = (byte) ((ch & '?') | 128);
            } else {
                int j3 = j + 1;
                dst[j] = (byte) ((ch >> '\f') | UsbConstants.USB_CLASS_WIRELESS_CONTROLLER);
                int j4 = j3 + 1;
                dst[j3] = (byte) (((ch >> 6) & 63) | 128);
                dst[j4] = (byte) ((ch & '?') | 128);
                j = j4 + 1;
            }
        }
        return j;
    }

    public static byte[] chars2utf(char[] src, int sindex, int len) {
        byte[] dst = new byte[len * 3];
        int len1 = chars2utf(src, sindex, dst, 0, len);
        byte[] result = new byte[len1];
        System.arraycopy(dst, 0, result, 0, len1);
        return result;
    }

    public static byte[] chars2utf(char[] src) {
        return chars2utf(src, 0, src.length);
    }

    public static byte[] string2utf(String s) {
        return chars2utf(s.toCharArray());
    }

    public static String quote(String s) {
        StringBuilder buf = new StringBuilder();
        for (int i = 0; i < s.length(); i++) {
            buf.append(quote(s.charAt(i)));
        }
        return buf.toString();
    }

    public static String quote(char ch) {
        switch (ch) {
            case '\b':
                return "\\b";
            case '\t':
                return "\\t";
            case '\n':
                return "\\n";
            case '\f':
                return "\\f";
            case '\r':
                return "\\r";
            case '\"':
                return "\\\"";
            case '\'':
                return "\\'";
            case '\\':
                return "\\\\";
            default:
                if (isPrintableAscii(ch)) {
                    return String.valueOf(ch);
                }
                return String.format("\\u%04x", Integer.valueOf(ch));
        }
    }

    private static boolean isPrintableAscii(char ch) {
        return ch >= ' ' && ch <= '~';
    }

    public static String escapeUnicode(String s) {
        int len = s.length();
        int i = 0;
        while (i < len) {
            if (s.charAt(i) > 255) {
                StringBuilder buf = new StringBuilder();
                buf.append(s.substring(0, i));
                while (i < len) {
                    char ch = s.charAt(i);
                    if (ch > 255) {
                        buf.append("\\u");
                        buf.append(Character.forDigit((ch >> '\f') % 16, 16));
                        buf.append(Character.forDigit((ch >> '\b') % 16, 16));
                        buf.append(Character.forDigit((ch >> 4) % 16, 16));
                        buf.append(Character.forDigit(ch % 16, 16));
                    } else {
                        buf.append(ch);
                    }
                    i++;
                }
                s = buf.toString();
            } else {
                i++;
            }
        }
        return s;
    }

    public static Name shortName(Name classname) {
        return classname.subName(classname.lastIndexOf((byte) 46) + 1, classname.getByteLength());
    }

    public static String shortName(String classname) {
        return classname.substring(classname.lastIndexOf(46) + 1);
    }

    public static Name packagePart(Name classname) {
        return classname.subName(0, classname.lastIndexOf((byte) 46));
    }

    public static String packagePart(String classname) {
        int lastDot = classname.lastIndexOf(46);
        return lastDot < 0 ? "" : classname.substring(0, lastDot);
    }

    public static List<Name> enclosingCandidates(Name name) {
        List<Name> names = List.nil();
        while (true) {
            int index = name.lastIndexOf((byte) 36);
            if (index > 0) {
                name = name.subName(0, index);
                names = names.prepend(name);
            } else {
                return names;
            }
        }
    }
}
