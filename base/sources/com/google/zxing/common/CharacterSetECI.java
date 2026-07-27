package com.google.zxing.common;

import com.google.zxing.FormatException;
import com.sun.tools.javac.jvm.ByteCodes;
import java.util.HashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public enum CharacterSetECI {
    Cp437(new int[]{0, 2}, new String[0]),
    ISO8859_1(new int[]{1, 3}, "ISO-8859-1"),
    ISO8859_2(4, "ISO-8859-2"),
    ISO8859_3(5, "ISO-8859-3"),
    ISO8859_4(6, "ISO-8859-4"),
    ISO8859_5(7, "ISO-8859-5"),
    ISO8859_6(8, "ISO-8859-6"),
    ISO8859_7(9, "ISO-8859-7"),
    ISO8859_8(10, "ISO-8859-8"),
    ISO8859_9(11, "ISO-8859-9"),
    ISO8859_10(12, "ISO-8859-10"),
    ISO8859_11(13, "ISO-8859-11"),
    ISO8859_13(15, "ISO-8859-13"),
    ISO8859_14(16, "ISO-8859-14"),
    ISO8859_15(17, "ISO-8859-15"),
    ISO8859_16(18, "ISO-8859-16"),
    SJIS(20, "Shift_JIS"),
    Cp1250(21, "windows-1250"),
    Cp1251(22, "windows-1251"),
    Cp1252(23, "windows-1252"),
    Cp1256(24, "windows-1256"),
    UnicodeBigUnmarked(25, "UTF-16BE", "UnicodeBig"),
    UTF8(26, "UTF-8"),
    ASCII(new int[]{27, ByteCodes.tableswitch}, "US-ASCII"),
    Big5(28),
    GB18030(29, StringUtils.GB2312, "EUC_CN", "GBK"),
    EUC_KR(30, "EUC-KR");

    private final String[] otherEncodingNames;
    private final int[] values;
    private static final Map<Integer, CharacterSetECI> VALUE_TO_ECI = new HashMap();
    private static final Map<String, CharacterSetECI> NAME_TO_ECI = new HashMap();

    static {
        for (CharacterSetECI eci : values()) {
            for (int value : eci.values) {
                VALUE_TO_ECI.put(Integer.valueOf(value), eci);
            }
            NAME_TO_ECI.put(eci.name(), eci);
            for (String name : eci.otherEncodingNames) {
                NAME_TO_ECI.put(name, eci);
            }
        }
    }

    CharacterSetECI(int value) {
        this(new int[]{value}, new String[0]);
    }

    CharacterSetECI(int value, String... otherEncodingNames) {
        this.values = new int[]{value};
        this.otherEncodingNames = otherEncodingNames;
    }

    CharacterSetECI(int[] values, String... otherEncodingNames) {
        this.values = values;
        this.otherEncodingNames = otherEncodingNames;
    }

    public int getValue() {
        return this.values[0];
    }

    public static CharacterSetECI getCharacterSetECIByValue(int value) throws FormatException {
        if (value < 0 || value >= 900) {
            throw FormatException.getFormatInstance();
        }
        return VALUE_TO_ECI.get(Integer.valueOf(value));
    }

    public static CharacterSetECI getCharacterSetECIByName(String name) {
        return NAME_TO_ECI.get(name);
    }
}
