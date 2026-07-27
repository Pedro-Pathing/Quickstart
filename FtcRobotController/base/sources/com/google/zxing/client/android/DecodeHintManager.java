package com.google.zxing.client.android;

import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.util.Log;
import com.google.zxing.DecodeHintType;
import com.sun.tools.doclint.DocLint;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public final class DecodeHintManager {
    private static final String TAG = DecodeHintManager.class.getSimpleName();
    private static final Pattern COMMA = Pattern.compile(DocLint.TAGS_SEPARATOR);

    private DecodeHintManager() {
    }

    private static Map<String, String> splitQuery(String query) {
        String text;
        String name;
        Map<String, String> map = new HashMap<>();
        int pos = 0;
        while (true) {
            if (pos >= query.length()) {
                break;
            }
            if (query.charAt(pos) == '&') {
                pos++;
            } else {
                int amp = query.indexOf(38, pos);
                int equ = query.indexOf(61, pos);
                if (amp < 0) {
                    if (equ < 0) {
                        String name2 = query.substring(pos);
                        name = Uri.decode(name2.replace('+', ' '));
                        text = "";
                    } else {
                        String name3 = query.substring(pos, equ);
                        String name4 = Uri.decode(name3.replace('+', ' '));
                        String text2 = query.substring(equ + 1);
                        text = Uri.decode(text2.replace('+', ' '));
                        name = name4;
                    }
                    if (!map.containsKey(name)) {
                        map.put(name, text);
                    }
                } else if (equ < 0 || equ > amp) {
                    String name5 = query.substring(pos, amp);
                    String name6 = Uri.decode(name5.replace('+', ' '));
                    if (!map.containsKey(name6)) {
                        map.put(name6, "");
                    }
                    pos = amp + 1;
                } else {
                    String name7 = query.substring(pos, equ);
                    String name8 = Uri.decode(name7.replace('+', ' '));
                    String text3 = query.substring(equ + 1, amp);
                    String text4 = Uri.decode(text3.replace('+', ' '));
                    if (!map.containsKey(name8)) {
                        map.put(name8, text4);
                    }
                    pos = amp + 1;
                }
            }
        }
        return map;
    }

    static Map<DecodeHintType, ?> parseDecodeHints(Uri inputUri) {
        String parameterText;
        String query = inputUri.getEncodedQuery();
        if (query == null || query.isEmpty()) {
            return null;
        }
        Map<String, String> parameters = splitQuery(query);
        Map<DecodeHintType, Object> hints = new EnumMap<>(DecodeHintType.class);
        DecodeHintType[] decodeHintTypeArrValues = DecodeHintType.values();
        int length = decodeHintTypeArrValues.length;
        int i = 0;
        int i2 = 0;
        while (i2 < length) {
            DecodeHintType hintType = decodeHintTypeArrValues[i2];
            if (hintType != DecodeHintType.CHARACTER_SET && hintType != DecodeHintType.NEED_RESULT_POINT_CALLBACK && hintType != DecodeHintType.POSSIBLE_FORMATS) {
                String parameterName = hintType.name();
                String parameterText2 = parameters.get(parameterName);
                if (parameterText2 != null) {
                    if (hintType.getValueType().equals(Object.class)) {
                        hints.put(hintType, parameterText2);
                    } else if (hintType.getValueType().equals(Void.class)) {
                        hints.put(hintType, Boolean.TRUE);
                    } else if (hintType.getValueType().equals(String.class)) {
                        hints.put(hintType, parameterText2);
                    } else if (hintType.getValueType().equals(Boolean.class)) {
                        if (parameterText2.isEmpty()) {
                            hints.put(hintType, Boolean.TRUE);
                        } else if ("0".equals(parameterText2) || "false".equalsIgnoreCase(parameterText2) || "no".equalsIgnoreCase(parameterText2)) {
                            hints.put(hintType, Boolean.FALSE);
                        } else {
                            hints.put(hintType, Boolean.TRUE);
                        }
                    } else if (hintType.getValueType().equals(int[].class)) {
                        if (!parameterText2.isEmpty() && parameterText2.charAt(parameterText2.length() - 1) == ',') {
                            parameterText = parameterText2.substring(i, parameterText2.length() - 1);
                        } else {
                            parameterText = parameterText2;
                        }
                        String[] values = COMMA.split(parameterText);
                        int[] array = new int[values.length];
                        for (int i3 = 0; i3 < values.length; i3++) {
                            try {
                                array[i3] = Integer.parseInt(values[i3]);
                            } catch (NumberFormatException e) {
                                Log.w(TAG, "Skipping array of integers hint " + hintType + " due to invalid numeric value: '" + values[i3] + '\'');
                                array = null;
                            }
                        }
                        if (array != null) {
                            hints.put(hintType, array);
                        }
                    } else {
                        Log.w(TAG, "Unsupported hint type '" + hintType + "' of type " + hintType.getValueType());
                    }
                }
            }
            i2++;
            i = 0;
        }
        Log.i(TAG, "Hints from the URI: " + hints);
        return hints;
    }

    public static Map<DecodeHintType, Object> parseDecodeHints(Intent intent) {
        Bundle extras = intent.getExtras();
        if (extras == null || extras.isEmpty()) {
            return null;
        }
        Map<DecodeHintType, Object> hints = new EnumMap<>(DecodeHintType.class);
        for (DecodeHintType hintType : DecodeHintType.values()) {
            if (hintType != DecodeHintType.CHARACTER_SET && hintType != DecodeHintType.NEED_RESULT_POINT_CALLBACK && hintType != DecodeHintType.POSSIBLE_FORMATS) {
                String hintName = hintType.name();
                if (extras.containsKey(hintName)) {
                    if (hintType.getValueType().equals(Void.class)) {
                        hints.put(hintType, Boolean.TRUE);
                    } else {
                        Object hintData = extras.get(hintName);
                        if (hintType.getValueType().isInstance(hintData)) {
                            hints.put(hintType, hintData);
                        } else {
                            Log.w(TAG, "Ignoring hint " + hintType + " because it is not assignable from " + hintData);
                        }
                    }
                }
            }
        }
        Log.i(TAG, "Hints from the Intent: " + hints);
        return hints;
    }
}
