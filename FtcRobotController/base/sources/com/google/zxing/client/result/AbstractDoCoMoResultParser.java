package com.google.zxing.client.result;

/* JADX INFO: loaded from: classes8.dex */
abstract class AbstractDoCoMoResultParser extends ResultParser {
    AbstractDoCoMoResultParser() {
    }

    static String[] matchDoCoMoPrefixedField(String prefix, String rawText) {
        return matchPrefixedField(prefix, rawText, ';', true);
    }

    static String matchSingleDoCoMoPrefixedField(String prefix, String rawText, boolean trim) {
        return matchSinglePrefixedField(prefix, rawText, ';', trim);
    }
}
