package com.google.zxing.client.result;

import com.google.zxing.Result;

/* JADX INFO: loaded from: classes8.dex */
public final class WifiResultParser extends ResultParser {
    @Override // com.google.zxing.client.result.ResultParser
    public WifiParsedResult parse(Result result) {
        String rawText;
        String ssid;
        String type;
        boolean hidden;
        String phase2Method;
        String rawText2 = getMassagedText(result);
        if (!rawText2.startsWith("WIFI:") || (ssid = matchSinglePrefixedField("S:", (rawText = rawText2.substring(5)), ';', false)) == null || ssid.isEmpty()) {
            return null;
        }
        String pass = matchSinglePrefixedField("P:", rawText, ';', false);
        String type2 = matchSinglePrefixedField("T:", rawText, ';', false);
        if (type2 != null) {
            type = type2;
        } else {
            type = "nopass";
        }
        String phase2Method2 = matchSinglePrefixedField("PH2:", rawText, ';', false);
        String hValue = matchSinglePrefixedField("H:", rawText, ';', false);
        if (hValue == null) {
            hidden = false;
            phase2Method = phase2Method2;
        } else if (phase2Method2 == null && !"true".equalsIgnoreCase(hValue) && !"false".equalsIgnoreCase(hValue)) {
            hidden = false;
            phase2Method = hValue;
        } else {
            boolean hidden2 = Boolean.parseBoolean(hValue);
            hidden = hidden2;
            phase2Method = phase2Method2;
        }
        String identity = matchSinglePrefixedField("I:", rawText, ';', false);
        String anonymousIdentity = matchSinglePrefixedField("A:", rawText, ';', false);
        String eapMethod = matchSinglePrefixedField("E:", rawText, ';', false);
        return new WifiParsedResult(type, ssid, pass, hidden, identity, anonymousIdentity, eapMethod, phase2Method);
    }
}
