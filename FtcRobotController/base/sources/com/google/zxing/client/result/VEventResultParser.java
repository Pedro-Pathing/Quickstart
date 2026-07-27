package com.google.zxing.client.result;

import com.google.zxing.Result;
import java.util.List;

/* JADX INFO: loaded from: classes8.dex */
public final class VEventResultParser extends ResultParser {
    @Override // com.google.zxing.client.result.ResultParser
    public CalendarParsedResult parse(Result result) {
        double d;
        double d2;
        String massagedText = getMassagedText(result);
        if (massagedText.indexOf("BEGIN:VEVENT") < 0) {
            return null;
        }
        String strMatchSingleVCardPrefixedField = matchSingleVCardPrefixedField("SUMMARY", massagedText);
        String strMatchSingleVCardPrefixedField2 = matchSingleVCardPrefixedField("DTSTART", massagedText);
        if (strMatchSingleVCardPrefixedField2 == null) {
            return null;
        }
        String strMatchSingleVCardPrefixedField3 = matchSingleVCardPrefixedField("DTEND", massagedText);
        String strMatchSingleVCardPrefixedField4 = matchSingleVCardPrefixedField("DURATION", massagedText);
        String strMatchSingleVCardPrefixedField5 = matchSingleVCardPrefixedField("LOCATION", massagedText);
        String strStripMailto = stripMailto(matchSingleVCardPrefixedField("ORGANIZER", massagedText));
        String[] strArrMatchVCardPrefixedField = matchVCardPrefixedField("ATTENDEE", massagedText);
        if (strArrMatchVCardPrefixedField != null) {
            for (int i = 0; i < strArrMatchVCardPrefixedField.length; i++) {
                strArrMatchVCardPrefixedField[i] = stripMailto(strArrMatchVCardPrefixedField[i]);
            }
        }
        String strMatchSingleVCardPrefixedField6 = matchSingleVCardPrefixedField("DESCRIPTION", massagedText);
        String strMatchSingleVCardPrefixedField7 = matchSingleVCardPrefixedField("GEO", massagedText);
        if (strMatchSingleVCardPrefixedField7 == null) {
            d = Double.NaN;
            d2 = Double.NaN;
        } else {
            int iIndexOf = strMatchSingleVCardPrefixedField7.indexOf(59);
            if (iIndexOf < 0) {
                return null;
            }
            try {
                d = Double.parseDouble(strMatchSingleVCardPrefixedField7.substring(0, iIndexOf));
                d2 = Double.parseDouble(strMatchSingleVCardPrefixedField7.substring(iIndexOf + 1));
            } catch (NumberFormatException e) {
                return null;
            }
        }
        try {
            return new CalendarParsedResult(strMatchSingleVCardPrefixedField, strMatchSingleVCardPrefixedField2, strMatchSingleVCardPrefixedField3, strMatchSingleVCardPrefixedField4, strMatchSingleVCardPrefixedField5, strStripMailto, strArrMatchVCardPrefixedField, strMatchSingleVCardPrefixedField6, d, d2);
        } catch (IllegalArgumentException e2) {
            return null;
        }
    }

    private static String matchSingleVCardPrefixedField(CharSequence prefix, String rawText) {
        List<String> values = VCardResultParser.matchSingleVCardPrefixedField(prefix, rawText, true, false);
        if (values == null || values.isEmpty()) {
            return null;
        }
        return values.get(0);
    }

    private static String[] matchVCardPrefixedField(CharSequence prefix, String rawText) {
        List<List<String>> values = VCardResultParser.matchVCardPrefixedField(prefix, rawText, true, false);
        if (values == null || values.isEmpty()) {
            return null;
        }
        int size = values.size();
        String[] result = new String[size];
        for (int i = 0; i < size; i++) {
            result[i] = values.get(i).get(0);
        }
        return result;
    }

    private static String stripMailto(String s) {
        if (s == null) {
            return s;
        }
        if (s.startsWith("mailto:") || s.startsWith("MAILTO:")) {
            return s.substring(7);
        }
        return s;
    }
}
