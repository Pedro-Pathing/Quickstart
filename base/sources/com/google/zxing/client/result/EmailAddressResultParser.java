package com.google.zxing.client.result;

import com.google.zxing.Result;
import com.sun.tools.doclint.DocLint;
import java.util.Map;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public final class EmailAddressResultParser extends ResultParser {
    private static final Pattern COMMA = Pattern.compile(DocLint.TAGS_SEPARATOR);

    @Override // com.google.zxing.client.result.ResultParser
    public EmailAddressParsedResult parse(Result result) {
        String[] strArrSplit;
        String[] strArr;
        String[] strArr2;
        String[] strArr3;
        String str;
        String str2;
        String[] strArrSplit2;
        String str3;
        String massagedText = getMassagedText(result);
        if (massagedText.startsWith("mailto:") || massagedText.startsWith("MAILTO:")) {
            String strSubstring = massagedText.substring(7);
            int iIndexOf = strSubstring.indexOf(63);
            if (iIndexOf >= 0) {
                strSubstring = strSubstring.substring(0, iIndexOf);
            }
            try {
                String strUrlDecode = urlDecode(strSubstring);
                if (strUrlDecode.isEmpty()) {
                    strArrSplit = null;
                } else {
                    strArrSplit = COMMA.split(strUrlDecode);
                }
                Map<String, String> nameValuePairs = parseNameValuePairs(massagedText);
                if (nameValuePairs == null) {
                    strArr = strArrSplit;
                    strArr2 = null;
                    strArr3 = null;
                    str = null;
                    str2 = null;
                } else {
                    if (strArrSplit == null && (str3 = nameValuePairs.get("to")) != null) {
                        strArrSplit = COMMA.split(str3);
                    }
                    String str4 = nameValuePairs.get("cc");
                    if (str4 == null) {
                        strArrSplit2 = null;
                    } else {
                        strArrSplit2 = COMMA.split(str4);
                    }
                    String str5 = nameValuePairs.get("bcc");
                    String[] strArrSplit3 = str5 != null ? COMMA.split(str5) : null;
                    String str6 = nameValuePairs.get("subject");
                    str2 = nameValuePairs.get("body");
                    strArr = strArrSplit;
                    strArr3 = strArrSplit3;
                    strArr2 = strArrSplit2;
                    str = str6;
                }
                return new EmailAddressParsedResult(strArr, strArr2, strArr3, str, str2);
            } catch (IllegalArgumentException e) {
                return null;
            }
        }
        if (EmailDoCoMoResultParser.isBasicallyValidEmailAddress(massagedText)) {
            return new EmailAddressParsedResult(massagedText);
        }
        return null;
    }
}
