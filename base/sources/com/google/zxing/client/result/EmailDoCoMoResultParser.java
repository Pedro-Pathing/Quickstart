package com.google.zxing.client.result;

import com.google.zxing.Result;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public final class EmailDoCoMoResultParser extends AbstractDoCoMoResultParser {
    private static final Pattern ATEXT_ALPHANUMERIC = Pattern.compile("[a-zA-Z0-9@.!#$%&'*+\\-/=?^_`{|}~]+");

    @Override // com.google.zxing.client.result.ResultParser
    public EmailAddressParsedResult parse(Result result) {
        String[] tos;
        String rawText = getMassagedText(result);
        if (!rawText.startsWith("MATMSG:") || (tos = matchDoCoMoPrefixedField("TO:", rawText)) == null) {
            return null;
        }
        for (String str : tos) {
            if (!isBasicallyValidEmailAddress(str)) {
                return null;
            }
        }
        String subject = matchSingleDoCoMoPrefixedField("SUB:", rawText, false);
        String body = matchSingleDoCoMoPrefixedField("BODY:", rawText, false);
        return new EmailAddressParsedResult(tos, null, null, subject, body);
    }

    static boolean isBasicallyValidEmailAddress(String email) {
        return email != null && ATEXT_ALPHANUMERIC.matcher(email).matches() && email.indexOf(64) >= 0;
    }
}
