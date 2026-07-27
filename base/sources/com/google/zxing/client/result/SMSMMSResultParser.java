package com.google.zxing.client.result;

import com.google.zxing.Result;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public final class SMSMMSResultParser extends ResultParser {
    @Override // com.google.zxing.client.result.ResultParser
    public SMSParsedResult parse(Result result) {
        String smsURIWithoutQuery;
        String rawText = getMassagedText(result);
        if (!rawText.startsWith("sms:") && !rawText.startsWith("SMS:") && !rawText.startsWith("mms:") && !rawText.startsWith("MMS:")) {
            return null;
        }
        Map<String, String> nameValuePairs = parseNameValuePairs(rawText);
        String subject = null;
        String body = null;
        boolean querySyntax = false;
        if (nameValuePairs != null && !nameValuePairs.isEmpty()) {
            subject = nameValuePairs.get("subject");
            body = nameValuePairs.get("body");
            querySyntax = true;
        }
        int queryStart = rawText.indexOf(63, 4);
        if (queryStart < 0 || !querySyntax) {
            smsURIWithoutQuery = rawText.substring(4);
        } else {
            smsURIWithoutQuery = rawText.substring(4, queryStart);
        }
        int lastComma = -1;
        List<String> numbers = new ArrayList<>(1);
        List<String> vias = new ArrayList<>(1);
        while (true) {
            int comma = smsURIWithoutQuery.indexOf(44, lastComma + 1);
            if (comma > lastComma) {
                String numberPart = smsURIWithoutQuery.substring(lastComma + 1, comma);
                addNumberVia(numbers, vias, numberPart);
                lastComma = comma;
            } else {
                addNumberVia(numbers, vias, smsURIWithoutQuery.substring(lastComma + 1));
                return new SMSParsedResult((String[]) numbers.toArray(EMPTY_STR_ARRAY), (String[]) vias.toArray(EMPTY_STR_ARRAY), subject, body);
            }
        }
    }

    private static void addNumberVia(Collection<String> numbers, Collection<String> vias, String numberPart) {
        String via;
        int numberEnd = numberPart.indexOf(59);
        if (numberEnd >= 0) {
            numbers.add(numberPart.substring(0, numberEnd));
            String maybeVia = numberPart.substring(numberEnd + 1);
            if (maybeVia.startsWith("via=")) {
                via = maybeVia.substring(4);
            } else {
                via = null;
            }
            vias.add(via);
            return;
        }
        numbers.add(numberPart);
        vias.add(null);
    }
}
