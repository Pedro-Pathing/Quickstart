package com.google.zxing.client.result;

import com.google.zxing.Result;
import java.util.ArrayList;
import java.util.List;

/* JADX INFO: loaded from: classes8.dex */
public final class AddressBookAUResultParser extends ResultParser {
    @Override // com.google.zxing.client.result.ResultParser
    public AddressBookParsedResult parse(Result result) {
        String rawText = getMassagedText(result);
        if (!rawText.contains("MEMORY") || !rawText.contains("\r\n")) {
            return null;
        }
        String name = matchSinglePrefixedField("NAME1:", rawText, '\r', true);
        String pronunciation = matchSinglePrefixedField("NAME2:", rawText, '\r', true);
        String[] phoneNumbers = matchMultipleValuePrefix("TEL", rawText);
        String[] emails = matchMultipleValuePrefix("MAIL", rawText);
        String note = matchSinglePrefixedField("MEMORY:", rawText, '\r', false);
        String address = matchSinglePrefixedField("ADD:", rawText, '\r', true);
        String[] addresses = address != null ? new String[]{address} : null;
        return new AddressBookParsedResult(maybeWrap(name), null, pronunciation, phoneNumbers, null, emails, null, null, note, addresses, null, null, null, null, null, null);
    }

    private static String[] matchMultipleValuePrefix(String prefix, String rawText) {
        String value;
        List<String> values = null;
        for (int i = 1; i <= 3 && (value = matchSinglePrefixedField(prefix + i + ':', rawText, '\r', true)) != null; i++) {
            if (values == null) {
                values = new ArrayList<>(3);
            }
            values.add(value);
        }
        if (values == null) {
            return null;
        }
        return (String[]) values.toArray(EMPTY_STR_ARRAY);
    }
}
