package com.google.zxing.client.result;

import com.google.zxing.Result;
import java.util.ArrayList;
import java.util.List;

/* JADX INFO: loaded from: classes8.dex */
public final class BizcardResultParser extends AbstractDoCoMoResultParser {
    @Override // com.google.zxing.client.result.ResultParser
    public AddressBookParsedResult parse(Result result) {
        String rawText = getMassagedText(result);
        if (rawText.startsWith("BIZCARD:")) {
            String firstName = matchSingleDoCoMoPrefixedField("N:", rawText, true);
            String lastName = matchSingleDoCoMoPrefixedField("X:", rawText, true);
            String fullName = buildName(firstName, lastName);
            String title = matchSingleDoCoMoPrefixedField("T:", rawText, true);
            String org2 = matchSingleDoCoMoPrefixedField("C:", rawText, true);
            String[] addresses = matchDoCoMoPrefixedField("A:", rawText);
            String phoneNumber1 = matchSingleDoCoMoPrefixedField("B:", rawText, true);
            String phoneNumber2 = matchSingleDoCoMoPrefixedField("M:", rawText, true);
            String phoneNumber3 = matchSingleDoCoMoPrefixedField("F:", rawText, true);
            String email = matchSingleDoCoMoPrefixedField("E:", rawText, true);
            return new AddressBookParsedResult(maybeWrap(fullName), null, null, buildPhoneNumbers(phoneNumber1, phoneNumber2, phoneNumber3), null, maybeWrap(email), null, null, null, addresses, null, org2, null, title, null, null);
        }
        return null;
    }

    private static String[] buildPhoneNumbers(String number1, String number2, String number3) {
        List<String> numbers = new ArrayList<>(3);
        if (number1 != null) {
            numbers.add(number1);
        }
        if (number2 != null) {
            numbers.add(number2);
        }
        if (number3 != null) {
            numbers.add(number3);
        }
        int size = numbers.size();
        if (size == 0) {
            return null;
        }
        return (String[]) numbers.toArray(new String[size]);
    }

    private static String buildName(String firstName, String lastName) {
        if (firstName == null) {
            return lastName;
        }
        return lastName == null ? firstName : firstName + ' ' + lastName;
    }
}
