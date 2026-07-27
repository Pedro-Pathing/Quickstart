package com.google.zxing.client.result;

import com.google.zxing.Result;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public abstract class ResultParser {
    private static final String BYTE_ORDER_MARK = "\ufeff";
    private static final ResultParser[] PARSERS = {new BookmarkDoCoMoResultParser(), new AddressBookDoCoMoResultParser(), new EmailDoCoMoResultParser(), new AddressBookAUResultParser(), new VCardResultParser(), new BizcardResultParser(), new VEventResultParser(), new EmailAddressResultParser(), new SMTPResultParser(), new TelResultParser(), new SMSMMSResultParser(), new SMSTOMMSTOResultParser(), new GeoResultParser(), new WifiResultParser(), new URLTOResultParser(), new URIResultParser(), new ISBNResultParser(), new ProductResultParser(), new ExpandedProductResultParser(), new VINResultParser()};
    private static final Pattern DIGITS = Pattern.compile("\\d+");
    private static final Pattern AMPERSAND = Pattern.compile("&");
    private static final Pattern EQUALS = Pattern.compile("=");
    static final String[] EMPTY_STR_ARRAY = new String[0];

    public abstract ParsedResult parse(Result result);

    protected static String getMassagedText(Result result) {
        String text = result.getText();
        return text.startsWith(BYTE_ORDER_MARK) ? text.substring(1) : text;
    }

    public static ParsedResult parseResult(Result theResult) {
        for (ResultParser resultParser : PARSERS) {
            ParsedResult result = resultParser.parse(theResult);
            if (result != null) {
                return result;
            }
        }
        return new TextParsedResult(theResult.getText(), null);
    }

    protected static void maybeAppend(String value, StringBuilder result) {
        if (value != null) {
            result.append('\n');
            result.append(value);
        }
    }

    protected static void maybeAppend(String[] value, StringBuilder result) {
        if (value != null) {
            for (String s : value) {
                result.append('\n');
                result.append(s);
            }
        }
    }

    protected static String[] maybeWrap(String value) {
        if (value == null) {
            return null;
        }
        return new String[]{value};
    }

    protected static String unescapeBackslash(String escaped) {
        int backslash = escaped.indexOf(92);
        if (backslash < 0) {
            return escaped;
        }
        int max = escaped.length();
        StringBuilder unescaped = new StringBuilder(max - 1);
        unescaped.append(escaped.toCharArray(), 0, backslash);
        boolean nextIsEscaped = false;
        for (int i = backslash; i < max; i++) {
            char c = escaped.charAt(i);
            if (nextIsEscaped || c != '\\') {
                unescaped.append(c);
                nextIsEscaped = false;
            } else {
                nextIsEscaped = true;
            }
        }
        return unescaped.toString();
    }

    protected static int parseHexDigit(char c) {
        if (c >= '0' && c <= '9') {
            return c - '0';
        }
        if (c >= 'a' && c <= 'f') {
            return (c - 'a') + 10;
        }
        if (c >= 'A' && c <= 'F') {
            return (c - 'A') + 10;
        }
        return -1;
    }

    protected static boolean isStringOfDigits(CharSequence value, int length) {
        return value != null && length > 0 && length == value.length() && DIGITS.matcher(value).matches();
    }

    protected static boolean isSubstringOfDigits(CharSequence value, int offset, int length) {
        int max;
        return value != null && length > 0 && value.length() >= (max = offset + length) && DIGITS.matcher(value.subSequence(offset, max)).matches();
    }

    static Map<String, String> parseNameValuePairs(String uri) {
        int paramStart = uri.indexOf(63);
        if (paramStart < 0) {
            return null;
        }
        Map<String, String> result = new HashMap<>(3);
        for (String str : AMPERSAND.split(uri.substring(paramStart + 1))) {
            appendKeyValue(str, result);
        }
        return result;
    }

    private static void appendKeyValue(CharSequence keyValue, Map<String, String> result) {
        String[] keyValueTokens = EQUALS.split(keyValue, 2);
        if (keyValueTokens.length == 2) {
            String key = keyValueTokens[0];
            String value = keyValueTokens[1];
            try {
                result.put(key, urlDecode(value));
            } catch (IllegalArgumentException e) {
            }
        }
    }

    static String urlDecode(String encoded) {
        try {
            return URLDecoder.decode(encoded, "UTF-8");
        } catch (UnsupportedEncodingException uee) {
            throw new IllegalStateException(uee);
        }
    }

    static String[] matchPrefixedField(String prefix, String rawText, char endChar, boolean trim) {
        List<String> matches = null;
        int i = 0;
        int max = rawText.length();
        while (i < max) {
            int i2 = rawText.indexOf(prefix, i);
            if (i2 < 0) {
                break;
            }
            int start = prefix.length() + i2;
            i = start;
            boolean more = true;
            while (more) {
                int i3 = rawText.indexOf(endChar, i);
                if (i3 < 0) {
                    i = rawText.length();
                    more = false;
                } else if (countPrecedingBackslashes(rawText, i3) % 2 != 0) {
                    i = i3 + 1;
                } else {
                    if (matches == null) {
                        matches = new ArrayList<>(3);
                    }
                    String element = unescapeBackslash(rawText.substring(start, i3));
                    if (trim) {
                        element = element.trim();
                    }
                    if (!element.isEmpty()) {
                        matches.add(element);
                    }
                    i = i3 + 1;
                    more = false;
                }
            }
        }
        if (matches == null || matches.isEmpty()) {
            return null;
        }
        return (String[]) matches.toArray(EMPTY_STR_ARRAY);
    }

    private static int countPrecedingBackslashes(CharSequence s, int pos) {
        int count = 0;
        for (int i = pos - 1; i >= 0 && s.charAt(i) == '\\'; i--) {
            count++;
        }
        return count;
    }

    static String matchSinglePrefixedField(String prefix, String rawText, char endChar, boolean trim) {
        String[] matches = matchPrefixedField(prefix, rawText, endChar, trim);
        if (matches == null) {
            return null;
        }
        return matches[0];
    }
}
