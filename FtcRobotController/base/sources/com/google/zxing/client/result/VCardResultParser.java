package com.google.zxing.client.result;

import com.google.zxing.Result;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.sun.tools.doclint.DocLint;
import java.io.ByteArrayOutputStream;
import java.io.UnsupportedEncodingException;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public final class VCardResultParser extends ResultParser {
    private static final Pattern BEGIN_VCARD = Pattern.compile("BEGIN:VCARD", 2);
    private static final Pattern VCARD_LIKE_DATE = Pattern.compile("\\d{4}-?\\d{2}-?\\d{2}");
    private static final Pattern CR_LF_SPACE_TAB = Pattern.compile("\r\n[ \t]");
    private static final Pattern NEWLINE_ESCAPE = Pattern.compile("\\\\[nN]");
    private static final Pattern VCARD_ESCAPES = Pattern.compile("\\\\([,;\\\\])");
    private static final Pattern EQUALS = Pattern.compile("=");
    private static final Pattern SEMICOLON = Pattern.compile(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
    private static final Pattern UNESCAPED_SEMICOLONS = Pattern.compile("(?<!\\\\);+");
    private static final Pattern COMMA = Pattern.compile(DocLint.TAGS_SEPARATOR);
    private static final Pattern SEMICOLON_OR_COMMA = Pattern.compile("[;,]");

    @Override // com.google.zxing.client.result.ResultParser
    public AddressBookParsedResult parse(Result result) {
        List<String> birthday;
        String rawText = getMassagedText(result);
        Matcher m = BEGIN_VCARD.matcher(rawText);
        if (!m.find() || m.start() != 0) {
            return null;
        }
        List<List<String>> listMatchVCardPrefixedField = matchVCardPrefixedField("FN", rawText, true, false);
        List<List<String>> names = listMatchVCardPrefixedField;
        if (listMatchVCardPrefixedField == null) {
            List<List<String>> listMatchVCardPrefixedField2 = matchVCardPrefixedField("N", rawText, true, false);
            names = listMatchVCardPrefixedField2;
            formatNames(listMatchVCardPrefixedField2);
        }
        List<String> nicknameString = matchSingleVCardPrefixedField("NICKNAME", rawText, true, false);
        String[] nicknames = nicknameString == null ? null : COMMA.split(nicknameString.get(0));
        List<List<String>> phoneNumbers = matchVCardPrefixedField("TEL", rawText, true, false);
        List<List<String>> emails = matchVCardPrefixedField("EMAIL", rawText, true, false);
        List<String> note = matchSingleVCardPrefixedField("NOTE", rawText, false, false);
        List<List<String>> addresses = matchVCardPrefixedField("ADR", rawText, true, true);
        List<String> org2 = matchSingleVCardPrefixedField("ORG", rawText, true, true);
        List<String> birthday2 = matchSingleVCardPrefixedField("BDAY", rawText, true, false);
        if (birthday2 != null && !isLikeVCardDate(birthday2.get(0))) {
            birthday = null;
        } else {
            birthday = birthday2;
        }
        List<String> title = matchSingleVCardPrefixedField("TITLE", rawText, true, false);
        List<List<String>> urls = matchVCardPrefixedField("URL", rawText, true, false);
        List<String> instantMessenger = matchSingleVCardPrefixedField("IMPP", rawText, true, false);
        List<String> geoString = matchSingleVCardPrefixedField("GEO", rawText, true, false);
        String[] strArrSplit = geoString != null ? SEMICOLON_OR_COMMA.split(geoString.get(0)) : null;
        String[] geo = strArrSplit;
        if (strArrSplit != null && geo.length != 2) {
            geo = null;
        }
        return new AddressBookParsedResult(toPrimaryValues(names), nicknames, null, toPrimaryValues(phoneNumbers), toTypes(phoneNumbers), toPrimaryValues(emails), toTypes(emails), toPrimaryValue(instantMessenger), toPrimaryValue(note), toPrimaryValues(addresses), toTypes(addresses), toPrimaryValue(org2), toPrimaryValue(birthday), toPrimaryValue(title), toPrimaryValues(urls), geo);
    }

    static List<List<String>> matchVCardPrefixedField(CharSequence charSequence, String str, boolean z, boolean z2) {
        ArrayList arrayList;
        int i;
        String str2;
        String str3;
        int iIndexOf;
        String strReplaceAll;
        int i2;
        int length = str.length();
        int i3 = 0;
        int i4 = 0;
        ArrayList arrayList2 = null;
        while (i4 < length) {
            int i5 = 2;
            Matcher matcher = Pattern.compile("(?:^|\n)" + ((Object) charSequence) + "(?:;([^:]*))?:", 2).matcher(str);
            if (i4 > 0) {
                i4--;
            }
            if (!matcher.find(i4)) {
                break;
            }
            int iEnd = matcher.end(i3);
            String strGroup = matcher.group(1);
            if (strGroup == null) {
                arrayList = null;
                i = 0;
                str2 = null;
                str3 = null;
            } else {
                String[] strArrSplit = SEMICOLON.split(strGroup);
                int length2 = strArrSplit.length;
                int i6 = i3;
                i = i6;
                arrayList = null;
                str2 = null;
                str3 = null;
                while (i6 < length2) {
                    String str4 = strArrSplit[i6];
                    if (arrayList == null) {
                        arrayList = new ArrayList(1);
                    }
                    arrayList.add(str4);
                    String[] strArrSplit2 = EQUALS.split(str4, i5);
                    if (strArrSplit2.length > 1) {
                        String str5 = strArrSplit2[0];
                        String str6 = strArrSplit2[1];
                        if ("ENCODING".equalsIgnoreCase(str5) && "QUOTED-PRINTABLE".equalsIgnoreCase(str6)) {
                            i = 1;
                        } else if ("CHARSET".equalsIgnoreCase(str5)) {
                            str2 = str6;
                        } else if ("VALUE".equalsIgnoreCase(str5)) {
                            str3 = str6;
                        }
                    }
                    i6++;
                    i5 = 2;
                }
            }
            int i7 = iEnd;
            while (true) {
                iIndexOf = str.indexOf(10, i7);
                if (iIndexOf < 0) {
                    break;
                }
                if (iIndexOf < str.length() - 1) {
                    int i8 = iIndexOf + 1;
                    if (str.charAt(i8) == ' ' || str.charAt(i8) == '\t') {
                        i7 = iIndexOf + 2;
                    }
                }
                if (i == 0 || ((iIndexOf <= 0 || str.charAt(iIndexOf - 1) != '=') && (iIndexOf < 2 || str.charAt(iIndexOf - 2) != '='))) {
                    break;
                }
                i7 = iIndexOf + 1;
            }
            if (iIndexOf < 0) {
                i4 = length;
                i3 = 0;
            } else if (iIndexOf > iEnd) {
                if (arrayList2 == null) {
                    arrayList2 = new ArrayList(1);
                }
                if (iIndexOf > 0 && str.charAt(iIndexOf - 1) == '\r') {
                    iIndexOf--;
                }
                String strSubstring = str.substring(iEnd, iIndexOf);
                if (z) {
                    strSubstring = strSubstring.trim();
                }
                if (i != 0) {
                    String strDecodeQuotedPrintable = decodeQuotedPrintable(strSubstring, str2);
                    if (z2) {
                        strReplaceAll = UNESCAPED_SEMICOLONS.matcher(strDecodeQuotedPrintable).replaceAll("\n").trim();
                    } else {
                        strReplaceAll = strDecodeQuotedPrintable;
                    }
                } else {
                    if (z2) {
                        strSubstring = UNESCAPED_SEMICOLONS.matcher(strSubstring).replaceAll("\n").trim();
                    }
                    strReplaceAll = VCARD_ESCAPES.matcher(NEWLINE_ESCAPE.matcher(CR_LF_SPACE_TAB.matcher(strSubstring).replaceAll("")).replaceAll("\n")).replaceAll("$1");
                }
                if ("uri".equals(str3)) {
                    try {
                        strReplaceAll = URI.create(strReplaceAll).getSchemeSpecificPart();
                    } catch (IllegalArgumentException e) {
                    }
                }
                if (arrayList == null) {
                    ArrayList arrayList3 = new ArrayList(1);
                    arrayList3.add(strReplaceAll);
                    arrayList2.add(arrayList3);
                    i2 = 0;
                } else {
                    i2 = 0;
                    arrayList.add(0, strReplaceAll);
                    arrayList2.add(arrayList);
                }
                i4 = iIndexOf + 1;
                i3 = i2;
            } else {
                i4 = iIndexOf + 1;
                i3 = 0;
            }
        }
        return arrayList2;
    }

    private static String decodeQuotedPrintable(CharSequence value, String charset) {
        char nextChar;
        int length = value.length();
        StringBuilder result = new StringBuilder(length);
        ByteArrayOutputStream fragmentBuffer = new ByteArrayOutputStream();
        int i = 0;
        while (i < length) {
            char c = value.charAt(i);
            switch (c) {
                case '\n':
                case '\r':
                    break;
                case '=':
                    if (i < length - 2 && (nextChar = value.charAt(i + 1)) != '\r' && nextChar != '\n') {
                        char nextNextChar = value.charAt(i + 2);
                        int firstDigit = parseHexDigit(nextChar);
                        int secondDigit = parseHexDigit(nextNextChar);
                        if (firstDigit >= 0 && secondDigit >= 0) {
                            fragmentBuffer.write((firstDigit << 4) + secondDigit);
                        }
                        i += 2;
                    }
                    break;
                default:
                    maybeAppendFragment(fragmentBuffer, charset, result);
                    result.append(c);
                    break;
            }
            i++;
        }
        maybeAppendFragment(fragmentBuffer, charset, result);
        return result.toString();
    }

    private static void maybeAppendFragment(ByteArrayOutputStream fragmentBuffer, String charset, StringBuilder result) {
        String fragment;
        if (fragmentBuffer.size() > 0) {
            byte[] fragmentBytes = fragmentBuffer.toByteArray();
            if (charset == null) {
                fragment = new String(fragmentBytes, StandardCharsets.UTF_8);
            } else {
                try {
                    fragment = new String(fragmentBytes, charset);
                } catch (UnsupportedEncodingException e) {
                    fragment = new String(fragmentBytes, StandardCharsets.UTF_8);
                }
            }
            fragmentBuffer.reset();
            result.append(fragment);
        }
    }

    static List<String> matchSingleVCardPrefixedField(CharSequence prefix, String rawText, boolean trim, boolean parseFieldDivider) {
        List<List<String>> values = matchVCardPrefixedField(prefix, rawText, trim, parseFieldDivider);
        if (values == null || values.isEmpty()) {
            return null;
        }
        return values.get(0);
    }

    private static String toPrimaryValue(List<String> list) {
        if (list == null || list.isEmpty()) {
            return null;
        }
        return list.get(0);
    }

    private static String[] toPrimaryValues(Collection<List<String>> lists) {
        if (lists == null || lists.isEmpty()) {
            return null;
        }
        List<String> result = new ArrayList<>(lists.size());
        Iterator<List<String>> it = lists.iterator();
        while (it.hasNext()) {
            String value = it.next().get(0);
            if (value != null && !value.isEmpty()) {
                result.add(value);
            }
        }
        return (String[]) result.toArray(EMPTY_STR_ARRAY);
    }

    private static String[] toTypes(Collection<List<String>> lists) {
        if (lists == null || lists.isEmpty()) {
            return null;
        }
        List<String> result = new ArrayList<>(lists.size());
        for (List<String> list : lists) {
            String value = list.get(0);
            if (value != null && !value.isEmpty()) {
                String type = null;
                int i = 1;
                while (true) {
                    if (i >= list.size()) {
                        break;
                    }
                    String metadatum = list.get(i);
                    int equals = metadatum.indexOf(61);
                    if (equals < 0) {
                        type = metadatum;
                        break;
                    }
                    if (!"TYPE".equalsIgnoreCase(metadatum.substring(0, equals))) {
                        i++;
                    } else {
                        type = metadatum.substring(equals + 1);
                        break;
                    }
                }
                result.add(type);
            }
        }
        return (String[]) result.toArray(EMPTY_STR_ARRAY);
    }

    private static boolean isLikeVCardDate(CharSequence value) {
        return value == null || VCARD_LIKE_DATE.matcher(value).matches();
    }

    private static void formatNames(Iterable<List<String>> names) {
        int end;
        if (names != null) {
            for (List<String> list : names) {
                String name = list.get(0);
                String[] components = new String[5];
                int start = 0;
                int componentIndex = 0;
                while (componentIndex < 4 && (end = name.indexOf(59, start)) >= 0) {
                    components[componentIndex] = name.substring(start, end);
                    componentIndex++;
                    start = end + 1;
                }
                components[componentIndex] = name.substring(start);
                StringBuilder newName = new StringBuilder(100);
                maybeAppendComponent(components, 3, newName);
                maybeAppendComponent(components, 1, newName);
                maybeAppendComponent(components, 2, newName);
                maybeAppendComponent(components, 0, newName);
                maybeAppendComponent(components, 4, newName);
                list.set(0, newName.toString().trim());
            }
        }
    }

    private static void maybeAppendComponent(String[] components, int i, StringBuilder newName) {
        if (components[i] != null && !components[i].isEmpty()) {
            if (newName.length() > 0) {
                newName.append(' ');
            }
            newName.append(components[i]);
        }
    }
}
