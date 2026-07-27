package com.google.zxing.client.result;

/* JADX INFO: loaded from: classes8.dex */
public final class AddressBookParsedResult extends ParsedResult {
    private final String[] addressTypes;
    private final String[] addresses;
    private final String birthday;
    private final String[] emailTypes;
    private final String[] emails;
    private final String[] geo;
    private final String instantMessenger;
    private final String[] names;
    private final String[] nicknames;
    private final String note;

    /* JADX INFO: renamed from: org, reason: collision with root package name */
    private final String f0org;
    private final String[] phoneNumbers;
    private final String[] phoneTypes;
    private final String pronunciation;
    private final String title;
    private final String[] urls;

    public AddressBookParsedResult(String[] names, String[] phoneNumbers, String[] phoneTypes, String[] emails, String[] emailTypes, String[] addresses, String[] addressTypes) {
        this(names, null, null, phoneNumbers, phoneTypes, emails, emailTypes, null, null, addresses, addressTypes, null, null, null, null, null);
    }

    public AddressBookParsedResult(String[] names, String[] nicknames, String pronunciation, String[] phoneNumbers, String[] phoneTypes, String[] emails, String[] emailTypes, String instantMessenger, String note, String[] addresses, String[] addressTypes, String org2, String birthday, String title, String[] urls, String[] geo) {
        super(ParsedResultType.ADDRESSBOOK);
        if (phoneNumbers != null && phoneTypes != null && phoneNumbers.length != phoneTypes.length) {
            throw new IllegalArgumentException("Phone numbers and types lengths differ");
        }
        if (emails != null && emailTypes != null && emails.length != emailTypes.length) {
            throw new IllegalArgumentException("Emails and types lengths differ");
        }
        if (addresses == null || addressTypes == null || addresses.length == addressTypes.length) {
            this.names = names;
            this.nicknames = nicknames;
            this.pronunciation = pronunciation;
            this.phoneNumbers = phoneNumbers;
            this.phoneTypes = phoneTypes;
            this.emails = emails;
            this.emailTypes = emailTypes;
            this.instantMessenger = instantMessenger;
            this.note = note;
            this.addresses = addresses;
            this.addressTypes = addressTypes;
            this.f0org = org2;
            this.birthday = birthday;
            this.title = title;
            this.urls = urls;
            this.geo = geo;
            return;
        }
        throw new IllegalArgumentException("Addresses and types lengths differ");
    }

    public String[] getNames() {
        return this.names;
    }

    public String[] getNicknames() {
        return this.nicknames;
    }

    public String getPronunciation() {
        return this.pronunciation;
    }

    public String[] getPhoneNumbers() {
        return this.phoneNumbers;
    }

    public String[] getPhoneTypes() {
        return this.phoneTypes;
    }

    public String[] getEmails() {
        return this.emails;
    }

    public String[] getEmailTypes() {
        return this.emailTypes;
    }

    public String getInstantMessenger() {
        return this.instantMessenger;
    }

    public String getNote() {
        return this.note;
    }

    public String[] getAddresses() {
        return this.addresses;
    }

    public String[] getAddressTypes() {
        return this.addressTypes;
    }

    public String getTitle() {
        return this.title;
    }

    public String getOrg() {
        return this.f0org;
    }

    public String[] getURLs() {
        return this.urls;
    }

    public String getBirthday() {
        return this.birthday;
    }

    public String[] getGeo() {
        return this.geo;
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        StringBuilder result = new StringBuilder(100);
        maybeAppend(this.names, result);
        maybeAppend(this.nicknames, result);
        maybeAppend(this.pronunciation, result);
        maybeAppend(this.title, result);
        maybeAppend(this.f0org, result);
        maybeAppend(this.addresses, result);
        maybeAppend(this.phoneNumbers, result);
        maybeAppend(this.emails, result);
        maybeAppend(this.instantMessenger, result);
        maybeAppend(this.urls, result);
        maybeAppend(this.birthday, result);
        maybeAppend(this.geo, result);
        maybeAppend(this.note, result);
        return result.toString();
    }
}
