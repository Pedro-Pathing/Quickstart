package com.google.zxing.client.result;

/* JADX INFO: loaded from: classes8.dex */
public final class WifiParsedResult extends ParsedResult {
    private final String anonymousIdentity;
    private final String eapMethod;
    private final boolean hidden;
    private final String identity;
    private final String networkEncryption;
    private final String password;
    private final String phase2Method;
    private final String ssid;

    public WifiParsedResult(String networkEncryption, String ssid, String password) {
        this(networkEncryption, ssid, password, false);
    }

    public WifiParsedResult(String networkEncryption, String ssid, String password, boolean hidden) {
        this(networkEncryption, ssid, password, hidden, null, null, null, null);
    }

    public WifiParsedResult(String networkEncryption, String ssid, String password, boolean hidden, String identity, String anonymousIdentity, String eapMethod, String phase2Method) {
        super(ParsedResultType.WIFI);
        this.ssid = ssid;
        this.networkEncryption = networkEncryption;
        this.password = password;
        this.hidden = hidden;
        this.identity = identity;
        this.anonymousIdentity = anonymousIdentity;
        this.eapMethod = eapMethod;
        this.phase2Method = phase2Method;
    }

    public String getSsid() {
        return this.ssid;
    }

    public String getNetworkEncryption() {
        return this.networkEncryption;
    }

    public String getPassword() {
        return this.password;
    }

    public boolean isHidden() {
        return this.hidden;
    }

    public String getIdentity() {
        return this.identity;
    }

    public String getAnonymousIdentity() {
        return this.anonymousIdentity;
    }

    public String getEapMethod() {
        return this.eapMethod;
    }

    public String getPhase2Method() {
        return this.phase2Method;
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        StringBuilder result = new StringBuilder(80);
        maybeAppend(this.ssid, result);
        maybeAppend(this.networkEncryption, result);
        maybeAppend(this.password, result);
        maybeAppend(Boolean.toString(this.hidden), result);
        return result.toString();
    }
}
