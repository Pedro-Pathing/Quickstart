package com.google.zxing.client.result;

import com.google.zxing.Result;

/* JADX INFO: loaded from: classes8.dex */
public final class SMTPResultParser extends ResultParser {
    @Override // com.google.zxing.client.result.ResultParser
    public EmailAddressParsedResult parse(Result result) {
        String rawText = getMassagedText(result);
        if (rawText.startsWith("smtp:") || rawText.startsWith("SMTP:")) {
            String emailAddress = rawText.substring(5);
            String subject = null;
            String body = null;
            int colon = emailAddress.indexOf(58);
            if (colon >= 0) {
                subject = emailAddress.substring(colon + 1);
                emailAddress = emailAddress.substring(0, colon);
                int colon2 = subject.indexOf(58);
                if (colon2 >= 0) {
                    body = subject.substring(colon2 + 1);
                    subject = subject.substring(0, colon2);
                }
            }
            return new EmailAddressParsedResult(new String[]{emailAddress}, null, null, subject, body);
        }
        return null;
    }
}
