package com.google.zxing.client.result;

import com.google.zxing.Result;
import com.qualcomm.hardware.lynx.LynxServoController;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/* JADX INFO: loaded from: classes8.dex */
public final class GeoResultParser extends ResultParser {
    private static final Pattern GEO_URL_PATTERN = Pattern.compile("geo:([\\-0-9.]+),([\\-0-9.]+)(?:,([\\-0-9.]+))?(?:\\?(.*))?", 2);

    @Override // com.google.zxing.client.result.ResultParser
    public GeoParsedResult parse(Result result) {
        double altitude;
        CharSequence rawText = getMassagedText(result);
        Matcher matcher = GEO_URL_PATTERN.matcher(rawText);
        if (!matcher.matches()) {
            return null;
        }
        String query = matcher.group(4);
        try {
            double latitude = Double.parseDouble(matcher.group(1));
            if (latitude <= 90.0d && latitude >= -90.0d) {
                double longitude = Double.parseDouble(matcher.group(2));
                if (longitude <= 180.0d && longitude >= -180.0d) {
                    if (matcher.group(3) == null) {
                        altitude = 0.0d;
                    } else {
                        double altitude2 = Double.parseDouble(matcher.group(3));
                        if (altitude2 < LynxServoController.apiPositionFirst) {
                            return null;
                        }
                        altitude = altitude2;
                    }
                    return new GeoParsedResult(latitude, longitude, altitude, query);
                }
                return null;
            }
            return null;
        } catch (NumberFormatException e) {
            return null;
        }
    }
}
