package com.google.zxing.oned;

import com.google.zxing.client.result.ExpandedProductParsedResult;
import java.util.ArrayList;
import java.util.List;
import org.opencv.videoio.Videoio;

/* JADX INFO: loaded from: classes8.dex */
final class EANManufacturerOrgSupport {
    private final List<int[]> ranges = new ArrayList();
    private final List<String> countryIdentifiers = new ArrayList();

    EANManufacturerOrgSupport() {
    }

    String lookupCountryIdentifier(String productCode) {
        int[] range;
        int start;
        initIfNeeded();
        int prefix = Integer.parseInt(productCode.substring(0, 3));
        int max = this.ranges.size();
        for (int i = 0; i < max && prefix >= (start = (range = this.ranges.get(i))[0]); i++) {
            int end = range.length == 1 ? start : range[1];
            if (prefix <= end) {
                return this.countryIdentifiers.get(i);
            }
        }
        return null;
    }

    private void add(int[] range, String id) {
        this.ranges.add(range);
        this.countryIdentifiers.add(id);
    }

    private synchronized void initIfNeeded() {
        if (this.ranges.isEmpty()) {
            add(new int[]{0, 19}, "US/CA");
            add(new int[]{30, 39}, "US");
            add(new int[]{60, 139}, "US/CA");
            add(new int[]{300, 379}, "FR");
            add(new int[]{380}, "BG");
            add(new int[]{383}, "SI");
            add(new int[]{385}, "HR");
            add(new int[]{387}, "BA");
            add(new int[]{400, Videoio.CAP_PROP_XI_AEAG_ROI_OFFSET_Y}, "DE");
            add(new int[]{Videoio.CAP_PROP_XI_WB_KB, Videoio.CAP_PROP_XI_LIMIT_BANDWIDTH}, "JP");
            add(new int[]{Videoio.CAP_PROP_XI_SENSOR_DATA_BIT_DEPTH, Videoio.CAP_PROP_XI_HOUS_TEMP}, "RU");
            add(new int[]{Videoio.CAP_PROP_XI_APPLY_CMS}, "TW");
            add(new int[]{Videoio.CAP_PROP_XI_IMAGE_IS_COLOR}, "EE");
            add(new int[]{Videoio.CAP_PROP_XI_COLOR_FILTER_ARRAY}, "LV");
            add(new int[]{Videoio.CAP_PROP_XI_GAMMAY}, "AZ");
            add(new int[]{Videoio.CAP_PROP_XI_GAMMAC}, "LT");
            add(new int[]{Videoio.CAP_PROP_XI_SHARPNESS}, "UZ");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_00}, "LK");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_01}, "PH");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_02}, "BY");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_03}, "UA");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_11}, "MD");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_12}, "AM");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_13}, "GE");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_20}, "KZ");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_22}, "HK");
            add(new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_23, Videoio.CAP_PROP_XI_ACQ_FRAME_BURST_COUNT}, "JP");
            add(new int[]{Videoio.CAP_QT, Videoio.CAP_PROP_XI_DEBOUNCE_T1}, "GB");
            add(new int[]{520}, "GR");
            add(new int[]{528}, ExpandedProductParsedResult.POUND);
            add(new int[]{Videoio.CAP_PROP_XI_IMAGE_DATA_FORMAT_RGB32_ALPHA}, "CY");
            add(new int[]{Videoio.CAP_PROP_XI_TRANSPORT_PIXEL_FORMAT}, "MK");
            add(new int[]{Videoio.CAP_PROP_XI_FRAMERATE}, "MT");
            add(new int[]{Videoio.CAP_PROP_XI_AVAILABLE_BANDWIDTH}, "IE");
            add(new int[]{Videoio.CAP_PROP_XI_BUFFER_POLICY, Videoio.CAP_PROP_XI_ACQ_BUFFER_SIZE_UNIT}, "BE/LU");
            add(new int[]{Videoio.CAP_PROP_XI_HDR_KNEEPOINT_COUNT}, "PT");
            add(new int[]{569}, "IS");
            add(new int[]{570, 579}, "DK");
            add(new int[]{Videoio.CAP_PROP_XI_HOUS_BACK_SIDE_TEMP}, "PL");
            add(new int[]{Videoio.CAP_PROP_XI_FFS_FILE_ID}, "RO");
            add(new int[]{599}, "HU");
            add(new int[]{600, 601}, "ZA");
            add(new int[]{603}, "GH");
            add(new int[]{608}, "BH");
            add(new int[]{609}, "MU");
            add(new int[]{611}, "MA");
            add(new int[]{613}, "DZ");
            add(new int[]{616}, "KE");
            add(new int[]{618}, "CI");
            add(new int[]{619}, "TN");
            add(new int[]{621}, "SY");
            add(new int[]{622}, "EG");
            add(new int[]{624}, "LY");
            add(new int[]{625}, "JO");
            add(new int[]{626}, "IR");
            add(new int[]{627}, "KW");
            add(new int[]{628}, "SA");
            add(new int[]{629}, "AE");
            add(new int[]{640, 649}, "FI");
            add(new int[]{690, 695}, "CN");
            add(new int[]{Videoio.CAP_DSHOW, 709}, "NO");
            add(new int[]{729}, "IL");
            add(new int[]{730, 739}, "SE");
            add(new int[]{740}, "GT");
            add(new int[]{741}, "SV");
            add(new int[]{742}, "HN");
            add(new int[]{743}, "NI");
            add(new int[]{744}, "CR");
            add(new int[]{745}, "PA");
            add(new int[]{746}, "DO");
            add(new int[]{750}, "MX");
            add(new int[]{754, 755}, "CA");
            add(new int[]{759}, "VE");
            add(new int[]{760, 769}, "CH");
            add(new int[]{770}, "CO");
            add(new int[]{773}, "UY");
            add(new int[]{775}, "PE");
            add(new int[]{777}, "BO");
            add(new int[]{779}, "AR");
            add(new int[]{780}, "CL");
            add(new int[]{784}, "PY");
            add(new int[]{785}, "PE");
            add(new int[]{786}, "EC");
            add(new int[]{789, 790}, "BR");
            add(new int[]{Videoio.CAP_PVAPI, 839}, "IT");
            add(new int[]{840, 849}, "ES");
            add(new int[]{850}, "CU");
            add(new int[]{858}, "SK");
            add(new int[]{859}, "CZ");
            add(new int[]{860}, "YU");
            add(new int[]{865}, "MN");
            add(new int[]{867}, "KP");
            add(new int[]{868, 869}, "TR");
            add(new int[]{870, 879}, "NL");
            add(new int[]{880}, "KR");
            add(new int[]{885}, "TH");
            add(new int[]{888}, "SG");
            add(new int[]{890}, "IN");
            add(new int[]{893}, "VN");
            add(new int[]{896}, "PK");
            add(new int[]{899}, "ID");
            add(new int[]{900, 919}, "AT");
            add(new int[]{930, 939}, "AU");
            add(new int[]{940, 949}, "AZ");
            add(new int[]{955}, "MY");
            add(new int[]{958}, "MO");
        }
    }
}
