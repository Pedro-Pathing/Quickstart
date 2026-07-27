package com.google.zxing.maxicode.decoder;

import com.google.zxing.common.BitMatrix;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.sun.tools.javac.jvm.ByteCodes;
import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.Videoio;

/* JADX INFO: loaded from: classes8.dex */
final class BitMatrixParser {
    private static final int[][] BITNR;
    private final BitMatrix bitMatrix;

    static {
        int[] iArr = new int[30];
        // fill-array-data instruction
        iArr[0] = 419;
        iArr[1] = 418;
        iArr[2] = 425;
        iArr[3] = 424;
        iArr[4] = 431;
        iArr[5] = 430;
        iArr[6] = 107;
        iArr[7] = 106;
        iArr[8] = 59;
        iArr[9] = 58;
        iArr[10] = -3;
        iArr[11] = -3;
        iArr[12] = -3;
        iArr[13] = -3;
        iArr[14] = -3;
        iArr[15] = -3;
        iArr[16] = -3;
        iArr[17] = -3;
        iArr[18] = -3;
        iArr[19] = 23;
        iArr[20] = 89;
        iArr[21] = 88;
        iArr[22] = 437;
        iArr[23] = 436;
        iArr[24] = 443;
        iArr[25] = 442;
        iArr[26] = 449;
        iArr[27] = 448;
        iArr[28] = 836;
        iArr[29] = 835;
        BITNR = new int[][]{new int[]{121, 120, 127, 126, 133, 132, 139, 138, 145, 144, 151, 150, ByteCodes.ifgt, ByteCodes.ifge, ByteCodes.if_icmpgt, ByteCodes.if_icmpge, ByteCodes.ret, 168, ByteCodes.dreturn, ByteCodes.freturn, ByteCodes.putfield, ByteCodes.getfield, ByteCodes.new_, ByteCodes.invokedynamic, ByteCodes.instanceof_, ByteCodes.checkcast, ByteCodes.if_acmp_nonnull, ByteCodes.if_acmp_null, -2, -2}, new int[]{123, 122, 129, 128, 135, 134, 141, 140, 147, 146, 153, 152, ByteCodes.if_icmpeq, ByteCodes.ifle, ByteCodes.if_acmpeq, ByteCodes.if_icmple, ByteCodes.lookupswitch, ByteCodes.tableswitch, ByteCodes.return_, ByteCodes.areturn, ByteCodes.invokespecial, ByteCodes.invokevirtual, ByteCodes.anewarray, ByteCodes.newarray, ByteCodes.monitorexit, ByteCodes.monitorenter, ByteCodes.jsr_w, 200, 816, -3}, new int[]{125, 124, 131, 130, 137, 136, 143, 142, 149, 148, 155, 154, ByteCodes.if_icmplt, ByteCodes.if_icmpne, ByteCodes.goto_, ByteCodes.if_acmpne, 173, ByteCodes.ireturn, ByteCodes.putstatic, ByteCodes.getstatic, ByteCodes.invokeinterface, ByteCodes.invokestatic, ByteCodes.athrow, ByteCodes.arraylength, ByteCodes.multianewarray, ByteCodes.wide, ByteCodes.ByteCodeCount, ByteCodes.breakpoint, 818, 817}, new int[]{283, 282, ByteCodes.error, ByteCodes.nullchk, ByteCodes.lshll, ByteCodes.ishll, 265, 264, 259, 258, 253, 252, 247, 246, 241, 240, 235, 234, 229, 228, 223, 222, 217, 216, 211, 210, 205, 204, 819, -3}, new int[]{285, 284, 279, Imgcodecs.IMWRITE_TIFF_ROWSPERSTRIP, ByteCodes.lshrl, 272, 267, 266, 261, 260, 255, 254, 249, 248, 243, 242, 237, 236, 231, 230, 225, UsbConstants.USB_CLASS_WIRELESS_CONTROLLER, 219, 218, 213, 212, 207, 206, 821, 820}, new int[]{287, 286, 281, 280, ByteCodes.lushrl, ByteCodes.iushrl, 269, 268, 263, 262, 257, 256, 251, SyncdDevice.msAbnormalReopenInterval, 245, 244, UsbConstants.USB_CLASS_MISC, 238, 233, 232, 227, 226, 221, 220, 215, 214, 209, 208, 822, -3}, new int[]{289, 288, 295, 294, Videoio.CAP_PROP_PVAPI_FRAMESTARTTRIGGERMODE, 300, 307, Videoio.CAP_PROP_PVAPI_PIXELFORMAT, 313, 312, 319, 318, 325, 324, 331, 330, 337, 336, 343, 342, 349, 348, 355, 354, 361, 360, 367, 366, 824, 823}, new int[]{291, 290, 297, 296, Videoio.CAP_PROP_PVAPI_DECIMATIONVERTICAL, Videoio.CAP_PROP_PVAPI_DECIMATIONHORIZONTAL, 309, 308, 315, 314, 321, 320, 327, 326, 333, 332, 339, 338, 345, 344, 351, 350, 357, 356, 363, 362, 369, 368, 825, -3}, new int[]{293, 292, 299, 298, Videoio.CAP_PROP_PVAPI_BINNINGY, Videoio.CAP_PROP_PVAPI_BINNINGX, 311, 310, Imgcodecs.IMWRITE_TIFF_PREDICTOR, 316, 323, 322, 329, 328, 335, 334, 341, 340, 347, 346, 353, 352, 359, 358, 365, 364, 371, 370, 827, 826}, new int[]{Videoio.CAP_PROP_XI_GPO_SELECTOR, Videoio.CAP_PROP_XI_GPI_LEVEL, Videoio.CAP_PROP_XI_OFFSET_Y, Videoio.CAP_PROP_XI_OFFSET_X, 397, 396, 391, 390, 79, 78, -2, -2, 13, 12, 37, 36, 2, -1, 44, 43, 109, 108, 385, 384, 379, 378, 373, 372, 828, -3}, new int[]{Videoio.CAP_PROP_XI_LED_SELECTOR, Videoio.CAP_PROP_XI_GPO_MODE, Videoio.CAP_PROP_XI_TRG_SOFTWARE, Videoio.CAP_PROP_XI_TRG_SOURCE, 399, 398, 393, 392, 81, 80, 40, -2, 15, 14, 39, 38, 3, -1, -1, 45, 111, 110, 387, 386, 381, 380, 375, 374, 830, 829}, new int[]{Videoio.CAP_PROP_XI_MANUAL_WB, Videoio.CAP_PROP_XI_LED_MODE, Videoio.CAP_PROP_XI_GPI_MODE, Videoio.CAP_PROP_XI_GPI_SELECTOR, Videoio.CAP_PROP_XI_DATA_FORMAT, 400, 395, 394, 83, 82, 41, -3, -3, -3, -3, -3, 5, 4, 47, 46, ByteCodes.lmod, 112, 389, 388, 383, 382, 377, 376, 831, -3}, new int[]{Videoio.CAP_PROP_XI_AEAG, Videoio.CAP_PROP_XI_AUTO_WB, Videoio.CAP_PROP_XI_EXPOSURE, Videoio.CAP_PROP_XI_TIMEOUT, Videoio.CAP_PROP_XI_BINNING_SELECTOR, Videoio.CAP_PROP_XI_DOWNSAMPLING_TYPE, 103, 102, 55, 54, 16, -3, -3, -3, -3, -3, -3, -3, 20, 19, 85, 84, Videoio.CAP_PROP_XI_DECIMATION_HORIZONTAL, Videoio.CAP_PROP_XI_DECIMATION_VERTICAL, Videoio.CAP_PROP_XI_AEAG_ROI_OFFSET_X, 438, Videoio.CAP_PROP_XI_BPC, 444, 833, 832}, new int[]{Videoio.CAP_PROP_XI_AE_MAX_LIMIT, Videoio.CAP_PROP_XI_EXP_PRIORITY, Videoio.CAP_PROP_XI_GAIN_SELECTOR, Videoio.CAP_PROP_XI_EXPOSURE_BURST_COUNT, Videoio.CAP_PROP_XI_BINNING_HORIZONTAL, Videoio.CAP_PROP_XI_BINNING_VERTICAL, 105, 104, 57, 56, -3, -3, -3, -3, -3, -3, -3, -3, 22, 21, 87, 86, Videoio.CAP_PROP_XI_IMAGE_DATA_FORMAT, Videoio.CAP_PROP_XI_DECIMATION_PATTERN, Videoio.CAP_PROP_XI_AEAG_ROI_WIDTH, Videoio.CAP_PROP_XI_AEAG_ROI_OFFSET_Y, 447, 446, 834, -3}, iArr, new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_02, Videoio.CAP_PROP_XI_CC_MATRIX_01, Videoio.CAP_PROP_XI_COLOR_FILTER_ARRAY, Videoio.CAP_PROP_XI_IMAGE_IS_COLOR, Videoio.CAP_PROP_XI_HOUS_TEMP, Videoio.CAP_PROP_XI_CHIP_TEMP, 48, -2, 30, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, 0, 53, 52, Videoio.CAP_PROP_XI_OUTPUT_DATA_PACKING, Videoio.CAP_PROP_XI_IMAGE_DATA_BIT_DEPTH, 457, 456, Videoio.CAP_PROP_XI_WIDTH, Videoio.CAP_PROP_XI_WB_KB, 837, -3}, new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_10, Videoio.CAP_PROP_XI_CC_MATRIX_03, Videoio.CAP_PROP_XI_GAMMAC, Videoio.CAP_PROP_XI_GAMMAY, Videoio.CAP_PROP_XI_APPLY_CMS, Videoio.CAP_PROP_XI_CMS, 49, -1, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2, -1, Videoio.CAP_PROP_XI_IS_COOLED, Videoio.CAP_PROP_XI_OUTPUT_DATA_PACKING_TYPE, Videoio.CAP_PROP_XI_LIMIT_BANDWIDTH, 458, 453, Videoio.CAP_PROP_XI_HEIGHT, 839, 838}, new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_12, Videoio.CAP_PROP_XI_CC_MATRIX_11, Videoio.CAP_PROP_XI_CC_MATRIX_00, Videoio.CAP_PROP_XI_SHARPNESS, 473, 472, 51, 50, 31, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, 1, -2, 42, Videoio.CAP_PROP_XI_TARGET_TEMP, Videoio.CAP_PROP_XI_COOLING, Videoio.CAP_PROP_XI_OUTPUT_DATA_BIT_DEPTH, Videoio.CAP_PROP_XI_SENSOR_DATA_BIT_DEPTH, 455, 454, 840, -3}, new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_20, Videoio.CAP_PROP_XI_CC_MATRIX_13, Videoio.CAP_PROP_XI_CC_MATRIX_32, Videoio.CAP_PROP_XI_CC_MATRIX_31, Videoio.CAP_PROP_XI_ACQ_FRAME_BURST_COUNT, Videoio.CAP_PROP_XI_TRG_SELECTOR, 97, 96, 61, 60, -3, -3, -3, -3, -3, -3, -3, -3, -3, 26, 91, 90, 505, 504, 511, Videoio.CAP_PROP_XI_DEBOUNCE_POL, Videoio.CAP_PROP_XI_LENS_FEATURE_SELECTOR, Videoio.CAP_PROP_XI_LENS_FOCAL_LENGTH, 842, 841}, new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_22, Videoio.CAP_PROP_XI_CC_MATRIX_21, Videoio.CAP_PROP_XI_DEFAULT_CC_MATRIX, Videoio.CAP_PROP_XI_CC_MATRIX_33, 501, Videoio.CAP_QT, 99, 98, 63, 62, -3, -3, -3, -3, -3, -3, -3, -3, 28, 27, 93, 92, Videoio.CAP_PROP_XI_DEBOUNCE_EN, 506, 513, 512, 519, Videoio.CAP_PROP_XI_LENS_FEATURE, 843, -3}, new int[]{Videoio.CAP_PROP_XI_CC_MATRIX_30, Videoio.CAP_PROP_XI_CC_MATRIX_23, 497, 496, 503, 502, 101, 100, 65, 64, 17, -3, -3, -3, -3, -3, -3, -3, 18, 29, 95, 94, Videoio.CAP_PROP_XI_DEBOUNCE_T1, Videoio.CAP_PROP_XI_DEBOUNCE_T0, Videoio.CAP_PROP_XI_LENS_FOCUS_DISTANCE, 514, Videoio.CAP_PROP_XI_DEVICE_MODEL_ID, 520, 845, 844}, new int[]{Videoio.CAP_PROP_XI_HDR, Videoio.CAP_PROP_XI_SENSOR_MODE, Videoio.CAP_PROP_XI_RECENT_FRAME, Videoio.CAP_PROP_XI_ACQ_TRANSPORT_BUFFER_COMMIT, Videoio.CAP_PROP_XI_IS_DEVICE_EXIST, Videoio.CAP_PROP_XI_TS_RST_SOURCE, Videoio.CAP_PROP_XI_LUT_EN, Videoio.CAP_PROP_XI_BUFFER_POLICY, 73, 72, 32, -3, -3, -3, -3, -3, -3, 10, 67, 66, 115, ByteCodes.fmod, Videoio.CAP_PROP_XI_FRAMERATE, Videoio.CAP_PROP_XI_SENSOR_OUTPUT_CHANNEL_COUNT, Videoio.CAP_PROP_XI_IMAGE_DATA_FORMAT_RGB32_ALPHA, 528, 523, Videoio.CAP_PROP_XI_DEVICE_SN, 846, -3}, new int[]{Videoio.CAP_PROP_XI_HDR_T1, Videoio.CAP_PROP_XI_HDR_KNEEPOINT_COUNT, Videoio.CAP_PROP_XI_COLUMN_FPN_CORRECTION, Videoio.CAP_PROP_XI_DEVICE_RESET, Videoio.CAP_PROP_XI_ACQ_BUFFER_SIZE_UNIT, Videoio.CAP_PROP_XI_ACQ_BUFFER_SIZE, Videoio.CAP_PROP_XI_LUT_VALUE, Videoio.CAP_PROP_XI_LUT_INDEX, 75, 74, -2, -1, 7, 6, 35, 34, 11, -2, 69, 68, 117, 116, Videoio.CAP_PROP_XI_COUNTER_VALUE, Videoio.CAP_PROP_XI_COUNTER_SELECTOR, Videoio.CAP_PROP_XI_TRANSPORT_PIXEL_FORMAT, Videoio.CAP_PROP_XI_IMAGE_PAYLOAD_SIZE, 525, 524, 848, 847}, new int[]{Videoio.CAP_PROP_XI_KNEEPOINT1, Videoio.CAP_PROP_XI_HDR_T2, 557, 556, Videoio.CAP_PROP_XI_BUFFERS_QUEUE_SIZE, Videoio.CAP_PROP_XI_ACQ_TRANSPORT_BUFFER_SIZE, Videoio.CAP_PROP_XI_TS_RST_MODE, 544, 77, 76, -2, 33, 9, 8, 25, 24, -1, -2, 71, 70, 119, 118, Videoio.CAP_PROP_XI_AVAILABLE_BANDWIDTH, Videoio.CAP_PROP_XI_ACQ_TIMING_MODE, Videoio.CAP_PROP_XI_SENSOR_CLOCK_FREQ_INDEX, Videoio.CAP_PROP_XI_SENSOR_CLOCK_FREQ_HZ, 527, 526, 849, -3}, new int[]{Videoio.CAP_PROP_XI_IMAGE_BLACK_LEVEL, Videoio.CAP_PROP_XI_KNEEPOINT2, Videoio.CAP_PROP_XI_HW_REVISION, 570, 577, 576, Videoio.CAP_PROP_XI_FFS_ACCESS_KEY, Videoio.CAP_PROP_XI_USED_FFS_SIZE, Videoio.CAP_PROP_XI_REGION_SELECTOR, Videoio.CAP_PROP_XI_TEST_PATTERN, Videoio.CAP_PROP_XI_REGION_MODE, Videoio.CAP_PROP_XI_FFS_FILE_ID, 601, 600, 607, 606, 613, 612, 619, 618, 625, 624, 631, 630, 637, 636, 643, 642, 851, 850}, new int[]{567, 566, Videoio.CAP_PROP_XI_AUTO_BANDWIDTH_CALCULATION, Videoio.CAP_PROP_XI_DEBUG_LEVEL, 579, 578, Videoio.CAP_PROP_XI_SENSOR_FEATURE_SELECTOR, 584, Videoio.CAP_PROP_XI_ROW_FPN_CORRECTION, Videoio.CAP_PROP_XI_HOUS_BACK_SIDE_TEMP, 597, Videoio.CAP_PROP_XI_SENSOR_BOARD_TEMP, 603, 602, 609, 608, 615, 614, 621, 620, 627, 626, 633, 632, 639, 638, 645, 644, 852, -3}, new int[]{569, 568, 575, 574, Videoio.CAP_PROP_XI_FREE_FFS_SIZE, Videoio.CAP_PROP_XI_FFS_FILE_SIZE, Videoio.CAP_PROP_XI_TEST_PATTERN_GENERATOR_SELECTOR, Videoio.CAP_PROP_XI_SENSOR_FEATURE_VALUE, 593, 592, 599, 598, 605, 604, 611, 610, 617, 616, 623, 622, 629, 628, 635, 634, 641, 640, 647, 646, 854, 853}, new int[]{727, 726, 721, 720, 715, 714, 709, 708, 703, 702, 697, 696, 691, 690, 685, 684, 679, 678, 673, 672, 667, 666, 661, 660, 655, UsbConstants.PRODUCT_ID_MICROSOFT_XBOX360_WIRED, 649, 648, 855, -3}, new int[]{729, 728, 723, 722, 717, 716, 711, 710, 705, 704, 699, 698, 693, 692, 687, 686, 681, 680, 675, 674, 669, 668, 663, 662, 657, 656, 651, 650, 857, 856}, new int[]{731, 730, 725, 724, 719, 718, 713, 712, 707, 706, 701, Videoio.CAP_DSHOW, 695, 694, 689, 688, 683, 682, 677, 676, 671, 670, 665, 664, 659, 658, 653, 652, 858, -3}, new int[]{733, 732, 739, 738, 745, 744, 751, 750, 757, 756, 763, 762, 769, 768, 775, 774, 781, 780, 787, 786, 793, 792, 799, 798, 805, 804, 811, 810, 860, 859}, new int[]{735, 734, 741, 740, 747, 746, 753, 752, 759, 758, 765, 764, 771, 770, 777, 776, 783, 782, 789, 788, 795, 794, 801, Videoio.CAP_PVAPI, 807, 806, 813, 812, 861, -3}, new int[]{737, 736, 743, 742, 749, 748, 755, 754, 761, 760, 767, 766, 773, 772, 779, 778, 785, 784, 791, 790, 797, 796, 803, 802, 809, 808, 815, 814, 863, 862}};
    }

    BitMatrixParser(BitMatrix bitMatrix) {
        this.bitMatrix = bitMatrix;
    }

    byte[] readCodewords() {
        byte[] result = new byte[144];
        int height = this.bitMatrix.getHeight();
        int width = this.bitMatrix.getWidth();
        for (int y = 0; y < height; y++) {
            int[] bitnrRow = BITNR[y];
            for (int x = 0; x < width; x++) {
                int bit = bitnrRow[x];
                if (bit >= 0 && this.bitMatrix.get(x, y)) {
                    int i = bit / 6;
                    result[i] = (byte) (result[i] | ((byte) (1 << (5 - (bit % 6)))));
                }
            }
        }
        return result;
    }
}
