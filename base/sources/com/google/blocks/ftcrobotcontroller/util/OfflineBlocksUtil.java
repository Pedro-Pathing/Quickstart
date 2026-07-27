package com.google.blocks.ftcrobotcontroller.util;

import android.content.res.AssetManager;
import android.graphics.Color;
import android.text.Html;
import androidx.core.view.ViewCompat;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItemMap;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareUtil;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotserver.internal.webserver.AppThemeColors;

/* JADX INFO: loaded from: classes8.dex */
public class OfflineBlocksUtil {
    public static InputStream fetchOfflineBlocksEditor() throws IOException {
        String configName = HardwareUtil.getConfigurationName();
        HardwareItemMap hardwareItemMap = HardwareItemMap.newHardwareItemMap();
        AssetManager assetManager = AppUtil.getDefContext().getAssets();
        Set<String> assetsToInclude = new HashSet<>();
        assetsToInclude.add("js/split.min.js");
        assetsToInclude.add("js/split.min.js.map");
        assetsToInclude.add("blocks/images.css");
        for (String blocksImagesFile : assetManager.list("blocks/images")) {
            assetsToInclude.add("blocks/images/" + blocksImagesFile);
        }
        assetsToInclude.add("css/blocks_offline.css");
        assetsToInclude.add("css/blocks_common.css");
        assetsToInclude.add("blockly/blockly_compressed.js");
        for (String blocklyMediaFile : assetManager.list("blockly/media")) {
            assetsToInclude.add("blockly/media/" + blocklyMediaFile);
        }
        assetsToInclude.add("blockly/msg/messages.js");
        assetsToInclude.add("blockly/blocks_compressed.js");
        assetsToInclude.add("blockly/javascript_compressed.js");
        assetsToInclude.add("ftcblockly/generators/javascript.js");
        assetsToInclude.add("ftcblockly/generators/ftcjava.js");
        assetsToInclude.add("ftcblockly/generators/ftcjava/lists.js");
        assetsToInclude.add("ftcblockly/generators/ftcjava/logic.js");
        assetsToInclude.add("ftcblockly/generators/ftcjava/loops.js");
        assetsToInclude.add("ftcblockly/generators/ftcjava/math.js");
        assetsToInclude.add("ftcblockly/generators/ftcjava/procedures.js");
        assetsToInclude.add("ftcblockly/generators/ftcjava/text.js");
        assetsToInclude.add("ftcblockly/generators/ftcjava/variables.js");
        assetsToInclude.add("blocks/FtcBlocks_common.js");
        assetsToInclude.add("blocks/FtcBlocksProjects_common.js");
        assetsToInclude.add("blocks/acceleration.js");
        assetsToInclude.add("blocks/acceleration_sensor.js");
        assetsToInclude.add("blocks/analog_input.js");
        assetsToInclude.add("blocks/analog_output.js");
        assetsToInclude.add("blocks/android_accelerometer.js");
        assetsToInclude.add("blocks/android_gyroscope.js");
        assetsToInclude.add("blocks/android_orientation.js");
        assetsToInclude.add("blocks/android_sound_pool.js");
        assetsToInclude.add("blocks/android_text_to_speech.js");
        assetsToInclude.add("blocks/andymark_imu_orientation_on_robot.js");
        assetsToInclude.add("blocks/andy_mark_color_sensor.js");
        assetsToInclude.add("blocks/angular_velocity.js");
        assetsToInclude.add("blocks/april_tag.js");
        assetsToInclude.add("blocks/blackboard.js");
        assetsToInclude.add("blocks/bno055imu.js");
        assetsToInclude.add("blocks/bno055imu_parameters.js");
        assetsToInclude.add("blocks/clipboard_util.js");
        assetsToInclude.add("blocks/color.js");
        assetsToInclude.add("blocks/color_blob_locator.js");
        assetsToInclude.add("blocks/color_range_sensor.js");
        assetsToInclude.add("blocks/color_sensor.js");
        assetsToInclude.add("blocks/compass_sensor.js");
        assetsToInclude.add("blocks/cr_servo.js");
        assetsToInclude.add("blocks/dbg_log.js");
        assetsToInclude.add("blocks/dc_motor.js");
        assetsToInclude.add("blocks/digital_channel.js");
        assetsToInclude.add("blocks/distance_sensor.js");
        assetsToInclude.add("blocks/elapsed_time2.js");
        assetsToInclude.add("blocks/exposure_control.js");
        assetsToInclude.add("blocks/focus_control.js");
        assetsToInclude.add("blocks/gain_control.js");
        assetsToInclude.add("blocks/gamepad.js");
        assetsToInclude.add("blocks/gobilda_pinpoint.js");
        assetsToInclude.add("blocks/gyro_sensor.js");
        assetsToInclude.add("blocks/hardware_util.js");
        assetsToInclude.add("blocks/husky_lens.js");
        assetsToInclude.add("blocks/imu.js");
        assetsToInclude.add("blocks/imu_parameters.js");
        assetsToInclude.add("blocks/ir_seeker_sensor.js");
        assetsToInclude.add("blocks/led.js");
        assetsToInclude.add("blocks/led_effect.js");
        assetsToInclude.add("blocks/light_sensor.js");
        assetsToInclude.add("blocks/limelight_3a.js");
        assetsToInclude.add("blocks/linear_op_mode.js");
        assetsToInclude.add("blocks/locale.js");
        assetsToInclude.add("blocks/magnetic_flux.js");
        assetsToInclude.add("blocks/matrix_f.js");
        assetsToInclude.add("blocks/max_sonar_i2cxl.js");
        assetsToInclude.add("blocks/misc.js");
        assetsToInclude.add("blocks/mr_i2c_compass_sensor.js");
        assetsToInclude.add("blocks/mr_i2c_range_sensor.js");
        assetsToInclude.add("blocks/navigation.js");
        assetsToInclude.add("blocks/octoquad.js");
        assetsToInclude.add("blocks/open_gl_matrix.js");
        assetsToInclude.add("blocks/opencv.js");
        assetsToInclude.add("blocks/optical_distance_sensor.js");
        assetsToInclude.add("blocks/orientation.js");
        assetsToInclude.add("blocks/pidf_coefficients.js");
        assetsToInclude.add("blocks/pose2d.js");
        assetsToInclude.add("blocks/position.js");
        assetsToInclude.add("blocks/predominant_color.js");
        assetsToInclude.add("blocks/project_util.js");
        assetsToInclude.add("blocks/ptz_control.js");
        assetsToInclude.add("blocks/quaternion.js");
        assetsToInclude.add("blocks/range.js");
        assetsToInclude.add("blocks/rev_blinkin_led_driver.js");
        assetsToInclude.add("blocks/rev_hub_orientation_on_robot.js");
        assetsToInclude.add("blocks/rumble_effect.js");
        assetsToInclude.add("blocks/servo.js");
        assetsToInclude.add("blocks/servo_controller.js");
        assetsToInclude.add("blocks/sort_order.js");
        assetsToInclude.add("blocks/sparkfun_led_stick.js");
        assetsToInclude.add("blocks/sparkfun_otos.js");
        assetsToInclude.add("blocks/system.js");
        assetsToInclude.add("blocks/telemetry.js");
        assetsToInclude.add("blocks/temperature.js");
        assetsToInclude.add("blocks/toolbox_util.js");
        assetsToInclude.add("blocks/touch_sensor.js");
        assetsToInclude.add("blocks/ultrasonic_sensor.js");
        assetsToInclude.add("blocks/vars.js");
        assetsToInclude.add("blocks/vector_f.js");
        assetsToInclude.add("blocks/velocity.js");
        assetsToInclude.add("blocks/vision_portal.js");
        assetsToInclude.add("blocks/voltage_sensor.js");
        assetsToInclude.add("blocks/white_balance_control.js");
        assetsToInclude.add("blocks/yaw_pitch_roll_angles.js");
        assetsToInclude.add("FtcOfflineBlocksProjects.html");
        assetsToInclude.add("FtcOfflineBlocks.html");
        assetsToInclude.add("favicon.ico");
        assetsToInclude.add("blocks/obsolete/elapsed_time.js");
        assetsToInclude.add("blocks/obsolete/obsolete.js");
        assetsToInclude.add("blocks/obsolete/tensor_flow.js");
        assetsToInclude.add("blocks/obsolete/tfod.js");
        assetsToInclude.add("blocks/obsolete/tfod_base.js");
        assetsToInclude.add("blocks/obsolete/tfod_current_game.js");
        assetsToInclude.add("blocks/obsolete/tfod_custom_model.js");
        assetsToInclude.add("blocks/obsolete/tfod_recognition.js");
        assetsToInclude.add("blocks/obsolete/tfod_rover_ruckus.js");
        assetsToInclude.add("blocks/obsolete/tfod_sky_stone.js");
        assetsToInclude.add("blocks/obsolete/vuforia.js");
        assetsToInclude.add("blocks/obsolete/vuforia_current_game.js");
        assetsToInclude.add("blocks/obsolete/vuforia_localizer.js");
        assetsToInclude.add("blocks/obsolete/vuforia_localizer_parameters.js");
        assetsToInclude.add("blocks/obsolete/vuforia_relic_recovery.js");
        assetsToInclude.add("blocks/obsolete/vuforia_rover_ruckus.js");
        assetsToInclude.add("blocks/obsolete/vuforia_sky_stone.js");
        assetsToInclude.add("blocks/obsolete/vuforia_trackable.js");
        assetsToInclude.add("blocks/obsolete/vuforia_trackable_default_listener.js");
        assetsToInclude.add("blocks/obsolete/vuforia_trackables.js");
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        ZipOutputStream zos = new ZipOutputStream(baos);
        try {
            zos.putNextEntry(new ZipEntry("index.html"));
            copyAsset(assetManager, "FtcOfflineFrame.html", zos);
            zos.closeEntry();
            zos.putNextEntry(new ZipEntry("css/blocks_common_less.css"));
            zos.write(convertLessToCss(assetManager, "css/blocks_common.less").getBytes());
            zos.closeEntry();
            zos.putNextEntry(new ZipEntry("css/frame_offline_less.css"));
            zos.write(convertLessToCss(assetManager, "css/frame_offline.less").getBytes());
            zos.closeEntry();
            zos.putNextEntry(new ZipEntry("js/FtcOfflineBlocks.js"));
            zos.write(getFtcOfflineBlocksJs(configName, hardwareItemMap).getBytes());
            zos.closeEntry();
            for (String assetPath : assetsToInclude) {
                zos.putNextEntry(new ZipEntry(assetPath));
                copyAsset(assetManager, assetPath, zos);
                zos.closeEntry();
            }
            zos.close();
            return new ByteArrayInputStream(baos.toByteArray());
        } catch (Throwable th) {
            try {
                zos.close();
            } catch (Throwable th2) {
                th.addSuppressed(th2);
            }
            throw th;
        }
    }

    private static void copyAsset(AssetManager assetManager, String assetPath, OutputStream outputStream) throws IOException {
        InputStream inputStream = assetManager.open(assetPath);
        try {
            byte[] buffer = new byte[4096];
            while (true) {
                int n = inputStream.read(buffer);
                if (n <= 0) {
                    break;
                } else {
                    outputStream.write(buffer, 0, n);
                }
            }
            if (inputStream != null) {
                inputStream.close();
            }
        } catch (Throwable th) {
            if (inputStream != null) {
                try {
                    inputStream.close();
                } catch (Throwable th2) {
                    th.addSuppressed(th2);
                }
            }
            throw th;
        }
    }

    private static String convertLessToCss(AssetManager assetManager, String assetPath) throws IOException {
        AppThemeColors colors = AppThemeColors.fromTheme();
        StringBuilder cssStringBuilder = new StringBuilder();
        FileUtil.readAsset(cssStringBuilder, assetManager, assetPath);
        float[] hsvTextBright = new float[3];
        Color.colorToHSV(colors.textBright, hsvTextBright);
        return cssStringBuilder.toString().replace("@import \"/css/core.less\";", "").replace("hue(@textBright)", String.format("%d", Integer.valueOf(Math.round(hsvTextBright[0])))).replace("saturation(@textBright)", String.format("%d%%", Integer.valueOf(Math.round(hsvTextBright[1] * 100.0f)))).replace("darken(@backgroundMedium, 5%)", String.format("#%06x", Integer.valueOf(darken(colors.backgroundMedium, 0.05f) & ViewCompat.MEASURED_SIZE_MASK))).replace("@textError", String.format("#%06x", Integer.valueOf(colors.textError & ViewCompat.MEASURED_SIZE_MASK))).replace("@textWarning", String.format("#%06x", Integer.valueOf(colors.textWarning & ViewCompat.MEASURED_SIZE_MASK))).replace("@textOkay", String.format("#%06x", Integer.valueOf(colors.textOkay & ViewCompat.MEASURED_SIZE_MASK))).replace("@textBright", String.format("#%06x", Integer.valueOf(colors.textBright & ViewCompat.MEASURED_SIZE_MASK))).replace("@textLight", String.format("#%06x", Integer.valueOf(colors.textLight & ViewCompat.MEASURED_SIZE_MASK))).replace("@textMediumDark", String.format("#%06x", Integer.valueOf(colors.textMediumDark & ViewCompat.MEASURED_SIZE_MASK))).replace("@textMedium", String.format("#%06x", Integer.valueOf(colors.textMedium & ViewCompat.MEASURED_SIZE_MASK))).replace("@textVeryDark", String.format("#%06x", Integer.valueOf(colors.textVeryDark & ViewCompat.MEASURED_SIZE_MASK))).replace("@textVeryVeryDark", String.format("#%06x", Integer.valueOf(colors.textVeryVeryDark & ViewCompat.MEASURED_SIZE_MASK))).replace("@backgroundLight", String.format("#%06x", Integer.valueOf(colors.backgroundLight & ViewCompat.MEASURED_SIZE_MASK))).replace("@backgroundMediumLight", String.format("#%06x", Integer.valueOf(colors.backgroundMediumLight & ViewCompat.MEASURED_SIZE_MASK))).replace("@backgroundMediumMedium", String.format("#%06x", Integer.valueOf(colors.backgroundMediumMedium & ViewCompat.MEASURED_SIZE_MASK))).replace("@backgroundMediumDark", String.format("#%06x", Integer.valueOf(colors.backgroundMediumDark & ViewCompat.MEASURED_SIZE_MASK))).replace("@backgroundMedium", String.format("#%06x", Integer.valueOf(colors.backgroundMedium & ViewCompat.MEASURED_SIZE_MASK))).replace("@backgroundAlmostDark", String.format("#%06x", Integer.valueOf(colors.backgroundAlmostDark & ViewCompat.MEASURED_SIZE_MASK))).replace("@backgroundDark", String.format("#%06x", Integer.valueOf(colors.backgroundDark & ViewCompat.MEASURED_SIZE_MASK))).replace("@backgroundVeryDark", String.format("#%06x", Integer.valueOf(colors.backgroundVeryDark & ViewCompat.MEASURED_SIZE_MASK))).replace("@backgroundVeryVeryDark", String.format("#%06x", Integer.valueOf(colors.backgroundVeryVeryDark & ViewCompat.MEASURED_SIZE_MASK))).replace("@lineBright", String.format("#%06x", Integer.valueOf(colors.lineBright & ViewCompat.MEASURED_SIZE_MASK))).replace("@lineLight", String.format("#%06x", Integer.valueOf(colors.lineLight & ViewCompat.MEASURED_SIZE_MASK))).replace("@feedbackBackground", String.format("#%06x", Integer.valueOf(colors.feedbackBackground & ViewCompat.MEASURED_SIZE_MASK))).replace("@feedbackBorder", String.format("#%06x", Integer.valueOf(colors.feedbackBorder & ViewCompat.MEASURED_SIZE_MASK)));
    }

    private static int darken(int color, float amount) {
        float[] hsl = new float[3];
        colorToHSL(color, hsl);
        hsl[2] = hsl[2] - amount;
        return hslToColor(hsl);
    }

    private static void colorToHSL(int color, float[] hsl) {
        float f;
        int r255 = Color.red(color);
        int g255 = Color.green(color);
        int b255 = Color.blue(color);
        int max255 = Math.max(Math.max(r255, g255), b255);
        int min255 = Math.min(Math.min(r255, g255), b255);
        float max = max255 / 255.0f;
        float min = min255 / 255.0f;
        hsl[2] = (max + min) / 2.0f;
        if (max255 != min255) {
            if (hsl[2] > 0.5d) {
                f = (max - min) / ((2.0f - max) - min);
            } else {
                f = (max - min) / (max + min);
            }
            hsl[1] = f;
            hsl[0] = hue(r255, g255, b255);
            return;
        }
        hsl[1] = 0.0f;
        hsl[0] = 0.0f;
    }

    private static int hslToColor(float[] hsl) {
        float b;
        float q;
        float p;
        if (hsl[1] == 0.0f) {
            q = hsl[2];
            b = q;
            p = q;
        } else {
            float b2 = hsl[2];
            float q2 = b2 < 0.5f ? hsl[2] * (hsl[1] + 1.0f) : (hsl[2] + hsl[1]) - (hsl[2] * hsl[1]);
            float p2 = (hsl[2] * 2.0f) - q2;
            float r = hue2rgb(p2, q2, hsl[0] + 0.33333334f);
            float g = hue2rgb(p2, q2, hsl[0]);
            b = hue2rgb(p2, q2, hsl[0] - 0.33333334f);
            q = r;
            p = g;
        }
        int r255 = Math.round(q * 255.0f);
        int g255 = Math.round(p * 255.0f);
        int b255 = Math.round(255.0f * b);
        return Color.rgb(r255, g255, b255);
    }

    private static float hue2rgb(float p, float q, float t) {
        if (t < 0.0f) {
            t += 1.0f;
        }
        if (t > 1.0f) {
            t -= 1.0f;
        }
        if (t < 0.16666667f) {
            return ((q - p) * 6.0f * t) + p;
        }
        if (t < 0.5f) {
            return q;
        }
        if (t < 0.6666667f) {
            return ((q - p) * (0.6666667f - t) * 6.0f) + p;
        }
        return p;
    }

    private static float hue(int r255, int g255, int b255) {
        float[] hsv = new float[3];
        Color.RGBToHSV(r255, g255, b255, hsv);
        return hsv[0] / 360.0f;
    }

    private static String getFtcOfflineBlocksJs(String configName, HardwareItemMap hardwareItemMap) throws Throwable {
        StringBuilder jsStringBuilder = new StringBuilder();
        jsStringBuilder.append("function getBlkFiles() {\n").append("  var BLK_FILES = [\n");
        List<OfflineBlocksProject> offlineBlocksProjects = new ArrayList<>();
        ProjectsUtil.fetchProjectsForOfflineBlocksEditor(offlineBlocksProjects);
        String delimiter = "";
        for (OfflineBlocksProject offlineBlocksProject : offlineBlocksProjects) {
            jsStringBuilder.append(delimiter).append("    {\n").append("      'FileName': '").append(ProjectsUtil.escapeSingleQuotes(offlineBlocksProject.fileName)).append("',\n").append("      'Content': '").append(ProjectsUtil.escapeSingleQuotes(offlineBlocksProject.content)).append("',\n").append("      'name': '").append(ProjectsUtil.escapeSingleQuotes(offlineBlocksProject.name)).append("',\n").append("      'escapedName' : '").append(ProjectsUtil.escapeSingleQuotes(Html.escapeHtml(offlineBlocksProject.name))).append("',\n").append("      'dateModifiedMillis': ").append(offlineBlocksProject.dateModifiedMillis).append(",\n").append("      'enabled': ").append(offlineBlocksProject.enabled).append("\n").append("    }");
            delimiter = ",\n";
        }
        jsStringBuilder.append("\n").append("  ];\n").append("  return BLK_FILES;\n").append("}\n\n").append("function getOfflineConfigurationName() {\n").append("  return '").append(ProjectsUtil.escapeSingleQuotes(configName)).append("';\n").append("}\n\n").append("function getSampleNamesJson() {\n").append("  var SAMPLE_NAMES = '").append(ProjectsUtil.fetchSampleNames()).append("';\n").append("  return SAMPLE_NAMES;\n").append("}\n\n").append("function getSampleBlkFileContent(sampleName) {\n").append("  switch (sampleName) {\n");
        for (Map.Entry<String, String> entry : ProjectsUtil.getSamples(hardwareItemMap).entrySet()) {
            String sampleName = entry.getKey();
            String blkFileContent = entry.getValue().replace("\n", " ").replaceAll("\\> +\\<", "><");
            if (sampleName.isEmpty()) {
                jsStringBuilder.append("    default:\n");
            }
            jsStringBuilder.append("    case '").append(ProjectsUtil.escapeSingleQuotes(sampleName)).append("':\n").append("      return '").append(ProjectsUtil.escapeSingleQuotes(blkFileContent)).append("';\n");
        }
        jsStringBuilder.append("  }\n").append("}\n").append(HardwareUtil.fetchJavaScriptForHardware(hardwareItemMap));
        return jsStringBuilder.toString();
    }
}
