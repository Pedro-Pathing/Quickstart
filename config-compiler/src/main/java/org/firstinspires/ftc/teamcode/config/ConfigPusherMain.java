package org.firstinspires.ftc.teamcode.config;

import java.io.BufferedReader;
import java.io.File;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

/**
 * Pure Java utility for pushing config.yaml to the robot via ADB.
 * Eliminates external Python interpreter dependency.
 */
public class ConfigPusherMain {

  public static void main(String[] args) {
    boolean reset = false;
    String projectDir = ".";

    for (String arg : args) {
      if ("--reset".equals(arg)) {
        reset = true;
      } else if (!arg.startsWith("-")) {
        projectDir = arg;
      }
    }

    String device = getDevice();
    if (device == null) {
      System.err.println("ERROR: No ADB device found. Is the Control Hub connected and ADB enabled?");
      System.exit(1);
    }

    if (reset) {
      resetConfig(device);
    } else {
      pushConfig(device, projectDir);
    }
  }

  private static String getDevice() {
    try {
      Process p = new ProcessBuilder("adb", "devices").start();
      BufferedReader reader = new BufferedReader(new InputStreamReader(p.getInputStream()));
      String line;
      List<String> devices = new ArrayList<>();
      while ((line = reader.readLine()) != null) {
        line = line.trim();
        if (!line.isEmpty() && !line.contains("List of devices") && line.contains("device") && !line.contains("offline")) {
          devices.add(line.split("\\s+")[0]);
        }
      }
      p.waitFor();
      if (devices.isEmpty()) return null;
      return devices.get(0);
    } catch (Exception e) {
      return null;
    }
  }

  private static void pushConfig(String device, String projectDir) {
    System.out.println("Pushing config.yaml → robot (" + device + ")");
    File localYaml = new File(projectDir, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml");
    if (!localYaml.exists()) {
      System.err.println("ERROR: Could not find config.yaml at " + localYaml.getAbsolutePath());
      System.exit(1);
    }
    // A file the robot cannot parse would be ignored there; say so now instead.
    try (java.io.InputStream in = new java.io.FileInputStream(localYaml)) {
      Object parsed = new org.yaml.snakeyaml.Yaml().load(in);
      if (!(parsed instanceof java.util.Map)) {
        System.err.println("ERROR: config.yaml is not a YAML mapping; not pushed.");
        System.exit(1);
      }
    } catch (Exception e) {
      System.err.println("ERROR: config.yaml is not valid YAML; not pushed.\n" + e.getMessage());
      System.exit(1);
    }

    exec("adb", "-s", device, "shell", "mkdir", "-p", "/sdcard/FIRST/teamcode", "/sdcard/FIRST");
    exec("adb", "-s", device, "push", localYaml.getAbsolutePath(), "/sdcard/FIRST/teamcode/config.yaml.tmp");
    exec("adb", "-s", device, "shell", "mv", "/sdcard/FIRST/teamcode/config.yaml.tmp", "/sdcard/FIRST/teamcode/config.yaml");
    exec("adb", "-s", device, "push", localYaml.getAbsolutePath(), "/sdcard/FIRST/config.yaml.tmp");
    exec("adb", "-s", device, "shell", "mv", "/sdcard/FIRST/config.yaml.tmp", "/sdcard/FIRST/config.yaml");
    System.out.println("✓ Pushed config.yaml atomically to robot (/sdcard/FIRST/teamcode/config.yaml)");
    System.out.println("  Re-init your OpMode on the robot to pick up the new values.");
    System.out.println(
        "  If INIT shows 'Pushed config ignored', read the reason: a key added since the last"
            + " deploy needs a rebuild and install first.");
  }

  private static void resetConfig(String device) {
    System.out.println("Removing config override from robot (" + device + ")");
    exec("adb", "-s", device, "shell", "rm", "-f", "/sdcard/FIRST/teamcode/config.yaml", "/sdcard/FIRST/config.yaml");
    System.out.println("✓ Removed override files — robot will use bundled config on next init.");
  }

  /** Runs one adb step; any failure stops the push with adb's own output, never a false ✓. */
  private static void exec(String... command) {
    try {
      Process p = new ProcessBuilder(command).redirectErrorStream(true).start();
      String output = new String(p.getInputStream().readAllBytes()).trim();
      int code = p.waitFor();
      // `adb shell` reports some remote failures only in its output.
      boolean failed =
          code != 0
              || output.contains("No such file")
              || output.contains("Permission denied")
              || output.contains("error:");
      if (failed) {
        System.err.println("ERROR: " + String.join(" ", command) + " failed (exit " + code + ")");
        if (!output.isEmpty()) {
          System.err.println(output);
        }
        System.exit(1);
      }
    } catch (Exception e) {
      System.err.println("ERROR: could not run " + String.join(" ", command) + ": " + e.getMessage());
      System.exit(1);
    }
  }
}
