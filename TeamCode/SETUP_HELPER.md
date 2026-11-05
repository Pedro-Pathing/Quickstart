# Engineering Workspace Setup Guide (v2.5)

This guide walks you through setting up your local development environment for the **decode** FTC Robot Controller codebase.

---

## 1. System Requirements

| Tool | Required Version | Notes |
|---|---|---|
| **JDK** | Java 21 | Configured automatically via the Foojay toolchain resolver |
| **Gradle** | 9.6.1+ | Invoked via the wrapper script `./gradlew` — do not install separately |
| **Android Studio** | Hedgehog (2023.1.1)+ | Required for deploying to the robot Control Hub |
| **Android SDK** | API 30+ | Installed through Android Studio SDK Manager |
| **Git** | Any recent version | For cloning and version control |

> **Note:** You do not need to install Gradle manually. The `./gradlew` wrapper downloads and caches the correct version automatically.

---

## 2. Clone & Initial Setup

```bash
git clone <repo-url> decode
cd decode
```

**Then in Android Studio:**
1. `File → Open` and select the `decode/` directory.
2. Wait for the initial Gradle sync to complete (may take 2–5 minutes on first run).
3. Accept any SDK or JDK prompts that appear.

---

## 3. Verify Your Build

Run a compilation check to confirm all dependencies resolved correctly:

```bash
./gradlew compileDebugJavaWithJavac
```

A successful output ends with `BUILD SUCCESSFUL`. With Gradle configuration cache enabled, subsequent runs typically complete in under 2 seconds.

To do a full clean rebuild:
```bash
./gradlew clean compileDebugJavaWithJavac
```

---

## 4. Configuration Files

The robot loads all tuning constants at runtime from YAML — nothing is hardcoded in Java. These files live under:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/
```

| File | Contents |
|---|---|
| `config.yaml` | Consolidated configuration values for subsystems, safety, tuning, teleop, and auto |
| `config-docs.yaml` | Human-written description (and min/max) for each config leaf key — hand-maintained |
| `RobotConfig.java` | **Generated** typed facade over the config — do not hand-edit |
| `config-schema.json` | **Generated** JSON Schema used to validate `config.yaml` in tests — do not hand-edit |

Runtime loading is handled by a separate Gradle module, `config-compiler/` (see project root), which Android Studio will sync as its own module. When you add or change a config key:

1. Edit `config.yaml` (and `config-docs.yaml` for its description).
2. Run `./gradlew generateConfig` (or `./gradlew verifyBuild`) to regenerate `config.java` and `config-schema.json`.
3. Access the value in Java via `config` (e.g. `config.shooter.max_rpm`), not by hand-writing a lookup.

See [CONTRIBUTING.md](CONTRIBUTING.md) §6 for the full configuration workflow, including the `checkConfigKeys` alliance-symmetry check and the `pushConfig`/`resetConfig` ADB hot-deploy tasks.


---

## 5. Deploying to the Robot

1. Connect to the Control Hub's Wi-Fi network or via USB.
2. In Android Studio, select the **TeamCode** run configuration.
3. Click **Run** (▶) and select the Control Hub from the device list.
4. The OpMode will appear in the Driver Station app under its registered group.

> **Tip:** Use `./gradlew assembleDebug` to build the APK without deploying, useful for CI validation.
>
> **Tip:** Once deployed, you can hot-reload `config.yaml` alone — no rebuild/redeploy — with `./gradlew pushConfig` (and revert with `./gradlew resetConfig`). Re-init the OpMode on the Driver Station afterward to pick up the new values.

Before opening a PR, run the full local verification pass (formatting, compile, unit tests, and the config-symmetry check) without deploying:
```bash
./gradlew verifyBuild
```

---

## 6. Managing AI Assistant Rules

AI assistant behavior (Claude Code, Codex, GitHub Copilot, Antigravity) is governed by [`AGENTS.md`](../AGENTS.md) at the repository root. Every one of these tools reads it directly, so it is the single file to edit; there is no generation step.

**When to update rules:**
- You've changed a coding convention that AI assistants should follow.
- You've added a new subsystem or pattern that AI assistants should be aware of.
- A rule is producing incorrect AI behavior.

**Layout:**
- `AGENTS.md`: coding conventions and architecture. Edit and commit it like any other file.
- `CLAUDE.md` and `GEMINI.md`: one-line `@AGENTS.md` imports, so older Claude Code versions and Gemini-based tools load the same rules. Keep them as bare imports: anything added there reaches only one tool.
- `.agents/skills/`: shared skills (Codex, Antigravity). `.claude/skills` and `.github/skills` are symlinks to it for Claude Code and Copilot. On Windows, enable symlinks (`git config core.symlinks true` with Developer Mode on) or those two paths check out as plain text files.

---

## 7. Project Documentation

For a comprehensive guide to the codebase architecture, subsystem descriptions, and contributing conventions, see:

**[CONTRIBUTING.md](CONTRIBUTING.md)**


This document covers:
- What every package and class does
- How the configuration system works
- How to add new config values
- How the safety systems (Casablanca & Sentinel) work
- Hardware caching patterns
- Autonomous and TeleOp patterns

---

## 8. Troubleshooting

**Gradle sync fails with "Cannot find JDK 21"**
→ In Android Studio: `File → Project Structure → SDK Location → JDK Location`. Set it to a Java 21 JDK install, or let the Foojay resolver fetch one automatically by setting `java.toolchain.version=21`.

**A `RobotConfig` field is unexpectedly `0.0`/`false`/`null` at runtime**
→ The reflection-based loader silently leaves a field at its default if the corresponding key is missing from `config.yaml`, rather than throwing. Check the key name and nesting match `config.yaml` exactly, and re-run `python3 scripts/generate_config.py` if you added the field recently. `ConfigValidationTest` (`./gradlew testDebugUnitTest`) will catch a key that's missing entirely against the schema, but not a misspelled one that happens to leave a sibling field unset.

**Compilation error: "cannot find symbol CachingDcMotorEx"**
→ Ensure `build.dependencies.gradle` includes the Dairy CachingHardware dependency and Android Studio has synced. Also confirm the `config-compiler` module synced correctly (`settings.gradle` includes it) — it's a separate Gradle module from `TeamCode`.

**Robot drives in the wrong direction**
→ Check that joystick inputs are negated in `TeleOp`'s drive command. The FTC SDK returns inverted Y-axis values (pushing forward = negative); `leftStickY` and `leftStickX` should both be negated before use.

**Shooter doesn't spin up in autonomous**
→ Confirm `shooter.max_rpm` is set in `config.yaml` (default: `1500.0`). A missing or zero value causes all velocity calculations to evaluate to zero.

**Rings not fed during shooting in autonomous**
→ Verify that `robot.update()` is called inside `loop()`. The subsystems and shot controller rely on the `periodic()` contract executed inside `robot.update()`.
