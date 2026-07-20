# Team 24171 — Pedro Pathing Quickstart

FTC robot code for the **BIOBUZZ (2026–2027)** season, built on a fork of the
[Pedro Pathing Quickstart](https://github.com/Pedro-Pathing/Quickstart).
This is the real FTC SDK + Pedro Pathing project that gets deployed to the Control Hub.

> Not to be confused with our `virtual_robot` simulator repo — that's a separate
> desktop app for testing logic without the physical robot. This repo is the one
> that runs on the actual bot.

---

## What you need (one-time, per machine)

Any laptop that will build or deploy code needs this set up **once**:

- **Git** — https://git-scm.com/downloads
- **Android Studio** — Quail 2 (2026.1.2) or later, https://developer.android.com/studio
  (bundles its own JDK, so no separate Java install needed)
- **Android SDK Platform API 34** — install via Android Studio's SDK Manager
  (Settings ▸ Languages & Frameworks ▸ Android SDK ▸ SDK Platforms tab ▸
  check "Android 14.0 (UpsideDownCake)" / API 34)
- **SDK Tools** (SDK Manager ▸ SDK Tools tab): Android SDK Build-Tools,
  Platform-Tools, and Command-line Tools (latest)

> **Note on compileSdk vs. your machine's SDKs:** the project compiles against
> **API 34**. It's fine to have other API levels (16.0, etc.) installed alongside
> it — the build uses whichever `compileSdk` is set, not whatever's newest.

---

## Getting the code onto a new machine

We clone over SSH (make sure your GitHub SSH key is set up on the machine first).

```bash
git clone git@github.com:noahetaylor/24171-Quickstart.git
cd 24171-Quickstart
git remote add upstream https://github.com/Pedro-Pathing/Quickstart.git   # optional: lets you pull Pedro's updates later
```

Then in Android Studio: **File ▸ Open** ▸ select the `24171-Quickstart` folder ▸
**Trust Project**. Let the first Gradle sync finish (5–15 min — it downloads all
dependencies).

---

## Building

Confirm everything compiles (no robot needed):

```bash
./gradlew assembleDebug
```

or in Android Studio: **Build ▸ Make Project** (Ctrl+F9). Success = `BUILD SUCCESSFUL`.
APKs land in `TeamCode/build/outputs/apk/`.

---

## Deploying to the Control Hub (in-season, needs the robot)

1. Connect the laptop to the Control Hub (USB, or the hub's WiFi Direct network).
2. Select the run configuration and hit Run/Deploy to install the app on the hub.
3. Watch live telemetry over the robot's WiFi.

*(This is the only step that requires the physical laptop-plus-robot combo.)*

---

## Pedro Pathing notes

- **Pinned Pedro version:** `com.pedropathing:ftc:2.1.2`
  (defined in `build.dependencies.gradle` — bump deliberately, then re-sync)
- **Pedro constants / tuning files** live in the `pedroPathing` package under
  `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/`
- Docs: https://pedropathing.com/docs/pathing

---

## Working conventions

- **Don't commit straight to `master`.** Use a feature branch, then merge via PR.
- Keep OpModes and subsystems organized under `TeamCode/`.
- After pulling upstream Pedro updates, re-sync Gradle and re-run a build before trusting anything.

---

## Heads-up for the fall

The FTC SDK version pinned here may need bumping once the official 2026–2027
season SDK is released (FIRST usually publishes it in September). When that lands,
update the FTC SDK dependency, re-sync, and re-build before deploying.
