package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;

/**
 * Saves and restores the global state Casablanca property tests change: the generated config values
 * Sentinel and Casablanca read, and Casablanca's Panels-tunable statics. Build it before changing
 * anything and call {@link #restore()} when the property finishes.
 */
final class CasablancaHarness {
  private final double frameDepth = config.field.hive.frame_depth_in;
  private final boolean protectFullFootprint = config.sentinel.protect_full_hive_footprint;
  private final double strafeLinear = config.pedro.braking.strafe.linear;
  private final double strafeQuadratic = config.pedro.braking.strafe.quadratic;

  private final boolean friction = Casablanca.enableFrictionComp;
  private final boolean smoothing = Casablanca.enableInputSmoothing;
  private final boolean headingLock = Casablanca.enableHeadingLock;
  private final boolean fieldCentric = Casablanca.fieldCentric;
  private final boolean depthProtection = Casablanca.enableDepthProtection;
  private final boolean sideProtection = Casablanca.enableSideProtection;
  private final double sideSlowDown = Casablanca.sideSlowDown;
  private final double sideHardStop = Casablanca.sideHardStop;

  /**
   * Leaves only the protected-zone limiter in the drive path: friction compensation, slew
   * smoothing, heading lock and field-centric input all off, both axes protected. Call after
   * constructing the Casablanca, whose constructor reloads these statics from config.
   */
  static void isolateZoneProtection() {
    Casablanca.enableFrictionComp = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableHeadingLock = false;
    Casablanca.fieldCentric = false;
    Casablanca.enableDepthProtection = true;
    Casablanca.enableSideProtection = true;
  }

  void restore() {
    config.field.hive.frame_depth_in = frameDepth;
    config.sentinel.protect_full_hive_footprint = protectFullFootprint;
    config.pedro.braking.strafe.linear = strafeLinear;
    config.pedro.braking.strafe.quadratic = strafeQuadratic;

    Casablanca.enableFrictionComp = friction;
    Casablanca.enableInputSmoothing = smoothing;
    Casablanca.enableHeadingLock = headingLock;
    Casablanca.fieldCentric = fieldCentric;
    Casablanca.enableDepthProtection = depthProtection;
    Casablanca.enableSideProtection = sideProtection;
    Casablanca.sideSlowDown = sideSlowDown;
    Casablanca.sideHardStop = sideHardStop;
  }
}
