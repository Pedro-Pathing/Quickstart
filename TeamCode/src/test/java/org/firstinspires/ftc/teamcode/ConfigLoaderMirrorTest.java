package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.junit.jupiter.api.Test;

/**
 * Covers the ConfigLoader "mirror" system (an {@code m}/{@code mirror} value means "derive this
 * from the opposite alliance's value"). For poses the BIOBUZZ field is 180-degree rotationally
 * symmetric, so {@code m} is a half-turn about the field centre ({@link
 * ConfigLoader#allianceTransform}); for scalars it is the identity plus an optional operation. What
 * the shipped config binds is covered by {@link ConfigSeasonAndSymmetryTest}; this class covers the
 * operation parser.
 */
public class ConfigLoaderMirrorTest {

  private static final double EPS = 1e-4;

  /** Non-alliance pair math (high/low): verify "k+0.5" adds the offset to the partner's value. */
  @Test
  public void nonAlliancePair_mathResolution() throws Exception {
    java.lang.reflect.Method eval =
        ConfigLoader.class.getDeclaredMethod(
            "evaluateMirrorOperation", Object.class, Object.class, Class.class, String.class);
    eval.setAccessible(true);

    // Sample: base low_position = 0.1 → high_position with "k+0.5" should be 0.6
    double sampleBase = 0.1;
    assertEquals(0.6, (Double) eval.invoke(null, sampleBase, "k+0.5", double.class, "dummy"), EPS);

    // Sample: base = 0.3 → "k-0.1" should give 0.2
    assertEquals(0.2, (Double) eval.invoke(null, 0.3, "k-0.1", double.class, "dummy"), EPS);

    // Sample: base = 0.4 → "k*2" should give 0.8
    assertEquals(0.8, (Double) eval.invoke(null, 0.4, "k*2", double.class, "dummy"), EPS);

    // Sample: base = 0.6 → "k/2" should give 0.3
    assertEquals(0.3, (Double) eval.invoke(null, 0.6, "k/2", double.class, "dummy"), EPS);
  }

  @Test
  public void mirrorMathOperations_mAndShorthandForms() throws Exception {
    java.lang.reflect.Method eval =
        ConfigLoader.class.getDeclaredMethod(
            "evaluateMirrorOperation", Object.class, Object.class, Class.class, String.class);
    eval.setAccessible(true);

    // For scalars, 'm' is identity (no spatial mirror), so m+N = base+N
    // Sample: base=0.5 → "m+0.5" gives 1.0, shorthand "+0.5" also gives 1.0
    double base = 0.5;
    assertEquals(1.0, (Double) eval.invoke(null, base, "m+0.5", double.class, "dummy"), EPS);
    assertEquals(1.0, (Double) eval.invoke(null, base, "+0.5", double.class, "dummy"), EPS);
    // Sample: base=0.5 → "m*2" / "*2" gives 1.0
    assertEquals(1.0, (Double) eval.invoke(null, base, "*2", double.class, "dummy"), EPS);
    // Sample: base=0.8 → "*2" gives 1.6
    assertEquals(1.6, (Double) eval.invoke(null, 0.8, "*2", double.class, "dummy"), EPS);
  }

  @Test
  public void mirrorVsKey_poseTransformations() throws Exception {
    java.lang.reflect.Method eval =
        ConfigLoader.class.getDeclaredMethod(
            "evaluateMirrorOperation", Object.class, Object.class, Class.class, String.class);
    eval.setAccessible(true);

    Pose basePose = new Pose(27.0, 128.0, 135.0);
    Pose expectedMirror = ConfigLoader.allianceTransform(basePose);

    // The alliance transform is a half-turn about the field centre, not a reflection.
    double size = ConfigLoader.fieldSize();
    assertEquals(size - basePose.x(), expectedMirror.x(), EPS);
    assertEquals(size - basePose.y(), expectedMirror.y(), EPS);

    // 'm' rotates the pose
    Pose mirrored = (Pose) eval.invoke(null, basePose, "m", Pose.class, "dummy");
    assertEquals(expectedMirror.x(), mirrored.x(), EPS);
    assertEquals(expectedMirror.y(), mirrored.y(), EPS);

    // 'm+2' mirrors the pose first, then adds 2
    Pose mirroredPlusTwo = (Pose) eval.invoke(null, basePose, "m+2", Pose.class, "dummy");
    assertEquals(expectedMirror.x() + 2.0, mirroredPlusTwo.x(), EPS);
    assertEquals(expectedMirror.y() + 2.0, mirroredPlusTwo.y(), EPS);

    // 'k' keeps the original pose UN-MIRRORED
    Pose original = (Pose) eval.invoke(null, basePose, "k", Pose.class, "dummy");
    assertEquals(basePose.x(), original.x(), EPS);
    assertEquals(basePose.y(), original.y(), EPS);

    // 'k+2' keeps original pose and adds 2
    Pose originalPlusTwo = (Pose) eval.invoke(null, basePose, "k+2", Pose.class, "dummy");
    assertEquals(basePose.x() + 2.0, originalPlusTwo.x(), EPS);
    assertEquals(basePose.y() + 2.0, originalPlusTwo.y(), EPS);
  }
}
