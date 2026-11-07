package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pcurve;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

/**
 * Starts facing CELL A (the CELL that begins upward-facing) and TIPS it with the pre-load.
 *
 * <p>CELL A starts with three NECTAR in it, and the official field calibration tips a CELL holding
 * three NECTAR with three more POLLEN, so four pre-loaded POLLEN should earn the first TIP (20) on
 * their own. Then, if {@code auto.cell_a_second_volley}, it drives around the HIVE on its own side
 * by the FLOWER-side lane and launches into CELL B once the camera confirms it is upward-facing:
 * with {@code auto.cell_a_collect_flower} it first refills from the side-wall FLOWER's bottom
 * opening (G418); without it (this robot has no FLOWER pickup) the volley fires only POLLEN left
 * when the TIP stopped the first volley. It always ends parked for LEAVE + PARK.
 */
// Paths are assigned by buildPaths(), which init() calls, not by a constructor.
@SuppressWarnings("NullAway.Init")
public abstract class CellAAuto extends AllianceAutoBase<config.CellAAuto> {

  private Path toScoreA;
  private Path toFlower;
  private Path toScoreB;
  private Path toParkFromB;

  protected CellAAuto(Alliance alliance) {
    super(alliance, config.CellAAuto.class, "auto_poses.cell_a");
  }

  @Override
  protected void buildPaths() {
    toScoreA = pline(poses.start, poses.scoreA);
    toFlower = pline(poses.scoreA, poses.flower);
    toScoreB = pcurve(poses.flower, poses.scoreBCp, poses.scoreB);
    toParkFromB = pline(poses.scoreB, poses.park);
  }

  @Override
  protected CommandBuilder buildAuto() {
    List<Command> steps = new ArrayList<>();
    steps.add(follow(follower, toScoreA));
    steps.add(volley(HiveCell.A));
    if (config.auto.cell_a_second_volley) {
      if (config.auto.cell_a_collect_flower) {
        steps.add(collectAt(toFlower));
        steps.add(follow(follower, toScoreB));
      } else {
        // Use the same clear lane even when skipping collection.
        steps.add(follow(follower, toFlower));
        steps.add(follow(follower, toScoreB));
      }
      steps.add(volley(HiveCell.B));
      steps.add(idleMechanisms());
      steps.add(follow(follower, toParkFromB));
    } else {
      // A straight line from CELL A's side to the LOADING ZONE clips the HIVE frame; detour.
      steps.add(idleMechanisms());
      steps.add(followRoute(poses.scoreA, sentinel.parkRoute(poses.scoreA, poses.park)));
    }
    return sequential(steps.toArray(new Command[0]));
  }

  @Override
  protected Pose startPose() {
    return poses.start;
  }

  @Override
  protected Pose parkPose() {
    return poses.park;
  }

  @Override
  protected Pose firstVolleyPose() {
    return poses.scoreA;
  }
}
