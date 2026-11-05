package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.lazy;
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
 * The partner routine to {@link CellAAuto}: starts on the CELL B side and waits for the alliance
 * partner's pre-load to TIP CELL A, which turns CELL B upward. It then launches its pre-load into
 * CELL B, optionally refills from its rear-wall FLOWER for a second volley (eight POLLEN in an
 * empty CELL is the official TIP calibration), and parks.
 *
 * <p>If CELL B never comes up (no partner TIP within {@code auto.tip_wait_ms}), it drives around
 * the HIVE on its own side and empties its pre-load into CELL A instead, so the pre-load still
 * scores.
 */
// Paths are assigned by buildPaths(), which init() calls, not by a constructor.
@SuppressWarnings("NullAway.Init")
public abstract class CellBAuto extends AllianceAutoBase<config.CellBAuto> {

  private Path toScoreB;
  private Path toFlower;
  private Path fromFlower;
  private Path toAltVia;
  private Path toAltA;
  private Path toParkFromB;
  private Path toParkFromAlt;

  protected CellBAuto(Alliance alliance) {
    super(alliance, config.CellBAuto.class, "auto_poses.cell_b");
  }

  @Override
  protected void buildPaths() {
    toScoreB = pcurve(poses.start, poses.scoreBCp, poses.scoreB);
    toFlower = pline(poses.scoreB, poses.flower);
    fromFlower = pline(poses.flower, poses.scoreB);
    toAltVia = pline(poses.scoreB, poses.altVia);
    toAltA = pline(poses.altVia, poses.altA);
    toParkFromB = pline(poses.scoreB, poses.park);
    toParkFromAlt = pline(poses.altA, poses.park);
  }

  @Override
  protected CommandBuilder buildAuto() {
    return sequential(follow(follower, toScoreB), volley(HiveCell.B), lazy(this::afterFirstVolley));
  }

  /** CELL B scored: refill and fire again. CELL B never came up: take the pre-load to CELL A. */
  private Command afterFirstVolley() {
    List<Command> steps = new ArrayList<>();
    if (hive.upCell() == HiveCell.B || robot.shotController.getBallsFired() > 0) {
      if (config.auto.cell_b_collect_flower) {
        steps.add(collectAt(toFlower));
        steps.add(follow(follower, fromFlower));
        steps.add(volley(HiveCell.B));
      }
      steps.add(idleMechanisms());
      steps.add(follow(follower, toParkFromB));
    } else {
      steps.add(follow(follower, toAltVia));
      steps.add(follow(follower, toAltA));
      steps.add(volley(HiveCell.A));
      steps.add(idleMechanisms());
      steps.add(follow(follower, toParkFromAlt));
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
    return poses.scoreB;
  }

  @Override
  protected HiveCell firstVolleyCell() {
    return HiveCell.B;
  }
}
