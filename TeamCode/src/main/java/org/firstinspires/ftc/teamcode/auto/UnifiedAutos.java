package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.records.Alliance;

/** Registers each BIOBUZZ routine once per alliance. Preselects TeleOp to follow AUTO. */
public final class UnifiedAutos {

  private UnifiedAutos() {}

  @Autonomous(name = "Red A: tip first", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class RedCellA extends CellAAuto {
    public RedCellA() {
      super(Alliance.RED);
    }
  }

  @Autonomous(name = "Blue A: tip first", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class BlueCellA extends CellAAuto {
    public BlueCellA() {
      super(Alliance.BLUE);
    }
  }

  @Autonomous(name = "Red B: second CELL", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class RedCellB extends CellBAuto {
    public RedCellB() {
      super(Alliance.RED);
    }
  }

  @Autonomous(name = "Blue B: second CELL", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class BlueCellB extends CellBAuto {
    public BlueCellB() {
      super(Alliance.BLUE);
    }
  }

  @Autonomous(name = "Red Park Only", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class RedParkOnly extends ParkOnlyAuto {
    public RedParkOnly() {
      super(Alliance.RED);
    }
  }

  @Autonomous(name = "Blue Park Only", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class BlueParkOnly extends ParkOnlyAuto {
    public BlueParkOnly() {
      super(Alliance.BLUE);
    }
  }
}
