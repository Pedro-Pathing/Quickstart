package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.junit.jupiter.api.Test;

public class InventoryHandoffTest {
  @Test
  public void missingOrUnknownHandoffCannotBecomeKnownEmptyWithASensor() {
    for (boolean sensed : new boolean[] {false, true}) {
      TeleOp op = new TeleOp();
      op.intake = mock(Intake.class);
      op.hive = new HiveTracker(3, 1000, 500, () -> 0);
      BallInventory inventory = new BallInventory(4, sensed);
      when(op.intake.inventory()).thenReturn(inventory);
      for (OpModeUtil.Handoff handoff :
          new OpModeUtil.Handoff[] {null, new OpModeUtil.Handoff(HiveCell.B, 1, -1, 0)}) {
        inventory.set(3);
        op.restoreHandoff(handoff);
        assertFalse(
            inventory.isKnown(), "an entry sensor cannot infer how many balls were already held");
        assertFalse(inventory.isEmpty());
        assertFalse(inventory.isFull());
        inventory.updateEntrySensor(true, 0, 0);
        assertFalse(inventory.isKnown(), "a new entry cannot reconstruct the missing total");
        inventory.set(0);
        assertTrue(inventory.isKnown(), "driver correction re-establishes the count");
        op.restoreHandoff(new OpModeUtil.Handoff(HiveCell.B, 1, 2, 0));
        assertTrue(inventory.isKnown());
        assertEquals(2, inventory.count());
      }
    }
  }
}
