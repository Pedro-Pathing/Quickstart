package org.firstinspires.ftc.teamcode.records;

/**
 * One of the two CELLS of an alliance's HIVE, named relative to the alliance rather than the field.
 *
 * <p>{@link #A} is the CELL that starts the match upward-facing: RED's audience-side CELL
 * (AprilTags 34-37) and BLUE's rear CELL (42-45). {@link #B} is the other CELL of the same HIVE,
 * which becomes upward-facing after every odd-numbered TIP. Because the field is 180-degree
 * rotationally symmetric, naming CELLS this way lets one RED-frame pose describe both alliances.
 */
public enum HiveCell {
  A,
  B;

  /** The other CELL of the same HIVE. */
  public HiveCell other() {
    return this == A ? B : A;
  }
}
