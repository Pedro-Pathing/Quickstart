package org.firstinspires.ftc.teamcode.records;

/** BIOBUZZ SCORING ELEMENTS (manual 9.8). Each needs its own shot calibration. */
public enum BallType {
  /** 2.8 in yellow ball; 40 per match. */
  POLLEN(2.8),
  /** 3.6 in red or blue ball; 8 per alliance. */
  NECTAR(3.6);

  public final double diameterInches;

  BallType(double diameterInches) {
    this.diameterInches = diameterInches;
  }

  public BallType toggled() {
    return this == POLLEN ? NECTAR : POLLEN;
  }
}
