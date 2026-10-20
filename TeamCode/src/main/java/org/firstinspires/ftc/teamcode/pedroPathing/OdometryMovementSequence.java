package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

/** Hardware-independent, edge-triggered four-leg odometry diagnostic sequence. */
public final class OdometryMovementSequence {
  public static final double DISTANCE_INCHES = 12.0;

  public enum Step {
    FORWARD("Forward", DISTANCE_INCHES, 0),
    LEFT("Strafe left", 0, DISTANCE_INCHES),
    BACKWARD("Backward", -DISTANCE_INCHES, 0),
    RIGHT("Strafe right", 0, -DISTANCE_INCHES);

    private final String label;
    private final double forwardInches;
    private final double leftInches;

    Step(String label, double forwardInches, double leftInches) {
      this.label = label;
      this.forwardInches = forwardInches;
      this.leftInches = leftInches;
    }

    public String getLabel() { return label; }
    public double getForwardInches() { return forwardInches; }
    public double getLeftInches() { return leftInches; }

    /** Each leg is relative to the robot's pose when that Y press is accepted. */
    public Pose targetFrom(Pose start) {
      double cos = Math.cos(start.getHeading());
      double sin = Math.sin(start.getHeading());
      return new Pose(start.getX() + forwardInches * cos - leftInches * sin,
          start.getY() + forwardInches * sin + leftInches * cos, start.getHeading());
    }

    public Step next() { return values()[(ordinal() + 1) % values().length]; }
  }

  private Step nextStep = Step.FORWARD;
  private Step activeStep;
  private boolean running;
  private boolean previousY;

  /** Busy or disabled presses are discarded, never queued. Holding Y cannot launch another leg. */
  public Step updateY(boolean yDown, boolean canStart) {
    boolean pressed = yDown && !previousY;
    previousY = yDown;
    if (!pressed || !canStart || running) {
      return null;
    }
    activeStep = nextStep;
    running = true;
    return activeStep;
  }

  public void complete() {
    if (running) {
      nextStep = activeStep.next();
      running = false;
    }
  }

  /** An interrupted leg is retried on the next accepted press. */
  public void abort() { running = false; }
  public boolean isRunning() { return running; }
  public Step getNextStep() { return nextStep; }
  public Step getActiveStep() { return activeStep; }
}
