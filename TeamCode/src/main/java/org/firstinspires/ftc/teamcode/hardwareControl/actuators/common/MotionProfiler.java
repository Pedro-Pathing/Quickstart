package org.firstinspires.ftc.teamcode.hardwareControl.actuators.common;

public class MotionProfiler {

    private final double maxVelocity;
    private final double maxAcceleration;

    private double peakVelocity;
    private double initialPosition;
    private double targetPosition;

    private double accelTime;
    private double cruiseTime;
    private double totalTime;

    private boolean isProfileGenerated = false;

    public MotionProfiler(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public void generateProfile(double initialPosition, double targetPosition) {
        this.initialPosition = initialPosition;
        this.targetPosition = targetPosition;

        double distance = Math.abs(targetPosition - initialPosition);
        double accelDistance = (maxVelocity * maxVelocity) / (2.0 * maxAcceleration);

        if (2.0 * accelDistance >= distance) {
            // Triangular profile
            peakVelocity = Math.sqrt(maxAcceleration * distance);
            accelTime = peakVelocity / maxAcceleration;
            cruiseTime = 0.0;
        } else {
            // Trapezoidal profile
            peakVelocity = maxVelocity;
            accelTime = maxVelocity / maxAcceleration;
            cruiseTime = (distance - 2.0 * accelDistance) / maxVelocity;
        }

        totalTime = 2.0 * accelTime + cruiseTime;
        isProfileGenerated = true;
    }

    public MotionState getMotionState(double time) {
        if (!isProfileGenerated) {
            return new MotionState(initialPosition, 0.0);
        }

        double direction = Math.signum(targetPosition - initialPosition);
        double position;
        double velocity;

        if (time <= 0.0) {
            position = initialPosition;
            velocity = 0.0;

        } else if (time < accelTime) {
            // Acceleration
            velocity = maxAcceleration * time;
            position = initialPosition
                    + direction * 0.5 * maxAcceleration * time * time;

        } else if (time < accelTime + cruiseTime) {
            // Cruise
            velocity = peakVelocity;
            position = initialPosition
                    + direction * (
                    0.5 * maxAcceleration * accelTime * accelTime
                            + peakVelocity * (time - accelTime)
            );

        } else if (time < totalTime) {
            // Deceleration
            double t = time - (accelTime + cruiseTime);
            velocity = peakVelocity - maxAcceleration * t;
            position = initialPosition
                    + direction * (
                    0.5 * maxAcceleration * accelTime * accelTime
                            + peakVelocity * cruiseTime
                            + peakVelocity * t
                            - 0.5 * maxAcceleration * t * t
            );

        } else {
            position = targetPosition;
            velocity = 0.0;
        }

        return new MotionState(position, direction * velocity);
    }

    public double getTotalTime() {
        return totalTime;
    }

    public static class MotionState {
        public final double position;
        public final double velocity;

        public MotionState(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }
}