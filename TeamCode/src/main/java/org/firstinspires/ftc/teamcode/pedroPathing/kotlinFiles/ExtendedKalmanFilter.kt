package org.firstinspires.ftc.teamcode.pedroPathing.kotlinFiles

import kotlin.math.PI

/**
 * ExtendedKalmanFilter - A non-linear state estimator for robot pose.
 * Fuses Odometry (Prediction) with Absolute Measurements (Update).
 */
class ExtendedKalmanFilter {
    var x = 0.0
    var y = 0.0
    var theta = 0.0

    private var P = Array(3) { DoubleArray(3) } // State Covariance
    private val Q = Array(3) { DoubleArray(3) } // Process Noise
    private val R = Array(3) { DoubleArray(3) } // Measurement Noise

    init {
        for (i in 0..2) {
            P[i][i] = 1.0   // Initial uncertainty
            Q[i][i] = 0.01  // Odometry process noise
            R[i][i] = 0.1   // Vision measurement noise
        }
    }

    /**
     * Prediction step using relative movement from odometry.
     */
    fun predict(deltaX: Double, deltaY: Double, deltaTheta: Double) {
        x += deltaX
        y += deltaY
        theta += deltaTheta

        // Increase uncertainty as we move
        for (i in 0..2) P[i][i] += Q[i][i]
    }

    /**
     * Update step using an absolute measurement (e.g., AprilTag).
     */
    fun update(zX: Double, zY: Double, zTheta: Double) {
        for (i in 0..2) {
            val kGain = P[i][i] / (P[i][i] + R[i][i])
            when (i) {
                0 -> x += kGain * (zX - x)
                1 -> y += kGain * (zY - y)
                2 -> {
                    var angleDiff = zTheta - theta
                    while (angleDiff > PI) angleDiff -= 2.0 * PI
                    while (angleDiff < -PI) angleDiff += 2.0 * PI
                    theta += kGain * angleDiff
                }
            }
            // Update covariance: P = (I - KH)P
            P[i][i] *= (1.0 - kGain)
        }
    }
}