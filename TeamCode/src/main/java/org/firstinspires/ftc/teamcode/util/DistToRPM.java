package org.firstinspires.ftc.teamcode.util;

import java.util.TreeMap;

public class DistToRPM {

    //Tree map, the thing that calculates closest vals:
    private static TreeMap<Double, Double> lookupTable = new TreeMap<>();
    //data entries
    static {
        // Close range
        lookupTable.put(12.0, 2500.0);   // Very close
        lookupTable.put(24.0, 3000.0);   // Close shot
        lookupTable.put(36.0, 3500.0);   // Medium-close

        // Mid range
        lookupTable.put(48.0, 4000.0);   // Mid distance
        lookupTable.put(60.0, 4500.0);   // Medium-far

        // Far range
        lookupTable.put(72.0, 5000.0);   // Far shot
        lookupTable.put(84.0, 5500.0);   // Very far
        lookupTable.put(96.0, 6000.0);   // Maximum distance
    }
    public static Double CalcRPM(double distance) {


        //Uses fancy and complicated Hermite Interpolation.


        //set endpoints
        if (distance <= lookupTable.firstKey()) {
            return lookupTable.firstEntry().getValue();  // Use closest value
        }
        if (distance >= lookupTable.lastKey()) {
            return lookupTable.lastEntry().getValue();  // Use closest value
        }

        //get 2 points below value:
        double p1 = lookupTable.floorKey(distance);
        Double p0 = lookupTable.lowerKey(p1);

        //get 2 points above value:
        double p2 = lookupTable.ceilingKey(distance);
        Double p3 = lookupTable.higherKey(p2);
        //get rpm
        double p1RPM = lookupTable.get(p1);
        double p2RPM = lookupTable.get(p2);
        //init slope vars
        double m1, m2;

        //handle null case + slope calculation
        if (p0 != null) {
            double p0RPM = lookupTable.get(p0);  // Safe to get now
            m1 = (p2RPM - p0RPM) / (p2 - p0);    // Use centered difference
        } else {
            m1 = (p2RPM - p1RPM) / (p2 - p1);    // Use forward difference (no p0 available)
        }

        if (p3 != null) {
            double p3RPM = lookupTable.get(p3);  // Safe to get now
            m2 = (p3RPM - p1RPM) / (p3 - p1);    // Use centered difference
        } else {
            m2 = (p2RPM - p1RPM) / (p2 - p1);    // Use backward difference (no p3 available)
        }


        //find position where we are between p1 and p2:
        double t = (distance - p1) / (p2 - p1);

        //Calculate the weights for the values(difficult par t):
        double t2 = t * t;      // t squared
        double t3 = t2 * t;     // t cubed

        double h00 = 2*t3 - 3*t2 + 1;     // Weight for P1 value
        double h10 = t3 - 2*t2 + t;       // Weight for P1 slope
        double h01 = -2*t3 + 3*t2;        // Weight for P2 value
        double h11 = t3 - t2;             // Weight for P2 slope

        //interval is how far apart
        double interval = p2 - p1;

        //weird stuff to get the actual rpm:
        return h00 * p1RPM +           // Starting RPM value
                h10 * m1 * interval +    // Starting slope influence
                h01 * p2RPM +            // Ending RPM value
                h11 * m2 * interval;     // Ending slope influence




    }
}
