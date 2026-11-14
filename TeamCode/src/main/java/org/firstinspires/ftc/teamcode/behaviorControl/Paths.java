package org.firstinspires.ftc.teamcode.behaviorControl;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths {

    public final PoseFactory poseFactory = PoseFactory.degrees();

    // Path start position
    public final Pose start = poseFactory.of(88.372, 131.4403, 90);

    // Path end position
    public final Pose end = poseFactory.of(136.876, 32.0003, 90);

    // Creates the path from start to end
    public Path path1() {
        return com.pedropathing.api.Paths.line(start, end).linear(start, end);
    }
}