import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public static class PathsM {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;

    public Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 36.000), new Pose(15.000, 36.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.000, 36.000), new Pose(56.000, 36.000))
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 36.000), new Pose(56.000, 84.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 84.000), new Pose(56.000, 60.000))
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 60.000), new Pose(15.000, 60.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.000, 60.000), new Pose(56.000, 60.000))
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 60.000), new Pose(56.000, 84.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 84.000), new Pose(15.000, 84.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.000, 84.000), new Pose(56.000, 84.000))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}