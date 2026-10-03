package org.firstinspires.ftc.teamcode.control

import com.pedropathing.api.Paths.path
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.opmodes.teleop.teleOp
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Intake


fun shoot(flywheel: Flywheel) = instant{flywheel.on()}

fun IntakeShoot(robot: Robot, toIntake: Path, toShoot: Path): Command {
    return sequential(follow(robot.follower, toIntake),
        robot.intake.intake(),
        follow(robot.follower, toShoot),
        shoot(robot.flywheel))
}