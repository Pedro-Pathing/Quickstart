package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import dev.nextftc.core.commands.Command;

public class Commands {
    public class myCommand extends Command {

        public myCommand() {
            requires(/* subsystems */);
            setInterruptible(true); // this is the default, so you don't need to specify
        }

        @Override
        public boolean isDone() {
            return false; // whether or not the command is done
        }

        @Override
        public void start() {
            // executed when the command begins
        }

        @Override
        public void update() {
            // executed on every update of the command
        }

        @Override
        public void stop(boolean interrupted) {
            // executed when the command ends
        }
    }
}
