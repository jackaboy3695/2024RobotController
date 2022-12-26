package org.firstinspires.ftc.teamcode.powerplay;

import com.arcrobotics.ftclib.command.CommandBase;


public class ClawCommand extends CommandBase {
    private final ClawSubsystem claw;
    private ClawSubsystem.State clawState;

    public ClawCommand(ClawSubsystem claw) {
        this.claw = claw;
        addRequirements(this.claw);
    }

    @Override
    public void initialize() {
        clawState = claw.getState();

        if (clawState == ClawSubsystem.State.RELEASE)
            claw.grab();
        else if (clawState == ClawSubsystem.State.REST)
            claw.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

