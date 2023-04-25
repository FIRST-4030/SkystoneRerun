package org.firstinspires.ftc.teamcode.util.general.subsystems.claw;

import org.firstinspires.ftc.teamcode.util.statemachine.State;

public class ClawCmd implements State {

    private ClawController clawController;
    private boolean open;

    public ClawCmd(ClawController clawController, boolean open){
        this.clawController = clawController;
        this.open = open;
    }

    @Override
    public void init() {
        clawController.setOpen(open);
        clawController.setBusy(true);
        clawController.update();
    }

    @Override
    public void run() {

    }

    @Override
    public void end() {
        clawController.setBusy(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
