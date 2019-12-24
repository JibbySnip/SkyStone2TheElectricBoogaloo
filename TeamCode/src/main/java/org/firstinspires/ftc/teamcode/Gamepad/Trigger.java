package org.firstinspires.ftc.teamcode.Gamepad;

class Trigger {
    private gamepadEx gamepad;
    private gamepadKeys.trigger trigger;

    Trigger(gamepadEx gamepad, gamepadKeys.trigger trigger) {
        this.gamepad = gamepad;
        this.trigger = trigger;
    }

    double getValue(){
        return gamepad.getTrigger(trigger);
    }
    boolean isDown() {
        return gamepad.getTrigger(trigger) < 0.5;
    }


}
