package org.firstinspires.ftc.teamcode.Gamepad;

class Button {

    gamepadKeys.key key;
    gamepadEx gamepad;
    boolean lastValue;
    boolean currValue;
    private boolean doubleTrigger;


    Button(gamepadKeys.key key, gamepadEx gamepad, boolean canDoubleTrigger) {
        this.key = key;
        this.gamepad = gamepad;
        this.doubleTrigger = canDoubleTrigger;
    }
    Button(gamepadKeys.key key, gamepadEx gamepad) {
        this.key = key;
        this.gamepad = gamepad;
        doubleTrigger = false;
    }

    boolean justPressed(){
        return (!lastValue && currValue);
    }

    boolean justReleased(){
        return(lastValue && !currValue);
    }

    boolean readState() {
        lastValue = currValue;
        currValue = gamepad.getButton(key);
        return currValue;
    }

    boolean stateChanged() {
        return (currValue != lastValue);
    }

    boolean canActNow() {
        return (doubleTrigger  || lastValue != currValue);
    }



}
