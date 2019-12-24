package org.firstinspires.ftc.teamcode.Gamepad;

public class Toggle extends Button {
    private boolean isToggled = false;
    Toggle(gamepadKeys.key key, gamepadEx gamepad, boolean canDoubleTrigger){
        super(key,gamepad,canDoubleTrigger);
    }
    @Override
    boolean readState( ) {
        lastValue = currValue;
        currValue = gamepad.getButton(key);
        isToggled = (lastValue && !currValue) != isToggled;
        return isToggled;
    }
    boolean isToggled() {
        return isToggled;
    }

}
