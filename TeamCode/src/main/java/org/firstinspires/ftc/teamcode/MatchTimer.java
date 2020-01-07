package org.firstinspires.ftc.teamcode;

class MatchTimer {
    private long time;
    enum matchPhase {
        INIT,
        AUTO,
        PICK_UP_REMOTES,
        TELEOP,
        ENDGAME
    }

    long getTime(){
        return time;
    }
}
