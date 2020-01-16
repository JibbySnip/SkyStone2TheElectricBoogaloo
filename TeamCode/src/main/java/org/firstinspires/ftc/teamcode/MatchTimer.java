package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

class MatchTimer {
    private ElapsedTime matchTimer;
    private ElapsedTime phaseTimer;

    enum matchPhase {
        NULL,
        AUTO,
        PICK_UP_REMOTES,
        TELEOP,
        ENDGAME,
        GG
    }
    private matchPhase phase;

    MatchTimer(matchPhase start) {
        phase = start;
        matchTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        phaseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }
    MatchTimer() {
        phase = matchPhase.NULL;
        matchTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        phaseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    void setPhase(matchPhase newPhase, boolean resetAll) {
        phase = newPhase;
        resetTimers(resetAll);
    }

    private void resetTimers(boolean resetAll) {
        phaseTimer.reset();
        if (resetAll) matchTimer.reset();
    }

    long getMatchTime(){
        return (long) phaseTimer.milliseconds();
    }
    long getPhaseTime(){
        return (long) phaseTimer.milliseconds();
    }

    private double sToMs(double ms) {
        return ms*1000;
    }


    matchPhase checkPhase(){
        switch (phase) {
            case AUTO:
                if (phaseTimer.milliseconds() >= sToMs(30)) {
                    phaseTimer.reset();
                    phase = matchPhase.PICK_UP_REMOTES;
                }
            case PICK_UP_REMOTES:
                if (phaseTimer.milliseconds() >= sToMs(8)) {
                    phaseTimer.reset();
                    phase = matchPhase.TELEOP;
                }
            case TELEOP:
                if (phaseTimer.milliseconds() >= sToMs(120)) {
                    phaseTimer.reset();
                    phase = matchPhase.ENDGAME;
                }

            case ENDGAME:
                if (phaseTimer.milliseconds() >= sToMs(30)) {
                    phaseTimer.reset();
                    phase = matchPhase.GG;
                }
            case GG:

            case NULL:

        }
        return phase;
    }
}
