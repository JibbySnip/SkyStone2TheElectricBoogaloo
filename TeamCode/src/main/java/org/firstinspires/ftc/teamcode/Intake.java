package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

class Intake {
    private DcMotorEx left,right;
    final private double UNJAM_SPEED = 0.2;
    enum jammedState{
        RIGHT,
        LEFT,
        NONE,
        REVERSED
    }
    private jammedState jammedMotor;

    Intake(DcMotorEx right, DcMotorEx left){
        this.left = left;
        this.right = right;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
    } // left and right are when viewed from the front of the robot

    void turnIntakeOn(boolean isForward) {
        if (isForward) {
            left.setPower(-0.6);
            right.setPower(-0.6);
        } else {
            left.setPower(0.6);
            right.setPower(0.6);
        }
    }


    void turnIntakeOff() {
        left.setPower(0);
        right.setPower(0);
    }




    private void unjamIntake(jammedState jamState) {
        jammedMotor = jamState;

        switch (jammedMotor) {
            case LEFT:
                left.setPower(UNJAM_SPEED);
                right.setPower(-UNJAM_SPEED);
                jammedMotor = jammedState.REVERSED;


            case RIGHT:
                left.setPower(-UNJAM_SPEED);
                right.setPower(UNJAM_SPEED);
                jammedMotor = jammedState.REVERSED;

            case REVERSED:
                turnIntakeOn(true);
                jammedMotor = jammedState.NONE;

        }
    }
    boolean isIntakeJammed() { //0 is left, 1 is right, 2 is none
        if (jammedMotor == jammedState.REVERSED) {
            unjamIntake(jammedMotor);
            return false;
        } else if (-left.getVelocity() <= (2*right.getVelocity())/3) {
            unjamIntake(jammedState.LEFT);
            return true;
        } else if (right.getVelocity() <= (2*-left.getVelocity())/3) {
            unjamIntake(jammedState.RIGHT);
            return true;
        } else {
            jammedMotor = jammedState.NONE;
            return false;
        }

    }
}
