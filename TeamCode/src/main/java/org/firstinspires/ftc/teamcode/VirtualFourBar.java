package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

class VirtualFourBar {
    CRServo left,right;
    Servo gripper;

    VirtualFourBar(CRServo left, CRServo right, Servo gripper, boolean leftReversed, boolean rightReversed){
        this.left = left;
        this.right = right;
        this.gripper = gripper;
        this.left.setDirection(leftReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        this.right.setDirection(rightReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }
    void rotate(double power){
         left.setPower(power);
         right.setPower(power);
    }
    void stop(){
        rotate(0);
    }
    void openGripper(){
        gripper.setPosition(0);
    }
    void closeGripper(){
        gripper.setPosition(1);
    }
    void toggleGripper(){
        if (gripper.getPosition() == 1) {
            openGripper();
        } else {
            closeGripper();
        }
    }
}
