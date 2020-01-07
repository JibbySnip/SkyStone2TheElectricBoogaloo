package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;


/**
 * A class based on the crappy use of the SRS Programmer by yours truly.
 */
public class Drawbridge {

    private final double leftUp = 1; //left and right are viewed from behind the robot
    private final double rightUp = 1;
    private Servo left,right;
    private pos desiredPos = pos.UP;
    private enum pos{
        UP,
        DOWN
    }

    Drawbridge(Servo left, Servo right) {
        this.left = left;
        this.right = right;
        left.setPosition(leftUp);
        right.setPosition(rightUp);
    }

    private void move(double leftPos,double rightPos) {
        left.setPosition(leftPos);
        right.setPosition(rightPos);
    }

    boolean isUp() {
        return (left.getPosition() == leftUp && right.getPosition() == rightUp);
    }

    public void flipPos(pos desiredPos) {
        this.desiredPos = (desiredPos == pos.UP) ? pos.DOWN : pos.UP;
    }

    void update(){
        if (desiredPos == pos.UP && !isUp()) {
            move(leftUp,rightUp);
        } else if (desiredPos == pos.DOWN && isUp()) {
            move(1-leftUp,1-rightUp);
        }
    }

}
