package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Elevator {
    private DcMotorEx left,right; //when viewed from back
    private double spoolDiameter;
    private PIDFCoefficients coeffs;
    private double currentPosL,currentPosR = 0;
    private double ticksPerRev, maxExtension, currentVelocity;
    private boolean isLeftReversed,isRightReversed;



    Elevator(DcMotor left, DcMotor right, double spoolDiameter, double ticksPerRev,double maxExtension, boolean isLeftReversed,boolean isRightReversed){
        this.left = (DcMotorEx) left;
        this.right = (DcMotorEx) right;
        this.spoolDiameter = spoolDiameter;
        this.ticksPerRev = ticksPerRev;
        this.isLeftReversed = isLeftReversed;
        this.isRightReversed = isRightReversed;
        this.maxExtension = maxExtension;
        motorInits();

    }

    private void motorInits(){
        left.setDirection(isLeftReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(isRightReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setVelocityPIDFCoefficients(coeffs.p,coeffs.i,coeffs.d,coeffs.f);
        right.setVelocityPIDFCoefficients(coeffs.p,coeffs.i,coeffs.d,coeffs.f);

    }


    private double ticksToInches(double ticks,boolean isReversed){
        double count = (ticks/ticksPerRev) * spoolDiameter * 2 * Math.PI;
        return isReversed ? -count : count;
    }
    void updateCoeffs(PIDFCoefficients coeffs){
        this.coeffs = coeffs;
        motorInits();
    }
    void setVelocity(double velocity){
        right.setVelocity(velocity);
        left.setVelocity(velocity);
    }
    void update(){
        currentPosL = ticksToInches(left.getCurrentPosition(),isLeftReversed);
        currentPosR = ticksToInches(right.getCurrentPosition(),isRightReversed);
        if ((currentPosL >= maxExtension - 0.25 || currentPosR >= maxExtension - 0.25) && (currentVelocity > 0)){
            setVelocity(0);
        }


    }
}
