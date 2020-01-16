package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;



public class Elevator {
    private DcMotorEx left,right; //when viewed from back
    private double spoolDiameter;
    private PIDFCoefficients coeffs;
    private double currentPosL,currentPosR = 0;
    private double ticksPerRev, maxExtension, desiredVelocity;
    private boolean isLeftReversed,isRightReversed;
    private int tolerance;
    FtcDashboard dashboard = FtcDashboard.getInstance();




    Elevator(DcMotor left, DcMotor right, double spoolDiameter, double ticksPerRev, double maxExtension, boolean isLeftReversed, boolean isRightReversed, int tolerance){
        this.left = (DcMotorEx) left;
        this.right = (DcMotorEx) right;
        this.spoolDiameter = spoolDiameter;
        this.ticksPerRev = ticksPerRev;
        this.isLeftReversed = isLeftReversed;
        this.isRightReversed = isRightReversed;
        this.maxExtension = maxExtension;
        this.tolerance = tolerance;
        motorInits();

    }

    private void motorInits(){
        left.setDirection(isLeftReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(isRightReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setVelocityPIDFCoefficients(coeffs.p,coeffs.i,coeffs.d,coeffs.f);
        left.setPositionPIDFCoefficients(0);
        left.setTargetPositionTolerance(tolerance);
        right.setTargetPositionTolerance(tolerance);

    }


    private double ticksToInches(double ticks,boolean isReversed){
        double count = (ticks/ticksPerRev) * spoolDiameter * 2 * Math.PI;
        return isReversed ? -count : count;
    }

    private double inchesToTicks(double inches,boolean isReversed) {
        double count = (inches*ticksPerRev)/(spoolDiameter*2*Math.PI);
        return isReversed ? -count : count;
    }

    void updateCoeffs(PIDFCoefficients coeffs){
        this.coeffs = coeffs;
        motorInits();
    }
    void setVelocity(double velocity){
        desiredVelocity = velocity;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setVelocity(velocity);
        left.setVelocity(velocity);
    }
    void update() {
        currentPosL = ticksToInches(left.getCurrentPosition(), isLeftReversed);
        currentPosR = ticksToInches(right.getCurrentPosition(), isRightReversed);
        if ((currentPosL >= maxExtension - 0.25 || currentPosR >= maxExtension - 0.25) && (desiredVelocity > 0)) {
            setVelocity(0);
        }

    }

    void runToPosition(double pos){
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setTargetPosition((int) inchesToTicks(pos,isLeftReversed));
        right.setTargetPosition((int) inchesToTicks(pos,isRightReversed));
    }

    void disableLift() {
        left.setMotorDisable();
        right.setMotorDisable();
    }

    void sendTelemetry() {
        TelemetryPacket positions = new TelemetryPacket();
        positions.put("left_position",currentPosL);
        positions.put("right_position",currentPosR);
        positions.put("left_desired_position",ticksToInches(left.getTargetPosition(),isLeftReversed));
        positions.put("right_desired_position",ticksToInches(right.getTargetPosition(),isRightReversed));
        positions.put("left_velocity",left.getVelocity());
        positions.put("right_velocity",right.getVelocity());
        positions.put("desired_velocity",desiredVelocity);
        dashboard.sendTelemetryPacket(positions);
    }
}
