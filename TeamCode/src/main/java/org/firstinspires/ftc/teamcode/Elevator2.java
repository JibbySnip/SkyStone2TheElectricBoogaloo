package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


class Elevator2 {

    private DcMotorEx left,right; //when viewed from back
    private PIDFCoefficients coeffs;
    private double currentPosL,currentPosR = 0;
    private double ticksPerRev, maxExtension, desiredVelocity,spoolDiameter,manualAdjust;
    private boolean isLeftReversed,isRightReversed, useTelemetry;
    private int tolerance,pos;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * An enum of elevator states.
     */
    enum elevState {
        AUTOMATIC,
        DISABLED,
        MANUAL,
        STOP_AND_RESET_ENCODERS,


    }

    private elevState currentState = elevState.MANUAL;


    /**
     * Returns an instance of the elevator class, intended to drive linear slides
     * @author Finn
     * @param left  the left slide when viewed from the back
     * @param right the right slide when viewed from the back
     * @param coeffs    coefficients for the PID control
     * @param spoolDiameter     diameter of the spool in inches
     * @param ticksPerRev   the ticks per full motor revolution
     * @param maxExtension  the length of the full extension from the top of the lowest stage to the top of the highest stage
     * @param isLeftReversed    whether the extension direction for the left motor is the motor's backwards direction
     * @param isRightReversed   whether the extension direction for the right motor is the motor's backwards direction
     * @param tolerance the desired tolerance of run_to_position() in ticks
     * @see VirtualFourBar
     * @see DcMotorEx
     */
    Elevator2(DcMotor left, DcMotor right, PIDFCoefficients coeffs, double spoolDiameter, double ticksPerRev, double maxExtension, boolean isLeftReversed, boolean isRightReversed, int tolerance, boolean useTelemetry){
        this.left = (DcMotorEx) left;
        this.right = (DcMotorEx) right;
        this.spoolDiameter = spoolDiameter;
        this.ticksPerRev = ticksPerRev;
        this.isLeftReversed = isLeftReversed;
        this.isRightReversed = isRightReversed;
        this.maxExtension = maxExtension;
        this.tolerance = tolerance;
        this.coeffs = coeffs;
        this.useTelemetry = useTelemetry;

        motorInits();

    }


    private void motorInits(){
        left.setDirection(isLeftReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setDirection(isRightReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        updateCoeffs(coeffs);
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

    /**
     * Pretty much does everything
     */
    void update() {
        currentPosL = ticksToInches(left.getCurrentPosition(), isLeftReversed);
        currentPosR = ticksToInches(right.getCurrentPosition(), isRightReversed);
        if (Math.abs(currentPosL-currentPosR) > 10) {
            this.currentState = elevState.DISABLED;
        }
        switch (currentState) {
            case MANUAL:

                enableLift();
                if ((currentPosL >= maxExtension - 0.15 || currentPosR >= maxExtension - 0.15) && (desiredVelocity > 0)) {
                    stop();
                } else if ((currentPosL <= 0.15 || currentPosR <= 0.15) && desiredVelocity < 0) {
                    stop();
                } else if (Math.abs(desiredVelocity-left.getVelocity()) > 0.01){
                    setVelocity(desiredVelocity);
                }

            case AUTOMATIC:
                enableLift();
                if (pos != currentPosL) {
                    setPosition(pos);
                }
            case DISABLED:
                disableLift();

            case STOP_AND_RESET_ENCODERS:
                setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                currentState = elevState.AUTOMATIC;

        }
        if (useTelemetry) {
            sendTelemetry();
        }

    }

    /**
     * Updates the PIDF coefficients while the program is running
     * @param coeffs Proportional, integral, derivative, and feedforward gains
     */
    void updateCoeffs(PIDFCoefficients coeffs){
        this.coeffs = coeffs;
        left.setVelocityPIDFCoefficients(coeffs.p,coeffs.i,coeffs.d,coeffs.f);
        right.setVelocityPIDFCoefficients(coeffs.p,coeffs.i,coeffs.d,coeffs.f);
        left.setPositionPIDFCoefficients(0);
        right.setPositionPIDFCoefficients(0);
    }


    private void setMotors(double velocity){
        desiredVelocity = velocity;
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setVelocity(velocity);
        left.setVelocity(velocity);
    }

    /**
     * Sets the velocity of the motors in the manual control state
     * @param velocity the desired velocity between -1 and 1
     */
    void setVelocity(double velocity){
        this.desiredVelocity = velocity;
    }

    /**
     * Sets the current operating state of the elevator
     * @param state the desired state
     */
    void setMode(elevState state) {
        this.currentState = state;
    }

    elevState getMode() {
        return currentState;
    }
    /**
     * Runs to a position in inches. setPosition() must be used first.
     */
    void runToPosition(){
        double adjustL = isLeftReversed ? -manualAdjust : manualAdjust;
        double adjustR = isRightReversed ? -manualAdjust : manualAdjust;

        left.setTargetPosition((int) (inchesToTicks(pos,isLeftReversed) + adjustL));
        right.setTargetPosition((int) (inchesToTicks(pos,isRightReversed) + adjustR));
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Sets a position for the runToPosition() function
     * @param pos the desired position in inches
     */
    void setPosition(int pos){
        this.pos = pos;
    }

    /**
     * Disables the lift system
     */
    void disableLift() {
        left.setMotorDisable();
        right.setMotorDisable();
    }
    void enableLift() {
        left.setMotorEnable();
        right.setMotorEnable();
    }

    /**
     * Sends telemetry data to the FTC Dashboard
     */
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

    /**
     * Gives the current position of the slides
     * @return The current position of the slides, in inches
     */
    double getCurrentPos() {
        return currentPosL;
    }

    /**
     * Gives a custom increase to the target position. Used for consistently undershooting or overshooting slides.
     * @param manualAdjust the amount of adjustment, in inches.
     */
    void changeManualAdjust(double manualAdjust) {
        this.manualAdjust += manualAdjust;
    }

    /**
     * Directly changes manual adjustment
     * @param manualAdjust the desired manual adjustment
     */
    void setManualAdjust(double manualAdjust) {
        this.manualAdjust = manualAdjust;
    }

    /**
     * Gives an increase to manualAdjust of the default size, 0.25in
     */
    void incrManualAdjust() {
        manualAdjust += 0.25;
    }

    /**
     * Gives a decrease to manualAdjust of the default size, 0.25in
     */
    void decrManualAdjust() {
        manualAdjust -= 0.25;
    }


    /**
     * Stops the slides immediately, without going through pre-processing
     */
    void stop(){
        setMotors(0);
    }

    /**
     * Brings the slides to the minimum extension.
     */
    void goToBottom(){
        setPosition(0);
    }

    private void setRunMode(DcMotor.RunMode runMode) {
        left.setMode(runMode);
        right.setMode(runMode);
    }
}
