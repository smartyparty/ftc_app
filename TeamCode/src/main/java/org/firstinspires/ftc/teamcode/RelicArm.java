/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import ftclib.FtcDcMotor;
import ftclib.FtcDigitalInput;
import ftclib.FtcServo;
import trclib.TrcEnhancedServo;
import trclib.TrcPidActuator;
import trclib.TrcPidActuator2;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcPidMotor2;

public class RelicArm implements TrcPidController.PidInput//, TrcPidMotor2.PowerCompensation
{

    //Private motor variable used to create TrcPidActuator motor that supports limit sensors/PID/joystick control
    private FtcDcMotor extender_motor;
    //PID controllable motor
    private TrcPidMotor extender_pid;
    //PID controller for extender
    private TrcPidController extender_controller;
    //Expose the actuator
    public TrcPidActuator extender;

    //Touch sensors to limit elbow range
    public FtcDigitalInput elbow_lower_limit_switch;

    //Private motor variables used to create enhanced motor that support limit sensors/PID/min-max range/joystick control
    //TrcPidMotor2 allows control of two motors with one class
    public FtcDcMotor elbow1;
    public FtcDcMotor elbow2;
    //Provide PID control of 2 motors via one class
    private TrcPidMotor2 elbow_pid;
    //PID controller for both elbow motors
    private TrcPidController elbow_controller;
    //Use 2 PID-controlled motors as single actuator
    public TrcPidActuator2 elbow;

    public FtcServo claw;

    //Servo that stows/deploys the cup/grabber subassembly
    public FtcServo claw_flipper;

    //Servo that selects cup or grabber by rotating 180deg
    public FtcServo claw_rotator;

    //Private motor variable used to create PID-controllable motor
    private FtcDcMotor syringe_motor;
    //PID-controllable motor
    private TrcPidMotor syringe_pid;
    //PID controller for syringe motor
    private TrcPidController syringe_controller;
    //Use PID-controlled motor as actuator
    public TrcPidActuator syringe;

    //Servo that operates syringe stopper
    public FtcServo syringe_stopper;

    /**
     * Constructor: Create an instance of the object  .
     */
    public RelicArm()
    {
        //
        // Relic arm consists of these motors/servos:
        // - Linear actuator (Rev Core Hex motor)
        // - Elbow (Rev Core Hex motor)
        // - Claw (180 servo)
        // - Rotator to select between claw/suction (continuous servo)
        // - Flipper to stow/deploy claw (180 servo)
        // - Syringe to provide suction to cup (AndyMark NeveRest motor)
//
//
//        //Elbow
//        elbow_lower_limit_switch = new FtcDigitalInput("extenderLowerLimit");

        elbow1 = new FtcDcMotor("left_arm");
        elbow1.setInverted(false);
        //Stop motor abruptly, rather than coast/float
        elbow1.setBrakeModeEnabled(true);

        elbow2 = new FtcDcMotor( "right_arm");
        //Invert this motor if mounted opposite
        elbow2.setInverted(false);
        //Stop motor abruptly, rather than coast/float
        elbow2.setBrakeModeEnabled(true);

//        elbow_controller = new TrcPidController(
//                "elbowPidCtrl",
//                new TrcPidController.PidCoefficients(
//                        RobotInfo.RELIC_ELBOW_KP, RobotInfo.RELIC_ELBOW_KI, RobotInfo.RELIC_ELBOW_KD),
//                RobotInfo.RELIC_ELBOW_TOLERANCE, this);
//
//        elbow_pid = new TrcPidMotor2("elbow_pid",elbow1,elbow2,0.0,elbow_controller, null);
//
//        elbow = new TrcPidActuator2("elbow",elbow_pid,elbow_lower_limit_switch, RobotInfo.RELIC_ELBOW_MIN_POS, RobotInfo.RELIC_ELBOW_MAX_POS);
//        elbow.setPositionScale(RobotInfo.RELIC_ELBOW_DEGREES_PER_COUNT, RobotInfo.RELIC_ELBOW_POS_OFFSET);
//        elbow.setManualOverride(true);

//        extender_motor = new FtcDcMotor("linear_acc");
//        extender_controller = new TrcPidController("extender_motor", new TrcPidController.PidCoefficients(
//                RobotInfo.RELIC_EXTENDER_KP, RobotInfo.RELIC_EXTENDER_KI, RobotInfo.RELIC_EXTENDER_KD),
//                RobotInfo.RELIC_EXTENDER_TOLERANCE, this);
//        extender = new TrcPidActuator("extender",extender_motor,null,extender_controller,RobotInfo.RELIC_EXTENDER_MIN_POS, RobotInfo.RELIC_EXTENDER_MAX_POS);

//        claw = new FtcServo("claw_servo");
//        claw.setInverted(true);

//        syringe_motor = new FtcDcMotor("syringe_motor");
//        syringe_controller = new TrcPidController("syringe_motor", new TrcPidController.PidCoefficients(
//                RobotInfo.RELIC_SYRINGE_KP, RobotInfo.RELIC_SYRINGE_KI, RobotInfo.RELIC_SYRINGE_KD),
//                RobotInfo.RELIC_SYRINGE_TOLERANCE, this);
//        syringe = new TrcPidActuator("syringe",syringe_motor,null,syringe_controller,RobotInfo.RELIC_SYRINGE_MIN_POS, RobotInfo.RELIC_SYRINGE_MAX_POS);
//
//
//        //TODO: uncomment if syringe stopper servo installed
//        //syringe_stopper_servo = new FtcServo("relicArmStopper");
//
//        claw_rotator = new FtcServo("claw_rotator");
//
//        claw_flipper = new FtcServo("claw_flipper");

    }   //RelicArm

    //
    // Implements TrcPidController.PidInput.
    //

    /**
     * This method is called by the PID controller to get the current height of the elevator.
     *
     * @param pidCtrl specifies the PID controller who is inquiring.
     *
     * @return current elevator height.
     */
    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        return 0;   //elbow.getPosition();
    }   //getInput

    //
    // Implements TrcPidMotor.PowerCompensation interface
    //

    /**
     * This method is called to compute the power compensation to counteract the varying non-linear load.
     * This value is calculated by the following:
     *  1. Convert the current elbow position in degrees to radians to get the scalar/vertical component of the force vector
     *  2. Multiply that value by the motor power required to hold the harm at 0 degrees.
     *
     * @return compensation value of the actuator.
     */
//    @Override
//    public double getCompensation()
//    {
//        return Math.cos(Math.toRadians(elbow.getPosition())) * RobotInfo.RELIC_ELBOW_LEVEL_MOTOR_POWER;
//    }   //getCompensation

}   //class RelicArm