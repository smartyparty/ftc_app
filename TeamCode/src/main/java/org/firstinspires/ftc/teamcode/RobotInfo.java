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

import com.qualcomm.robotcore.hardware.DcMotor;

class RobotInfo
{
    static final float MM_PER_INCH                      = 25.4f;

    //
    // Color sensor values.
    //
    static final double RED1_LOW_THRESHOLD              = 1.0;
    static final double RED1_HIGH_THRESHOLD             = 30.0;
    static final double BLUE_LOW_THRESHOLD              = 40.0;
    static final double BLUE_HIGH_THRESHOLD             = 210.0;
    static final double RED2_LOW_THRESHOLD              = 340.0;
    static final double RED2_HIGH_THRESHOLD             = 359.0;

    //
    // DriveBase subsystem.
    //
    static final DcMotor.RunMode DRIVE_MOTOR_MODE       = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final double TURN_POWER_LIMIT                = 0.5;

    static final double ENCODER_X_KP                    = 0.15;
    static final double ENCODER_X_KI                    = 0.0;
    static final double ENCODER_X_KD                    = 0.015;
    static final double ENCODER_X_TOLERANCE             = 2.0;
    static final double ENCODER_X_INCHES_PER_COUNT      = 12.6/1120;  //Inches per revolution / counts per revolution

//    static final double SMALL_X_THRESHOLD               = 8.0;
//    static final double ENCODER_SMALL_X_KP              = 0.2;
//    static final double ENCODER_SMALL_X_KI              = 0.0;
//    static final double ENCODER_SMALL_X_KD              = 0.0;

    static final double ENCODER_Y_KP                    = 0.02;
    static final double ENCODER_Y_KI                    = 0.0;
    static final double ENCODER_Y_KD                    = 0.0022;
    static final double ENCODER_Y_TOLERANCE             = 1.0;
    static final double ENCODER_Y_INCHES_PER_COUNT      = 12.6/1120;  //Inches per revolution / counts per revolution

//    static final double SMALL_Y_THRESHOLD               = 8.0;
//    static final double ENCODER_SMALL_Y_KP              = 0.045;
//    static final double ENCODER_SMALL_Y_KI              = 0.0;
//    static final double ENCODER_SMALL_Y_KD              = 0.001;

    static final double ANALOG_GYRO_SCALE               = 1.0136;
    static final double ANALOG_GYRO_VOLT_PER_DEG_PER_SEC= 0.007;

    static final double GYRO_KP                         = 0.018;
    static final double GYRO_KI                         = 0.0;
    static final double GYRO_KD                         = 0.002;
    static final double GYRO_TOLERANCE                  = 1.0;

//    static final double SMALL_TURN_THRESHOLD            = 15.0;
//    static final double GYRO_SMALL_TURN_KP              = 0.03;
//    static final double GYRO_SMALL_TURN_KI              = 0.0;
//    static final double GYRO_SMALL_TURN_KD              = 0.001;

    static final double PIDDRIVE_STALL_TIMEOUT          = 0.25;     //in msec.

    static final double VISION_KP                       = 0.0125;
    static final double VISION_KI                       = 0.0;
    static final double VISION_KD                       = 0.0;
    static final double VISION_TOLERANCE                = 1.0;

    static final double AUTO_PARKING_POWER              = 0.8;

    //
    // JewelBar subsystem.
    //
    //These values depend on whether servo is continuous or 180-deg.
    static final double JEWEL_ARM_RETRACTED             = 0.1;
    static final double JEWEL_ARM_EXTENDED              = 0.6;
    static final double JEWEL_DISPLACEMENT_DRIVE_TIME   = 0.3;
    static final double JEWEL_DISPLACEMENT_DRIVE_POWER  = 0.3;

    //
    // RelicArm subsystem.
    //
    static final double RELIC_EXTENDER_POWER               = 0.0;
    static final double RELIC_EXTENDER_INCHES_PER_COUNT    = 0.0;
    static final double RELIC_EXTENDER_KP                  = 0.1;  //???
    static final double RELIC_EXTENDER_KI                  = 0.0;
    static final double RELIC_EXTENDER_KD                  = 0.0;
    static final double RELIC_EXTENDER_TOLERANCE           = 2.0;
    static final double RELIC_EXTENDER_MIN_POS             = 0.1;   //Do not allow retraction past this position
    static final double RELIC_EXTENDER_MAX_POS             = 200.0;  //Do not allow extension past this positiuon
    static final double RELIC_EXTENDER_CAL_POWER           = 0.3;

    static final double RELIC_CLAW_POWER                = 0.5;
    static final double RELIC_CLAW_CLOSE                = 0.0;
    static final double RELIC_CLAW_OPEN                 = 0.5;

    static final double RELIC_ROTATOR_POWER             = 0.5;      //Power for rotator/selector servo
    static final double RELIC_ROTATOR_CLAW              = 0.0;      //Position to select grabber
    static final double RELIC_ROTATOR_CUP               = 0.0;      //Position to select suction cup

    static final double RELIC_FLIPPER_POWER             = 0.5;      //Power for deploy/retract of grabber/cup assembly
    static final double RELIC_FLIPPER_STOW              = 0.0;      //Position to stow grabber/cup assembly
    static final double RELIC_FLIPPER_DEPLOY            = 0.0;      //Position to deploy grabber/cup assembly

    static final double RELIC_SYRINGE_POWER             = 0.5;      //Power for syringe
    static final double RELIC_SYRINGE_KP                  = 0.1;  //???
    static final double RELIC_SYRINGE_KI                  = 0.0;
    static final double RELIC_SYRINGE_KD                  = 0.0;
    static final double RELIC_SYRINGE_TOLERANCE           = 2.0;
    static final double RELIC_SYRINGE_MIN_POS           = 0.0;
    static final double RELIC_SYRINGE_MAX_POS           = 0.0;

    static final double RELIC_STOPPER_POWER             = 0.5;      //Power for syringe stopper
    static final double RELIC_STOPPER_MIN_POS           = 0.0;
    static final double RELIC_STOPPER_MAX_POS           = 0.0;

    // NOTE: The calculation of RELIC_ELBOW_LEVEL_MOTOR_POWER is not needed unless we want to hold
    // the arm steady at certain elevations in autonomous mode.
    //
    // To counteract gravity, we need to add power compensation to the elbow motor.
    // We are using a Rev Core Hex motor. The performance spec of this motor is:
    //    Free Speed: 125 RPM
    //    Stall Torque: 3.2 N-m = 453.1581832897921 oz-in
    //    Stall Current: 4.4 A
    //    Gear Ratio: 72:1
    //    Encoder Counts per Revolution
    //          At the motor - 4 counts/revolution
    //          At the output - 288 counts/revolution (4 x 72:1 gear ratio)
    // The greatest effort to hold the elbow is when the arm is horizontal.
    // Arm length = 19 inches.
    // Weight of arm at 19-inch from fulcrum = 21.16 oz.
    // Torque at fulcrum = Weight of arm * arm length = 21.16*19 = 402.04 oz-in.
    // Additional gear ratio = 72:1.
    // Torque at motor = 402.04/72 = 201.02 oz-in.
    // To maintain arm at horizontal position, we need to apply 201.02/593 = 0.339 of full power.
    // To calculate Degrees/EncoderCount, one revolution of the motor will give 288 counts.
    // One revolution of the motor will yield 5-degree movement of the arm because of the 72:1 gear ratio.
    // Therefore, the degrees_per_count = 5.0/288.0.
    // The arm is resting at 40 degrees below the horizontal position. A lower limit switch will clear the encoder
    // when the arm is at rest. So the scaled position read from the encoder should subtract 40 degrees from it.
    //
    static final double RELIC_ELBOW_DEGREES_PER_COUNT   = (5.0/288.0);
    static final double RELIC_ELBOW_POS_OFFSET          = -40.0;        //Offset in degrees for elbow resting position
    static final double RELIC_ELBOW_LEVEL_MOTOR_POWER   = 0.339*1.0;
    static final double RELIC_ELBOW_KP                  = 0.1;  //???
    static final double RELIC_ELBOW_KI                  = 0.0;
    static final double RELIC_ELBOW_KD                  = 0.0;
    static final double RELIC_ELBOW_TOLERANCE           = 2.0;
    static final double RELIC_ELBOW_MIN_POS             = RELIC_ELBOW_POS_OFFSET;   //Do not allow rotation past resting position
    static final double RELIC_ELBOW_MAX_POS             = 200.0;    //???
    static final double RELIC_ELBOW_CAL_POWER           = 0.3;

}   //class RobotInfo