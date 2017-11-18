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
    static final double RED1_HIGH_THRESHOLD             = 40.0;
    static final double BLUE_LOW_THRESHOLD              = 150.0;
    static final double BLUE_HIGH_THRESHOLD             = 220.0;
    static final double RED2_LOW_THRESHOLD              = 350.0;
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

}   //class RobotInfo