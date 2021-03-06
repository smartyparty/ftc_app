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

import ftclib.FtcServo;


/**
 *
 * This class represents the arm on the robot which is used to displace the
 * jewel of the appropriate color.
 * The arm consists of
 *  1. A single servo which raises and lowers the arm a fixed distance
 *  2. A color sensor mounted at the end of the arm to detect jewel color
 *
 *  This class handles the raising/lowering of the arm, while the color
 *  detection logic is handled in the Robot class itself, where an analog
 *  trigger for the jewel color sensor raises an event when it detects
 *  colors
* */
public class JewelArm
{

    private String instanceName;
    private FtcServo verticalServo;
    private boolean armExtended = false;
    private double extendedPos = RobotInfo.JEWEL_ARM_EXTENDED;
    private double retractedPos = RobotInfo.JEWEL_ARM_RETRACTED;

    /**
     * Constructor: Create an instance of the object  .
     */
    public JewelArm(String instanceName)
    {
        this.instanceName = instanceName;
        verticalServo = new FtcServo("jewel_servo");
        //verticalServo.setPhysicalRange(0,180);
        //verticalServo.setLogicalRange(0.1,0.9);

        //Define the physical range of the HS485HB servo
        //Logical range will be 0 to 1 and scaled accordingly
        verticalServo.setInverted(false);
        verticalServo.setPosition(retractedPos);
    }   //JewelArm

    public String toString()
    {
        return instanceName;
    }

    public double getExtendedPos() {
        return extendedPos;
    }

    public double getRetractedPos() {
        return retractedPos;
    }

    public void setExtendedPos(double pos) {
        extendedPos = pos;
    }

    public void setRetractedPos(double pos) {
        retractedPos = pos;
    }

    public double getPosition() {
        return verticalServo.getPosition();
    }

    public void setExtended(boolean extended)
    {
        armExtended = extended;
        verticalServo.setPosition(extended ? extendedPos: retractedPos);
    }

//    public void setSweepPosition(double pos)
//    {
//        horizontalServo.setPosition(pos);
//    }

}   //class JewelArm