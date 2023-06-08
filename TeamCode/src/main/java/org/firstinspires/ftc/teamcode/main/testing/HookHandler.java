package org.firstinspires.ftc.teamcode.main.testing;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HookHandler {

    public static double leftHookUp = 0.23, leftHookDown = 0.73;
    public static double rightHookUp = 0.69, rightHookDown = 0;


    /**
     * This method returns what the value at the button state that the hooks should be in
     * <br>[0] is the left hook value; [1] is the right hook value
     * @param buttonToggle
     * @return
     */
    public double[] updatePosition(boolean buttonToggle){
        double[] output = new double[2];

        if (buttonToggle) {
            output[0] = leftHookDown;
            output[1] = rightHookDown;
        }
        if (!buttonToggle) {
            output[0] = leftHookUp;
            output[1] = rightHookUp;
        }

        return output;
    }
}
