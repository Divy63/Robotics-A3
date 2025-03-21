package AS3;

import jp.vstone.RobotLib.CRobotMem;
import jp.vstone.RobotLib.CRobotPose;
import jp.vstone.RobotLib.CRobotUtil;
import jp.vstone.RobotLib.CSotaMotion;
import java.util.*;
import AS3.ServoRangeTool;
import org.apache.commons.math3.*;

public class AS3_2 {
    static final String TAG = "AS3_2"; // set this to support the Sota logging system

    // private variables
    CRobotPose _sotaPose = new CRobotPose();
    CRobotMem _sotaMem = new CRobotMem();
    CSotaMotion _sotaMotion = new CSotaMotion(_sotaMem);

    private static final Byte[] SERVO_IDS = { CSotaMotion.SV_BODY_Y, CSotaMotion.SV_L_SHOULDER, CSotaMotion.SV_L_ELBOW,
            CSotaMotion.SV_R_SHOULDER, CSotaMotion.SV_R_ELBOW, CSotaMotion.SV_HEAD_Y, CSotaMotion.SV_HEAD_P,
            CSotaMotion.SV_HEAD_R };
    ServoRangeTool _servoRangeTool;

    AS3_2() {
        CRobotUtil.Log(TAG, "Start " + TAG);
    }

    boolean connect() {
        if (!_sotaMem.Connect()) {
            CRobotUtil.Log(TAG, "Sota connection failure " + TAG);
            return false;
        }

        CRobotUtil.Log(TAG, "connected " + TAG);
        _sotaMotion.InitRobot_Sota();
        return true;
    }

    void changePos(ServoRangeTool _servoRangeTool) {
        
        ServoRangeTool srt=_servoRangeTool.Load();
        srt.printMotorRanges();
        _sotaMotion.ServoOn();
        CRobotUtil.Log(TAG, "Centering Motors. Press power button to stop.");

        // Getting all pose
        CRobotPose srtMidPose=srt.getMidPose();
        CRobotPose srtMaxPose = srt.getMaxPose();
        CRobotPose srtMinPose = srt.getMinPose();


        // Converting the pose to the Short[]
        Short[] minPos=srtMinPose.getServoAngles(SERVO_IDS);
        Short[] maxPos=srtMaxPose.getServoAngles(SERVO_IDS);
        Short[] midPos=srtMidPose.getServoAngles(SERVO_IDS);

        // Go to Mid position
        Boolean isCentered=_sotaMotion.play(srtMidPose, 1000);
        _sotaMotion.waitEndinterpAll();
        CRobotUtil.wait(500);
        CRobotUtil.Log(TAG, isCentered?"Centered":"Not Centered");
        
        CRobotPose tempPose = new CRobotPose();

       for(int i=0;i<SERVO_IDS.length;i++){
            Byte servoID=SERVO_IDS[i];

            // Go to Min position
            midPos[i]=minPos[i];
            tempPose.SetPose(SERVO_IDS, midPos);
            _sotaMotion.play(tempPose, 3000);
            _sotaMotion.waitEndinterpAll();
            CRobotUtil.wait(500);

            // Go to Max position
            midPos[i] = maxPos[i];
            tempPose.SetPose(SERVO_IDS, midPos);
            _sotaMotion.play(tempPose, 3000);
            _sotaMotion.waitEndinterpAll();
            CRobotUtil.wait(500);
            midPos= srtMidPose.getServoAngles(SERVO_IDS);

            // Go to Mid position
            _sotaMotion.play(srt.getMidPose(), 3000);
            _sotaMotion.waitEndinterpAll();
            CRobotUtil.wait(500);
       }
    }

    public static void main(String args[]) {
        AS3_2 sota = new AS3_2();
        if (!sota.connect())
            return;
        CRobotUtil.Log(TAG, "Startup Successful");
        sota._servoRangeTool = new ServoRangeTool(SERVO_IDS);
        sota.changePos(sota._servoRangeTool);
        CRobotUtil.Log(TAG, "Program End Reached");
    }

}
