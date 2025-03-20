package AS3;

import jp.vstone.RobotLib.CRobotMem;
import jp.vstone.RobotLib.CRobotPose;
import jp.vstone.RobotLib.CRobotUtil;
import jp.vstone.RobotLib.CSotaMotion;

public class AS3 {
    protected static final Byte[] SERVO_IDS = {
            CSotaMotion.SV_BODY_Y,
            CSotaMotion.SV_L_SHOULDER,
            CSotaMotion.SV_L_ELBOW,
            CSotaMotion.SV_R_SHOULDER,
            CSotaMotion.SV_R_ELBOW,
            CSotaMotion.SV_HEAD_Y,
            CSotaMotion.SV_HEAD_P,
            CSotaMotion.SV_HEAD_R
    };
    static final String TAG = "AS3_1";   // set this to support the Sota logging system
    // private variables
    CRobotPose _sotaPose = new CRobotPose();
    CRobotMem _sotaMem = new CRobotMem();
    CSotaMotion _sotaMotion = new CSotaMotion(_sotaMem);
    ServoRangeTool _servoRangeTool;

    boolean connect() {
        if (!_sotaMem.Connect()) {
            CRobotUtil.Log(TAG, "Sota connection failure " + TAG);
            return false;
        }

        CRobotUtil.Log(TAG, "connected " + TAG);
        _sotaMotion.InitRobot_Sota();
        return true;
    }
}
