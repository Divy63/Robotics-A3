package AS3;

import jp.vstone.RobotLib.CRobotPose;
import jp.vstone.RobotLib.CRobotUtil;
import jp.vstone.RobotLib.CSotaMotion;

public class AS3_3 extends AS3 {
    
    public void readPosLoop() {
        _sotaMotion.ServoOff();
        CRobotUtil.Log(TAG, "Tracking motor positions. Press power button to stop.");
        while(!_sotaMotion.isButton_Power()) {

            Short[] pos = _sotaMotion.getReadpos();

            CRobotPose pose = _sotaMotion.getReadPose();
            CRobotUtil.Log(TAG, "-------------");
            CRobotUtil.Log(TAG, "Head R: "+pose.getServoAngle(CSotaMotion.SV_HEAD_R)+"          ");  // add spaces since we are not clearing screen
            CRobotUtil.Log(TAG, "Head P: "+pose.getServoAngle(CSotaMotion.SV_HEAD_P)+"          ");
            CRobotUtil.Log(TAG, "Head Y: "+pose.getServoAngle(CSotaMotion.SV_HEAD_Y)+"          ");
            CRobotUtil.Log(TAG, "Body Y: "+pose.getServoAngle(CSotaMotion.SV_BODY_Y)+"          ");
            CRobotUtil.Log(TAG, "L Elbow: "+pose.getServoAngle(CSotaMotion.SV_L_ELBOW)+"          ");
            CRobotUtil.Log(TAG, "L Shoulder: "+pose.getServoAngle(CSotaMotion.SV_L_SHOULDER)+"          ");
            CRobotUtil.Log(TAG, "R Elbow: "+pose.getServoAngle(CSotaMotion.SV_R_ELBOW)+"          ");
            CRobotUtil.Log(TAG, "R Shoulder: "+pose.getServoAngle(CSotaMotion.SV_R_SHOULDER)+"          ");
            
            _servoRangeTool.printMotorRanges(pos);
            MatrixHelp.printVector("angles",_servoRangeTool.calcAngles(pose));
            

            System.out.flush();  // force stdout flush before waiting to avoid tearing / flicker.

            CRobotUtil.wait(100);

        }
    }
    
    public static void main(String[] args) {
        AS3_3 sota = new AS3_3();
        if(!sota.connect())
            return;
        CRobotUtil.Log(TAG, "Startup Successful");
        sota._servoRangeTool=new ServoRangeTool(SERVO_IDS);
        
        

        CRobotUtil.Log(TAG, "Program End Reached");
    }
}
