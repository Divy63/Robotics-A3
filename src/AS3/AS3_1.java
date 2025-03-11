import jp.vstone.RobotLib.CRobotMem;
import jp.vstone.RobotLib.CRobotPose;
import jp.vstone.RobotLib.CRobotUtil;
import jp.vstone.RobotLib.CSotaMotion;
import java.util.*;
import AS3.ServoRangeTool;
import org.apache.commons.math3.*;

public class AS3_1 {
    static final String TAG = "AS3_1";   // set this to support the Sota logging system

	// private variables
	CRobotPose _sotaPose = new CRobotPose();
	CRobotMem _sotaMem = new CRobotMem();
	CSotaMotion _sotaMotion = new CSotaMotion(_sotaMem);

	private static final Byte[] SERVO_IDS={CSotaMotion.SV_BODY_Y,CSotaMotion.SV_L_SHOULDER,CSotaMotion.SV_L_ELBOW,CSotaMotion.SV_R_SHOULDER,CSotaMotion.SV_R_ELBOW,CSotaMotion.SV_HEAD_Y,CSotaMotion.SV_HEAD_P,CSotaMotion.SV_HEAD_R};
	ServoRangeTool _servoRangeTool;

	AS3_1() {
		CRobotUtil.Log(TAG, "Start " + TAG);
	}

	boolean connect() {		
		if(!_sotaMem.Connect()) { 
			CRobotUtil.Log(TAG, "Sota connection failure " + TAG);
			return false;
		}

		CRobotUtil.Log(TAG, "connected " + TAG);
		_sotaMotion.InitRobot_Sota(); 
		return true;
	}

	void readMotorPositionLoop(ServoRangeTool _servoRangeTool) {
		// Turn servo Motor off
		_sotaMotion.ServoOff();
		CRobotUtil.Log(TAG, "Tracking motor positions. Press power button to stop.");
		while(!_sotaMotion.isButton_Power()) {

					Short[] pos = _sotaMotion.getReadpos();
					
					// CRobotUtil.Log(TAG, "Motor positions: " + Arrays.toString(pos));

					if(pos!=null){
						_servoRangeTool.register(pos);
						CRobotUtil.Log(TAG,"Registered Positions");
					}else{
						CRobotUtil.Log(TAG,"Failed to read positions");
					}

					CRobotUtil.wait(100);
					
		}
		_servoRangeTool.save();
		_servoRangeTool.printMotorRanges();
		CRobotUtil.Log(TAG, "Motor positions data saved.");
	}
    public static void main(String args[]) {
        AS3_1 sota = new AS3_1();
        if (!sota.connect())
            return;
        CRobotUtil.Log(TAG, "Startup Successful");
		sota._servoRangeTool=new ServoRangeTool(SERVO_IDS);
		sota.readMotorPositionLoop(sota._servoRangeTool);
        CRobotUtil.Log(TAG, "Program End Reached");
    }

    
}
