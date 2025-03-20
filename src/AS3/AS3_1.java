package AS3;

import jp.vstone.RobotLib.CRobotMem;
import jp.vstone.RobotLib.CRobotPose;
import jp.vstone.RobotLib.CRobotUtil;
import jp.vstone.RobotLib.CSotaMotion;
import java.util.*;
import AS3.ServoRangeTool;
import org.apache.commons.math3.*;

public class AS3_1 extends AS3 {

	AS3_1() {
		CRobotUtil.Log(TAG, "Start " + TAG);
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
    public static void main(String[] args) {
        AS3_1 sota = new AS3_1();
        if (!sota.connect())
            return;
        CRobotUtil.Log(TAG, "Startup Successful");
		sota._servoRangeTool=new ServoRangeTool(SERVO_IDS);
		sota.readMotorPositionLoop(sota._servoRangeTool);
        CRobotUtil.Log(TAG, "Program End Reached");
    }

    
}
