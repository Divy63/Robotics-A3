package AS3;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.Collections;
import java.util.Map;
import java.util.TreeMap;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import jp.vstone.RobotLib.CRobotPose;
import jp.vstone.RobotLib.CRobotUtil;
import jp.vstone.RobotLib.CSotaMotion;

public class ServoRangeTool implements Serializable {
    private static final long serialVersionUID = 1L;
    
    private Short[] _minpos = null;  // internal arrays for precalcualted values
    private Short[] _maxpos = null;
    private Short[] _midpos = null;

    final static String FILENAME = "../resources/servo_ranges.dat";
    private TreeMap<Byte, Integer> _IDtoIndex;
    private Byte[] _servoIDs;

    public Map<Byte, RotLimit> RotationLimits;

    public ServoRangeTool(Byte[] servoIDs) {

        this._servoIDs=servoIDs;
        _minpos=new Short[servoIDs.length];
        _maxpos=new Short[servoIDs.length];
        _midpos=new Short[servoIDs.length];
        _IDtoIndex=new TreeMap<>();

        for (int i = 0; i < servoIDs.length; i++) {
            _IDtoIndex.put(servoIDs[i], i);
            _minpos[i] = Short.MAX_VALUE;
            _maxpos[i] = Short.MIN_VALUE;
            _midpos[i] = 0;
        }

        RotationLimits = new TreeMap<>();
        RotationLimits.put(CSotaMotion.SV_BODY_Y,RotLimit.of(-1.077363736, 1.077363736));
        RotationLimits.put(CSotaMotion.SV_L_SHOULDER, RotLimit.of(-2.617993878,1.745329252));
        RotationLimits.put(CSotaMotion.SV_L_ELBOW, RotLimit.of(-1.745329252,1.221730476));
        RotationLimits.put(CSotaMotion.SV_R_SHOULDER, RotLimit.of(-1.745329252,2.617993878));
        RotationLimits.put(CSotaMotion.SV_R_ELBOW, RotLimit.of(-1.221730476,1.745329252));
        RotationLimits.put(CSotaMotion.SV_HEAD_Y, RotLimit.of(-2.617993878, 2.617993878)); // todo not 100% sure these are the correct ranges for the head (naming in the urdf is confusing)
        RotationLimits.put(CSotaMotion.SV_HEAD_P, RotLimit.of(-1.495996502,1.495996502));
        RotationLimits.put(CSotaMotion.SV_HEAD_R, RotLimit.of(-1.495996502,1.495996502));
    }

        
    public void register(CRobotPose pose) {
        // register(pose.getServoAngles(_servoIDs));

     }
     public void register(Short[] pos) {
         for(int i=0;i<pos.length;i++){
            Short thisPos=pos[i];
            CRobotUtil.Log("MIN", "Servo ID: " + i + " Position: " + pos[i]);

            // Getting the minimum value of the position
            if(thisPos<_minpos[i]){
                
                _minpos[i]=pos[i];
            }
            // Getting the maximum value of the position
            if(thisPos>_maxpos[i]){
                _maxpos[i]=pos[i];
            }

            // Getting the middle value of the position
            _midpos[i]=(short)((_minpos[i]+_maxpos[i])/2);
         }
     }

    
    ///==================== Export as CRobotPose objects
    ///====================
    private CRobotPose makePose(Short[] pos) {  // convert short[] to CRobotPose object
        CRobotPose pose=new CRobotPose();
        pose.SetPose(_servoIDs, pos);
        return pose;
    }

    public CRobotPose getMinPose() { return makePose(_minpos);}
    public CRobotPose getMaxPose() { return makePose(_maxpos);}
    public CRobotPose getMidPose() { return makePose(_midpos);}

    ///==================== Angle <-> motor pos conversions
    ///====================
    public RealVector calcAngles(CRobotPose pose) { // convert pose in motor positions to radians
        Short[] pos=pose.getServoAngles(_servoIDs);
        RealVector angles=new ArrayRealVector(pos.length);
        for(int i=0;i<pos.length;i++){
            angles.setEntry(i, posToRad((byte)(i - 1), pos[i]));
        }
        return angles; 
    }

    public CRobotPose calcMotorValues(RealVector angles) { // convert pose in angles to motor positions
        CRobotPose pose=new CRobotPose();
        Short[] pos = new Short[_servoIDs.length];
        for(int i=0;i<pos.length;i++){
            pos[i] = radToPos(_servoIDs[i], angles.getEntry(i));
        }
        pose.SetPose(AS3.SERVO_IDS,pos);
        return pose;
    }

    private double posToRad(Byte servoID, Short pos) { // convert motor position to angle, in radians 
        RotLimit rotLimit = RotationLimits.get(servoID);
        Short[] posLimit = getPosArray(servoID);
        return rotLimit.minRad() + (pos - posLimit[0]) * (rotLimit.maxRad() - rotLimit.minRad())/(posLimit[2] - posLimit[0]);
    }

    private short radToPos(Byte servoID, double angle) { // convert angles, in radians, to motor position
        RotLimit rotLimit = RotationLimits.get(servoID);
        Short[] posLimit = getPosArray(servoID);
        return (short)(posLimit[0] + (angle - rotLimit.minRad()) * (posLimit[2] - posLimit[0])/(rotLimit.maxRad() - rotLimit.minRad()));
    }
    
    private Short[] getPosArray(Byte servoID) {
        int idx = _IDtoIndex.get(servoID);
        return new Short[]{_minpos[idx], _midpos[idx], _maxpos[idx]};
    }
    
	///==================== Pretty Print
    /// ///====================
	private String formattedLine(String title, Byte servoID, Short[] minpos, Short[] maxpos, Short[] middle, Short[] pos) {
        int i = _IDtoIndex.get(servoID);
        double rad = 0;
        if (pos != null) rad = posToRad(servoID, pos[i]);
		String format = "%14s %8d %8d %8d    %.2f rad";
		return String.format(format, title, minpos[i], middle[i], maxpos[i], rad);
	}

    public void printMotorRanges() {printMotorRanges(null);}
	public void printMotorRanges(Short[] pos) {  // will print the current position as given by the pos array
		System.out.println("-------------");
		System.out.println( formattedLine("Body Y: ", CSotaMotion.SV_BODY_Y, _minpos, _maxpos, _midpos, pos));
		System.out.println( formattedLine("L Shoulder: ", CSotaMotion.SV_L_SHOULDER, _minpos, _maxpos, _midpos, pos));
        System.out.println( formattedLine("L Elbow: ", CSotaMotion.SV_L_ELBOW, _minpos, _maxpos, _midpos, pos));
		System.out.println( formattedLine("R Shoulder: ", CSotaMotion.SV_R_SHOULDER, _minpos, _maxpos, _midpos, pos));
		System.out.println( formattedLine("R Elbow: ", CSotaMotion.SV_R_ELBOW, _minpos, _maxpos, _midpos, pos));
        System.out.println( formattedLine("Head Y: ", CSotaMotion.SV_HEAD_Y, _minpos, _maxpos, _midpos, pos));
		System.out.println( formattedLine("Head P: ", CSotaMotion.SV_HEAD_P, _minpos, _maxpos, _midpos, pos));
        System.out.println( formattedLine("Head R: ", CSotaMotion.SV_HEAD_R, _minpos, _maxpos, _midpos, pos));
	}

    ///==================== LOAD AND SAVE
    ///====================
    public static ServoRangeTool Load(){ return ServoRangeTool.Load(FILENAME);}
    public static ServoRangeTool Load(String filename){
        try (ObjectInputStream input=new ObjectInputStream(new FileInputStream(filename))){
            return (ServoRangeTool)input.readObject();
            
        }catch(IOException ioe){
            ioe.printStackTrace();
            return null; 
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
            return null;
        }
    }

    public void save() { save(FILENAME);}
    public void save(String filename) {
        try (ObjectOutputStream output=new ObjectOutputStream(new FileOutputStream(filename))){
            output.writeObject(this);
        }catch(IOException ioe){
            ioe.printStackTrace();
        }
    }
}