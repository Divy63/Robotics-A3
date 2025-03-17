package AS3;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.TreeMap;

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
        
        return null; // TODO
    }

    public CRobotPose calcMotorValues(RealVector angles) { // convert pose in angles to motor positions
        return null; // TODO
    }

    private double posToRad(Byte servoID, Short pos) { // convert motor position to angle, in radians 
        return 0; // TODO
    }

    private short radToPos(Byte servoID, double angle) { // convert angles, in radians, to motor position
        return 0; // TODO
    }
    
	///==================== Pretty Print
    /// ///====================
	private String formattedLine(String title, Byte servoID, Short[] minpos, Short[] maxpos, Short[] middle, Short[] pos) {
		// int i = 0;
        int i = _IDtoIndex.get(servoID);
        double rad = 0;
        // if (pos != null) rad = posToRad(servoID, pos[i]);
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