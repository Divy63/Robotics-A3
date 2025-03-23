package AS3;

import java.util.HashMap;
import java.util.Map;
import AS3.Frames.FrameKeys;
import jp.vstone.RobotLib.CSotaMotion;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class JacobianMaker {
    private static JacobianMaker instance;
    private Map<FrameKeys, Byte[]> keyLUT;
    private JacobianMaker() {
        this.keyLUT = new HashMap<>();
        this.keyLUT.put(FrameKeys.L_HAND, new Byte[]{CSotaMotion.SV_BODY_Y,CSotaMotion.SV_L_SHOULDER, CSotaMotion.SV_L_ELBOW});
        this.keyLUT.put(FrameKeys.R_HAND, new Byte[]{CSotaMotion.SV_BODY_Y,CSotaMotion.SV_R_SHOULDER, CSotaMotion.SV_R_ELBOW});
        this.keyLUT.put(FrameKeys.HEAD, new Byte[]{CSotaMotion.SV_BODY_Y, CSotaMotion.SV_HEAD_Y, CSotaMotion.SV_HEAD_R, CSotaMotion.SV_HEAD_P});
    }
    
    public static JacobianMaker getInstance() {
        if (instance == null) {
            instance = new JacobianMaker();
        }
        return instance;
    }
    
    public RealMatrix makeJacobian(RealVector currentAngles, FrameKeys frameType) {
        SotaForwardK FKCurr = new SotaForwardK(currentAngles);
        RealVector FKPosCurr = MatrixHelp.getTrans(FKCurr.frames.get(frameType));
        
        Byte[] keys = keyLUT.get(frameType);
        double[][] j = new double[3][keys.length];
        
        double[] angles; 
        for (int i = 0; i < keys.length; i++) {
            angles = currentAngles.toArray();
            angles[Motors.get(keys[i])] = angles[i] + SotaInverseK.NUMERICAL_DELTA_rad;
            SotaForwardK FKNext = new SotaForwardK(angles);
            RealVector FKPosNext = MatrixHelp.getTrans(FKNext.frames.get(frameType));
            for (int k = 0; k < 3; k++) {
                j[k][i] = (FKPosNext.getEntry(k) - FKPosCurr.getEntry(k)) / SotaInverseK.NUMERICAL_DELTA_rad;
            }
        }

        return MatrixUtils.createRealMatrix(j);
    }
}
