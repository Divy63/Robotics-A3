package AS3;

import java.util.HashMap;
import java.util.Map;

import jp.vstone.RobotLib.CSotaMotion;
import org.apache.commons.math3.linear.*;
import AS3.Frames.FrameKeys;

public class SotaForwardK {

    public final Map<FrameKeys, RealMatrix> frames = new HashMap<>();

    public RealVector endEffectorState = null; // a single vector representing the combined state of the end effector. needs to be in the same order as in the IK

    public SotaForwardK(double[] angles) { this(MatrixUtils.createRealVector(angles)); }
    public SotaForwardK(RealVector angles) {
        // constructs all the frame matrices and stores them in a Map that maps
        //  a frame type (FrameKey) to the frame matrix.
        
        //# Intermediate Transforms
        RealMatrix BaseBody = MatrixHelp.T(
                MatrixHelp.rotZ(angles.getEntry(Motors.get(CSotaMotion.SV_BODY_Y))),
                0,0,0.005);
        RealMatrix BodyHeadY = MatrixHelp.T(
                MatrixHelp.rotZ(Motors.get(CSotaMotion.SV_HEAD_Y)),
                0,0,0.19
        );
        RealMatrix HeadP=MatrixHelp.T(
                MatrixHelp.rotX(Motors.get(CSotaMotion.SV_HEAD_P)),
                0,0,0
        );
        RealMatrix HeadR = MatrixHelp.T(
                MatrixHelp.rotY(Motors.get(CSotaMotion.SV_HEAD_R)),
                0,0,0
        );
        
        //# Head
        frames.put(FrameKeys.HEAD, BaseBody); //todo
        
    }
}