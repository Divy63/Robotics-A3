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
                MatrixHelp.rotZ(angles.getEntry(Motors.get(CSotaMotion.SV_HEAD_Y))),
                0,0,0.19
        );
        RealMatrix HeadYP=MatrixHelp.T(
                MatrixHelp.rotX(angles.getEntry(Motors.get(CSotaMotion.SV_HEAD_P))),
                0,0,0
        );
        RealMatrix HeadPR = MatrixHelp.T(
                MatrixHelp.rotY(angles.getEntry(Motors.get(CSotaMotion.SV_HEAD_R))),
                0,0,0
        );
        
        MatrixHelp.printMatrix("BaseBody", BaseBody, 1,3);
        MatrixHelp.printMatrix("BodyHeadY", BodyHeadY, 1,3);
        
        //# Head
        RealMatrix BaseY = BodyHeadY.multiply(BaseBody);
        RealMatrix BaseP = BaseY.multiply(HeadYP);
        RealMatrix BaseR = BaseP.multiply(HeadPR);
        
        frames.put(FrameKeys.HEAD, BaseR); //todo
        
    }
}