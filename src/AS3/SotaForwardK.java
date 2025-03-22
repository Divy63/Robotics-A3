package AS3;

import java.util.HashMap;
import java.util.Map;

import jp.vstone.RobotLib.CSotaMotion;
import org.apache.commons.math3.linear.*;
import AS3.Frames.FrameKeys;

public class SotaForwardK {

    public final Map<FrameKeys, RealMatrix> frames = new HashMap<>();
    private static final double[] ELBOW_AXIS_LEFT = {0.6258053, 0.329192519, 0.707106769};
    private static final double[] ELBOW_AXIS_RIGHT = {-0.6258053, 0.329192519, 0.707106769};
    private static final double ARM_LENGTH = 0.047;

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
        
        RealMatrix BodyShouldLeft = MatrixHelp.T(
                MatrixHelp.rotX(-angles.getEntry(Motors.get(CSotaMotion.SV_L_SHOULDER))),
                0.039,0,0.1415
        );
        RealMatrix BodyShouldRight = MatrixHelp.T(
                MatrixHelp.rotX(angles.getEntry(Motors.get(CSotaMotion.SV_R_SHOULDER))),
                -0.039,0,0.1415
        );

        RealMatrix ShouldElbowLeft = MatrixHelp.T(
                MatrixHelp.rotRodrigues(ELBOW_AXIS_LEFT[0],ELBOW_AXIS_LEFT[1],ELBOW_AXIS_LEFT[2],
                        angles.getEntry(Motors.get(CSotaMotion.SV_L_ELBOW))),
                0.0225,-0.03897,0
        );
        RealMatrix ShouldElbowRight = MatrixHelp.T(
                MatrixHelp.rotRodrigues(ELBOW_AXIS_RIGHT[0],ELBOW_AXIS_RIGHT[1],ELBOW_AXIS_RIGHT[2],
                        angles.getEntry(Motors.get(CSotaMotion.SV_R_ELBOW))),
                -0.0225,-0.03897,0
        );
        
        
        
        RealMatrix ElbowHandLeft = MatrixHelp.T(
                MatrixUtils.createRealIdentityMatrix(4),
                0.0225,-0.03897,0//todo this should take this direction then normalize it and multiply by length
        );
        
        //# Head
        RealMatrix BaseY = BodyHeadY.multiply(BaseBody);
        RealMatrix BaseP = BaseY.multiply(HeadYP);
        RealMatrix BaseR = BaseP.multiply(HeadPR);
        
        // # Hand Left
        RealMatrix BaseShouldL = BaseBody.multiply(BodyShouldLeft);
        RealMatrix BaseElbowL = BaseShouldL.multiply(ShouldElbowLeft);
        RealMatrix BaseHandL = BaseElbowL.multiply(ElbowHandLeft);
        
        // # Hand Right
        //todo
        RealMatrix BaseShouldR = BaseBody.multiply(BodyShouldRight);
        RealMatrix BaseElbowR = BaseShouldR.multiply(ShouldElbowRight);
        
        
        
        
        frames.put(FrameKeys.HEAD, BaseR); 
        frames.put(FrameKeys.L_HAND, BaseHandL);
        frames.put(FrameKeys.R_HAND, BaseElbowR);//todo
        
    }
}