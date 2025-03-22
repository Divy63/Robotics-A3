package AS3;

import java.util.HashMap;
import java.util.Map;

public class Motors {
    // I'm very confused as to where these should actually be this project is a mess
    // all this class does is provide a singleton of the motor idx map
    private static Map<Byte, Integer> motorMap;
    
    private static Map<Byte, Integer> getInstance() {  
        if (motorMap == null) {
            motorMap = new HashMap<>();
            for (int i = 0; i < AS3.SERVO_IDS.length; i++) {
                motorMap.put(AS3.SERVO_IDS[i], i);
            }
        }
        return motorMap;
    }
    
    public static int get(Byte idx){
        return getInstance().get(idx);
    }
}
