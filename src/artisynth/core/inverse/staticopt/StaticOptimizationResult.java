package artisynth.core.inverse.staticopt;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Results of single-posture static optimization.
 */
public class StaticOptimizationResult {

    /**
     * Activation per exciter name (keyed by {@code MuscleExciter} or individual muscle name).
     */
    public final Map<String, Double> activationsByName = new LinkedHashMap<>();

    /**
     * Estimated muscle force per muscle/exciter name.
     */
    public final Map<String, Double> forcesByName = new LinkedHashMap<>();

    /**
     * Achieved wrench per targeted FrameSpring (name -> [Fx,Fy,Fz,Mx,My,Mz]).
     */
    public final Map<String, double[]> achievedWrenches = new LinkedHashMap<>();

    /**
     * Target wrench per targeted FrameSpring (name -> [Fx,Fy,Fz,Mx,My,Mz]).
     */
    public final Map<String, double[]> targetWrenches = new LinkedHashMap<>();
}


