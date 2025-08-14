package artisynth.core.inverse.staticopt;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;

/**
 * Minimal MOT static posture importer (first data row only) with z-axis rotations.
 *
 * Heuristics mirror TL4 logic: columns named like "pelvic_tilt", "L1_FE", "T7_FE", etc.,
 * are mapped to corresponding bodies and applied as rotations about z around a given pivot body.
 */
public class MotPostureLoader {

    /**
     * Apply first-frame posture from a MOT file: rotate sacrum (pelvic_tilt) then each vertebra column
     * (e.g., L1_FE) and everything above it by that many degrees around z, using the indicated pivot chain.
     */
    public static void applyFirstFrameZRotations(String motFile, MechModel mech, String[] vertebraeOrder, String pivotForSacrum) throws IOException {
        try (BufferedReader reader = new BufferedReader(new FileReader(motFile))) {
            // Seek header line with "time"
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.trim().startsWith("time")) break;
            }
            if (line == null) return;

            String[] columnNames = line.trim().split("\\s+");
            Map<Integer,String> columnToBody = new HashMap<>();

            for (int i = 0; i < columnNames.length; i++) {
                String col = columnNames[i];
                String bodyName = null;
                if (col.equals("pelvic_tilt")) {
                    bodyName = "sacrum";
                } else if (col.endsWith("_FE")) {
                    String first = col.split("_")[0];
                    if (first.startsWith("L")) bodyName = "lumbar" + first.substring(1);
                    else if (first.startsWith("T")) bodyName = "thoracic" + first.substring(1);
                    else if (first.startsWith("S")) bodyName = "sacrum";
                }
                if (bodyName != null) columnToBody.put(i, bodyName);
            }

            // First data line
            line = reader.readLine();
            if (line == null) return;
            String[] values = line.trim().split("\\s+");

            @SuppressWarnings("unchecked")
            ComponentList<RigidBody> bodies = (ComponentList<RigidBody>) mech.get("bodyset");

            // Sacrum: rotate everything above
            for (Map.Entry<Integer,String> e : columnToBody.entrySet()) {
                if ("sacrum".equals(e.getValue())) {
                    double deg = Double.parseDouble(values[e.getKey()]);
                    for (String v : vertebraeOrder) {
                        applyZRotationAboutPivot(bodies, v, deg, pivotForSacrum);
                    }
                    break;
                }
            }

            // Each vertebra and above
            for (String vertebra : vertebraeOrder) {
                for (Map.Entry<Integer,String> e : columnToBody.entrySet()) {
                    if (vertebra.equals(e.getValue())) {
                        double deg = Double.parseDouble(values[e.getKey()]);
                        int startIdx = indexOf(vertebraeOrder, vertebra);
                        for (int i = startIdx; i < vertebraeOrder.length; i++) {
                            applyZRotationAboutPivot(bodies, vertebraeOrder[i], deg, vertebra);
                        }
                        break;
                    }
                }
            }
            mech.updateForces(0.0);
        }
    }

    private static int indexOf(String[] arr, String val) {
        for (int i = 0; i < arr.length; i++) if (arr[i].equals(val)) return i;
        return -1;
    }

    private static void applyZRotationAboutPivot(ComponentList<RigidBody> bodies, String bodyName, double rotationDeg, String pivotVertebraName) {
        RigidBody body = bodies.get(bodyName);
        if (body == null) return;
        RigidBody pivot = bodies.get(pivotVertebraName);
        if (pivot == null) return;

        Point3d pivotPos = new Point3d(pivot.getPosition());
        double a = pivotPos.x, b = pivotPos.y, c = pivotPos.z;
        AxisAngle rotAxis = new AxisAngle(0, 0, 1, Math.toRadians(rotationDeg));

        RigidTransform3d toOrigin = new RigidTransform3d(-a, -b, -c, 0, 0, 0);
        RigidTransform3d rot = new RigidTransform3d();
        rot.setRotation(new RotationMatrix3d());
        rot.setRotation(rotAxis);
        RigidTransform3d back = new RigidTransform3d(a, b, c, 0, 0, 0);

        RigidTransform3d finalT = new RigidTransform3d();
        finalT.mul(back, rot);
        finalT.mul(finalT, toOrigin);

        RigidTransform3d newPose = new RigidTransform3d(body.getPose());
        newPose.mul(finalT, newPose);
        body.setPose(newPose);
    }
}


