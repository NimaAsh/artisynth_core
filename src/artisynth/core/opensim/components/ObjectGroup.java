package artisynth.core.opensim.components;

import java.util.ArrayList;
import java.util.List;
import artisynth.core.mechmodels.MuscleComponent;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.opensim.OpenSimParser;

public class ObjectGroup extends OpenSimObject {
   
   ArrayList<String> members;
   
   public ObjectGroup() {
      members = new ArrayList<>();
   }
   
   public void addMember(String member) {
      members.add (member);
   }
   
   public ArrayList<String> members() {
      return members;
   }
   
   public List<String> getMembers() {
      return new ArrayList<>(members);
   }
   
   public void clear() {
      members.clear ();
   }
   
   /**
    * Creates a MuscleExciter from this ObjectGroup by finding muscles
    * in the specified MechModel that match the member names.
    * 
    * @param mech MechModel to search for muscles
    * @param componentMap component mapping for finding muscles
    * @return created MuscleExciter, or null if no muscles found
    */
   public MuscleExciter createMuscleExciter(MechModel mech, ModelComponentMap componentMap) {
      if (members.isEmpty()) {
         return null;
      }
      
      MuscleExciter exciter = new MuscleExciter(getName());
      boolean hasMembers = false;
      ArrayList<String> foundMuscles = new ArrayList<>();
      
      for (String memberName : members) {
         MuscleComponent muscle = findMuscleByName(mech, memberName);
         
         if (muscle != null) {
            exciter.addTarget(muscle, 1.0);
            hasMembers = true;
            foundMuscles.add(muscle.getName());
         }
      }
      
      // if (hasMembers) {
      //    System.out.println(getName() + " -> " + String.join(", ", foundMuscles));
      // }
      
      return hasMembers ? exciter : null;
   }
   
   /**
    * Creates separate MuscleExciters for left and right muscles from this ObjectGroup.
    * Automatically detects left/right muscles based on common suffixes (_l, _r, _left, _right).
    * 
    * @param mech MechModel to search for muscles
    * @param componentMap component mapping for finding muscles
    * @return list of created MuscleExciters (left, right, and/or combined)
    */
   public List<MuscleExciter> createLeftRightMuscleExciters(MechModel mech, ModelComponentMap componentMap) {
      if (members.isEmpty()) {
         return new ArrayList<>();
      }
      
      List<MuscleExciter> exciters = new ArrayList<>();
      
      // First pass: identify left muscles and collect all muscles
      List<MuscleComponent> allMuscles = new ArrayList<>();
      List<MuscleComponent> leftMuscles = new ArrayList<>();
      
      for (String memberName : members) {
         MuscleComponent muscle = findMuscleByName(mech, memberName);
         
         if (muscle != null) {
            allMuscles.add(muscle);
            String muscleName = muscle.getName().trim().toLowerCase();
            
            // Check for left muscles
            boolean isLeft = muscleName.endsWith("_l") || muscleName.endsWith("_left") || 
                           muscleName.endsWith("_l_") || muscleName.endsWith("_left_") ||
                           muscleName.contains("_l_") || muscleName.contains("_left_");
            
            if (isLeft) {
               leftMuscles.add(muscle);
            }
         }
      }
      
      // Second pass: categorize remaining muscles
      List<MuscleComponent> rightMuscles = new ArrayList<>();
      List<MuscleComponent> otherMuscles = new ArrayList<>();
      
      for (MuscleComponent muscle : allMuscles) {
         if (leftMuscles.contains(muscle)) {
            continue; // Already categorized as left
         }
         
         String muscleName = muscle.getName().trim().toLowerCase();
         
         // Check for right muscles
         boolean isRight = muscleName.endsWith("_r") || muscleName.endsWith("_right") ||
                          muscleName.endsWith("_r_") || muscleName.endsWith("_right_") ||
                          muscleName.contains("_r_") || muscleName.contains("_right_");
         
         // Special case: if muscle has no suffix and there are left muscles in the same group,
         // treat it as right muscle (common naming convention)
         if (!isRight && !leftMuscles.isEmpty()) {
            isRight = true;
         }
         
         if (isRight) {
            rightMuscles.add(muscle);
         } else {
            otherMuscles.add(muscle);
         }
         

      }
      
      // Create left exciter if left muscles found
      if (!leftMuscles.isEmpty()) {
         MuscleExciter leftExciter = new MuscleExciter(getName() + "_left");
         for (MuscleComponent muscle : leftMuscles) {
            leftExciter.addTarget(muscle, 1.0);
         }
         exciters.add(leftExciter);
      }
      
      // Create right exciter if right muscles found
      if (!rightMuscles.isEmpty()) {
         MuscleExciter rightExciter = new MuscleExciter(getName() + "_right");
         for (MuscleComponent muscle : rightMuscles) {
            rightExciter.addTarget(muscle, 1.0);
         }
         exciters.add(rightExciter);
      }
      
      // Create combined exciter for muscles without left/right designation
      if (!otherMuscles.isEmpty()) {
         MuscleExciter combinedExciter = new MuscleExciter(getName());
         for (MuscleComponent muscle : otherMuscles) {
            combinedExciter.addTarget(muscle, 1.0);
         }
         exciters.add(combinedExciter);
      }
      
      return exciters;
   }
   
   /**
    * Helper method to find a muscle by name in the MechModel
    */
   private MuscleComponent findMuscleByName(MechModel mech, String name) {
      MuscleComponent muscle = null;
      
      // Use different search strategy based on muscle path point configuration
      if (OpenSimParser.getMusclesContainPathPoints()) {
         // Direct path when muscles contain path points
         muscle = (MuscleComponent) mech.findComponent("forceset/" + name);
      } else {
         // Nested path when muscles don't contain path points
         muscle = (MuscleComponent) mech.findComponent("forceset/" + name + "/" + name);
      }
      
      if (muscle != null) {
         return muscle;
      }
      
      // Fallback: try the other path strategy
      if (OpenSimParser.getMusclesContainPathPoints()) {
         muscle = (MuscleComponent) mech.findComponent("forceset/" + name + "/" + name);
      } else {
         muscle = (MuscleComponent) mech.findComponent("forceset/" + name);
      }
      
      if (muscle != null) {
         return muscle;
      }
      
      // Final fallback: recursive search through all components
      List<MuscleComponent> allMuscles = new ArrayList<>();
      mech.recursivelyFind(allMuscles, MuscleComponent.class);
      
      for (MuscleComponent m : allMuscles) {
         if (name.equals(m.getName())) {
            return m;
         }
      }
      
      return null;
   }
   
   @Override
   public ObjectGroup clone () {
      ObjectGroup og = (ObjectGroup)super.clone ();
      // duplicate member array
      og.members = new ArrayList<String>();
      og.members.addAll (members);
      return og;
   }

}
