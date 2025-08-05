package artisynth.core.opensim.components;

import java.util.ArrayList;
import java.util.List;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MuscleExciter;

/**
 * ObjectGroupSet component for managing collections of ObjectGroups
 * in OpenSim models. This class follows the pattern of ComponentSet
 * and stores ObjectGroups as metadata.
 */
public class ObjectGroupSet extends SetBase<ObjectGroup> {
   
   public ObjectGroupSet() {
      super();
   }
   
   @Override
   public ObjectGroupSet clone() {
      return (ObjectGroupSet) super.clone();
   }
   
   /**
    * Creates muscle exciters for all ObjectGroups in this set.
    * 
    * @param mech MechModel to add exciters to
    * @param componentMap component mapping for finding muscles
    * @param separateLeftRight if true, create separate exciters for left/right muscles
    * @return list of created MuscleExciters
    */
   public List<MuscleExciter> createMuscleExciters(MechModel mech, ModelComponentMap componentMap, boolean separateLeftRight) {
      List<MuscleExciter> exciters = new ArrayList<>();
      
      for (ObjectGroup group : objects()) {
         List<MuscleExciter> groupExciters;
         if (separateLeftRight) {
            groupExciters = group.createLeftRightMuscleExciters(mech, componentMap);
         } else {
            MuscleExciter exciter = group.createMuscleExciter(mech, componentMap);
            groupExciters = new ArrayList<>();
            if (exciter != null) {
               groupExciters.add(exciter);
            }
         }
         
         for (MuscleExciter exciter : groupExciters) {
            mech.addMuscleExciter(exciter);
            exciters.add(exciter);
         }
      }
      return exciters;
   }
   
   /**
    * Creates muscle exciters for all ObjectGroups in this set.
    * Uses the default left/right separation setting.
    * 
    * @param mech MechModel to add exciters to
    * @param componentMap component mapping for finding muscles
    * @return list of created MuscleExciters
    */
   public List<MuscleExciter> createMuscleExciters(MechModel mech, ModelComponentMap componentMap) {
      return createMuscleExciters(mech, componentMap, true);
   }
   
   /**
    * Finds an ObjectGroup by name.
    * 
    * @param name name of the ObjectGroup to find
    * @return the ObjectGroup, or null if not found
    */
   public ObjectGroup findObjectGroup(String name) {
      for (ObjectGroup group : objects()) {
         if (name.equals(group.getName())) {
            return group;
         }
      }
      return null;
   }
   
   /**
    * Gets all muscle names from all ObjectGroups in this set.
    * 
    * @return list of all muscle names referenced by ObjectGroups
    */
   public List<String> getAllMuscleNames() {
      List<String> allNames = new ArrayList<>();
      for (ObjectGroup group : objects()) {
         allNames.addAll(group.getMembers());
      }
      return allNames;
   }
} 