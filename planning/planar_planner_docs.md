# Planar Planner Stuff

## General Plan
* Take in point clouds
* Generate grasps
* Check collisions
* Calculate grasp scores
* Return best grasp candidate (that is possible in current orientation) and general graspability score (based on all grasp candidates & scores)

## Code Structure
* PlanarGraspSelector
  * Input: point cloud
  * Outputs: grasp selection, graspability score
* PlanarGraspPlanner
  * Input: grasp selection
  * Outputs: motion commands

## Notes
* Create a MultiBodyPlant in grasp selector for collision detection purposes
  * parser.AddModelsFromUrl the SDFs of the table and the gripper
  * create in init of grasp selector

## Current bugs/TODOs
* Seems like finger box is colliding with the gripper itself
  * visualize actual gripper in meshcat (can use a new meshcat session and just add gripper, finger box, pick point)
