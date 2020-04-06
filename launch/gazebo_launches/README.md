// editted by hs 05/04/18 //

empty_playgournd.launch: 
    no UR5, no camera, used for testing mapping.

playground_without_ur5.launch: 
    no UR5, used for testing SLAM.

gazebo_simulation_with_gripper.launch:
    whole environment set, mainly for testing UR5 gripper.

gazebo_simulation_whole.launch:
    whole environment set, for simulatiing the whole project.

playground_without_ur5_double_camera.launch:
    no UR5, used for testing SLAM. 
    NOTICE: program for ur5 transporting using two cameras is not complete yet.

gazebo_hectormapping.launch:
    for hectormapping in gazebo simulated world.

ur5_catcher.launch:
    for UR5 transporting.

ur5_gripper.launch:
    an empty world with a single gripper, used for testing gripper visual effect.

gazebo_move.launch:
    use move_base to autonomously move EduMIP to a desired position.