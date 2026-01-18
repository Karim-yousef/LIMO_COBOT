#!/bin/bash
# Wait for Gazebo to be ready
echo "Waiting for Gazebo to load world..."
sleep 10

# Wait for the spawn service to be available
echo "Waiting for Gazebo spawn service..."
until rosservice list | grep -q "/gazebo/spawn_urdf_model"; do
    echo "Waiting for spawn service..."
    sleep 1
done

echo "Spawning robot..."
# Spawn at origin (0, 0) facing +X direction (toward cubes at x=4)
rosrun gazebo_ros spawn_model -urdf -param robot_description -model limo_robot \
    -x 0 -y 0 -z 0.05 -R 0 -P 0 -Y 0 \
    -J grasping_frame_joint 0 \
    -J joint2_to_joint1 0 \
    -J joint3_to_joint2 0 \
    -J joint4_to_joint3 0 \
    -J joint5_to_joint4 0 \
    -J joint6_to_joint5 0 \
    -J joint6output_to_joint6 0

echo "Robot spawn complete!"
