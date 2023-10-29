# UoG_Robotics_TDP4
This repository holds codebase for Robotics TDP Project of Team 4 for the year 2023-2024

### Setting up Workspace
1.  Refer to the link for system requirement, installation & tutorials instructions
    - rmf_demo workspace: https://github.com/open-rmf/rmf_demos
2.  Clone the rmf_demos workspace, create a folder and name it 'Nao' in rmf_ws/src/demonstration.
3.  Load all the nao files into the Nao Folder.

#### Directory setup of rmf_ws:
.  
├── ...  
├── src  
│   ├── demonstrations    
│       ├── Nao  
│           ├── nao_demos  
│           ├── nao_demos_assets  
│           ├── nao_demos_gz  
│           ├── nao_demos_gz_classic  
│           ├── nao_demos_maps   
│       ├── rmf_demos  
│   ├── rmf      
│   └── thirdparty                
└── ...  

### Load Soccer Field 3D Model into Gazebo


### To launch the robocup world,
1. source /opt/ros/humble/setup.bash
2. cd ~/rmf_ws
3. source ~/rmf_ws/install/setup.bash   
4. Launch Robocup Simulation with Nao in Gazebo or Ignition,  
   Ignition: ros2 launch nao_demos_gz nao.launch.xml  
   Gazebo: ros2 launch nao_demos_gz_classic nao.launch.xml


