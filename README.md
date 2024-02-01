# UoG_Robotics_TDP4

## Project Overview

Welcome to the Webots RoboCup Soccer project! This project aims to simulate a RoboCup Soccer environment using Webots, a popular open-source robot simulator. The simulation involves the implementation of autonomous soccer-playing robots that can compete against each other.

## Getting Started

### Prerequisites

- Webots: Make sure you have Webots installed on your system. You can download it from [Webots official website](https://cyberbotics.com/).
- Programming Language: This project is implemented using python.
- RoboCup Soccer Server: The Webots simulation interacts with the RoboCup Soccer Server, which is integrated into Webots. No separate installation is required.

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/RajasJoshi/UoG_Robotics_TDP4.git
   ```

2. Navigate to the project directory:

   ```bash
   cd Wenao
   ```

3. Install dependencies (if any) and set up your development environment.
   ```
   pip3 -r requirements.txt
   ```

## Project Structure

```
Wenao/
├── controllers
│   ├── Base
│   │   └── SupervisorBase.py
│   ├── BlueDefender
│   │   └── BlueDefender.py
│   ├── BlueForwardA
│   │   └── BlueForwardA.py
│   ├── BlueForwardB
│   │   └── BlueForwardB.py
│   ├── BlueGoalkeeper
│   │   └── BlueGoalkeeper.py
│   ├── DefaultBase
│   │   ├── DefaultBase.py
│   ├── RedDefender
│   │   └── RedDefender.py
│   ├── RedForwardA
│   │   └── RedForwardA.py
│   ├── RedForwardB
│   │   └── RedForwardB.py
│   ├── RedGoalkeeper
│   │   └── RedGoalkeeper.py
│   ├── Supervisor
│   │   └── Supervisor.py
│   └── Utils
│       ├── best.pt
│       ├── Consts.py
│       ├── Functions.py
│       ├── ImageServer.py
│       ├── __init__.py
├── libraries
├── motions
│   ├── Backwards.motion
│   ├── ForwardLoop.motion
│   ├── Forwards50.motion
│   ├── Forwards.motion
│   ├── ForwardsSprint.motion
│   ├── HandWave.motion
│   ├── LongPass.motion
│   ├── RightShoot.motion
│   ├── Shoot.motion
│   ├── SidePass_Left.motion
│   ├── SidePass_Right.motion
│   ├── SideStepLeftLoop.motion
│   ├── SideStepLeft.motion
│   ├── SideStepRightLoop.motion
│   ├── SideStepRight.motion
│   ├── StandInit.motion
│   ├── Stand.motion
│   ├── StandUpFromBack.motion
│   ├── StandUpFromFront.motion
│   ├── TurnLeft10.motion
│   ├── TurnLeft10_V2.motion
│   ├── TurnLeft180.motion
│   ├── TurnLeft20.motion
│   ├── TurnLeft30.motion
│   ├── TurnLeft40.motion
│   ├── TurnLeft60.motion
│   ├── TurnRight10.motion
│   ├── TurnRight10_V2.motion
│   ├── TurnRight20.motion
│   ├── TurnRight40.motion
│   └── TurnRight60.motion
├── plugins
│   ├── physics
│   ├── remote_controls
│   └── robot_windows
├── protos
│   ├── meshes
│   ├── NaoH25V60.proto
│   ├── NAOH25V60.urdf
│   ├── NaoJersey.proto
│   ├── NaoLeftWristH25Realistic.proto
│   ├── Nao.proto
│   ├── NaoRightWristH25Realistic.proto
│   ├── NaoV6.proto
│   └── textures
└── worlds
    └── naotdp.wbt
```

- **controllers:** Contains the controller code for each soccer player.
- **worlds:** Includes the Webots world file defining the soccer field.

## Running the Simulation

1. Open Webots and load the project.

2. Set the simulation parameters and select the appropriate world file (`naotdp.wbt`).

3. Run the simulation.

4. Observe the behavior of the soccer players in the simulated environment.

## Customizing and Extending

Feel free to customize the project to implement your strategies and behaviors for the soccer players. You can modify the existing controllers or create new ones. Additionally, you can explore advanced features provided by Webots and the RoboCup Soccer Server.

## Contributing

If you would like to contribute to this project, please follow our [contribution guidelines](CONTRIBUTING.md). We welcome bug reports, feature requests, and pull requests.

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

- Thanks to the [Webots community](https://cyberbotics.com/community) for their support.
- Inspired by the RoboCup Soccer initiative.

- References:
  - `https://github.com/UofG-RoboticsTeam9/RoboCupSoccer.git`
  - `https://github.com/Bembelbots/WebotsLoLaController.git`
  - `https://github.com/amirhnourian/WebotsNaoWresteling`
