Voici un modèle de `README.md` pour votre projet de suivi de QR Code avec un robot Doosan M0609 équipé d'une pince OnRobot RG et d'une caméra eye-on-hand, le tout en simulation sur ROS Noetic et Gazebo. Vous pouvez copier et coller ce contenu directement dans votre fichier README sur GitHub.

```markdown
# Suivi de QR Code avec Robot Doosan M0609 en Simulation Gazebo

## Aperçu du Projet
Ce projet vise à utiliser le robot Doosan M0609 avec une pince OnRobot RG et une caméra eye-on-hand pour suivre un QR code en temps réel. Il est développé sur ROS Noetic et Ubuntu 20.04, avec une simulation dans Gazebo.

### Caractéristiques Principales
- Suivi en temps réel de QR Code avec le robot Doosan M0609.
- Utilisation d'une pince OnRobot RG comme effecteur.
- Intégration d'une caméra eye-on-hand pour la détection de QR Code.
- Simulation complète sous Gazebo avec ROS Noetic.

## Démarrage
Ces instructions vous permettront de démarrer une copie du projet sur votre machine locale à des fins de développement et de test.
Les packages serial et roboticsgroup_upatras_gazebo_plugins sont relatifs au doosan et OnRobot 

Les packages que j'ai crée sont : 
- doosan_gazebo_sim : Simulation du robot avec tracking du QR code ( Visp_Auto_tracker)
- doosan_moveit_config : MoveIt
- manipulator_description : Fichier URDF du robot, de l'effecteur et de la camera

### Prérequis
- Ubuntu 20.04
- ROS Noetic
- Gazebo (pour la simulation)

### Installation
1. Clonez le dépôt dans votre espace de travail ROS (par exemple, `catkin_ws`):
```bash
cd ~/dsr_ws/src
git clone https://github.com/Greg7900/MSFT_project.git
cd ..
catkin build 
```

### Lancer la Simulation
Pour lancer la simulation du robot Doosan M0609, suivez ces commandes :

1. Lancez la simulation Gazebo :
```bash
cd dsr_ws/src
source devel/setup.bash
roslaunch doosan_gazebo_sim gazebo_sim.launch
```

2. Dans un nouveau terminal, sourcez l'environnement et exécutez le script principal :
```bash
cd ~/dsr_ws
source devel/setup.bash
rosrun doosan_gazebo_sim gazebo_cube
```

## Utilisation
Une fois la simulation et le script en cours d'exécution, le robot Doosan commencera à suivre le QR code en utilisant la caméra eye-on-hand.

## Contribuer
Veuillez lire [CONTRIBUTING.md](lien-vers-votre-fichier-de-contribution) pour des détails sur notre code de conduite et le processus pour soumettre des pull requests.

## Auteurs
- Greg7900 - *Travail initial* - [Greg7900](https://github.com/Greg7900)


## Remerciements
- Moi
