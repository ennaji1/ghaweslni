# Contrôle matériel expérimental par caméra ArUco

Dépôt minimal : aucun Gazebo ni fichier de CAO requis.

## Installation, depuis la racine du projet

```bash
sudo apt install python3-venv
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv-robot
.venv-robot/bin/python -m pip install 'numpy==1.26.4' 'opencv-contrib-python==4.10.0.84' 'PyYAML==6.0.2' 'gymnasium==0.29.1' 'stable-baselines3==2.4.1' 'torch==2.5.1+cpu' --extra-index-url https://download.pytorch.org/whl/cpu
```

## Préparation

Le YAML personnel doit contenir image_width, image_height, camera_matrix et dist_coeff.
La résolution réelle doit correspondre exactement au YAML. Réutiliser le focus de
calibration (la calibration actuelle indique 0). Fermer les autres applications caméra.
Le script corrige les coins avec undistortPoints, puis les projette vers K de simulation.
Cela corrige les intrinsèques, pas une différence de montage ni de géométrie.

Hypothèses d'entraînement : ID 23, DICT_4X4_100, côté noir 10 cm ; caméra horizontale,
centrée, 5 cm devant l'essieu, hauteur 50 cm, station à la hauteur correspondante.
Un autre montage ou une station différente exigent une adaptation/réentraînement.
Le modèle simule un alignement AVANT contact : il ne réalise pas la recharge.

Démarrer le pilote matériel habituel du robot. Le topic est geometry_msgs/msg/Twist,
par défaut /cmd_vel. Arrêter les autres producteurs (clavier, Nav2, ancien docking).
Conserver le même ROS_DOMAIN_ID que le pilote, ne pas activer use_sim_time.
Vérifier le signe et les unités du pilote et un arrêt moteur sur absence de commande
(timeout du contrôleur <= 0,5 s). Le script ne peut pas garantir cet arrêt si le
processus Python se bloque ou si la connexion se coupe. Garder l'arrêt d'urgence accessible.

## Aperçu, sans commande publiée

```bash
bash tester_robot_reel.sh --calibration /chemin/vers/camera.yaml --camera /dev/video2 --focus 0
```

L'aperçu conserve un historique de commandes réellement envoyées égal à zéro ; il
n'est donc pas une reproduction d'une trajectoire exécutée. Contrôler la détection
et le montage avant le mode moteur. Une grande zone libre ne remplace pas ces contrôles.

## Essai moteur explicite

```bash
bash tester_robot_reel.sh --calibration /chemin/vers/camera.yaml --camera /dev/video2 --focus 0 --execute --duration 30
```

Limites : v entre -0,02 et +0,03 m/s, omega +/-0,08 rad/s. Ces saturations modifient
le comportement évalué en simulation. Les commandes saturées sont réinjectées dans
l'historique. Absence de marqueur ou image de plus de 0,3 s : zéro. Durée maximale,
Q, Échap, Ctrl+C : arrêt. Le modèle ne détecte pas les obstacles. Ce premier essai
matériel n'a pas été validé sur le robot ; seules les conversions et l'import des
modules ont été vérifiés ici.

Pas d'affichage disponible : ajouter --no-display. Topic différent : --cmd-topic /votre_topic.
Résultats conservés sans écrasement dans controle/data/real_rollouts/essai_... :
commandes.csv (coins corrigés, prédiction et commande envoyée séparées), configuration.json.
Les timestamps sont monotones locaux, pas l'heure ROS.

## Origine et licence

Projet dérivé des travaux E-Wslni : https://github.com/RedaMohssine/E-Wslni-Autonomous-Guide-Robot et de la chaîne ArUco : https://github.com/NithishGovind/Aruco-marker-based-Docking. Modules distribués sous GPL-3.0, voir LICENSE.

Après clonage, placer le YAML réel à la racine sous camera_reelle.yaml.
