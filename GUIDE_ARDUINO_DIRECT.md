# Caméra et modèle vers Arduino, sans ROS

Ce mode ne lance aucun nœud ROS, aucun topic et aucun Gazebo. Il remplace le pont
cmd_vel_to_arduino par une liaison PySerial directe. Le firmware n'est pas modifié.

## Installer et préparer

Depuis le PC robot connecté au Wi-Fi :

```bash
cd ~/ghaweslni
git pull --ff-only
.venv-robot/bin/python -m pip install 'pyserial==3.5'
```

Les autres dépendances du modèle sont celles déjà installées. Le YAML et les poids
restent ceux du dépôt. Fermer par Ctrl+C l'ancien pont ROS, le moniteur Arduino,
l'ancien aperçu et tout autre programme utilisant la caméra ou /dev/ttyACM0.

## Aperçu (port série NON ouvert)

```bash
bash tester_arduino_direct.sh --camera /dev/video0 --duration 30
```

## Essai moteur explicite

```bash
bash tester_arduino_direct.sh --camera /dev/video0 --port /dev/ttyACM0 --execute --duration 5
```

Le port de l'Arduino est /dev/ttyACM0 d'après l'identification USB fournie. Le CP2102
sur /dev/ttyUSB0 est un autre périphérique : ne pas le choisir pour cet Arduino.
Le pilote demande une ouverture exclusive, mais tous les anciens programmes
n'utilisent pas ce verrou ; aucun autre pont série ne doit rester actif.

Protocole du firmware lu : Serial.begin(57600), puis M<V>:<W> suivi d'un retour à
la ligne. Un exemple de trame est M-0.0300:0.0000. Cela prouve un envoi série,
pas la bonne exécution ni le sens physique des roues.

Les signes par défaut (-1,-1) reprennent la conversion de l'ancien pont logiciel :
V_Arduino=-V_modele, W_Arduino=-W_modele. La géométrie actuelle doit être vérifiée,
premier essai roues levées et arrêt d'urgence accessible. Si le sens doit changer,
les options --linear-sign 1 et --angular-sign 1 permettent une configuration
explicite. Ne pas corriger ces signes sans avoir identifié les sens physiques.
Le programme ne transmet jamais automatiquement RESUME ou RECAL.

V est limitée à [-0,02 ; 0,03] m/s et W à +/-0,08 rad/s. Caméra silencieuse,
cible perdue, image ancienne ou durée dépassée : commande nulle. Q, Échap,
fermeture de la fenêtre ou Ctrl+C quittent et envoient trois trames nulles.
L'aperçu ne publie aucune commande et garde un historique de commandes nulles.
Le modèle, la calibration et le calcul des coins restent identiques au mode ROS.

## Limite importante du firmware

Le firmware Arduino présent dans le dépôt n'a pas de timeout de réception série.
Le nouveau programme possède un watchdog dans un thread séparé : si aucune
commande neuve n'arrive pendant 0,3 seconde, il envoie zéro. Cela couvre une boucle
de calcul qui ralentit, pas une panne du PC, une coupure USB ou un arrêt brutal du
processus. Ce dispositif n'est pas un arrêt matériel garanti. Un timeout embarqué
et un arrêt d'urgence restent nécessaires pour des essais autonomes robustes.

## Enregistrement

Chaque essai crée controle/data/arduino_direct/essai_DATE_ID/ :
commandes.csv (coins canoniques, prédictions, vitesses limitées et trame série),
serie.jsonl (toutes les lignes TX et RX horodatées), configuration.json et resume.json.
Les retours Arduino ne sont pas utilisés comme entrées du modèle ; ils servent
au diagnostic. Les directions et unités doivent être vérifiées sur le matériel.

## Vérifications effectuées

Tests du format, des signes, du watchdog et de l'arrêt à la fermeture. Test du
pipeline avec image synthétique et pseudo-terminal série : trames non nulles,
retours simulés et trame finale nulle vérifiés. Le conteneur de test utilise un
adaptateur de détecteur pour son ancien OpenCV 4.6 ; le PC cible utilise OpenCV
4.10 installé avec le projet. Aucun essai sur un Arduino physique n'est revendiqué.
