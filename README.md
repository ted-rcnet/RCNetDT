Release du 11/06/2013
Code :
* Optimisations
- Amelioration du return to home
- La RCNet FC est prise en charge par défaut sans sélectionner quoi que ce soit
- Le module GPS RCNet est activable en dé-commentant une seul ligne
- Fiabilisation du code en général
- Optimisation d'une bonne partie du code
- Clean du code pour y voir plus clair et être plus efficace lors des modifications futurs.
- Les PIDs par défaut son optimisés pour la FC et ce directement dans le code, notamment les PIDs Altitude et GPS Hold / PH

* Nouveautés
- Ajout de l'atterrissage automatique au RTH (appelé RTH Autoland / RTHA)
- Ajout du désarmement moteur automatique a la fin du RTHA
- Ajout de la possibilité de faire du RTHA avec le failsafe multiwii <- Désactivé par défaut car non encore totalement validé, pas de bug connu
- Support du module SD RCNet DevTeam
- Log des données de la machine (Nombre d'armement, nombre de désarmement, temps armé, temps de fonctionnement total, nombre de failsafe, nombre d'erreurs I2C) dans un fichier sur la SD
- Log des données GPS et Barométriques de Vol dans un fichier sur la carte SD (voir plus bas pour l'utilisation de ces données)
- Mode mission GPS via un fichier google earth (voir plus bas comment faire) <- Partie du code non encore totalement optimisé, pas de bug connu
- Introduction du Smooth-GPS permettant un arrêt moins violent lors du passage en GPS Hold <- code en cours d'optimisation, comportement actuel -> présence d'encore un léger rollback (le multi semble revenir un peu en arrière), aucun bug connu


Comme vous pouvez le constaté nous avons fait le choix de prendre une version multiwii stable et abouti et de la remanier a notre guise pour en faire ce que l'on souhaite.
Ceci était pour nous la seul solution viable pour arriver au niveau de qualité de fonctionnement et de simplicité que nous souhaitions, multiwii supporte tellement de capteurs/contrôleurs que des compromis sont souvent nécessaire au bon fonctionnement du code sur tous ces matériels différents.
Notre but n'est pas de faire un code compatible avec tout une liste de cartes et capteurs, notre but est d'avoir le meilleur code possible pour le matériel que nous connaissons/maitrisons, de toutes façon, nous n'avons clairement pas le temps et les moyens de nous lancer dans le développement, les tests et l'optimisation de toutes nos nouvelles fonctionnalités autant de matériels différents, le simple fait de chercher une compatibilité avec le 328P rend énormément de sujet complexe et cela fini en compromis qui vont au détriment du fonctionnement optimal sur 2560, nous n'utilisons que du 2560 alors pas de compromis, on fait le mieux possible pour le 2560 ￼


Pour couronner le tout, deux utilitaires font leur apparition.
* Le premier s'adresse aux utilisateurs de mac, il permets de patcher l'application multiwii sans effort (habituellement il faut lancer une commande depuis le terminal), il se nomme Patch et se trouve dans le dossier de l'application multiwii pour MacOS, pour que cela fonctionne le patch doit être lancé depuis le même endroit que l'application.

* Le second (dont le doux nom est kmlConverter) permets : 
- de convertir les fichiers log GPS créés par la carte en fichier compatible google earth et ainsi visualiser votre vol dans l'application google earth.
- de convertir un fichier google earth contenant les waypoint du vol prévu pour la mission en fichier lisible par la carte


Vous vous en doutez, il est nécessaire pour tout le monde que ce logiciel ai un nom, ne serais-ce que pour la communauté multiwii officiel.
Vous avez pu aussi remarquer qu'il n'y en a pas ;) nous ne sommes pas très inspiré en fait et ne lui avons pas trouvé de nom sympa et simple/sobre.

Nous vous proposons donc un petit concours, chacun pourra donc proposer 1 ou 2 noms, a la fin tout le monde votera via un post a sondage, le nom qui fera l'unanimité sera alors choisi et le gagnant aura un petit cadeau ;)

EDIT : 11/06/2013 ; mise a jour en V091 ; changelog :
- Correction d'une inversion des latitudes/longitudes dans le kmlconverter
- Correction mineur dans le log des données GPS
- Correctif mode mission (nous parlerons de ce mode dans un post détaillé bientôt)
- Mise a jour du Patch Mac (il copie maintenant la lib SdFat dans le dossier arduino)
- Ajout de diverses options dans le config.h
- PIN A9 (ROLL) Pour le RSSI
- Ajout du RSSI FRSKY dont le calcul est différent du RSSI "normal", nécéssite un petit montage expliqué ici -> http://www.rcnet.com/tuto-multiwii/frsky-rssi-sur-multiwii-t1688.html
