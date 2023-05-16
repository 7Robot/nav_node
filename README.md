# Noeud de navigation

## Principe de fonctionnement

### Recherche d'un chemin
Dans la grande majorité des cas, le chemin le plus court entre deux points est une affine entre ces deux points.

Pour trouver le chemin le plus court on applique l'algorithme suivant :
* Première étape : On trace une droite entre les deux points $f(x)=ax+b$. 
    
    Avec $(a,b)=(\dfrac{y_1-y_0}{x_1-x_0}, y_0 - a\cdot x_0)$.
* Deuxième étape : On vérifie pour chaque point sur le chemin si il y a collision avec un obstacle sur le chemin.
  
* Si il y a collision, on place un point sur la normale à la trajectoire à mi-chemin entre le début et la fin de l'obstacle.
![Exemple d'uitlisation](https://zupimages.net/up/23/17/ouxn.png "3 points")

  
* On recommence tant que le chemin est obstrué sur une portion

**Cette algorithme est réalisé dans Master_Path**

### Mise à jour du chemin

Le chemin est mis à jour (c'est à dire recalculé) si et seulement si :

* Un nouvel objectif de position est donné

* Un des obstacles a bougé de plus de 5cm (*fonction obstacle_variation*) ET le chemin est dorénavant obstrué (*fonction verify_path*)

* Si un obstacle a bougé de plus de 5 cm et que le nouveau chemin qui serait trouvé est plus court que l'ancien

## Guide d'utilisation

### Lancement d'un objectif en position 

Pour lancer un objectif en position, il faut publier un objet de la classe std_msgs/Point sur le topic /robot_x/position_goal (La coordonnée en z n'importe pas).
Le robot va se mettre en mouvement pour atteindre cette position.

### Arrivé à destination

Lorsque le robot est arrivé à destination, il publie un message de type std_msgs/Bool sur le topic /robot_x/nav_node_result, il renvoie le message True, une fois arrivé.

### Il n'y a pas de chemin

Le nav_node va renvoyer dans le topic /robot_x/nav_node_result, le message False.

### Je souhaite stopper le nav_node

Pour stopper le nav_node, il suffit d'envoyer un message de type std_msgs/Bool sur le topic /robot_x/activation_nav_node.
ATTENTION, sachant qu'une position a été envoyée au PIC, le robot ne s'arrêtera pas pour autant.

## Je souhaite charger des obstacles

Pour charger des obstacles statiques il suffit d'envoyer sur le topic /robot_x/nav_node_obstacle_map un message de type std_msgs/String, contenant à la suite la liste des obstacles à éviter.
Exemple : "r2m1p3p4", évitera les gateaux r2 et m1 ainsi que les plats 3 et 4.
