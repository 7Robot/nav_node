# Noeud de navigation

## Principe de fonctionnement

### Recherche d'un chemin

La recherche de Chemin se fait par l'algorithme **P\***  (Pepino \*), cette algorithme de recherche naïve permet de trouver le meilleur chemin entre deux points.

L'algorithme part d'un tableau en 2D, ce tableau sera mis à jour avec le meilleur chemin pour accéder à chacune des cases, on reparcourt le tableau tant que le tableau des distances d'accès varie.

### Mise à jour du chemin

Le chemin est mis à jour (c'est à dire recalculé) si et seulement si :

* Un nouvel objectif de position est donné

* Un des obstacles a bougé de plus de 5cm ET le chemin est dorénavant obstrué.

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




## Interfaces
### Subscribers
Ce noeud dispose des Subscribers suivants :
| Nom | Type | Call Back |  Description |
| --- | ---- | --------- | ------------ |
| goal_sub | Point | goal_callback | Topic qui donne le point d'arrivée voulu. |
| sub_robot_data | RobotData | robot_data_callback | Topic qui donne les information d'odomérie (position) |
| stop_sub | Bool | stop_callback | Topic demande l'arret de la naviguation de la nav node. |




### Publishers
Ce noeud dispose des Publisher suivants :
| Nom | Type | Description |
| --- | ---- | ----------- |
| pub_action_pic | Pic_Action | Topic des actions a transmettre a la carte PIC. |
| result_pub | Bool | Topic qui publie la reussite / echec des actions |

## Classes et fonctions

### Nav_node


### PStar

| Fonctions | parametres | description |
| --------- | ---------- | ----------- |
| set_map | map : tabelau 2D du plateau false si ... et true si ... | recopie la map fournie et créé une map des couts (cout maximums partout) |
| get_cost | p : point du tableau dont on veux calculer le cout | Calcule le cout d'un point du tableau a partir du cout des 8 cases adjacentes. Retourne vrai si le cout a changé, faux sinon. |
| calculate_path | startX,startY : les coordonées du point de départ. endX,endY : les coordonées du point d'arrivée. result_path : le vecteur de points (chemin) déterminé (fournir l'adresse de la case memoire). | Calcule le chemin optimal .......... |