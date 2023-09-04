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
