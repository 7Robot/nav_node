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