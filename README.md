# Noeud de navigation

## Principe de fonctionnement

Dans la grande majorité des cas, le chemin le plus court entre deux points est une affine entre ces deux points.
* Première étape : On trace une droite entre les deux points $f(x)=ax+b$. 
    
    Avec $(a,b)=(\dfrac{y_1-y_0}{x_1-x_0}, y_0 - a\cdot x_0)$.
* Deuxième étape : On vérifie pour chaque point sur le chemin si il y a collision avec un obstacle sur le chemin.
  
* Si il y a collision, on trouve la taille de l'obstacle et on trace un arc de cercle autour de l'obstacle.

## Évitement d'obstacle

## État des tests

* [x] add_circle
* [x] add_affine
* [x] sfb

* [x] find_path_without_obstacles
* [ ] correct_path
* [x] is_collision
* [x] calc_circle
* [ ] circumvent
* [x] follow_path (Il faut changer la publication de la valeur)
* [x] obstacles_callback
