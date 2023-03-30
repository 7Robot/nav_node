# Noeud de navigation

## Principe de fonctionnement

Dans la grande majorité des cas, le chemin le plus court entre deux points est une affine entre ces deux points.
* Première étape : On trace une droite entre les deux points $f(x)=ax+b$. 
    
    Avec $(a,b)=(\dfrac{y_1-y_0}{x_1-x_0}, y_0 - a\cdot x_0)$.
* Deuxième étape : On vérifie pour chaque point sur le chemin si il y a collision avec un obstacle sur le chemin.
  
* Si il y a collision, on place un point sur la normale à la trajectoire à mi-chemin entre le début et la fin de l'obstacle.
  
* On recommence tant que le chemin est obstrué sur une portion

