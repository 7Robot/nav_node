import rospy, rospkg
from geometry_msgs.msg import Point
from obstacle_detector.msg import Obstacles
from cdf_msgs.msg import Pic_Action, MergedData, MergedDataBis, Trajectoire
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray, Marker
from tool_lidar.objet import ChooseColor


import tool_lidar.variable_globale as tool_glob
import tool_lidar.publisher as tool_pub
import numpy as np
import time

class NavNode():
    def __init__(self,
                 action_done_pub, 
                 action_orders_pub, margin=0.03, 
                 pos = np.array([0,0]), 
                 max_radius = 0.35, 
                 max_iter = 15, 
                 distance_interpoint = 0.03, 
                 emergency_stop_distance = 0,
                 color = "Green",
                 name_robot = "Han7",
                 debug = False,
                 activation_topic = "/robot_1/activation_nav_node",):
        self.pub_marker = rospy.Publisher(rospy.get_param("~pub_marker_next_pos", "/robot_1/marker_next_pos"), MarkerArray, queue_size = 10)
        self.path = []
        self.margin = margin
        self.position_goal = None
        self.max_iter = max_iter
        self.debug = debug
        self.activate = True
        self.new_path = False

        if self.debug:
            self.react_pub = rospy.Publisher('/merged_datas', MergedData, queue_size=1)


        rospack = rospkg.RosPack()
        rospack.list()
        if color == "Green":
            self.static_obstacles = np.load("{}/src/map/green_nostealing.npy".format(rospack.get_path('nav_node')))
            rospy.loginfo("Green map loaded")
        elif color == "Blue":
            self.static_obstacles = np.load("{}/src/map/blue_nostealing.npy".format(rospack.get_path('nav_node')))
            rospy.loginfo("Blue map loaded")
        elif color == "Debug":
            self.static_obstacles = np.load("{}/src/map/debug_map.npy".format(rospack.get_path('nav_node')))
            rospy.loginfo("Debug map loaded")
        else:
            rospy.logerr("Color not recognized")
            exit()
        self.map_obstacles = self.static_obstacles.copy()
        self.max_radius = max_radius
        self.shape_board = self.map_obstacles.shape
        
        self.position = pos
        self.velocity = np.zeros((1,3))
        self.orientation = 0

        self.distance_interpoint = distance_interpoint
        self.next_goal = None
        self.emergency_stop_distance = emergency_stop_distance

        self.action_orders_pub = action_orders_pub
        self.action_done_pub = action_done_pub
        self.name_robot = name_robot
        self.activation_sub = rospy.Subscriber(activation_topic, Bool, self.activation_callback)

        # Contain the list of the obstacles (valeurs initiales aberrantes)
        self.obstacles = np.array([-255 for _ in range(6)])

        self.is_in_obstacle = False

        rospy.loginfo("Nav node initialized")

    def find_middle_obstacles(self, path, path_portion : int):
        """
        Find the beginning and the end of the obstacle in the path portion

        Parameters
        ----------
        path_portion : int Portion de chemin

        Returns
        -------
        (start_obstacle, end_obstacle) : tuple
        False if no obstacle

        """
        start = np.array(path[path_portion])
        end = np.array(path[path_portion + 1])

        discretisation = np.linspace(start, end, int(np.linalg.norm(end-start)/(np.linalg.norm(start - end)*0.01)))

        index_in_disc = 0
        (x, y) = (discretisation[index_in_disc, 0], discretisation[index_in_disc, 1])

        # Find the beginning of the obstacle
        while not(self.is_obstacle(x,y)) and index_in_disc < len(discretisation) - 1:
            index_in_disc += 1
            (x, y) = (discretisation[index_in_disc, 0], discretisation[index_in_disc, 1])
        
        if index_in_disc == len(discretisation) - 1:
            return False
        elif index_in_disc == 0:
            #l'obstacle se trouve à l'index 0, nous sommes dans un obstacle
            start_obstacle = discretisation[0]
        else:
            #On a trouvé un obstacle
            start_obstacle = discretisation[index_in_disc-1]
        
        start_obstacle = discretisation[index_in_disc]

        # Find the end of the obstacle
        while self.is_obstacle(x,y) and index_in_disc < len(discretisation) - 1:
            index_in_disc += 1
            (x, y) = (discretisation[index_in_disc, 0], discretisation[index_in_disc, 1])
        
        if index_in_disc == len(discretisation) - 1:
            #rospy.logwarn("Obstacle non fermé")
            return None
        else:
            end_obstacle = discretisation[index_in_disc]

            return (start_obstacle, end_obstacle)
        
    def is_obstacle(self, x, y):    # modifier pour que collision soit un encadrement et pas une valeur exacte ?
        """
        Vérifie si le point (x, y) touche un obstacle ou est sorti de la map
        """
        (xf,yf) = (int(x*100), int(y*100))
        shape = self.map_obstacles.shape
        if xf < 0 or yf < 0 or xf >= shape[0] or yf >= shape[1]:
            return True
        return self.map_obstacles[xf, yf] != 0

    def find_normal_non_obstacle(self, start, end):
        """ Find the nearest point outside of the obstacle space to define the new path
        Everything is in meter

        entry: 
            start: (x,y): start point of the obstacle
            end: (x,y): end point of the obstacle
        
        return:
            detour: (x,y): the nearest point outside of the obstacle space

        use: 
            self.map_obstacle: map
            self.margin: margin around the obstacle
        """
        rospy.loginfo("Recherche d'un détour à partir de l'obstacle {} {}".format(start, end))

        vect_obstacle = np.array(end) - np.array(start)

        # Find the normal vector and normalize it with np.linalg.norm
        vec_norm = np.array([vect_obstacle[1], -vect_obstacle[0]])/np.linalg.norm(vect_obstacle) * 0.01

        # Find the middle point of the obstacle on the vector
        middle = np.array([(start[0]+end[0])/2, (start[1]+end[1])/2])

        middle_sup = middle.copy()
        middle_inf = middle.copy()

        # Max distance is 350 cm, don't want to go too far
        dist = 0

        # Find the nearest point outside of the obstacle space + margin
        while (self.is_obstacle(middle_sup[0],middle_sup[1]) and self.is_obstacle(middle_inf[0], middle_inf[1])) and dist < np.sqrt((self.shape_board[0]/100)**2 + (self.shape_board[1]/100)**2):
            middle_sup += vec_norm  
            middle_inf -= vec_norm    
            dist += 0.01

        # Return the nearest point
        if dist > 3.5:
            rospy.logwarn("Pas de point millieu trouvé")
            return None      # No point found
        else:
            if self.is_obstacle(middle_sup[0], middle_sup[1]):
                middle_inf_decal = middle_inf - self.margin * vec_norm * 100
                if not(self.is_obstacle(middle_inf_decal[0], middle_inf_decal[1])):
                    return middle_inf - self.margin * vec_norm * 100
                else:
                    rospy.logwarn("ATTENTION : Chemin étroit trouvé en ")
                    return middle_inf
            else:
                middle_sup_decal = middle_sup + self.margin * vec_norm * 100
                if not(self.is_obstacle(middle_sup_decal[0], middle_sup_decal[1])):
                    return middle_sup + self.margin * vec_norm * 100
                else:
                    rospy.logwarn("ATTENTION : Chemin étroit")
                    return middle_sup

    def get_out_of_obstacle(self):
        """
        Get out of the obstacle
        """
        self.is_in_obstacle = True
        escape_point = None
        r = 1
        while type(escape_point) == type(None):
            # Solve x+y = r
            for x in range(-r, r+1):

                y = r - abs(x)
                (x_conv, y_conv) = (x*0.01, y*0.01)
                if not(self.is_obstacle(self.position[0] + x_conv, self.position[1] + y_conv)):
                    escape_point = np.array([self.position[0] + x_conv, self.position[1] + y_conv])
                if not(self.is_obstacle(self.position[0] + x_conv, self.position[1] - y_conv)):
                    escape_point = np.array([self.position[0] + x_conv, self.position[1] - y_conv])
            r += 1
        rospy.loginfo("Sortie d'obstacle trouvé en " + str(escape_point))
        self.path = [escape_point]
        self.next_goal = escape_point
        #rospy.loginfo("Point de sortie trouvé : " + str(escape_point))
        self.publish_pic_msg(escape_point)

    def master_path(self, start, end, apply = True):
        """
        Calculate the path to go from start to end
        """

        if self.is_obstacle(start[0], start[1]) or self.is_obstacle(end[0], end[1]):
            # rospy.logwarn("Départ ou arrivée dans un obstacle")
            if self.is_obstacle(start[0], start[1]):
                # rospy.logwarn("Départ dans un obstacle, on le quitte")
                self.get_out_of_obstacle()
                return None
            else:
                rospy.logwarn("Arrivée dans un obstacle ! Pas de chemin")

        path = [start, end]
        self.position_goal = end

        path_portion = 0

        iter = 0
        max_iter = self.max_iter

        while path_portion<(len(path)-1) and iter < max_iter:
            iter += 1
            middle = self.find_middle_obstacles(path, path_portion)
            if type(middle) == type(None):
                self.path = [np.array(self.position)]
                self.new_path = True
                rospy.logwarn("Pas de trajectoire")
                return None
            elif middle:
                detour = self.find_normal_non_obstacle(middle[0], middle[1])
                rospy.loginfo("Detour trouvé : " + str(detour))
                if type(detour) != type(None):
                    path.insert(path_portion+1, detour)
                    rospy.loginfo("New Path : " + str(path))
                else:
                    self.path = [np.array(self.position)]
                    self.new_path = True
                    rospy.logwarn("Pas de trajectoire")
                    return None
            else:
                path_portion += 1


        path.pop(0)
        if iter == max_iter:
            rospy.logwarn("Trop d'itération :" + str(max_iter) + " itérations")
            return None
        else:
            #
            #rospy.loginfo("Found path" + str(path))
            pass
        
        if apply:
            self.path = path
            rospy.loginfo("Found path : " + str(self.path))
            self.new_path = True
        else:
            return path
        
        if self.verify_path():
            rospy.logerr("OBSTACLE SUR LE CHEMIN")

    def get_next_pos(self):
        """
        Get the next position to go and store it in self.next_goal
        """
        if len(self.path)!=0:
            self.next_goal = self.path[0]
        else:
            self.next_goal = self.position
        return self.next_goal
        
    def obstacle_variation(self, liste_obstacle):
        """
        Check if the obstacle list has changed and return a boolean
        """
        for prev_obs in self.obstacles:
            b=False
            for obs in liste_obstacle:
                #Trigger distance = 0.03
                b = b or np.linalg.norm(np.array(obs) - np.array(prev_obs)) < 5
            if b==False: # Aucun des nouveaux obstacles n'est proche de cet ancien
                return True
        return False
            
    def verify_path(self):
        """
        Verify if there is an obstacle on the path
        """
        for i in range(len(self.path)-1):
            # Find each cm of the path
            points = np.linspace(self.path[i], self.path[i+1], int(np.linalg.norm(np.array(self.path[i+1]) - np.array(self.path[i]))/0.01))
            for point in points:
                if self.is_obstacle(point[0], point[1]):
                    return True
        rospy.loginfo("No obstacle on the path")
        rospy.loginfo("Obstacles :" + str(self.obstacles))
        return False
    
    def add_obstacles(self, liste_obstacle):
        self.obstacles = liste_obstacle

        ## On ne garde que les obstacles qui sont sur le plateau ou à moins de 50cm du plateau
        Obstacles_coherents = []

        x_plateau = self.shape_board[0]
        y_plateau = self.shape_board[1]

        for obstacle in liste_obstacle:                            # Tout objet à plus de 50 cm du plateau est ignoré
            if (-50<obstacle[0] and obstacle[0]<x_plateau+50) and (-50<obstacle[1] and obstacle[1]<y_plateau+50):
                Obstacles_coherents.append(obstacle)

        ## On crée une aire de jeu plus grande que le plateau pour traiter les obstacles hors du plateau

        x_plateau_max = x_plateau + 200                    # On rajoute 100cm de marge sur chaque côté
        y_plateau_max = y_plateau + 200

        Cadrillage_max = np.zeros((x_plateau_max, y_plateau_max))

        ## On crée une matrice associée à chaque obstacle

        for obstacle in Obstacles_coherents: 
            x_obstacle = int(obstacle[0])+100
            y_obstacle = int(obstacle[1])+100
            rayon_obstacle = int(100*self.max_radius)

            # Placer des 1 dans le cercle de rayon rayon_obstacle autour de coordonnees_obstacle
            carre_rayon = rayon_obstacle**2
            M_obstacle = ((np.arange(2*rayon_obstacle)-rayon_obstacle)**2+((np.arange(2*rayon_obstacle)-rayon_obstacle)**2).reshape(2*rayon_obstacle,1)<=carre_rayon).astype(int)

            # On place la matrice dans la matrice Cadrillage_max aux bonnes coordonnées
            Cadrillage_max[x_obstacle-rayon_obstacle:x_obstacle+rayon_obstacle,y_obstacle-rayon_obstacle:y_obstacle+rayon_obstacle]= np.logical_or(Cadrillage_max[x_obstacle-rayon_obstacle:x_obstacle+rayon_obstacle,y_obstacle-rayon_obstacle:y_obstacle+rayon_obstacle],M_obstacle)

        ## On crée une matrice associée au plateau
        Cadrillage_rempli = Cadrillage_max[100:x_plateau_max-100,100:y_plateau+100]

        ## On met à jour les informations sur le plateau
        self.map_obstacles = np.logical_or(self.static_obstacles, Cadrillage_rempli)

    # Callbacks
    # ---------------------------------------------------------------------------------------------

    def position_goal_callback(self, msg):
        """
        Callback pour récupérer la position cible
        """
        dist = np.sqrt((msg.x - self.position[0])**2 + (msg.y - self.position[1])**2)
        if dist > 0.01: # Si la position cible est trop loin
            self.position_goal = [msg.x, msg.y]
            rospy.loginfo("Position goal : " + str(msg))
            color = ChooseColor(1, 0, 0)
            marker_array = MarkerArray()
            marker_pos_other = tool_pub.create_marker(800, 3, 0, 0.1, 0.1, msg.x, msg.y, color, scale_z=0.15, frame_id="fuzed") 
            marker_array.markers.append(marker_pos_other)
            self.pub_marker.publish(marker_array)
            self.master_path(self.position, self.position_goal)
        else : # La cible est proche
            rospy.loginfo("Cible atteinte")
            self.position_goal = self.position
            self.path = [np.array(self.position)]
        
    def publish_pic_msg(self, next_goal):
        """
        Publie un message ordonnant au robot de suivre le chemin "path"

        Paramètres
        ----------
            - path : liste\n
                Liste ordonnée des noeuds constituant le chemin
        """
        #rospy.loginfo("Going to : " + str(next_goal))
        if not(self.activate): # Si le nav_node n'est pas activé
            return None
        

        msg = Pic_Action()
        msg.action_destination = 'motor'
        msg.action_msg = 'moveseg'
        msg.action_msg += ' ' + str(next_goal[0]) + ' ' + str(next_goal[1])

        self.action_orders_pub.publish(msg)
        # marker_arr = MarkerArray()
        # if self.name_robot == "Han7":
        #     marker = tool_pub.create_marker(800, 3, 0, 0.1, 0.1, next_goal[0], next_goal[1], ChooseColor(0, 1, 0), scale_z=0.15, frame_id = "/robot_1/fuzed")
        # else:
        #     marker = tool_pub.create_marker(800, 3, 0, 0.1, 0.1, next_goal[0], next_goal[1], ChooseColor(0, 1, 0), scale_z=0.15, frame_id = "/robot_2/fuzed")
        # marker_arr.markers.append(marker)
        # pub_marker.publish(marker_arr)
        if self.debug:
            time.sleep(0.01)
            msg = MergedData()
            msg.robot_1 = Trajectoire()
            msg.robot_1.position = Point(next_goal[0], next_goal[1], 0)
            msg.robot_2 = Trajectoire()
            msg.robot_2.position = Point(1, 1, 0)
            #rospy.loginfo("[DEBUG] Publishing : " + str(Point(next_goal[0], next_goal[1], 0)))
            self.react_pub.publish(msg)

    def emergency_stop(self):
        """
        Stoppe le robot
        """
        self.path=[self.position]
        rospy.logwarn("STOP")
        
    def position_callback(self, msg):
        """
        Callback pour récupérer la position des robots
        """
        for i, elem in enumerate(self.path):
            color = ChooseColor(0, 1, 0)
            marker_array = MarkerArray()
            marker_pos_other = tool_pub.create_marker(2000 + i, 3, 0, 0.1, 0.1, elem[0], elem[1], color, scale_z=0.15, frame_id="/robot_1/base") 
            marker_array.markers.append(marker_pos_other)
            self.pub_marker.publish(marker_array)
        
        old_position = self.position

        if self.name_robot == "Han7":
            self.position = np.array([msg.robot_1[-1].position.x, msg.robot_1[-1].position.y])
            liste_obstacle = [np.array([msg.robot_2[-1].position.x, msg.robot_2[-1].position.y])]
            if not(msg.ennemi_1[-1].position.x == 0 and msg.ennemi_1[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_1[-1].position.x, msg.ennemi_1[-1].position.y]))
            if not(msg.ennemi_2[-1].position.x == 0 and msg.ennemi_2[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_2[-1].position.x, msg.ennemi_2[-1].position.y]))
            self.orientation = msg.robot_1[-1].position.z
            self.velocity = np.array([msg.robot_1[-1].vitesse.x, msg.robot_1[-1].vitesse.y, msg.robot_1[-1].vitesse.z])
        elif self.name_robot == "Gret7" :
            self.position = np.array([msg.robot_2[-1].position.x, msg.robot_2[-1].position.y])
            liste_obstacle = [np.array([msg.robot_1[-1].position.x, msg.robot_1[-1].position.y])]
            if not(msg.ennemi_1[-1].position.x == 0 and msg.ennemi_1[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_1[-1].position.x, msg.ennemi_1[-1].position.y]))
            if not(msg.ennemi_2[-1].position.x == 0 and msg.ennemi_2[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_2[-1].position.x, msg.ennemi_2[-1].position.y]))
            self.orientation = msg.robot_2[-1].position.z
            self.velocity = np.array([msg.robot_2[-1].vitesse.x, msg.robot_2[-1].vitesse.y, msg.robot_2[-1].vitesse.z])
        else :
            rospy.logerr("Nom de robot non reconnu")     

        if not(self.is_in_obstacle):
            self.obstacles_processing(liste_obstacle)

        if type(self.position_goal) != type(None):
            # Do not send another goal if the robot too close to the old position
            if np.linalg.norm(self.position - old_position) > self.distance_interpoint/2 or self.new_path:
                self.new_path = False

                rospy.loginfo("Path : " + str(self.path))     

                if len(self.path)!=0:
                    # Si il n'y a pas encore de position suivante on la récupère
                    if type(self.next_goal) == type(None):
                        self.get_next_pos()
                    else:
                        if np.linalg.norm(np.array(self.position) - np.array(self.next_goal)) < self.distance_interpoint:
                            # Si on est assez proche du point suivant
                            if len(self.path) == 0: # On a atteint la cible
                                rospy.loginfo("Cible atteinte")
                                self.action_done_pub.publish(True)
                                # Si l'on était dans un obstacle alors nous en sommes sorti
                                if self.is_in_obstacle:
                                    self.is_in_obstacle = False
                                    if self.is_obstacle(self.position[0], self.position[1]):
                                        # Si on est toujours dans un obstacle, on continue d'en sortir
                                        self.get_out_of_obstacle()
                                        rospy.loginfo("De nouveau dans un obstacle, sortie de l'obstacle")
                                    else:
                                        # Si on est sorti de l'obstacle, on peut continuer vers l'objectif
                                        rospy.loginfo("Recherche du chemin vers "+str(self.position_goal))
                                        self.master_path(self.position, self.position_goal)
                                else:
                                    # Si on n'était pas dans un obstacle, alors on est arrivé à l'objectif
                                    self.position_goal = None
                                    rospy.loginfo("Fin du chemin")
                                    return None
                            else:
                                rospy.loginfo("Advancing in path" + str(self.path))
                                self.path.pop(0)
                                self.get_next_pos()
                        else:
                            # Sinon on récupère la position suivante
                            self.get_next_pos()
                    if len(self.path) != 0:
                        # Dans tous les cas si l'on est pas arrivé, on publie la position suivante     
                        self.publish_pic_msg(self.next_goal)
            else:
                self.position = old_position
        else:
            pass 
            # rospy.loginfo("Cible non définie")

    def obstacles_processing(self, liste_obstacle):
        """
        Fonction pour récupérer les obstacles
        """
        
        # Emergency stop, le mettre à 0 désactive l'arret d'urgence
        """
        if self.emergency_stop_distance > 0:
            for obstacle in liste_obstacle:
                arr = self.chgt_base_plateau_to_robot(obstacle)
                if abs(arr[1]) < self.emergency_stop_distance and abs(arr[1]) < self.max_radius/2:
                    if self.orientation < np.pi/2 or self.orientation > 3*np.pi/2 : # Le robot va en avant
                        if arr[0] > 0:
                            self.emergency_stop()
                            rospy.logwarn("STOP")
                    else:
                        if arr[0]<0:
                            self.emergency_stop()
                            rospy.logwarn("STOP")
        """
        
        # Convert to cm
        liste_obstacle = np.array([obstacle*100 for obstacle in liste_obstacle])

        if self.obstacle_variation(liste_obstacle):

            # Save the map for debug
            rospack = rospkg.RosPack()
            rospack.list()
            np.save("{}/src/map/debug.npy".format(rospack.get_path('nav_node')), self.map_obstacles)

            #On ajoute les obstacles à la carte
            self.add_obstacles(liste_obstacle)

            # On met à jour le chemin si un obstacle se trouve dessus
            if self.position_goal is not None:
                if self.verify_path():
                    rospy.loginfo("Chemin obstrué")
                    # Si le chemin est obstrué on le recalcule
                    self.master_path(self.position, self.position_goal)
                elif np.linalg.norm(self.position - self.position_goal) > self.distance_interpoint:
                    # Sinon si l'objectif n'est pas trop proche
                    # On mesure le potentiel nouveau chemin path et on le compare à l'ancien
                    #self.path

                    path = self.master_path(self.position, self.position_goal, False)
                    length_of_path = np.linalg.norm(self.position - path[0])
                    for i in range(len(path)-1):
                        length_of_path += np.linalg.norm(np.array(path[i]) - np.array(path[i+1]))
                    length_of_old_path = np.linalg.norm(self.position - self.path[0])
                    for i in range(len(self.path)-1):
                        length_of_old_path += np.linalg.norm(np.array(self.path[i]) - np.array(self.path[i+1]))
                    
                    # Si le chemin est plus court on l'applique
                    if length_of_path < length_of_old_path:
                        self.path = path
                        rospy.loginfo("Found path : " + str(self.path))
                else:
                    rospy.loginfo("Chemin non obstrué")

    def chgt_base_plateau_to_robot(self, point):
        cos_angle = np.cos(self.orientation)
        sin_angle = np.sin(self.orientation)
        point_transforme_to_robot = np.array([0,0])
        point_transforme_to_robot[0] = (point[0] - self.position[0]) * cos_angle + (point[1] - self.position[1]) * sin_angle
        point_transforme_to_robot[1] = (self.position[0] - point[0]) * sin_angle + (point[1] - self.position[1]) * cos_angle
        return point_transforme_to_robot

    def activation_callback(self, msg):
        self.activation = msg.data

if __name__ == '__main__':

    rospy.init_node('nav_node', anonymous=False)
    pub_marker = rospy.Publisher(rospy.get_param("~pub_marker_other", "/robot_1/marker_obs"), MarkerArray, queue_size = 10)
    position_goal_topic = rospy.get_param('~position_goal_topic', '/robot_1/Pos_goal')
    action_orders_topic = rospy.get_param('~action_orders_topic', '/robot_1/action')
    action_done_topic = rospy.get_param('~action_done_topic', '/robot_1/action_done')
    max_iter = rospy.get_param('~max_iter', 15)
    distance_interpoint = rospy.get_param('~distance_interpoint', 0.03)
    margin = rospy.get_param('~margin', 0.1) #m
    emergency_stop_distance = rospy.get_param('~emergency_stop_distance', 0.2) #m
    positions_topic = rospy.get_param('~positions_topic', '/robot_1/positions_topic') 
    color = rospy.get_param('~color', 'Green')
    debug_mode = rospy.get_param('~debug_mode', True)
    name_robot = rospy.get_param('~name_robot', 'Han7')
    activate_topic = rospy.get_param('~activate_topic', '/robot_1/activation_nav_node')

    # Déclaration des Publishers
    action_orders_pub = rospy.Publisher(action_orders_topic, Pic_Action, queue_size=1)
    action_done_pub = rospy.Publisher(action_done_topic, Bool, queue_size=1)

    # Création de la classe NavigationNode
    Nav_node = NavNode(action_orders_pub=action_orders_pub, 
                       distance_interpoint=distance_interpoint, 
                       margin=margin, 
                       max_iter=max_iter,
                       emergency_stop_distance=emergency_stop_distance,
                       color=color,
                       name_robot=name_robot,
                       debug = debug_mode,
                       action_done_pub=action_done_pub)

    # Déclaration des Subscribers
    rospy.Subscriber(position_goal_topic, Point, Nav_node.position_goal_callback)
    rospy.Subscriber(positions_topic, MergedDataBis, Nav_node.position_callback)


    # Vérification de la présence d'obstacle sur le chemin du robot
    while not rospy.is_shutdown():

        rospy.sleep(0.05)