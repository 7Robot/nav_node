import rospy
from geometry_msgs.msg import Point
from obstacle_detector.msg import Obstacles
from cdf_msgs.msg import RobotData, Pic_Action
from virtual_robot.msg import Virtual_Robot_ActionAction, Virtual_Robot_ActionGoal
from std_msgs.msg import Bool
import numpy as np
import time
import cv2
import actionlib

class PathCircle():
    """
    Classe définissant un chemin comme arc de cercle

    x²+y²+2ax+2by+c=0

    Le robot utilise x_c, y_c et R/ a et b entre start[0] et end[0] et parcours la courbe dans le sens sens[0] (False sens horaire/ x décroissants, True sens trigo/x croissants)
    is_circle[0] est True si le chemin est un cercle, False si c'est une affine
    """
    def __init__(self):
        self.x_c = []
        self.y_c = []
        self.R = []

        self.a = []
        self.b = []

        self.is_circle = []

        self.start = []
        self.end = []
        self.sens = []    

    def add_circle(self, x_c, y_c, R, start, end, sens, path_portion):
        self.x_c.insert(path_portion,x_c)
        self.y_c.insert(path_portion,y_c)
        self.R.insert(path_portion,R)
        self.is_circle.insert(path_portion,True)
        self.start.insert(path_portion,start)
        self.end[path_portion-1] = start
        self.end.insert(path_portion,end)

        # On vérifie si en allant dans le sens trigo on se rapproche de la cible
        psi = np.angle(start[0] + 1j*start[1] - (x_c + 1j*y_c))
        eps = 0.174533 # 10° en radians
        self.sens.insert(path_portion,))

        self.a.insert(path_portion,0)
        self.b.insert(path_portion,0)

    def add_affine(self, start, end, path_portion):
        self.start.insert(path_portion,start)
        self.end[path_portion-1] = start
        self.end.insert(path_portion,end)

        self.is_circle.insert(path_portion,False)
        self.a.insert(path_portion,(end[1]-start[1])/(end[0]-start[0]))
        self.b.insert(path_portion,start[1] - start[0]*(end[1]-start[1])/(end[0]-start[0]))

        self.x_c.insert(path_portion,0)
        self.y_c.insert(path_portion,0)
        self.R.insert(path_portion,0)
        self.sens.insert(path_portion,end[0]>start[0])
    
    def sfb(self, b):
        """
        Retourne 1 si b est True, -1 sinon
        """
        return 1 if b else -1
        



class NavigationNode():
    def __init__(self, distance_interpoint, margin):
        # Initialize the node
        rospy.init_node('navigation_node', anonymous=True)

        # Initialize the robot data
        self.robot_data = RobotData()

        # Initialize the path
        self.path = PathCircle()
        self.path_portion = 0
        self.next_goal = []
        self.distance_interpoint = distance_interpoint
        self.margin = margin

        # Initialize static obstacles
        self.static_obstacles = np.load("cvmap.npy")
        self.map_obstacles = self.static_obstacles.copy()
        self.max_radius = 0.28

        # Initialize the board
        self.board = (200,300)

        
    def find_path_without_obstacles(self, start, goal):
        """
        Trouve un chemin entre deux points sans obstacles

        Paramètres
        ----------
            - start : tuple\n
                Position de départ du robot
            - goal : tuple\n
                Position cible du robot

        Retourne
        ----------
            - path : PathCubic\n
                Chemin cubique définit plus haut
        """
        # Réinitialisation du chemin
        self.path = PathCircle()
        self.path_portion = 0


        self.path.start += [start]
        self.path.end = [goal]

        # Calcul des coefficients de la courbe affine
        (x0, y0) = start
        (x1, y1) = goal
        try:
            self.path.a += [(y1-y0)/(x1-x0)]
            self.path.b += [y0 - x0*(y1-y0)/(x1-x0)]
        except ZeroDivisionError:
            self.path.a += [0]
            self.path.b += [0]
            print("Division par 0 !!!")


        # On vérifie si la courbe est bien croissante ou décroissante
        self.path.sens += [x1 > x0]


    def correct_path(self, path):
        """
        Corrige le chemin pour éviter les obstacles
        """

        for path_portion in range(len(self.path.x_c)):
            if self.path.is_circle[path_portion]:
                # Le chemin est un cercle
                # On vérifie sur chaque centimètre si on touche un obstacle
                pos = self.path.start[path_portion]
                dtheta = 1-0.01**2/(2*path.R[self.path_portion]**2)

                while np.linalg.norm(pos - self.path.end[path_portion]) > 0.01:
                    angle = np.angle(pos[0] + 1j*pos[1] - (path.x_c[self.path_portion] + 1j*path.y_c[self.path_portion]))
                    angle += path.sfb(path.sens[self.path_portion])*np.arccos(dtheta)
                    pos[0] = int(path.x_c[self.path_portion] + path.R[self.path_portion]*np.cos(angle))
                    pos[1] = int(path.y_c[self.path_portion] + path.R[self.path_portion]*np.sin(angle))
                    if self.is_collision(pos[0], pos[1]):
                        # On a touché un obstacle, on doit trouver un point de contournement
                        # On cherche un point sur la normale à la droite
                        self.circumvent(path_portion, pos[0], pos[1])
                        continue

            else:
                # Le chemin est une courbe affine
                # On vérifie sur chaque centimètre si on touche un obstacle
                for x in range(int(self.path.start[path_portion][0]), int(self.path.end[path_portion][0])):
                    y = x * self.path.a[path_portion] + self.path.b[path_portion]
                    if self.is_collision(x, y):
                        # On a touché un obstacle, on doit trouver un point de contournement
                        # On cherche un point sur la normale à la droite
                        self.circumvent(path_portion, x, y)
                        continue

    def is_collision(self, x, y):
        """
        Vérifie si le point (x, y) touche un obstacle
        """
        (xf,yf) = (int(x*100), int(y*100))
        return self.map_obstacles[xf, yf] != 0

    def calc_circle(self, start, goal, dodge_point):
        """
        Crée un cercle entre start et goal en passant par dodge_point
        """
        x = [start[0], dodge_point[0], goal[0]]
        y = [start[1], dodge_point[1], goal[1]]

        # Solve A*t = r
        A = np.zeros((3, 3))
        for i in range(3):
            A[i] = np.array([2*x[i], 2*y[i], 1])
        
        r = np.zeros((3,1))
        for i in range(3):
            r[i] = -(y[i]**2 + x[i]**2)
        
        t = np.linalg.solve(A, r)

        (a, b, c) = t
        xc = -a
        yc = -b
        R = np.sqrt(xc**2 + yc**2 - c)

        # Find the sens
        psi = np.angle(start[0] + 1j*start[1] - (xc + 1j*yc))
        eps = 0.174533 # 10° en radians
        sens = np.hypot(goal[0] - start[0], goal[1] - start[1]) > np.hypot(goal[0] - xc + R*np.cos(psi), goal[1] - yc + R*np.sin(psi))

        return (xc, yc, R, sens)

    def circumvent(self, path_portion, x, y):
        """
        Crée un arc de cercle entre la position et la fin de la portion de chemin
        """

        # Find the normal
        if self.path.is_circle:
            # The path is a circle
            (x0,y0) = self.robot_data.position
            (x1,y1) = self.path.end[path_portion]
            (a,b) = ((y1-y0)/(x1-x0), y0 - x0*(y1-y0)/(x1-x0))
        else:
            # The path is an affine
            (a,b) = (self.path.a[path_portion], self.path.b[path_portion])
        sens = x > self.robot_data.position[0]

        # Find the normal vector
        u = np.array([-a, 1])
        u = u/np.linalg.norm(u)

        # Find the center of the obstacle_zone
        x_end = x
        y_end = y

        while self.is_collision(x_end, y_end):
            x_end += 0.01 # Regarde un cm plus loin
            y_end += 0.01 * a
        
        x_middle = (x + x_end)/2
        y_middle = (y + y_end)/2

        # Find the dodge point
        while self.is_collision(x_middle, y_middle):
            x_middle += u[0] * 0.01
            y_middle += u[1] * 0.01
        
        """
        # Create margin around the obstacle
        x_start = x-self.path.sfb(sens)*self.path.margin
            # Si le chemin est croissant x_start doit être plus grand que pos[0] et inversement
        if sens and x_start<self.robot_data.position[0]:
            x_start = self.robot_data.position[0]
        elif not sens and x_start>self.robot_data.position[0]:
            x_start = self.robot_data.position[0]
        
        """
        # Find and create the circle
        (xc, yc, R) = self.calc_circle((x,y), (x_end, y_end), (x_middle, y_middle))
        self.add_circle(xc, yc, R, (x,y), (x_end, y_end), path_portion)

    def follow_path(self):
        """
        Fait suivre le chemin au robot
        """
        pos = self.robot_data.pos
        path = self.path
# On vérifie si on est arrivé à la fin d'une portion de chemin
        if (pos[0]-path.end[self.path_portion][0])**2 + (pos[1]-path.end[self.path_portion][1])**2 < self.distance_interpoint**2:
            self.path_portion += 1
            print("Portion suivante")

        if path.is_circle[self.path_portion]:
            # Trouver la position du robot sur le cercle
            angle = np.angle(pos[0] + 1j*pos[1] - (path.x_c[self.path_portion] + 1j*path.y_c[self.path_portion]))
            dtheta = 1-self.distance_interpoint**2/(2*path.R[self.path_portion]**2)
            if  dtheta > 1:
                pos = path.end[self.path_portion]
            else:
                angle += path.sfb(path.sens[self.path_portion])*np.arccos(dtheta)
                pos[0] = path.x_c[self.path_portion] + path.R[self.path_portion]*np.cos(angle)
                pos[1] = path.y_c[self.path_portion] + path.R[self.path_portion]*np.sin(angle)
        else:
                pos[0] += path.sfb(path.sens[self.path_portion])*self.distance_interpoint/np.sqrt(1+path.a[self.path_portion]**2)
                pos[1] = pos[0]*path.a[self.path_portion] + path.b[self.path_portion]
        rospy.log("Consigne : ", (pos[0],pos[1]))

        # On publie la consigne

        self.publish_pic_msg((pos[0],pos[1]))

    # Callbacks

    def position_goal_callback(self, msg):
        """
        Callback pour récupérer la position cible
        """
        print("Position goal : " + str(msg))
        self.position_goal = [msg.x, msg.y]
        
    def publish_pic_msg(self, next_goal):
        """
        Publie un message ordonnant au robot de suivre le chemin "path"

        Paramètres
        ----------
            - path : liste\n
                Liste ordonnée des noeuds constituant le chemin
        """
        msg = Pic_Action()
        msg.action_destination = 'motor'
        msg.action_msg = 'MOVE'
        msg.action_msg += ' ' + str(next_goal[0]) + ' ' + str(next_goal[1])
        self.action_orders_pub.publish(msg)

        if debug_mode:
            action_client_goal = Virtual_Robot_ActionGoal()
            action_client_goal.command = msg.action_msg
            action_client.send_goal(action_client_goal)
        
    def robot_data_callback(self, msg):
        self.robot_data = msg
  

    def obstacles_callback(self, msg):
        """
        Callback pour récupérer les obstacles
        """
        
        ## On récupère les obstacles
        Liste_obstacle = [(obstacle.position.x, obstacle.position.y, self.max_radius) for obstacle in msg.other_robot_lidar]

        ## On ne garde que les obstacles qui sont sur le plateau ou à moins de 50cm du plateau
        Obstacles_coherents = []

        x_plateau = self.board[0]
        y_plateau = self.board[1]

        for obstacle in Liste_obstacle:                            # Tout objet à plus de 50 cm du plateau est ignoré
            if (-50<obstacle[0] and obstacle[0]<x_plateau+50) and (-50<obstacle[1] and obstacle[1]<y_plateau+50) and (obstacle[2]<50):
                Obstacles_coherents.append(obstacle)

        ## On crée une aire de jeu plus grande que le plateau pour traiter les obstacles hors du plateau

        x_plateau_max = x_plateau + 200                    # On rajoute 100cm de marge sur chaque côté
        y_plateau_max = y_plateau + 200

        Cadrillage_max = np.zeros((x_plateau_max, y_plateau_max))

        ## On crée une matrice associée à chaque obstacle
        
        for obstacle in Liste_obstacle:
            x_obstacle = obstacle[0]+100
            y_obstacle = obstacle[1]+100
            rayon_obstacle = obstacle[2]

            # Placer des 1 dans le cercle de rayon rayon_obstacle autour de coordonnees_obstacle
            carre_rayon = rayon_obstacle**2
            M_obstacle = ((np.arange(2*rayon_obstacle)-rayon_obstacle)**2+((np.arange(2*rayon_obstacle)-rayon_obstacle)**2).reshape(2*rayon_obstacle,1)<=carre_rayon).astype(int)

            # On place la matrice dans la matrice Cadrillage_max aux bonnes coordonnées
            Cadrillage_max[x_obstacle-rayon_obstacle:x_obstacle+rayon_obstacle,y_obstacle-rayon_obstacle:y_obstacle+rayon_obstacle]= np.logical_or(Cadrillage_max[x_obstacle-rayon_obstacle:x_obstacle+rayon_obstacle,y_obstacle-rayon_obstacle:y_obstacle+rayon_obstacle],M_obstacle)

        ## On crée une matrice associée au plateau
        Cadrillage_rempli = Cadrillage_max[100:x_plateau_max-100,100:y_plateau+100]

        ## On met à jour les informations sur le plateau
        self.map_obstacles = np.logical_or(self.static_obstacles, Cadrillage_rempli)

                  
    def next_point_callback(self, msg):
        self.next_point = msg
    
    def motion_done_callback(self, msg):
        pass
        
if __name__ == '__main__':
    rospy.init_node('navigation_node', anonymous=False)

    position_goal_topic = rospy.get_param('~position_goal_topic', '/robot_x/Pos_goal')
    obstacles_topic = rospy.get_param('~obstacles_topic', '/obstacles')
    action_orders_topic = rospy.get_param('~action_orders_topic', '/robot_x/action')
    next_point_topic = rospy.get_param('~next_point_topic', '/robot_x/asserv_next_point')
    graph_file = rospy.get_param('~graph_file', 'default')
    cv_obstacle_file = rospy.get_param('~cv_obstacle_file', 'default')
    debug_mode = rospy.get_param('~debug_mode', False)
    distance_interpoint = rospy.get_param('~distance_interpoint', '15')
    margin = rospy.get_param('~margin', 0.1) #m
    emergency_stop_distance = rospy.get_param('~emergency_stop_distance', 0.2) #m
    robot_data_topic = rospy.get_param('~robot_data_topic', '/robot_x/Odom')
    robot_x_dimension = rospy.get_param('~robot_x_dimension', 0.5) #m
    robot_y_dimension = rospy.get_param('~robot_y_dimension', 0.2) #m

    # Déclaration des Publishers
    action_orders_pub = rospy.Publisher(action_orders_topic, Pic_Action, queue_size=1)

    if debug_mode:
        action_client = actionlib.SimpleActionClient('pic_action', Virtual_Robot_ActionAction)

    # Création de la classe NavigationNode
    Nav_node = NavigationNode(distance_interpoint, margin)

    # Déclaration des Subscribers
    rospy.Subscriber(position_goal_topic, Point, Nav_node.position_goal_callback)
    rospy.Subscriber(robot_data_topic, RobotData, Nav_node.robot_data_callback)
    rospy.Subscriber(obstacles_topic, Obstacles, Nav_node.obstacles_callback)
    rospy.Subscriber(next_point_topic, Point, Nav_node.next_point_callback)
    rospy.Subscriber('/robot_x/motion_done', Bool, Nav_node.motion_done_callback)

    # Vérification de la présence d'obstacle sur le chemin du robot
    while not rospy.is_shutdown():

        rospy.sleep(0.05)