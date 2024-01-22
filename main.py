import pygame
import numpy as np


def rotate_vector2(vector, axis, angle):
    
    # Convertir el ángulo a radianes
    angle_rad = np.radians(angle)

    # Normalizar el vector del eje para asegurar que tiene longitud 1
    axis = axis / np.linalg.norm(axis)

    # Calcular la matriz de rotación usando la fórmula de Rodrigues
    rotation_matrix = (
        np.cos(angle_rad) * np.eye(3) +
        (1 - np.cos(angle_rad)) * np.outer(axis, axis) +
        np.sin(angle_rad) * np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
    )

    # Aplicar la matriz de rotación al vector
    rotated_vector = np.dot(rotation_matrix, vector)

    return rotated_vector

def arduino_map(value, from_low, from_high, to_low, to_high):
    from_range = from_high - from_low
    to_range = to_high - to_low
    constrained_value = max(min(value, from_high), from_low)
    mapped_value = to_low + (constrained_value - from_low) * to_range / from_range

    return mapped_value

def normVector(a):
    if np.all(a==0):
        return a
    return a/np.linalg.norm(a)

def AngleBetweenUnitVecs(a,b):
    return np.arccos(np.dot(a,b))

def VectorBtw(VectorA,VectorB,VectorF,norm=True):

   
    if norm:
        VectorA = normVector(VectorA)
        VectorB = normVector(VectorB)
        VectorF = normVector(VectorF)
    Offset = np.dot(VectorA,VectorB)
    print(VectorA,VectorB)
    print("--")
    print(Offset,VectorF)
    print(np.dot(VectorA,VectorF))
    print(np.dot(VectorB,VectorF))
    return (np.dot(VectorA,VectorF) - Offset > 0) and (np.dot(VectorB,VectorF) - Offset > 0)


class Camera:

    CenterPlane = np.array([-3,-1,0.5],np.float64)
    FocalCenter = np.zeros(3,np.float64)
    FocalDist = 1
    NormalDirection = np.array([1,1,0],np.float64)
    NearPlaneConst = 0
    fov = 45
    farDist = 25
    look_up = np.array([0,1,0],np.float64)

    Axis = np.array([0,0,0],np.float64)
    Ordi = np.array([0,0,0],np.float64)
    BasesMatriz = None

    DeclinedVector =  np.zeros(3,np.float64)
    TurnedVector = np.zeros(3,np.float64)

    fovVectorScalar = 0

    DeclinedVector_ =  np.zeros(3,np.float64)
    TurnedVector_ =  np.zeros(3,np.float64)
    upVector = np.array([0,0,1])


    maxEscale = 0
    def __init__(self) -> None:
        self.update()
        self.MaxAndMins()


    def update(self):
        self.UpdatePlaneConst()
        self.UpdateFocalCenter()
        self.UpdateBases()
        self.getFovVectors()


    def getFovVectors(self):
        normalVec =  normVector(self.NormalDirection)


        self.DeclinedVector = rotate_vector2(normalVec,self.Axis,self.fov)
        #self.DeclinedVector_ = np.copy(self.DeclinedVector)
        #self.DeclinedVector_[2] *= -1 
        self.DeclinedVector_ = rotate_vector2(normalVec,self.Axis,-self.fov)

        
        
        self.TurnedVector = rotate_vector2(normalVec,self.upVector,self.fov)
        self.TurnedVector_ = rotate_vector2(normalVec,self.upVector,-self.fov)

        #self.TurnedVector_ = np.copy(self.TurnedVector)
        #self.TurnedVector_[2] *= -1 

        self.fovVectorScalar = np.dot(self.DeclinedVector,self.DeclinedVector_)

    def UpdateFocalCenter(self):
        self.FocalCenter = self.CenterPlane - (self.NormalDirection/np.linalg.norm(self.NormalDirection))*self.FocalDist

    def UpdatePlaneConst(self):
        self.NearPlaneConst = - np.dot(self.NormalDirection,self.CenterPlane)

    def NearPlaneInterception(self,point):
        pass

    def MoveCamera(self,direction):
        pass
    
    def HorizontalMove(self,steps):
        self.CenterPlane += self.Axis*steps

    def NormalMovement(self,steps):
        self.CenterPlane += self.NormalDirection*steps

    def RotateCamera(self,angle,axis):
        pass

    def btwVectorsAxis(self,vector):


        turnedScalar = np.dot(self.TurnedVector,vector) - self.fovVectorScalar
        turnedScalar_ = np.dot(self.TurnedVector_,vector) - self.fovVectorScalar

        return (turnedScalar > 0) and (turnedScalar_ > 0) 


    def btwVectorsOrdirnates(self,vector):
        declined = np.dot(self.DeclinedVector,vector) - self.fovVectorScalar
        declined_ = np.dot(self.DeclinedVector_,vector) - self.fovVectorScalar

        return (declined > 0) and (declined_ > 0) 

    def IsRenderablePoint(self,point):
        
        point = normVector(point- self.FocalCenter)



        AxisVec = normVector(np.array([point[0],point[1],self.TurnedVector[2]]))
        OrdiVec = normVector(np.array([point[0],self.DeclinedVector[1],point[2]]))



        return self.btwVectorsAxis(AxisVec) and self.btwVectorsOrdirnates(OrdiVec)

        
   
    def Plane3dProyect(self,punto1):

        a, b, c = self.NormalDirection
        d = self.NearPlaneConst

        x1, y1, z1 = punto1
        x2, y2, z2 = self.FocalCenter
        direccion_recta = (x2 - x1, y2 - y1, z2 - z1)
        t = (-a*x1 - b*y1 - c*z1 - d) / (a*direccion_recta[0] + b*direccion_recta[1] + c*direccion_recta[2])
        punto_interseccion = (x1 + t*direccion_recta[0], y1 + t*direccion_recta[1], z1 + t*direccion_recta[2])
        return punto_interseccion
   
    def R3ToR2Plnae(self,point):
        return  np.matmul(self.BasesMatriz,(point-self.CenterPlane))
   


    def UpdateBases(self):
        self.Axis =normVector(np.cross(self.NormalDirection ,self.upVector))
        self.Ordi = normVector(np.cross(self.Axis,self.NormalDirection ))



        self.BasesMatriz = np.array([self.Axis,self.Ordi])

    def MaxAndMins(self):
        """
        distances = (abs(self.CenterPlane-self.FocalCenter))

        point = []
        point2 = []
        convert = np.pi/180

        for i in distances:
            point.append(np.tan(self.fov*convert)*i)
            point2.append(np.tan(-self.fov*convert)*i)

        point = np.array(point)
        point2 = np.array(point2)

        self.maxEscale = np.linalg.norm(point)/2
        
        return  self.maxEscale
        """
        recta = ((self.FocalCenter),(self.TurnedVector))
        plano = (self.NormalDirection[0],self.NormalDirection[1],self.NormalDirection[2],self.NearPlaneConst)

        self.maxEscale = np.linalg.norm(self.interseccion_recta_plano(recta,plano) - self.CenterPlane) 
        return 
    
    def interseccion_recta_plano(self,recta, plano):
    # Desempaqueta la información de la recta
        punto_origen, vector_direccion = recta
        x0, y0, z0 = punto_origen
        u, v, w = vector_direccion

        # Desempaqueta la información del plano
        A, B, C, D = plano

        # Calcula el parámetro t en la ecuación de la recta (x = x0 + tu, y = y0 + tv, z = z0 + tw)
        t = (-A * x0 - B * y0 - C * z0 - D) / (A * u + B * v + C * w)

        # Calcula las coordenadas del punto de intersección
        punto_interseccion = (x0 + t * u, y0 + t * v, z0 + t * w)

        return punto_interseccion 
    

    def RenderPoint(self,point):

        if not self.IsRenderablePoint(point):

            return False
            
        proyect = self.Plane3dProyect(point)
        proyect = self.R3ToR2Plnae(proyect)

      
        
        return  arduino_map(proyect[0],-self.maxEscale,self.maxEscale,0,700),arduino_map(proyect[1],-self.maxEscale,self.maxEscale,0,700)
       

c = Camera()


cube =  [(0, 0, 0), (1, 0, 0), (0, 1, 0) ,(1, 1, 0), (0, 0, 1), (1, 0, 1),(0, 1, 1), (1, 1, 1)]




pygame.init()
    
    # Definir el tamaño de la pantalla
width, height = 700, 700
screen = pygame.display.set_mode((width, height))

while True:
    screen.fill((255, 255, 255))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

    
    for i in cube:
        point = c.RenderPoint(i)
      
        if point:
            pygame.draw.circle(screen,(0,0,0),point,4)
        

    keys = pygame.key.get_pressed()

    # Actualizar la posición del objeto basado en las teclas presionadas
    if keys[pygame.K_LEFT]:
        #c.CenterPlane +=[0,0.008,0] #[-0.008,0,0]
        c.HorizontalMove(0.008)
    if keys[pygame.K_RIGHT]:
        c.HorizontalMove(-0.008)
        #c.CenterPlane += [0,-0.008,0]#[0.008,0,0]
    if keys[pygame.K_UP]:
        c.NormalMovement(0.008)
        #c.CenterPlane += [0.008,0,0] #[0,0.008,0]
    if keys[pygame.K_DOWN]:
        c.NormalMovement(-0.008)
        #c.CenterPlane += [-0.008,0,0] #[0,-0.005,0]
    
    if keys.count(1) > 0:
        c.update()
    pygame.display.flip()

np.array([])
exit()
"""def rotate_vector(vector, right_rotation_angle, up_rotation_angle):
    # Normalizar el vector para obtener la dirección
    direction = vector / np.linalg.norm(vector)

    # Construir matrices de rotación
    rotation_right = np.array([
        [np.cos(right_rotation_angle), -np.sin(right_rotation_angle), 0],
        [np.sin(right_rotation_angle), np.cos(right_rotation_angle), 0],
        [0, 0, 1]
    ])

    rotation_up = np.array([
        [np.cos(up_rotation_angle), 0, np.sin(up_rotation_angle)],
        [0, 1, 0],
        [-np.sin(up_rotation_angle), 0, np.cos(up_rotation_angle)]
    ])

    # Aplicar las rotaciones a la dirección
    rotated_direction = rotation_right @ rotation_up @ direction

    # Escalar el vector resultante para mantener su magnitud original
    rotated_vector = rotated_direction * np.linalg.norm(vector)

    return rotated_vector"""







