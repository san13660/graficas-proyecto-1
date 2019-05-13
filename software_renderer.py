# Christopher Sandoval 13660
# Proyecto 1

from bitmap import Bitmap, color
from obj import Obj
from mtl import Mtl
from collections import namedtuple
from random import randint
from texture import Texture
from vector_operations import V2, V3, matrix_mult, unitary, dot, length, cross, sub
from math import cos, sin

class SoftwareRenderer(object):
    def __init__(self):
        self.bitmap = None

    # Inicializa el objeto bitmap
    def glInit(self):
        self.bitmap = Bitmap()

    # Sirve para definir el tamano de la imagen
    def glCreateWindow(self, width, height):
        self.bitmap.createWindow(width, height)

    # Sirve para definir el area en la que se desea dibujar
    def glViewPort(self, x, y, width, height):
        self.bitmap.setViewPort(x, y, width, height)

    # Sirve para definir el color con el que se limpia la ventana
    def glClearColor(self, r,g,b):
        r=int(r*255)
        g=int(g*255)
        b=int(b*255)
        self.bitmap.setClearColor(r,g,b)

    # Sirve para definir el color del vertex
    def glColor(self, r,g,b):
        r=int(r*255)
        g=int(g*255)
        b=int(b*255)
        self.bitmap.setVertexColor(r,g,b)

    # Sirve para definir la posicion de un punto
    def glVertex(self, x, y):
        newX = int(self.bitmap.viewPortX + ((self.bitmap.viewPortWidth)*((x+1)/2)))
        newY = int(self.bitmap.viewPortY + ((self.bitmap.viewPortHeigth)*((y+1)/2)))
        self.bitmap.drawPoint(newX, newY)

    # Sirve para definir la posicion absoluta de un punto
    def glVertexAbs(self, x, y):
        self.bitmap.drawPoint(x, y)

    # Sirve para limpiar toda la imagen con un color
    def glClear(self):
        self.bitmap.clearWindow()

    # Sirve para generar el archivo
    def glFinish(self, filename):
        self.bitmap.write(filename)

    # Sirve para dibujar una linea con posiciones absolutas
    def glLineAbsPos(self, x1, y1, x2, y2):

        # La distancia entre la posicion inicial y la final
        dy = abs(y2 - y1)
        dx = abs(x2 - x1)

        # Nos indica si el avance en y va a depender del avance en x o viceversa
        steep = dy > dx

        # Si la inclinacion es mayor a 45 grados entonces el avance en x dependera de y, se invierten las variables x y y
        if steep:
            tempX1 = x1
            x1 = y1
            y1 = tempX1

            tempX2 = x2
            x2 = y2
            y2 = tempX2

        # Si la linea va de derecha a izquierda se invierten las variables de x/y inicial y final para que siga funcionando el algoritmo
        if x1 > x2:
            tempX1 = x1
            x1 = x2
            x2 = tempX1

            tempY1 = y1
            y1 = y2
            y2 = tempY1

        # La distancia entre la posicion inicial y la final
        dy = abs(y2 - y1)
        dx = abs(x2 - x1)
        
        # El offset de la posicion y con respecto a la posicion inicial
        offset = 0

        # El limite que debe sobrepasar la posicion y para que se avance un pixel en y. Lo iniciamos en dx ya que cada ciclo de x le sumaremos 2*dy al offset y lo compararemos con 2*dx. Es otra forma de hacer lo siguiente pero SIN introducir decimales: iniciar el offset en 0, cada ciclo sumarle dx y compararlo con 0.5dy.
        threshold = dx

        # La variable y empieza en la y inicial
        y = y1

        # Se recorren todos los pixeles en x
        for x in range(x1, x2 + 1):
            # Si la linea tenia una pendiente mayor a 45 grados entonces se invirtieron las variables, por lo que se llama al comando con x y y invertidas
            if steep:
                self.bitmap.drawPoint(y, x)
            else:
                self.bitmap.drawPoint(x, y)
            
            # Se le suma al offset dos veces la diferencia de distancias en y 
            offset += dy * 2
            
            # Si el offset supera al threshold establecido entonces se mueve 1 en y
            if offset >= threshold:
                if y1 < y2:
                    y += 1
                else:
                    y -= 1

                # El treshold se ajusta para cuando toque moverse otro pixel en y sumandole dos veces dx
                threshold += dx * 2

    # Sirve para dibujar una linea con posiciones relativas (-1,1)
    def glLine(self, x1, y1, x2, y2):
        x1 = int(self.bitmap.viewPortWidth * ((x1+1)/2))
        y1 = int(self.bitmap.viewPortHeigth * ((y1+1)/2))
        x2 = int(self.bitmap.viewPortWidth * ((x2+1)/2))
        y2 = int(self.bitmap.viewPortHeigth * ((y2+1)/2))
        self.glLineAbsPos(x1, y1, x2, y2)

    # Sirve para cargar archivos .obj solo lineas
    def glLoadObjWireFrame(self, filename, translateX, translateY, scaleX, scaleY):

        # Se instancia la clase Obj
        obj = Obj(filename)

        for face in obj.faces:
            # Se establece la cantidad de vertices en cada cara
            vertexCount = len(face)

            for i in range(vertexCount):
                # Para hacer una linea se toma el vertice que indica i y el siguiente despues de i
                f1 = face[i] - 1
                nextVertex = i + 1

                # Si i es el ultimo vertice se conectara con el primero
                if nextVertex >= vertexCount:
                    nextVertex -= vertexCount

                f2 = face[nextVertex] - 1

                # Se encuentran las coordenadas de los vertices de la linea
                v1 = obj.vertices[f1]
                v2 = obj.vertices[f2]

                # Se establecen los puntos inicial y final de la linea, se aplica la traslacion y la escala
                x1 = (v1[0] + translateX) * scaleX
                y1 = (v1[1] + translateY) * scaleY
                x2 = (v2[0] + translateX) * scaleX
                y2 = (v2[1] + translateY) * scaleY

                # Se dibuja la linea
                self.glLine(x1, y1, x2, y2)

    def transform(self, vertex):
        # Se coloca el vector en una matriz para poder multiplicarlo con la funcion matrix_mult y se agrega una dimension mas para poder multiplicarlo con la matriz 4x4
        vertexR4 = [
            [vertex.x,0,0,0],
            [vertex.y,0,0,0],
            [vertex.z,0,0,0],
            [1,0,0,0]
        ]

        # Se hace la multiplicacion de la matriz con el vector
        transformed_vertex = matrix_mult(self.ViewPortMatrix,matrix_mult(self.ProjectionMatrix,matrix_mult(self.ViewMatrix,matrix_mult(self.ModelMatrix, vertexR4))))


        # Se selecciona solamente el los valores de la matriz que contienen el resultado de la multiplicacion y se forma un vector de tres dimensiones
        transformed_vertex = [
            int(transformed_vertex[0][0]/transformed_vertex[3][0]),
            int(transformed_vertex[1][0]/transformed_vertex[3][0]),
            int(transformed_vertex[2][0]/transformed_vertex[3][0]),
        ]

        return V3(*transformed_vertex)


    # Sirve para cargar archivos .obj con las caras pintadas
    def glLoadObjTexture(self, filename, filenameMTL, filenameT, translate, scale, rotate, shader):

        self.LoadModelMatrix(translate,scale,rotate)

        self.LoadViewPortMatrix()

        # Se instancia la clase Obj
        obj = Obj(filename)

        if(filenameMTL):
            # Se instancia la clase Mtl
            mtl = Mtl(filenameMTL)
        else:
            mtl = None

        if(filenameT):
            # Se instancia la clase Texture
            texture = Texture(filenameT)
        else:
            texture = None

        # Se hace un contador para saber en que cara vamos para despues asignar los materiales
        faceCount = 0

        for face in obj.faces:

            # Se obtienen las coordenadas de los tres vertices de cada cara
            x1 = obj.vertices[face[0][0]-1][0]
            y1 = obj.vertices[face[0][0]-1][1]
            z1 = obj.vertices[face[0][0]-1][2]
            x2 = obj.vertices[face[1][0]-1][0]
            y2 = obj.vertices[face[1][0]-1][1]
            z2 = obj.vertices[face[1][0]-1][2]
            x3 = obj.vertices[face[2][0]-1][0]
            y3 = obj.vertices[face[2][0]-1][1]
            z3 = obj.vertices[face[2][0]-1][2]

            v1 = V3(x1, y1, z1)
            v2 = V3(x2, y2, z2)
            v3 = V3(x3, y3, z3)

            # Se obtienen las normales de los tres vertices de cada cara
            nx1 = obj.normals[face[0][2]-1][0]
            ny1 = obj.normals[face[0][2]-1][1]
            nz1 = obj.normals[face[0][2]-1][2]
            nx2 = obj.normals[face[1][2]-1][0]
            ny2 = obj.normals[face[1][2]-1][1]
            nz2 = obj.normals[face[1][2]-1][2]
            nx3 = obj.normals[face[2][2]-1][0]
            ny3 = obj.normals[face[2][2]-1][1]
            nz3 = obj.normals[face[2][2]-1][2]

            n1 = V3(nx1, ny1, nz1)
            n2 = V3(nx2, ny2, nz2)
            n3 = V3(nx3, ny3, nz3)

            # Se establece un color base
            base_color = (0.5,0.5,0.5)

            if(mtl):
                # Para asignar el nuevo material en la lista miramos si ya pasamos el index que nos indica un cambio de material
                if(len(obj.materialIndex) > 0 and faceCount >= obj.materialIndex[0]):
                
                    # Se busca el nombre del material en la lista de materiales y luego se obtiene el RGB de ese material
                    base_color = mtl.materials[obj.materialNames[0]]

                    # Se eliminan esos materiales de la lista para que la siguiente vez que toque asignar material se asigne el siguiente de la lista
                    obj.materialNames.pop(0)
                    obj.materialIndex.pop(0)

            vt1 = V3(0, 0, 0)
            vt2 = V3(0, 0, 0)
            vt3 = V3(0, 0, 0)            

            if(texture):
                # Se obtienen las coordenadas de los vertices de la textura
                xt1 = int(obj.tVertices[face[0][1]-1][0] * texture.width) - 1
                yt1 = int(obj.tVertices[face[0][1]-1][1] * texture.height) - 1
                xt2 = int(obj.tVertices[face[1][1]-1][0] * texture.width) - 1
                yt2 = int(obj.tVertices[face[1][1]-1][1] * texture.height) - 1
                xt3 = int(obj.tVertices[face[2][1]-1][0] * texture.width) - 1
                yt3 = int(obj.tVertices[face[2][1]-1][1] * texture.height) - 1

                # Se colocan las coordenadas de los vertices de textura en vectores
                vt1 = V3(xt1, yt1, 0)
                vt2 = V3(xt2, yt2, 0)
                vt3 = V3(xt3, yt3, 0)

            # Se dibuja la cara y se rellena
            self.glFillTriangleBarycentricTexture((v1,v2,v3), base_color, texture, (vt1,vt2,vt3), (n1,n2,n3), shader)


            # Se le suma uno al contador de las caras
            faceCount += 1

    def LookAt(self, eye, center, up):
        # Se obtiene el eje z de la direccion desde la camara hasta el centro de la escena
        z = unitary(sub(eye,center))

        # Se obtiene el eje x de la direccion desde la camara hasta el centro de la escena
        x = unitary(cross(up,z))

        # Se obtiene el eje y de la direccion desde la camara hasta el centro de la escena
        y = unitary(cross(z,x))

        self.LoadViewMatrix(x, y, z, center)
        self.LoadProjectionMatrix(-1/length(sub(eye,center)))

    def LoadModelMatrix(self, translate, scale, rotate):
        # Se define la matriz de traslacion
        translateMatrix = [
                            [1,0,0,translate[0]],
                            [0,1,0,translate[1]],
                            [0,0,1,translate[2]],
                            [0,0,0,1]
                            ]
    
        # Se define la matriz de escala
        scaleMatrix = [
                            [scale[0],0,0,0],
                            [0,scale[1],0,0],
                            [0,0,scale[2],0],
                            [0,0,      0, 1]
                            ]

        # Se obtienen los tres ejes de la rotaci
        a = rotate[0]
        b = rotate[1]
        c = rotate[2]

        # Se define la matriz de rotacion en el eje x
        rotationMatrixX = [
                            [1,0,0,0],
                            [0,cos(a),-sin(a),0],
                            [0,sin(a),cos(a),0],
                            [0,0,0,1]
                            ]

        # Se define la matriz de rotacion en el eje y
        rotationMatrixY = [
                            [cos(b),0,sin(b),0],
                            [0,1,0,0],
                            [-sin(b),0,cos(b),0],
                            [0,0,0,1]
                            ]

        # Se define la matriz de rotacion en el eje z
        rotationMatrixZ = [
                            [cos(c),-sin(c),0,0],
                            [sin(c),cos(c),0,0],
                            [0,0,1,0],
                            [0,0,0,1]
                            ]
 
        # Se define la matriz de rotacion final
        rotationMatrix = matrix_mult(rotationMatrixX, matrix_mult(rotationMatrixY, rotationMatrixZ))

        # Se define la matriz de transformacion del modelo
        self.ModelMatrix = matrix_mult(translateMatrix, matrix_mult(rotationMatrix, scaleMatrix))


    def LoadViewMatrix(self, x, y, z, center):
        # Se definen las matrices de transformacion de la vista
        M = [
            [x.x, x.y, x.z, 0],
            [y.x, y.y, y.z, 0],
            [z.x, z.y, z.z, 0],
            [0,0,0,1]
            ]

        O = [
            [1,0,0,-center.x],
            [0,1,0,-center.y],
            [0,0,1,-center.z],
            [0,0,0,1]
        ]

        self.ViewMatrix = matrix_mult(M,O)

    def LoadProjectionMatrix(self, coeff):
        # Se define la matriz de proyeccion
        self.ProjectionMatrix = [
                                [1,0,0,0],
                                [0,1,0,0],
                                [0,0,1,0],
                                [0,0,coeff,1]
        ]
    
    def LoadViewPortMatrix(self, x=0, y=0):
        # Se define la matriz de transformacion del view port
        self.ViewPortMatrix = [
                            [self.bitmap.width/2, 0, 0, x+self.bitmap.width/2],
                            [0,self.bitmap.height/2, 0, y+self.bitmap.height/2],
                            [0,0,128,128],
                            [0,0,0,1]
        ]

    # Sirve para rellenar poligonos previamente dibujados
    def glFillPolygons(self):
        # Se establece un "color" para marcar las orillas, se usan numeros negativos para asegurarse que no se confunda con ningun otro color
        edgeColor = [-1, -1, -1]

        # Se convierten todas las orillas de los poligonos al color establecido arriba
        for y in range(self.bitmap.height):
            for x in range(self.bitmap.width):
                if self.bitmap.framebuffer[y][x] != self.bitmap.clearColor:
                    self.bitmap.framebuffer[y][x] = edgeColor

        # Se recorren todos los pixeles de el archivo de abajo para arriba y de izquierda a derecha
        for y in range(self.bitmap.height):
            # Esta varible  indica si el pixel esta adentro de algun poligono
            inside = False
            
            # Estas dos variables nos ayudan a determinar si se esta entrando o saliendo de un poligono
            edgeAbove = False
            edgeBelow = False
            for x in range(self.bitmap.width):
                # Si el pixel es del color de las orillas se inicia el proceso de verificacion para saber si se esta entrando o saliendo de un poligono
                if self.bitmap.framebuffer[y][x] == edgeColor:
                    for a in range(-1,2):
                        # Para determinar si se esta entrando o saliendo de un poligono se revisa que alguno de los tres pixeles adyacentes en la linea de arriba tambien sea una orilla
                        if self.bitmap.framebuffer[y+1][x+a] == edgeColor:
                            edgeAbove = True
                        # Para determinar si se esta entrando o saliendo de un poligono se revisa que alguno de los tres pixeles adyacentes en la linea de abajo tambien sea una orilla
                        if self.bitmap.framebuffer[y-1][x+a] == edgeColor:
                            edgeBelow = True
                    # Si las dos condiciones de arriba se cumplen quiere decir que se esta entrando o saliendo de un poligono
                    if edgeBelow and edgeAbove:
                        # Se invierte la variable que nos dice si esta adentro o afuera de un poligono 
                        inside = not inside
                        edgeBelow = False
                        edgeAbove = False
                else:
                    # Si actualmente se esta en un pixel que no sea una orilla y se esta adentro de un poligono entonces se debe pintar el pixel
                    if inside:
                        self.bitmap.framebuffer[y][x] = self.bitmap.vertexColor

        # Por ultimo se repintan todas las orillas de los poligonos a su color original
        for y in range(self.bitmap.height):
            for x in range(self.bitmap.width):
                if self.bitmap.framebuffer[y][x] == edgeColor:
                    self.bitmap.framebuffer[y][x] = self.bitmap.vertexColor

    # Sirve para definir la caja en la que se van a verificar las coordenadas barycentricas
    def boundingBox(self, A,B,C):
        x = sorted([A.x, B.x, C.x])
        y = sorted([A.y, B.y, C.y])
        return V2(x[0], y[0]), V2(x[2], y[2])

    # Sirve para encontrar las coordenadas barycentricas de un triangulo
    def barycentric(self, A, B, C, P):
        cx, cy, cz = cross(V3(B.x - A.x, C.x - A.x, A.x - P.x), V3(B.y - A.y, C.y - A.y, A.y - P.y))

        # Se contempla el caso en el que cz sea cero
        if cz == 0:
            w = -1
            v = -1
            u = -1
        else:
            # Se obtienen las coordenadas barycentricas
            u = cx/cz 
            v = cy/cz
            w = 1 - (u + v)
        return w, v, u

    def glDrawBackground(self, image):
        bg = Texture(image)
        for x in range(self.bitmap.width):
            for y in range(self.bitmap.height):
                self.bitmap.framebuffer[y][x] = color(*bg.pixels[y][x])

    # Se pintan los triangulos con ayuda de coordenadas barycentricas
    def glFillTriangleBarycentricTexture(self, obj_vertices, base_color, texture, texture_vertex, normal_vectors, shader):

        # 
        A = self.transform(obj_vertices[0])
        B = self.transform(obj_vertices[1])
        C = self.transform(obj_vertices[2])

        # Se calcula la caja en la que estara el triangulo
        bbox_min, bbox_max = self.boundingBox(A, B, C)

        # Para cada punto de la caja se calculara si se encuentra dentro del triangulo para saber si se pinta
        for x in range(bbox_min.x, bbox_max.x + 1):
            for y in range(bbox_min.y, bbox_max.y + 1):
                # Se obtienen las coordenadas barycentricas
                w, u, v = self.barycentric(A, B, C, V2(x, y))

                # Si alguna de las coordenadas es menor a cero se salta este punto ya que no esta 
                if w < 0 or v < 0 or u < 0:
                    continue

                if texture:
                    # Se calcula la x y y de la posicion en la imagen de textura
                    tA, tB, tC = texture_vertex
                    tx = tA.x * w + tB.x * v + tC.x * u
                    ty = tA.y * w + tB.y * v + tC.y * u

                    # Se obtiene el color de la textura y se calcula su intensidad
                    base_color = texture.get_color_at_pos(tx, ty)

                if shader:          
                    final_color = shader(base_color, obj_vertices, (w,v,u), normal_vectors)
                else:
                    final_color = base_color

                self.glColor(*final_color)

                # Se calcula el valor de z para saber donde va en el zbuffer
                z = A.z * w + B.z * v + C.z * u

                # Si la z de este punto de esta cara es mayor a la que ya estaba en esta posicion entonces se dibuja

                if x>0 and x<self.bitmap.width and y>0 and y<self.bitmap.height and z > self.bitmap.zbuffer[y][x]:
                    self.bitmap.drawPoint(x,y)
                    self.bitmap.zbuffer[y][x] = z

                


# Instanciacion del objeto
sr = SoftwareRenderer()

#---------------------Metodos Globales-----------------------

def glInit():
    sr.glInit()

def glCreateWindow(width, height):
    sr.glCreateWindow(width,height)

def glViewPort(x, y, width, height):
    sr.glViewPort(x,y,width,height)

def glClearColor(r,g,b):
    sr.glClearColor(r,g,b)

def glColor(r,g,b):
    sr.glColor(r,g,b)

def glVertex(x, y):
    sr.glVertex(x,y)

def glVertexAbs(x,y):
    sr.glVertexAbs(x,y)

def glClear():
    sr.glClear()

def glFinish(filename='out.bmp'):
    sr.glFinish(filename)

def glLine(x1, y1, x2, y2):
    sr.glLine(x1, y1, x2, y2)

def glLineAbsPos(x1, y1, x2, y2):
    sr.glLineAbsPos(x1, y1, x2, y2)

def glLoadObjWireFrame(filename, translateX=0, translateY=0, scaleX=1, scaleY=1):
    sr.glLoadObjWireFrame(filename, translateX, translateY, scaleX, scaleY)

def glLoadObjTexture(filename, filename_material='', filename_texture='', translate=(0,0,0), scale=(1,1,1), rotate=(0,0,0), shader=None):
    sr.glLoadObjTexture(filename, filename_material, filename_texture, translate, scale, rotate, shader)

def glFillPolygons():
    sr.glFillPolygons()

def glLookAt(eye, center, up):
    sr.LookAt(eye, center, up)

def glDrawBackground(image):
    sr.glDrawBackground(image)