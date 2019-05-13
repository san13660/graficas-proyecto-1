# Christopher Sandoval 13660
# Proyecto 1

from collections import namedtuple

V2 = namedtuple('Vertex2',['x', 'y'])
V3 = namedtuple('Vertex3',['x', 'y', 'z'])

# Funcion para multiplicar matrices
def matrix_mult(a,b):
    zip_b = zip(*b)
    zip_b = list(zip_b)
    return [[sum(ele_a*ele_b for ele_a, ele_b in zip(row_a, col_b)) 
             for col_b in zip_b] for row_a in a]

# Producto punto de vectores
def dot(v0, v1):
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z

# Producto cruz de vectores
def cross(v0, v1):
    return V3(
        v0.y * v1.z - v0.z * v1.y,
        v0.z * v1.x - v0.x * v1.z,
        v0.x * v1.y - v0.y * v1.x
    )

# Encontrar el vector unitario
def unitary(v):
    l = length(v)

    if not l:
        return V3(0, 0, 0)

    return V3(v.x/l, v.y/l, v.z/l)

# Suma de vectores
def add(v0, v1):
    return V3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z)

# Resta de vectores
def sub(v0, v1):
    return V3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z)

# Multiplicacion de vectores
def mul(v, c):
    return V3(v.x * c, v.y * c, v.z * c)

# Longitud del vector
def length(v):
    return (v.x**2 + v.y**2 + v.z**2)**0.5