# Christopher Sandoval 13660
# Proyecto 1

import software_renderer as sr
from collections import namedtuple
from vector_operations import V2, V3, dot

def shader_1(base_color, obj_vertices, barycentric_coord, normal_vectors):
        A, B, C = obj_vertices
        w, v, u = barycentric_coord
        nA, nB, nC = normal_vectors

        x = A.x * w + B.x * v + C.x * u
        y = A.y * w + B.y * v + C.y * u
        z = A.z * w + B.z * v + C.z * u

        nx = nA.x * w + nB.x * v + nC.x * u
        ny = nA.y * w + nB.y * v + nC.y * u
        nz = nA.z * w + nB.z * v + nC.z * u

        vn = V3(nx, ny, nz)

        intensity = dot(vn, V3(0,1,1))

        base_color = (base_color[0] + abs(0.4 * y), base_color[1], base_color[2])

        if y < -0.25:
            base_color = (base_color[0], base_color[1] + (abs(y+0.25)*4), base_color[2])

        if intensity < 0:
            intensity = 0

        intensity += 0.2

        if intensity > 1:
            intensity = 1

        return (
            base_color[0] * intensity if base_color[0] * intensity <= 1 else 1,
            base_color[1] * intensity if base_color[1] * intensity <= 1 else 1,
            base_color[2] * intensity if base_color[2] * intensity <= 1 else 1,
        )

def shader_2(base_color, obj_vertices, barycentric_coord, normal_vectors):
        A, B, C = obj_vertices
        w, v, u = barycentric_coord
        nA, nB, nC = normal_vectors

        x = A.x * w + B.x * v + C.x * u
        y = A.y * w + B.y * v + C.y * u
        z = A.z * w + B.z * v + C.z * u

        nx = nA.x * w + nB.x * v + nC.x * u
        ny = nA.y * w + nB.y * v + nC.y * u
        nz = nA.z * w + nB.z * v + nC.z * u

        vn = V3(nx, ny, nz)

        intensity = dot(vn, V3(0,1,1))

        base_color = (base_color[0], base_color[1] + abs(0.2 * y), base_color[2])

        if y < 0.20:
            base_color = (base_color[0], base_color[1] + (abs(y-0.20)*3), base_color[2])

        if intensity < 0:
            intensity = 0

        intensity += 0.2

        if intensity > 1:
            intensity = 1

        return (
            base_color[0] * intensity if base_color[0] * intensity <= 1 else 1,
            base_color[1] * intensity if base_color[1] * intensity <= 1 else 1,
            base_color[2] * intensity if base_color[2] * intensity <= 1 else 1,
        )

def shader_3(base_color, obj_vertices, barycentric_coord, normal_vectors):
        A, B, C = obj_vertices
        w, v, u = barycentric_coord
        nA, nB, nC = normal_vectors

        x = A.x * w + B.x * v + C.x * u
        y = A.y * w + B.y * v + C.y * u
        z = A.z * w + B.z * v + C.z * u

        nx = nA.x * w + nB.x * v + nC.x * u
        ny = nA.y * w + nB.y * v + nC.y * u
        nz = nA.z * w + nB.z * v + nC.z * u

        vn = V3(nx, ny, nz)

        intensity = dot(vn, V3(0,1,1))

        if y < 0.10:
            base_color = (base_color[0], base_color[1] + (abs(y-0.10)*2), base_color[2])

        if intensity < 0:
            intensity = 0

        intensity += 0.2

        if intensity > 1:
            intensity = 1

        return (
            base_color[0] * intensity if base_color[0] * intensity <= 1 else 1,
            base_color[1] * intensity if base_color[1] * intensity <= 1 else 1,
            base_color[2] * intensity if base_color[2] * intensity <= 1 else 1,
        )

def shader_4(base_color, obj_vertices, barycentric_coord, normal_vectors):
        A, B, C = obj_vertices
        w, v, u = barycentric_coord
        nA, nB, nC = normal_vectors

        x = A.x * w + B.x * v + C.x * u
        y = A.y * w + B.y * v + C.y * u
        z = A.z * w + B.z * v + C.z * u

        nx = nA.x * w + nB.x * v + nC.x * u
        ny = nA.y * w + nB.y * v + nC.y * u
        nz = nA.z * w + nB.z * v + nC.z * u

        vn = V3(nx, ny, nz)

        intensity = dot(vn, V3(0,1,1))

        if y < 0.15:
            base_color = (base_color[0], base_color[1] + (abs(y-0.15)*2), base_color[2])

        if intensity < 0:
            intensity = 0

        intensity += 0.2

        if intensity > 1:
            intensity = 1

        return (
            base_color[0] * intensity if base_color[0] * intensity <= 1 else 1,
            base_color[1] * intensity if base_color[1] * intensity <= 1 else 1,
            base_color[2] * intensity if base_color[2] * intensity <= 1 else 1,
        )

def shader_5(base_color, obj_vertices, barycentric_coord, normal_vectors):
        A, B, C = obj_vertices
        w, v, u = barycentric_coord
        nA, nB, nC = normal_vectors

        x = A.x * w + B.x * v + C.x * u
        y = A.y * w + B.y * v + C.y * u
        z = A.z * w + B.z * v + C.z * u

        nx = nA.x * w + nB.x * v + nC.x * u
        ny = nA.y * w + nB.y * v + nC.y * u
        nz = nA.z * w + nB.z * v + nC.z * u

        vn = V3(nx, ny, nz)

        intensity = dot(vn, V3(0,1,1))

        if y < -0.08:
            base_color = (base_color[0] + (abs(y+0.08)*5), base_color[1] + (abs(y+0.08)*5), base_color[2])

        if z > 0.02:
            base_color = (base_color[0] + (abs(z-0.02)*5), base_color[1] + (abs(z-0.02)*4), base_color[2])

        if intensity < 0:
            intensity = 0

        intensity += 0.2

        if intensity > 1:
            intensity = 1

        return (
            base_color[0] * intensity if base_color[0] * intensity <= 1 else 1,
            base_color[1] * intensity if base_color[1] * intensity <= 1 else 1,
            base_color[2] * intensity if base_color[2] * intensity <= 1 else 1,
        )

sr.glInit()
sr.glCreateWindow(800,800)
sr.glViewPort(0,0,800,800)
sr.glClearColor(0,0,0)

sr.glLookAt(V3(0,0,4),V3(0,0,0),V3(0,1,0))

sr.glDrawBackground('background.bmp')

sr.glLoadObjTexture(filename='dino4.obj', filename_texture='dino1.bmp', translate=(0.5,0.2,0), scale=(1,1,1), rotate=(0,1.1,0), shader=shader_1)

sr.glLoadObjTexture(filename='dino2.obj', filename_texture='dino3.bmp', translate=(-0.2,-0.18,0), scale=(0.5,0.5,0.5), rotate=(0,1.3,0), shader=shader_2)

sr.glLoadObjTexture(filename='dino3.obj', filename_texture='dino4.bmp', translate=(-0.5,-0.4,0.6), scale=(0.8,0.8,0.8), rotate=(0,1,0), shader=shader_3)

sr.glLoadObjTexture(filename='dino1.obj', filename_texture='dino2.bmp', translate=(0.5,-0.6,1), scale=(0.8,0.8,0.8), rotate=(0,-1,0), shader=shader_4)

sr.glLoadObjTexture(filename='dino5.obj', filename_texture='dino5.bmp', translate=(-0.5,0.8,0), scale=(0.8,0.8,0.8), rotate=(1.57,-0.4,1), shader=shader_5)

sr.glFinish()

    