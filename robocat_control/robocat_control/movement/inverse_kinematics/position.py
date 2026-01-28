'''
Per definir la posició, utilitzem Inverse Kinematics

Hi ha 3 tipus de sistemas de coordenades rellevants

1. Coord: (X, Y)
    definim dos valors (X, Y) pel punt que definim en el pla del moviment.
    (0, 0) es el eix central de la pota
    valors tal que X**2+Y**2>1 no son possibles.

2. Polar: (R, A)
    definim dos valors (R, A) pel cercle que representa el total de moviment
    (0, X) no es un valor acceptat, pero definiria el eix central de la pota
    R es troba entr 1 i 0, A entre 180, -90
    

3. Servo: (U, D)
    definim dos valors (U, D) com els angles que tenen els servos
    (0,0) representa la cama extesas cap abaix
    U te un rang de 90 a -90 i D de 0 a 180

Exemples de valors de una a altre:

            +-------+-------+-------+
            | Coord | Polar | Servo |
            +-------+-------+-------+
            | X=0   |R=1    |U=0    |
            | Y=-1  |A=0    |D=0    |
            +-------+-------+-------+
            | X=1   |R=1    |U=90   |
            | Y=0   |A=90   |D=0    |
            +-------+-------+-------+
            | X=0   |R=.5   |U=-60  |
            | Y=-.5 |A=0    |D=120  |
            +-------+-------+-------+
            | X=.5  |R=.5   |U=30   |
            | Y=0   |A=90   |D=120  |
            +-------+-------+-------+
            | X=-.5 |R=.5   |U=-120 <---ERROR
            | Y=0   |A=-90  |D=120  |
            +-------+-------+-------+
'''
import numpy as np

MIN_X = -1
MAX_X = 1
MIN_Y = -1
MAX_Y = 1
def valid_coord(coord):
    (X, Y) = coord

    assert MIN_X <= X <= MAX_X, f"error, X={X}"
    assert MIN_Y <= Y <= MAX_Y, f"error, Y={Y}"



MIN_R = 0.1 #Practic
MAX_R = 1
MIN_A = -np.pi
MAX_A = np.pi
def valid_polar(polar):
    (R, A) = polar

    assert MIN_R < R <= MAX_R, f"error, R={R}"
    assert MIN_A < A <= MAX_A, f"error, A={A}"



MIN_U = -np.pi/2
MAX_U = np.pi/2
MIN_D = 0
MAX_D = np.pi
def valid_servo(servo):
    (U, D) = servo

    assert MIN_U <= U <= MAX_U, f"error, U={U}"
    assert MIN_D <= D <= MAX_D, f"error, D={D}"

def coord_to_polar(coord, show=False):

    #valid_coord(coord)
    (X, Y) = coord

    R = np.sqrt(X**2 + Y**2)

    A = np.arctan2(Y, X)
    A = A + np.pi/2
    if A > np.pi: 
        A -= np.pi*2

    polar = (float(R), float(A))
    valid_polar(polar)

    if show: print(f"{coord} -> {polar}")
    return polar
def polar_to_coord(polar, show=False):
    #valid_polar(polar)
    (R, A) = polar

    A += np.pi*1.5

    X = R * np.cos(A)
    Y = R * np.sin(A)

    coord = (float(X), float(Y))
    valid_coord(coord)

    if show: print(f"{polar} -> {coord}")
    return coord

def polar_to_servo(polar, show=False):

    #valid_polar(polar)
    (R, A) = polar

    U = A - np.arccos(R)

    D = 2*np.arccos(R)

    servo = (float(U), float(D))
    valid_servo(servo)

    if show: print(f"{polar} -> {servo}")
    return servo

def servo_to_polar(servo, show=False):
    #valid_servo(servo)
    (U, D) = servo

    R = np.cos(D/2)

    A = U + np.arccos(R)

    polar = (float(R), float(A))
    valid_polar(polar)

    if show: print(f"{servo} -> {polar}")
    return polar

def coord_to_servo(coord, show=False):
    #valid_coord(coord)
    (X, Y) = coord

    R = np.sqrt(X**2 + Y**2)

    A = np.arctan2(Y, X)
    A = A + np.pi/2
    if A > np.pi: 
        A -= np.pi*2

    U = A - np.arccos(R)

    D = 2*np.arccos(R)

    servo = (float(U)*180/np.pi, float(D)*180/np.pi)
    #valid_servo(servo)

    if show: print(f"{coord} -> {servo}")
    return servo

def servo_to_coord(servo, show=False):
    #valid_servo(servo)
    (U, D) = servo

    R = np.cos(D/2)

    A = U + np.arccos(R)

    A += np.pi*1.5

    X = R * np.cos(A)
    Y = R * np.sin(A)

    coord = (float(X), float(Y))
    #valid_coord(coord)

    if show: print(f"{servo} -> {coord}")
    return coord


"""if __name__ == '__main__':
    coords = [
        (0, -1), 
        (1, 0), 
        (0, -0.5), 
        (0.5, 0), 
        (-0.5, -.5)
    ]

    #test coord_to_polar
    polars = []
    for i in range(len(coords)):
        polars.append(coord_to_polar(coords[i], show=True))

    #test polar_to_servo
    servos = []
    for i in range(len(coords)):
        servos.append(polar_to_servo(polars[i], show=True))

    for i in range(len(coords)):
        polar_to_coord(polars[i], show=True)"""

