'''
Per mediar el moviment entre posicions utilitzem inetploacions

Tenim 4 tipus de interpolacions:
    1. Linear (coord)
    2. Parabo (coord)
    3. Square (coord)
    4. Direct (servo)
'''
from . import position as ik
import numpy as np

def linear_interploation(prev_coord, next_coord, factor):
    (prev_X, prev_Y) = prev_coord
    (next_X, next_Y) = next_coord

    diff_X = next_X - prev_X
    diff_Y = next_Y - prev_Y

    X = prev_X + factor * diff_X
    Y = prev_Y + factor * diff_Y

    coord = (X, Y)
    ik.valid_coord(coord)
    return coord

def parabo_interploation(prev_coord, next_coord, factor, h=0.5):
    (prev_X, prev_Y) = prev_coord
    (next_X, next_Y) = next_coord

    diff_X = next_X - prev_X
    diff_Y = next_Y - prev_Y

    #linear part
    X = prev_X + factor * diff_X
    Y = prev_Y + factor * diff_Y

    #parabolic part
    h_factor = 4 * factor * (1 - factor)
    Y = Y + h*h_factor

    coord = (X, Y)
    ik.valid_coord(coord)

    return coord

def square_interploation(prev_coord, next_coord, factor, h=0.5):
    (prev_X, prev_Y) = prev_coord
    (next_X, next_Y) = next_coord

    diff_X = next_X - prev_X
    diff_Y = next_Y - prev_Y

    linear_distance = np.sqrt(diff_X**2 + diff_Y**2)
    total_distance =  linear_distance + 2*h

    factor_1 = (factor*total_distance)/h
    if factor_1 < 1:
        X = prev_X
        Y = prev_Y + factor_1 * h
    
        coord = (X, Y)
        ik.valid_coord(coord)
        return coord
    
    factor_2 = (factor*linear_distance - h)/linear_distance
    if factor_2 < 1:
        X = prev_X + factor * diff_X
        Y = prev_Y + factor * diff_Y + h
    
        coord = (X, Y)
        ik.valid_coord(coord)
        return coord
    
    factor_3 = (factor*linear_distance - (h+linear_distance))/h
    if factor_3 < 1:
        X = next_X
        Y = next_Y + (1-factor_3) * h
    
        coord = (X, Y)
        ik.valid_coord(coord)
        return coord
    
    return next_coord

def direct_interploation(prev_servo, next_servo, factor):
    (prev_U, prev_D) = prev_servo
    (next_U, next_D) = next_servo

    diff_U = next_U - prev_U
    diff_D = next_D - prev_D

    X = prev_U + factor * diff_U
    Y = prev_D + factor * diff_D

    servo = (X, Y)
    ik.valid_coord(servo)
    return servo

def interpolation(prev_pos, next_pos, factor, method='linear', height=0.5):
    if method=='linear':
        return linear_interploation(prev_pos, next_pos, factor)
    if method=='parabolic':
        return parabo_interploation(prev_pos, next_pos, factor, h=height)
    if method=='square':
        return square_interploation(prev_pos, next_pos, factor, h=height)
    if method=='direct':
        return direct_interploation(prev_pos, next_pos, factor)
    return None

if __name__ == '__main__':
    pass
