from math import sin,cos,atan2,sqrt
import sys
def cartesian_to_polar(x,y):
    r = sqrt(x*x + y*y)
    theta = atan2(y,x)
    return (r,theta)

def polar_to_cartesian(r,theta):
    x = r*cos(theta)
    y = r*sin(theta)
    return (x,y)

if __name__ == "__main__":
    try:    
        n = input('Enter option: \n 1.] cartesian to polar \n 2.] polar to cartesian \n')
        if n != 1 and n != 2:
            print('Enter 1 or 2 only!')
            sys.exit(1)
        
        if n == 1:
            (x,y) = input('Enter cartesian coordinates:\n')
            res = cartesian_to_polar(x,y)
            print(res)
        else:
            (r,theta) = input('Enter polar coordinates:\n')
            res = polar_to_cartesian(r,theta)
            print(res)
    
    except SyntaxError:
        print('Enter coordinates as (x,y) or (r,theta)')