import math
def calc_pweight(x, y, mux, muy, sx=0.3, sy=0.3):
    return 1./(2*math.pi*sx*sx)*math.exp(-(((x-mux)**2)/(2*sx**2) + ((y-muy)**2)/(2*sy**2)))

