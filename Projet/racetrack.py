from trackgen import Track
from math import pi
import numpy as np

def generate_track(Length = 1000, Width = 6):
    # Where are the corners?
    crns = np.array( [False,True,False,True], dtype=bool )

    # Change in angle (needs to be zero for straight)
    delTh = np.array( [0,pi,0,pi], dtype=float )

    # length parameter initial guess (radius for corner, length for straight)
    lpar = np.array( [Length/2,Length/5,Length/2,Length/5], dtype=float )

    track = Track( length = Length, width = Width, left = True, crns = crns )

    sol = track.solve( lpar, delTh, case = 2 )

    xe, ye, thcum = track.endpoint()

    print( "End point   = (%4.3f, %4.3f)" % (xe, ye) )
    print( "Final angle =  %4.3f" % (thcum) )

    return track

