import sympy
from sympy import symbols, ln, exp, solve, Eq

def diode():
    print( "This program uses 3 diode measurements to extract parameters." )
    print( "You will need to have taken these measurements beforehand." )
    print( "Enter each point as [ <diode current>, <diode voltage> ]." )
    print( "" )
    TA= int( input( "Enter the ambient temperature in Celsius (default is 27 C): " ) or "27" )
    print( "" )
    VT= 8.61733034e-5 * ( 273.15 + TA )
    vd, id, N, ISAT, RS= symbols( "vd id N ISAT RS" )
    # POINTS= [{id: 2e-3, vd: 1.18}, {vd: 1.44, id: 50e-3}, {vd: 1.68, id: 100e-3}]
    # POINTS= [{id: 10e-3, vd: 1}, {vd: 1.5, id: 100e-3}, {vd: 2.5, id: 850e-3}]
    POINTS= [{id: 1e-3, vd: 1.18}, {vd: 1.6, id: 110e-3}, {vd: 2.8, id: 850e-3}]
    # for i in range(3):	
    #    pid, pvd= input( "Enter point " + str(i) + ": " ).split()
    #    POINTS.append( { vd: float( pvd ), id: float( pid ) } )
    EQS= []
    for i in range(3):
        EQS.append( Eq( POINTS[i][vd], RS*POINTS[i][id] + N*VT*ln(POINTS[i][id]) - N*VT*ISAT ) )
    print( POINTS )
    print( EQS )
    ANS= solve( EQS, [ RS, N, ISAT ] )[0]
    print( "RS   = " + str(ANS[0]) )
    print( "N    = " + str(ANS[1]) )
    print( "ISAT = " + str(exp(ANS[2])) )



if __name__ == "__main__":
    diode()
