# -*- coding: Utf-8 -*-

### Generic imports ###
import sys
import os
import math

### Simcenter Amesim imports ###
from numpy import *

### libeng toolbox imports ###
from libeng_toolbox.ICEUnits import convert

### Application imports ###
from Combustion_tool.COMBImports import *
if 'Combustion_tool.Model.Engthpri' in sys.modules :
    del sys.modules['Combustion_tool.Model.Engthpri']
from Combustion_tool.Model.Engthpri import *

def IGR_Estimation(cpt, OPnum, fixed_f = None) :
    OP     = cpt['OPManager'].getOP(OPnum)
    Engine = cpt['ENGDEF']
    Fuel   = cpt['ENGMD']

    OPName     = OP.getParam('name').evaluated_value
    RPM        = OP.getParam('N').evaluated_value          # rev/min
    BMEP       = OP.getParam('BMEP').evaluated_value       # bar
    Tegr       = OP.getParam('EGR').evaluated_value        # %

    Padm       = OP.getParam('PADM').evaluated_value       # bar
    Tadm       = OP.getParam('TADM').evaluated_value       # K
    Pexh       = OP.getParam('PEXH').evaluated_value       # bar
    Texh       = OP.getParam('TEXH').evaluated_value       # K

    IVC        = OP.getParam('IVC').evaluated_value        # degree
    EVO        = OP.getParam('EVO').evaluated_value        # degree
    EVC        = OP.getParam('EVC').evaluated_value        # degree
    
    MFR_air    = OP.getParam('MFR').evaluated_value        # g/s
    M_fuel     = OP.getParam('MFuelInj').evaluated_value   # g

    data       = OP.getPcylExp(HP = True)
    Pivc       = data['Pressure'][0]                       # bar
    Vivc       = Engine.computeVol([IVC])[0]
    Vivc       = convert('cm**3', 'm**3', Vivc)            # m**3
    
    # Engine
    rod_length = Engine.getParam('L').evaluated_value      # mm
    stroke     = Engine.getParam('stroke').evaluated_value # mm
    bore       = Engine.getParam('bore').evaluated_value   # mm
    CR         = Engine.getParam('rc').evaluated_value     # mm
    rman       = Engine.getParam('Rman').evaluated_value   # mm

    # Fuel
    AFRSto     = Fuel.getParam('AFRSto').evaluated_value
    PCI        = Fuel.getParam('pci').evaluated_value      # kJ/kg
    W_C        = Fuel.getParam('W_C').evaluated_value      # g/mol
    W_N        = Fuel.getParam('W_N').evaluated_value      # g/mol
    W_O        = Fuel.getParam('W_O').evaluated_value      # g/mol
    W_H        = Fuel.getParam('W_H').evaluated_value      # g/mol
    Cx         = Fuel.getParam('xCxHyOz').evaluated_value
    Hy         = Fuel.getParam('yCxHyOz').evaluated_value
    Oz         = Fuel.getParam('zCxHyOz').evaluated_value

    if fixed_f != None :
        f = fixed_f
    else :
        f = None

    ################## Your code here ##################

    # File must have Pinit in bar and Tinit in K 
    File = os.path.expandvars(r'$AME/libeng/data/Data_combustion/Data_Tool_CFM/Param/OP_init_cond_iter02.txt')

    f = open(File, 'r')
    data = f.readlines()
    f.close()
    
    i = 0
    ok = False
    for row in data :
        if row.strip()[0] != '#' and row.split() != '' :
            if i == OPnum :
                ok = True
                f  = 0
                Pinit = row.split()[0]
                Tinit  = row.split()[1]
                Xinit1 = row.split()[2]
                Xinit2 = row.split()[3]
                Xinit3 = row.split()[4]
                
                
            i += 1
    if not ok :
        f  = 0
        Pinit = 1
        Tinit = 273.15
        Xinit1 = 1
        Xinit2 = 0
        Xinit3 = 0
        if OP.mode == BMF :
            M1init = 1
            M2init = 0
            M3init = 0

    ############## End of your code here ###############
    
    OP.getParam('f').text_value      = f
    OP.getParam('Pinit').text_value  = Pinit
    OP.getParam('Tinit').text_value  = Tinit
    OP.getParam('X1init').text_value = Xinit1
    OP.getParam('X2init').text_value = Xinit2
    OP.getParam('X3init').text_value = Xinit3
    if OP.mode == BMF :
        OP.getParam('M1init').text_value = M1init
        OP.getParam('M2init').text_value = M2init
        OP.getParam('M3init').text_value = M3init

