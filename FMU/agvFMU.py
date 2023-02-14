from fmpy import *
#from fmpy.utill import plot_result
import numpy as np



def fmu(path, setSpeed, posError, headingError):
    fmu = path
    dump(fmu)
    #read the model description
    model_description = read_model_description(fmu)
    vrs = {}
    for variable in model_description.modelVariables:
        vrs[variable.name] = variable.valueReference
    print(vrs)

    start_values = [setSpeed, posError, headingError]
    dtype = [('expseu_.SetSpeed_mps', float), ('expseu_.PosError_m', float), ('expseu_.HeadingError_rad', float)]

    signals = np.array(start_values, dtype=dtype)

   
    result = simulate_fmu(fmu, stop_time=0.5, input=signals)
    print(result)
    return result
    
def pushing_into_file(results):
    for result in results:
        PositionX_m = open("PositionX_m.txt", 'a+')
        PositionX_m.write(f"{result[0]},{result[1]} \n")
        PositionX_m.close()
        PositionY_m = open("PositionY_m.txt", 'a+')
        PositionY_m.write(f"{result[0]},{result[2]} \n")
        PositionY_m.close()
        PositionZ_m = open("PositionZ_m.txt", 'a+')
        PositionZ_m.write(f"{result[0]},{result[3]} \n")
        PositionZ_m.close()
        Roll_deg = open("Roll_deg.txt", 'a+')
        Roll_deg.write(f"{result[0]},{result[4]} \n")
        Roll_deg.close()
        Pitch_deg = open("Pitch_deg.txt", 'a+')
        Pitch_deg.write(f"{result[0]},{result[4]} \n")
        Pitch_deg.close()
        Yaw_deg = open("Yaw_deg.txt", 'a+')
        Yaw_deg.write(f"{result[0]},{result[4]} \n")
        Yaw_deg.close()
        WheelSpeedLeft_RPM = open("WheelSpeedLeft_RPM.txt", 'a+')
        WheelSpeedLeft_RPM.write(f"{result[0]},{result[4]} \n")
        WheelSpeedLeft_RPM.close()
        WheelSpeedRight_RPM = open("WheelSpeedRight_RPM.txt", 'a+')
        WheelSpeedRight_RPM.write(f"{result[0]},{result[4]} \n")
        WheelSpeedRight_RPM.close()





fmu("./AGV_turtle_electric_drive_Prescan_export.fmu",2.6, 0.1, 0.1)
pushing_into_file(fmu("./AGV_turtle_electric_drive_Prescan_export.fmu",0.26, 0.1, 0.1))


