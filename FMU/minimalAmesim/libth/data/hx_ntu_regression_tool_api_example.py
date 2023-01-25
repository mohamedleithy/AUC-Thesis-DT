#coding:latin-1
"""
LMS Imagine.Lab AMESim
NTU Regression Tool API example file

This file shows a simple example of what can be done with the Regression Tool API
The RegressionToolApi and its following methods are used:
    - open
    - save
    - getParameter
    - setParameter
    - doRegression
    - getErrorMax
    - getErrorMean
    - getRegressionCoefficient
    
Other available methods
    - getPsimu
    - getTableColumn
    - setTableColumn
    
"""

## IMPORT
import sys,os
import numpy as np
from toolbox_thm.hx_ntu_regression_tool_api import RegressionToolApi



## DATA
REGTOOL_DEFAULT_FILE=os.getenv('AME')+'/libthh/data/HX_NTU_Regression_Tool_example.data'

## MAIN
if __name__=='__main__':

    myRegTool=RegressionToolApi()
    
    # open default file
    print("\ncase 1 open the default table :")
    myRegTool.open(REGTOOL_DEFAULT_FILE)
    myRegTool.save('1_example_before_optim.data')
    print("nusselt correlation for flow 1 = %s"%myRegTool.getParameter("nuturb1"))
    print("nusselt correlation for flow 2 = %s"%myRegTool.getParameter("nuturb2"))
    print("error max = %s%%"%myRegTool.getErrorMax())
    print("error mean = %s%%"%myRegTool.getErrorMean())
    print("R^2 = %s"%myRegTool.getRegressionCoefficient())
    
    # perform regression with default parameterization
    print("\ncase 2 perform a regression :")
    myRegTool.doRegression()
    myRegTool.save('2_example_after_optim.data')
    print("nusselt correlation for flow 1 = %s"%myRegTool.getParameter("nuturb1"))
    print("nusselt correlation for flow 2 = %s"%myRegTool.getParameter("nuturb2"))
    print("error max = %s%%"%myRegTool.getErrorMax())
    print("error mean = %s%%"%myRegTool.getErrorMean())
    print("R^2 = %s"%myRegTool.getRegressionCoefficient())
    
    # setup a better first guess for the turbulent nusselt correlations
    print("\ncase 3 use a better first guess :")
    myRegTool.setParameter("nuturb1","2*Re^0.8*Pr^0.3333")
    myRegTool.setParameter("nuturb2","2*Re^0.8*Pr^0.3333")
    myRegTool.doRegression()
    myRegTool.save('3_example_better_first_guess.data')
    print("nusselt correlation for flow 1 = %s"%myRegTool.getParameter("nuturb1"))
    print("nusselt correlation for flow 2 = %s"%myRegTool.getParameter("nuturb2"))
    print("error max = %s%%"%myRegTool.getErrorMax())
    print("error mean = %s%%"%myRegTool.getErrorMean())
    print("R^2 = %s"%myRegTool.getRegressionCoefficient())
    
    # change air temperature data
    print("\ncase 4 change fluid 1 (air) temperature to 20 degC:")
    temp=myRegTool.getTableColumn(1) # this returns (data, title, unit)
    print("%s [%s] old data: %s"%(temp[1],temp[2],temp[0]))
    data=30*np.ones(temp[0].shape)
    myRegTool.setTableColumn(1,data)
    temp=myRegTool.getTableColumn(1) # this returns (data, title, unit)
    print("%s [%s] new data: %s"%(temp[1],temp[2],temp[0]))
    myRegTool.doRegression(gwallReg=True, nulam1Reg=True, nuturb1Reg=True, nulam2Reg=True, nuturb2Reg=True)
    myRegTool.save('4_example_changed_ext_temp.data')
    print("Gwall = %s"%myRegTool.getParameter("gwall"))
    print("nusselt correlation for flow 1 = %s"%myRegTool.getParameter("nuturb1"))
    print("nusselt correlation for flow 2 = %s"%myRegTool.getParameter("nuturb2"))
    print("error max = %s%%"%myRegTool.getErrorMax())
    print("error mean = %s%%"%myRegTool.getErrorMean())
    print("R^2 = %s"%myRegTool.getRegressionCoefficient())
    