#coding:latin-1
"""
LMS Imagine.Lab AMESim
SEC Regression Tool API example file

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
from toolbox_thm.hx_sec_regression_tool_api import RegressionToolApi


## DATA
REGTOOL_DEFAULT_FILE=os.getenv('AME')+'/libheat/data/HX_SEC_Regression_Tool_example.data'

## MAIN
if __name__=='__main__':

    myRegTool=RegressionToolApi()
    
    # open default file
    print "\ncase 1 open the default table :"
    myRegTool.open(REGTOOL_DEFAULT_FILE)
    myRegTool.save('1_example_before_optim.data')
    print "km   = %s"%myRegTool.getParameter("km")
    print "aint (high flow) = %s"%myRegTool.getParameter("aint")
    print "bint (high flow) = %s"%myRegTool.getParameter("bint")
    print "aext (high flow) = %s"%myRegTool.getParameter("aext")
    print "bext (high flow) = %s"%myRegTool.getParameter("bext")
    print "aint (low flow) = %s"%myRegTool.getParameter("aintlam")
    print "bint (low flow) = %s"%myRegTool.getParameter("bintlam")
    print "aext (low flow) = %s"%myRegTool.getParameter("aextlam")
    print "bext (low flow) = %s"%myRegTool.getParameter("bextlam")
    print "deviation max = %s%%"%myRegTool.getErrorMax()
    print "deviation mean = %s%%"%myRegTool.getErrorMean()
    print "R^2 = %s"%myRegTool.getRegressionCoefficient()
    
    # perform regression with default parameterization
    print "\ncase 2 perform a regression :"
    myRegTool.doRegression()
    myRegTool.save('2_example_after_optim.data')
    print "km   = %s"%myRegTool.getParameter("km")
    print "aint (high flow) = %s"%myRegTool.getParameter("aint")
    print "bint (high flow) = %s"%myRegTool.getParameter("bint")
    print "aext (high flow) = %s"%myRegTool.getParameter("aext")
    print "bext (high flow) = %s"%myRegTool.getParameter("bext")
    print "deviation max = %s%%"%myRegTool.getErrorMax()
    print "deviation mean = %s%%"%myRegTool.getErrorMean()
    print "R^2 = %s"%myRegTool.getRegressionCoefficient()
    
    # change internal fluid to glycol
    print "\ncase 3 change internal fluid to glycol at 2bar (<=> 3.013barA) \n\t perform a new regression on the 5 coefficients:"
    myRegTool.setParameter("fluidInt","glycol")
    myRegTool.setParameter("prefInt","3.013")
    myRegTool.doRegression(kmReg=True, aextReg=True, bextReg=True, aintReg=True, bintReg=True)
    myRegTool.save('3_example_glycol.data')
    print "km   = %s"%myRegTool.getParameter("km")
    print "aint (high flow) = %s"%myRegTool.getParameter("aint")
    print "bint (high flow) = %s"%myRegTool.getParameter("bint")
    print "aext (high flow) = %s"%myRegTool.getParameter("aext")
    print "bext (high flow) = %s"%myRegTool.getParameter("bext")
    print "deviation max = %s%%"%myRegTool.getErrorMax()
    print "deviation mean = %s%%"%myRegTool.getErrorMean()
    print "R^2 = %s"%myRegTool.getRegressionCoefficient()
    
    # change air temperature data
    print "\ncase 4 change external temperature to 20 degC:"
    temp=myRegTool.getTableColumn(1) # this returns (data, title, unit)
    print "%s [%s] old data: %s"%(temp[1],temp[2],temp[0])
    data=20*np.ones(temp[0].shape)
    myRegTool.setTableColumn(1,data)
    temp=myRegTool.getTableColumn(1) # this returns (data, title, unit)
    print "%s [%s] new data: %s"%(temp[1],temp[2],temp[0])
    myRegTool.doRegression(kmReg=True, aextReg=True, bextReg=True, aintReg=True, bintReg=True)
    myRegTool.save('4_example_changed_ext_temp.data')
    print "km   = %s"%myRegTool.getParameter("km")
    print "aint (high flow) = %s"%myRegTool.getParameter("aint")
    print "bint (high flow) = %s"%myRegTool.getParameter("bint")
    print "aext (high flow) = %s"%myRegTool.getParameter("aext")
    print "bext (high flow) = %s"%myRegTool.getParameter("bext")
    print "deviation max = %s%%"%myRegTool.getErrorMax()
    print "deviation mean = %s%%"%myRegTool.getErrorMean()
    print "R^2 = %s"%myRegTool.getRegressionCoefficient()
    
    # use different parameters for high flow and low flow
    print "\ncase 5 use different parameters for high flow and low flow:"
    # change default transition values:
    myRegTool.setParameter("gextTrans","3.5")
    myRegTool.setParameter("gintTrans","4")
    myRegTool.doRegression(kmReg=True, aextReg=True, bextReg=True, aintReg=True, bintReg=True, aextLamReg=True, bextLamReg=True, aintLamReg=True, bintLamReg=True)
    myRegTool.save('5_example_high_flow_low_flow.data')
    print "km   = %s"%myRegTool.getParameter("km")
    print "aint (high flow) = %s"%myRegTool.getParameter("aint")
    print "bint (high flow) = %s"%myRegTool.getParameter("bint")
    print "aext (high flow) = %s"%myRegTool.getParameter("aext")
    print "bext (high flow) = %s"%myRegTool.getParameter("bext")
    print "GextTrans = %s"%myRegTool.getParameter("gextTrans")
    print "GintTrans = %s"%myRegTool.getParameter("gintTrans")
    print "aint (low flow) = %s"%myRegTool.getParameter("aintlam")
    print "bint (low flow) = %s"%myRegTool.getParameter("bintlam")
    print "aext (low flow) = %s"%myRegTool.getParameter("aextlam")
    print "bext (low flow) = %s"%myRegTool.getParameter("bextlam")
    print "deviation max = %s%%"%myRegTool.getErrorMax()
    print "deviation mean = %s%%"%myRegTool.getErrorMean()
    print "R^2 = %s"%myRegTool.getRegressionCoefficient()
    