# -*- coding: utf-8 -*-
"""
Created on Mon Mar 20 10:48:32 2023

@author: Nick Wouters
"""

from numbers import Number
import numpy as np
from sklearn.neighbors import KernelDensity

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from rsQualitativeModel.QDataType import Sign
from rsLibrary.StatisticsUtils import JointDistribution
from rsLibrary.Variable import Variable, VariableType, QualType, OrdinalType, RobotVariable, DerivedVariable
from rsLibrary.DataUtils import List, Dict


class Col(list):
    def __init__(self, variable:Variable):
        self.variable = variable
    def __str__(self): return self.variable.name+":" +super().__str__()
    
    def name(self): 
        return self.variable.name
    
    
class DataSet:
    """
    Class to store all the data in a structured way, but without information
    about the context or links between the variables
    """
    def __init__(self, *variables:Variable):
        """Initialise empty list for each var in variables"""
        self.variables = List(variables)
        self.data = Dict({var:List([]) for var in variables}, newLine = True) # Dict for nice printing
        
    @property
    def varNames(self):
        return [var.name for var in self.variables]
        
    def __getitem__(self, var) -> list:
        """Return list corresponding to variable"""
        if isinstance(var, str):
            newVar = self.getVarWithName(var)
        else:
            newVar = var
        return self.data[newVar] 
    
    def __setitem__(self, var, data:list):
        """Set new list with data for var"""
        if not data == None and (type(data)==list or type(data)==List):
            # var = Variable(varName, VariableType.)
            if isinstance(var, str):
                if var not in self.varNames:
                    newVar = Variable.createSimpleVariables(var)[0]
                    self.variables.append(newVar)
                else:
                    newVar = self.variables[self.varNames.index(var)]
            else:
                newVar = var
                if newVar.name not in self.varNames:
                    self.variables.append(newVar)
            self.data[newVar] = data
    
    def __str__(self):
        return f"DataSet with \n\tVariables: {[str(var.name) for var in self.variables]}\n\tData: {self.data}"
    
    # Operators on data
    def Q(self, var):
        return List( [Sign.sign(d) for d in self[var]] )
    
    def d(self, var):
        return [0]+[self[var][idx+1]-self[var][idx] for idx in range(len(self[var])-1)]
    
    def Qd(self, var):
        self['d'+var] = self.d(var)
        return self.Q("d"+var)
    
    def old(self, var):
        # print(var, self)
        return [0] + self[var][:-1]
    
    def addDictToData(self, varDataDict):
        self.data = {**self.data, **varDataDict}
        
    def getVarWithName(self, var:str):
        return self.variables[self.varNames.index(var)] if var in self.varNames else None
    
    def view(self, *varNames):
        """Keep only variables in view, returns new DataSet"""
        varList = [var for var in self.variables if var.name in varNames]
        viewDat = DataSet(*varList)
        for varName in varNames:
            var = self.getVarWithName(varName)
            # If var not qualitative or already in variables, no need to apply operator
            if (varName[0] not in ['q', 'd'] and varName[-4:]!="_old") or self.getVarWithName(var) in self.variables:
                viewDat[varName] = self[varName]
            # Else apply operator
            elif varName[:2]=='qd' and varName[2:] in self.varNames:
                viewDat[varName] = self.Qd(varName[2:])
            elif varName[0]=='q' and varName[1:] in self.varNames:
                viewDat[varName] = self.Q(varName[1:])
            elif varName[0]=='d' and varName[1:] in self.varNames:
                viewDat[varName] = self.d(varName[1:])
            elif varName[-4:]=="_old" and varName[:-4] in self.varNames:
                viewDat[varName] = self.old(varName[:-4])
            else:
                print("Error for operation on non-existing var")
        return viewDat
    
    def addDerivedVariable(self, *varNames):
        for varName in varNames:
            if varName[:2]=='qd' and varName[2:] in self.varNames:
                self[varName] = self.Qd(varName[2:])
            elif varName[0]=='q' and varName[1:] in self.varNames:
                self[varName] = self.Q(varName[1:])
            elif varName[0]=='d' and varName[1:] in self.varNames:
                self[varName] = self.d(varName[1:])
            elif varName[-4:]=="_old" and varName[:-4] in self.varNames:
                self[varName] = self.old(varName[:-4])
            else:
                print("Do not understand this derived variable:", varName)

    def addT(self):
        self['t'] = [i for i in range(0, len(self[self.variables[0]]))]
        
    def createJointDistribution(self, bw=1.0, nr_bins=10, kernel='gaussian', rtol=0.25):
        # List to store probabilities
        probs = []
        # List to store all ranges of the vars
        # TODO: use built-in range type of python! list of ranges, with step size equal to (max-min)/nr_bins or amount of qualitative values
        # Possibly generalises over both quantitative as qualitative variables (with float casting of Sign) only possible for ordinal variables (with Sign for example, counterexample: Colors)
        all_ranges = []
        # Ordered dataset with qualitative variables first
        orderedDat = self.ordered()
        # print(orderedDat)
        
        # Store qualitative spaces (possible discrete values) as dict (list for each var)
        qual_spaces = {}
        # Store freqs of each combination in freqs (N-dimensional list)
        qual_freqs = []
        
        # Loop over all variables (qualitative first)
        for idx, var in enumerate(orderedDat.variables):
            # Store possible qualitative values, idx will be the nr of qual vars
            # print(var.name, var.qualType)
            if var.qualType==QualType.QUALITATIVE:
                # print(f"VAR {var.name} IS QUALITATIVE ")
                # print(orderedDat.data[var])
                lq = orderedDat.data[var][0].listAll()
                qual_spaces[var]=(lq)
                all_ranges.append(lq)
                # all_ranges.append(np.linspace(float(min(lq)), float(max(lq)), len(lq)))
                # Go to next var
                continue
            
            # When first quantitative variable is found:
            # fit a KDE for each qualitative combination, for all quantitative vars at once
            else:
                # Find indexes of qualitative variables
                indexes = [range(len(qual_spaces[var])) for var in qual_spaces]

                # Reorder data into frames
                frames = [[orderedDat.data[var][t] for var in orderedDat.variables]for t in range(len(orderedDat.data[orderedDat.variables[0]]))]
                
                # Find quantitative ranges and construct corresponding N-dimensional grid
                # print("OrderedDat Quant vars", [str(v) for v in orderedDat.variables[idx:]])
                ranges = [(min(orderedDat.data[v])-bw, max(orderedDat.data[v])+bw) for v in orderedDat.variables[idx:]]
                all_ranges+=ranges
                X_grid = np.meshgrid(*[np.linspace(r[0], r[1], nr_bins) for r in np.float_(ranges)], indexing='ij')
                
                # Dimensions of the (non-global) probability distribution
                dims = [nr_bins for i in range(len(ranges))]
                
                # Nested for loop emulation for N-dimensions (see itertools.product) = Cartesian product
                from itertools import product # Super convenient
                for combo in product(*indexes):
                    # start = combo[0]
                    # print("combo", combo)
                        
                    # Qualitative values of the combination of indexes
                    qual_comb = [qual_spaces[key][i] for i, key in zip(combo, qual_spaces)]
                    # print(qual_comb)
                    
                    # Keep count of this combination
                    count = 0
                    filtered = []
                    
                    # Check all frames
                    for f in frames:
                        # If qualitative value of var is different of q,
                        # then this sample f does not correspond to this combination
                        for q, var in zip(qual_comb, qual_spaces):
                            if not f[orderedDat.variables.index(var)]==q:
                                break
                        else:
                            # If no differences are found, increase count for this combination
                            count+=1
                            # And add to filtered list, to construct X_train later
                            filtered.append(f[idx:])
                    
                    # The frequency of this combination
                    freq = count/len(frames)
                    # Use the filtered values as X_train
                    X_train = filtered
                    
                    # If no values in X_train, probabilities should be 0
                    if len(X_train)==0: 
                        probs.append(np.zeros(dims))
                        continue
                    
                    # Fit KDE to estimate probabilities of quantitative variables (for this combination)
                    model = KernelDensity(kernel=kernel, bandwidth=bw, rtol=rtol, algorithm='auto')
                    model.fit(X_train)
                    
                    # Use grid to create prob_distribution
                    prob_KDE = np.exp(model.score_samples(np.stack([x.flatten() for x in X_grid],axis=-1)))*freq

                    # Append to global distribution
                    probs.append(prob_KDE.reshape(dims))
                
                # print(np.asarray(probs).shape)
                # Reshape global distribution into right dimensions
                probs = np.asarray(probs).reshape((*[len(qual_spaces[s]) for s in qual_spaces], *dims))
                break # You can also break before and write the block of code out of the for loop
        return JointDistribution(orderedDat.variables, all_ranges, probs)
                        
            
        
    def orderedVariables(self):
        """Returns list with first the qualitative, then the quantitative vars"""
        qual = []
        quant = []
        for var in self.variables:
            if var.qualType == QualType.QUALITATIVE:
                qual.append(var)
                # print("Qualitative")
            else:
                quant.append(var)
        return List(qual+quant)
    
    def ordered(self):
        dat = DataSet(*self.orderedVariables())
        dat.data = Dict({var:self.data[var] for var in dat.variables})
        return dat
            
    def plot(self, *inVars):
        import matplotlib.pyplot as plt
        
      #  plotVars = List([v for v in self.orderedVariables()[-1::-1] if str(v) in inVars]) # CHANGES ORDER!
        plotVars = List([ self.getVarWithName(v) for v in inVars])
        if len(inVars)==0:
            plotVars = self.variables
            
        nrOfVarsToPlot = len(plotVars)
        plotQual = False
        if plotVars[-1].qualType==QualType.QUALITATIVE:
            nrOfVarsToPlot -= 1
            plotQual = True
            qdata = self.data[plotVars[-1]]
         
       # print(plotVars, ' qual = ', plotQual)
          
        plot_chars = {Sign.PLUS:'+', Sign.ZERO:'o', Sign.MINUS:'_', Sign.UNKNOWN:'|'}
        if nrOfVarsToPlot==0:
            if plotQual:
                qualDat = [] 
                for s in list(Sign):
                    signDat = []
                    for i, d in enumerate(self[plotVars[-1]]):
                        if d==s:
                            signDat.append([self[d][i] for d in plotVars[:-1]])
                    qualDat.append(signDat)
            else:
                print("Ëmpty DataSet..")
        elif nrOfVarsToPlot==1:
            plt.xlabel(xlabel=plotVars[0].name)
            plt.title("Plot of " + str(plotVars))
            x = self.data[plotVars[0]]
            if plotQual:
                plt.ylabel(ylabel=plotVars[1].name)
                for sign in plot_chars:
                    sign_data = [x[i] for i in range(len(qdata)) if qdata[i] == sign]
                    plt.plot(sign_data[1:], [int(sign) for i in sign_data][1:], plot_chars[sign])
            else:
                plt.plot(x, [0 for i in x], 'o')
        elif nrOfVarsToPlot==2:
            plt.xlabel(xlabel=plotVars[0].name)
            plt.ylabel(ylabel=plotVars[1].name)
            plt.title("Plot of " + str(plotVars))
            x = self.data[plotVars[0]]
            y = self.data[plotVars[1]]
            if plotQual:
                for sign in plot_chars:
                    sign_data_x = [x[i] for i in range(len(qdata)) if qdata[i] == sign]
                    sign_data_y = [y[i] for i in range(len(qdata)) if qdata[i] == sign]
                    plt.plot(sign_data_x[1:], sign_data_y[1:], plot_chars[sign])
            else:
                plt.plot(x[1:], y[1:], 'o') # '-o'
        elif nrOfVarsToPlot==3:
            import seaborn as sns 
            sns.set(style = "darkgrid")
            ax = plt.axes(projection='3d')
            ax.set_xlabel(plotVars[0].name)
            ax.set_ylabel(plotVars[1].name)
            ax.set_zlabel(plotVars[2].name)
            plt.title("Plot of " + str(plotVars))
            x = self.data[plotVars[0]]
            y = self.data[plotVars[1]]
            z = self.data[plotVars[2]]
            if plotQual:
                for sign in plot_chars:
                    sign_data_x = [float(x[i]) for i in range(len(qdata)) if qdata[i] == sign]
                    sign_data_y = [float(y[i]) for i in range(len(qdata)) if qdata[i] == sign]
                    sign_data_z = [float(z[i]) for i in range(len(qdata)) if qdata[i] == sign]
                    ax.scatter(sign_data_x[1:], sign_data_y[1:], sign_data_z[1:], plot_chars[sign])
                plt.legend(plot_chars)
            else:
                ax.scatter(x[1:], y[1:], z[1:], 'o')
        else:
            print(f"Plot for {len(plotVars)} dimensions not possible")
        plt.show()
    def corrcoeff(self, v1: Variable | str, v2:Variable | str) -> float:
        _x = np.array(self[v1])        
        _y = np.array(self[v2])
        return np.corrcoef(_x, _y)[0, 1] # take from correlation matrix

        # https://www.geeksforgeeks.org/python-pearsons-chi-square-test/
        # https://gist.github.com/fabianp/9396204419c7b638d38f partial pearson?
        
    def corrcoeffs(self) -> np.ndarray:
        tuple_of_data = tuple ( np.array(self[v]) for v in self.variables if v.qualType == QualType.QUANTITATIVE)
        matrix_of_data = np.vstack(tuple_of_data)
        corr_matrix = np.corrcoef(matrix_of_data)
    
        import pandas as pd
        var_names = tuple( v.name for v in self.variables if v.qualType == QualType.QUANTITATIVE)
        df = pd.DataFrame(corr_matrix, columns = var_names, index=var_names)
        # pd.setOptions( display.float_format , callable)  # https://pandas.pydata.org/docs/reference/api/pandas.set_option.html#pandas.set_option
        print(df) # see https://learnpython.com/blog/print-table-in-python/
        return corr_matrix

    
    @staticmethod
    def fromDict(datDict:dict[str:list]):
        """Create DataSet from dict with data already included as list per var"""
        dat = DataSet(*datDict)
        dat.data = datDict
        return dat
        
    
if __name__ == "__main__":
    from rsQualitativeModel.QHistory import QHistory
    print("== DataSet Class Tests ==")
    TEST_FLAG = 4
    
    x = [0, 1, 2, 3, 4]
    s0 = RobotVariable("s0", VariableType.STATEVAR)
    c = Col(s0)
    
    if TEST_FLAG == 0:
        s0 = RobotVariable("s0", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        a0 = RobotVariable("a0", VariableType.ACTIONVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        dat = DataSet(s0, a0)
        dat["s0"] = [1, 2, 4, 3, 4, 4]
        dat["a0"] = [5, 3, 4, 3, 1, 2] 
        
        print("\n == Simple DataSet: == \n", dat)
        # Add qs0 as a variable to QHistory
        def forwardMath(data, t=-1):
            return Sign.sign(data[s0][t])
        qs0 = DerivedVariable("qs0", varType=VariableType.STATEVAR, qualType=QualType.QUALITATIVE, ordinalType = OrdinalType.ORDINAL, forwardMath=forwardMath)
        
        hist = QHistory(s0, a0, qs0)
        print("variables", hist.allVars)
        hist.add({s0:1, "a0":5})
        hist.add({"s0":2, "a0":3})
        hist.add({"s0":4, a0:4})
        hist.add({"s0":3, "a0":3}) 
        hist.add({s0:4, a0:1}) # Doesn't matter if string or Variable object is used
        hist.add({"s0":4, "a0":2})
        hist.plot(*hist.allVars)
        
        if False:
            dat2 = hist.toDataSet()
            
            dat2["bla"] = [Sign.MINUS, Sign.MINUS, Sign.ZERO, Sign.ZERO, Sign.PLUS, Sign.PLUS]
            dat2["qtest"] = [Sign.MINUS, Sign.ZERO, Sign.ZERO, Sign.PLUS, Sign.ZERO, Sign.PLUS]
            # dat2["qtest2"] = [Sign.MINUS, Sign.ZERO, Sign.ZERO, Sign.PLUS, Sign.ZERO, Sign.PLUS]
            
            print("\n == DataSet from History and with extra Qual vars: == ")
            print(dat2)
            
            print("\n == Ordered DataSet, with qualitative variables first == ")
            print(dat.view("s0", "qs0", "qa0", "a0").ordered())
            
            # print("\nDataSet to QHistory:")
            # hist3 = dat2.toHist(["s0"], ["a0"]) # Removed
            # dat3 = DataSet.fromHist(hist3) # Lossless?
            # print(dat3)

        
        if False:
            print("\n == JointDistribution from DataSet: == ")
            print(dat.view("s0", "a0", "qs0").variables[2].qualType)
            jointDistr = dat.view("s0", "a0", "qs0").createJointDistribution(nr_bins=10)
            print(jointDistr.variables)
            print(jointDistr.marginalise("s0", "a0").probabilities)
            
            dat.plot("a0", "s0")
            jointDistr.marginalise(s0, a0).plotProbs()
            
            jointDistr.marginalise(s0).plotProbs()
            jointDistr.marginalise(a0).plotProbs()

            
        if True:
            print("\n == View on DataSet: == ")
            viewDat = dat.view("s0", "qs0", "qds0", "da0", "s0_old")
            print(viewDat)
            
           # viewDat.plot("s0", "qds0")

           # viewDat.plot("s0", 'da0', "qds0")
            
            viewDat.plot("s0", "qs0", 'da0', "qds0")

            if False:
                print("\n == First a view, then JointDistribution: == ")
                view_vars = ["s0", "qds0"]
                dat4 = dat.view(*view_vars)
                print(dat4)
                dat4.plot(*view_vars)
                distr = dat4.createJointDistribution()
                distr.plotProbs()
                
                view_vars = ["s0", "qds0"]
                distr2 = viewDat.createJointDistribution().marginalise(*view_vars)
                print(distr.probabilities, distr2.probabilities)
                print("MSE =", np.sum((distr.probabilities-distr2.probabilities)**2)/len(distr.probabilities))
                
                print(dat.createJointDistribution().marginalise(*view_vars))
        
        # print("Q operator", dat4.Q("s0"))
        # print(dat4.view("qs0"))
        
        # dat2.view("qs0", "s0").plot()
        
        # print("d operator", dat4.d("s0"))
        
        # print("Qd operator", dat4.Qd("s0"))
        
        # print("old operator", dat4.old("s0"))
        
        # dat4.addDictToData(dat4.old("s0"))
        # print(dat4.data)
        
    elif TEST_FLAG == 1:
        print("Tests flow: QHistory->DataSet->JointDistribution")
        s0 = RobotVariable("s0", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        s1 = RobotVariable("s1", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        a0 = RobotVariable("a0", VariableType.ACTIONVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        
        hist = QHistory(s0, s1, a0)
        hist.add({"s0":1, "s1":20, "a0":5})
        hist.add({"s0":2, "s1":10, "a0":3})
        hist.add({"s0":4, "s1":15, "a0":4})
        hist.add({"s0":3, "s1":12, "a0":3})
        hist.add({"s0":4, "s1":-4, "a0":1})
        hist.add({"s0":4, "s1":5, "a0":2})
        
        print("\nCreated History:", hist)
        print(hist.stateActions)
        hist.plot(*hist.allVars)
        
        # dat = DataSet.fromHist(hist, oldVars=True)
      #  dat = hist.toDataSet(oldVars=True)
      #  print("\nCreated DataSet from QHistory:", dat)
        
        dat_view = hist.view("qs0", "a0", "ds0")
        print("\nCreated view from DataSet:", dat_view)
        
        distr = dat_view.createJointDistribution()
        print("\nCreated JointProbDist from view:", distr)
        
        distr.marginalise("qs0", "a0").plotProbs()
        
        print("\nMI(a0;s0) =", distr.mutualInformation("ds0", "s0"))
    
    elif TEST_FLAG == 2:
        print("Tests with Variable class in DataSet")
        s0 = RobotVariable("s0", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        s1 = RobotVariable("s1", VariableType.STATEVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        a0 = RobotVariable("a0", VariableType.ACTIONVAR, QualType.QUANTITATIVE, OrdinalType.ORDINAL)
        qs0 = RobotVariable("qs0", VariableType.STATEVAR, QualType.QUALITATIVE, OrdinalType.ORDINAL)
        
        dat = DataSet(s0, s1, a0, qs0)
        
        print("\nCreated DataSet:\n", dat)
        dat[s0] = [1,2,4,3,4,4]
        dat[s1] = [20, 10, 15, 12, -4, 5]
        dat[a0] = [5,3,4,3,1,2]
        dat[qs0] = dat.Q("s0")
        dat["ds0"] = dat.d(s0)
        
        print("dat[a0] =", dat[a0])
        print("dat[\"ds0\"] =", dat[dat.getVarWithName("ds0")]) # dat["ds0"] but via method
        
        print("dat.Qd(\"s0\") =", dat.Qd("s0"))
        print(f"{dat.old(s0) = }")
        
        dat_view = dat.view("s0", "a0", "s1")
        print("\nCreated view from DataSet:", dat_view)
        
        dat_view.plot("s0")
        
        distr = dat_view.createJointDistribution()
        print("\nCreated JointProbDist from view:", distr)        
        
        # distr.plotProbs()
        
        dat_view = dat.view("s0", "a0", "s1")
        print("\nCreated view from DataSet:", dat_view)
        
        dat_view.plot()
    
