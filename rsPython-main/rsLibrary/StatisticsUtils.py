# -*- coding: utf-8 -*-
"""
Created on Mon Jan 23 16:39:00 2023

@author: Nick
"""

import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import KernelDensity
from numbers import Number
from scipy.stats import entropy

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code
from rsLibrary.Variable import Variable, VariableType, QualType, OrdinalType
from rsLibrary.DataUtils import List, Dict

def generateRandData(nr_samples, N_dim=4, seed=2023):
    # Fix the seed to reproduce the results
    np.random.seed(seed)
    x = np.random.uniform(0, 100, size=(nr_samples, N_dim))
    return x

def plotKDE2D(KDE, targetVar, minVals=[0, 0], maxVals=[100, 100], nbins=30):
    """
    Plot a 2D colorgrid from minVals to maxVals for targetVar.
    """
    if KDE.n_features_in_ != 2:
        print("Plot only works for 2-dimensional data!")
    else:
        # Generate grid
        X = np.mgrid[minVals[0]:maxVals[0]:nbins*1j, minVals[1]:maxVals[1]:nbins*1j]
        # Evaluate KDE over grid
        probs = np.exp(KDE.score_samples(np.stack((X[0].flatten(),X[1].flatten()),axis=-1)))
        # Plot
        plt.title(f"Class density plot 2D")
        plt.xlabel("var1")
        plt.ylabel("var2")
        # Plot colormesh
        plt.pcolormesh(X[0], X[1], probs.reshape(X[0].shape), shading='auto')
        # Add color bar
        plt.colorbar()
        plt.show()    
        
def plotEntropy1D(probs, minVal = 0, maxVal = 10):
    ents = [entropyBase2(p) for p in np.transpose(probs)]
    X = np.linspace(minVal, maxVal, len(ents))
    plt.title("Entropy plot 1D")
    plt.xlabel("var")
    plt.ylabel("H")
    plt.plot(X, ents)

def plotEntropy2D(probs, minVals = [0, 0], maxVals = [100, 100]):
    """
    Plot an entropy colorgrid. 
    """
    nbins = probs.shape[1:]
    print(nbins)
    # Generate grid
    X = np.mgrid[minVals[0]:maxVals[0]:nbins[0]*1j, minVals[1]:maxVals[1]:nbins[1]*1j]
    # Evaluate KDE over grid
    norm_probs = probs/np.sum(probs, axis=0)
    # entropy = -np.sum(norm_probs*np.log(norm_probs), axis=0).reshape(X[0].shape)
    entropy = entropyBase2(norm_probs, axis=0).reshape(X[0].shape)
    # Make plot
    plt.title("Entropy plot 2D")
    plt.xlabel("var1")
    plt.ylabel("var2")
    # Plot entropy colormesh
    plt.pcolormesh(X[0], X[1], entropy, shading='auto')
    # Add color bar
    plt.colorbar()
    plt.show()
    
def entropyBase2(probs, axis=None):
    # if np.sum(probs)!=1:
    #     probs/=np.sum(probs)
    # return -np.sum(probs*np.log2(probs))
    return entropy(probs, base=2, axis=axis)

class ProbabilityDistribution():
    def __init__(self, variables:list[Variable], ranges:list[list], probs:list[list[float]]):
        self.variables = List(variables) # First variable should be target
        self.ranges = ranges # Each range corresponds to one variable
        self.probabilities = np.asarray(probs)
        
class ConditionalDistribution(ProbabilityDistribution):
    def __init__(self, variables:list[Variable], conditioned_vars:list[str], ranges:list[list[float]], probs:list[list[float]]):
        super().__init__(variables, ranges, probs)
        self.conditioned_vars = conditioned_vars
        
    def __str__(self):
        s = "P("
        s += ','.join( [v.name for v in self.variables])
        #for var in self.variables:
        #    s+=str(var)+', '
        if len(self.conditioned_vars) > 0:
            #s = s[:-2]+'| '
            s += '|' + ','.join( [v.name for v in self.conditioned_vars])
        #for var in self.conditioned_vars:
         #   s+=str(var)+', '
        s += ')'
        return s
        
class JointDistribution(ProbabilityDistribution):
    def __init__(self, variables:list[Variable], ranges:list[list[float]], probs:list[list[float]]):
        super().__init__(variables, ranges, probs)
        self.probabilities /= np.sum(self.probabilities)
        if len(variables) != len(self.probabilities.shape):
            print(variables, self.probabilities.shape)
            print("Something wrong with dimensions")
    
    def __str__(self):
    #    return f"P{tuple([str(var) for var in self.variables])}"  # NOK
        return "P(" + ','.join( [str(v) for v in self.variables]) + ')'
    
    def entropy(self):
        """Calculates the entropy of the discrete joint distribution"""
        return entropyBase2(self.probabilities)
    
    def conditionalEntropy(self, *cond_vars:str):
        """H(X|Y)=H(X,Y) - H(Y)"""
        return self.entropy()-self.marginalise(cond_vars).entropy()
    
    def mutualInformation(self, var1, var2=None):
        """
        Calculates the mutual information between 2 variables in the distribution
        First sum both marginal entropies, then substract joint entropy
        """
        # If no var2 is specified, use the target variable (the first in the list)
        if var2 == None:
            var2 = self.variables[0]
        # Entropy of first var
        P_var1 = self.marginalise(var1)
        H_var1 = P_var1.entropy()
        # Entropy of second var
        P_var2 = self.marginalise(var2)
        H_var2 = P_var2.entropy()
        # Joint distr of var1 and var2
        P_var1_var2 = self.marginalise(var1, var2)
        H_var1_var2 = P_var1_var2.entropy()
        return H_var1 + H_var2 - H_var1_var2 
    
    def conditionalMutualInformation(self, var1, var2, *condVars):
        """Calculates the conditional mutual information between var1 and
        var2, after conditioning on condVar"""
        # print("condVars", *condVars)
        H_var1_condVar = self.marginalise(var1, *condVars).entropy()
        H_var2_condVar = self.marginalise(var2, *condVars).entropy()
        H_var1_var2_condVar = self.marginalise(var1, var2, *condVars).entropy()
        H_condVar = self.marginalise(*condVars).entropy()
        return H_var1_condVar + H_var2_condVar - H_var1_var2_condVar - H_condVar
    
    def mutualInformationChain(self, var1, *otherVars):
        """https://cs.stackexchange.com/questions/151542/chain-rule-for-mutual-information"""
        # Not tested
        varList = []
        MI = 0
        for var in otherVars:
            MI += self.conditionalMutualInformation(var1, var, *varList)
        return MI
    
    def sumOverVar(self, var, keepdims=False):
        """Sum over var, return a new JointDistribution"""
        var_idx = self.variables.index(var)
        # Var will not be present in new distribution: remove from vars and ranges
        summed_probs = np.sum(self.probabilities, axis=var_idx, keepdims=keepdims)
        new_ranges = self.ranges.copy()
        new_ranges.remove(self.ranges[var_idx])
        new_vars = self.variables.copy()
        new_vars.remove(var)
        # Return new joint distribution with summed probs and remaining vars
        return JointDistribution(new_vars, new_ranges, summed_probs)
    
    def marginalise(self, *mar_vars, keepdims=False):
        """Return the marginal distribution for var, by summing over all other vars"""
        # print(mar_vars, [str(v) for v in self.variables])
        other_vars = [other_var for other_var in self.variables if other_var not in mar_vars]
        # print([str(v) for v in other_vars])
        distr = self
        for v in other_vars:
            distr = distr.sumOverVar(v, keepdims=keepdims)
        return distr
            
    def conditionOnVars(self, *cond_vars):
        """Create conditional distribution after conditioning on all variables
        in cond_vars. This returns a conditional probability distribution for
        alls vars that are in self.variables, but not in cond_vars, conditioned
        on cond_vars"""
        other_var_idxs = tuple([self.variables.index(var) \
                                for var in self.variables if var not in cond_vars])
        print("var_idxs", other_var_idxs)
        # Marginalise over other vars, keep dimensions for division
        marg_probs_same_dims = np.sum(self.probabilities, axis=other_var_idxs, keepdims=True)
        # P(X|Y) = P(X,Y)/P(X)
        probs = self.probabilities/marg_probs_same_dims
        new_vars = list(set(self.variables)-set(cond_vars))
        print([str(v) for v in new_vars])
        print([str(v) for v in cond_vars])
        new_ranges = [self.ranges[self.variables.index(var)] for var in new_vars]\
            + [self.ranges[self.variables.index(var)] for var in cond_vars]
        return ConditionalDistribution(new_vars, set(cond_vars), new_ranges, probs)
        
    def plotProbs(self):
        # print(len(self.variables))
       # print(len(self.variables))
        plt.title(str(self))
        if len(self.variables)==1:
            X = np.linspace(self.ranges[0][0], self.ranges[0][1], self.probabilities.shape[0])
            plt.xlabel(f"{self.variables[0]}")
            plt.plot(X, self.probabilities)
            plt.show()
        elif len(self.variables)==2:
            # print("Shape, probs:", self.probabilities.shape)
            X = np.meshgrid(*[np.linspace(float(r[0]), float(r[-1]), self.probabilities.shape[idx]) for idx, r in enumerate(self.ranges)], indexing='ij')
            plt.xlabel(f"{self.variables[0]}")
            plt.ylabel(f"{self.variables[1]}")
            plt.pcolormesh(X[0], X[1], self.probabilities)
            plt.colorbar()
            plt.show()
        # Only works if first variable is discrete 
        # print(len(self.variables))
        # if len(self.variables)==1:
        #     # Only target variable
        #     pass
        # if len(self.variables[1:])==1:
        #     # One other variable
        #     for idx, prob in enumerate(self.probabilities):
        #         X = np.linspace(self.ranges[1][0], self.ranges[1][1], self.probabilities.shape[1])
        #         print(X[0], prob)
        #         plt.title(f"Probability distribution: var {self.variables[0]} = {self.ranges[0][idx]}")
        #         plt.xlabel(f"{self.variables[1]}")
        #         plt.plot(X, prob)
        #         plt.show()
        # elif len(self.variables[1:])==2:
        #     # Two other variables
        #     for idx, prob in enumerate(self.probabilities):
        #         print(self.ranges)
        #         X = np.meshgrid(*[np.linspace(r[0], r[1], self.probabilities.shape[2]) for r in self.ranges[1:]], indexing='ij')
        #         plt.title(f"Probability distribution: var {self.variables[0]} = {self.ranges[0][idx]}")
        #         plt.xlabel(f"{self.variables[1]}")
        #         plt.ylabel(f"{self.variables[2]}")
        #         plt.pcolormesh(X[0], X[1], prob)
        #         plt.colorbar()
        #         plt.show()
            
    def plotEntropy(self):
        print(self.variables[1:])
        if len(self.variables[1:])==1:
            plotEntropy1D(self.probabilities, self.ranges[1][0], self.ranges[1][1])
        elif len(self.variables[1:])==2:
            print("shape", self.probabilities.shape)
            plotEntropy2D(self.probabilities)
    ###
    @staticmethod
    def createJointDistrFromData(variables, data, ranges, bw=1.0, nr_bins=10, kernel='gaussian', rtol=0.25):
        import time
        probs = []
        start = time.time()
        # TODO: generalise
        # If first var is discrete/class var, then one KDE per class
        if len(ranges)>0 and type(ranges[0][0])!=Number:
            X_grid = np.meshgrid(*[np.linspace(r[0], r[1], nr_bins) for r in np.float_(ranges[1:])], indexing='ij')
            # Distinct classes
            t_classes = ranges[0]
            # Fit a KDE for each class, and create probability grid
            for t in t_classes:
                # print("class", t)
                # print("data", data)
                count = len([d for d in data if d[0]==t])
                freq = count/len(data)
                # KDE on X_train
                X_train = [d[1:] for d in data if d[0]==t]
                # print(X_train)
                # If no data for this class, don't fit a KDE
                if len(X_train) == 0:
                    continue # TODO: Should be changed to something that has no influence on shape of probabilities
                model = KernelDensity(kernel=kernel, bandwidth=bw, rtol=rtol, algorithm='auto')
                model.fit(X_train)
                # Convert scores to probs and reshape
                prob = np.exp(model.score_samples(np.stack([x.flatten() for x in X_grid],axis=-1)))*freq
                dims = [nr_bins for i in range(len(ranges[1:]))]
                probs.append(prob.reshape(dims)) # TODO check for normalization here
        # Else: One KDE for all vars, since target var is continuous
        else:
            pass
        stop = time.time()
        print("Time to create JointDistribution", stop-start)
        return JointDistribution(variables, ranges, probs)
            
    
if __name__ == "__main__":
    print("Test statistic utils")
    
    if False:
        print(" == Test np.meshgrid ==")
        ranges = [[0, 10], [0, 100], [0, 1]]
        a = np.asarray(np.meshgrid(*[np.linspace(r[0], r[1], 10) for r in ranges]))
        
        import matplotlib.pyplot as plt
        ax = plt.axes(projection='3d')
        ax.scatter(a[0], a[1], a[2], 'o')
        plt.title(f"3D grid {ranges}")
        plt.show()
    
    if True:
        print(" == creation of JointDistr ==")
        data = [
            ["Red", 11, 2], ["Red", 10, 3], ["Blue", 14, 3], ["Blue", 13, 2], 
            ["Red", 10, 4], ["Red", 11, 1], ["Blue", 14, 1], ["Blue", 13, 0]
            ]
        variables = ["c", "x0", "x1"]
        ranges = [["Red", "Blue"], [10, 15], [0, 5]]
        distr = JointDistribution.createJointDistrFromData(variables, data, ranges, nr_bins=10)
        print(distr)
        print("index of x0:", distr.variables.index("x0"))
        distr.plotProbs()
    
        if True:
            print(" == Test JointDistr methods ==")
            distr.sumOverVar("x1")
            
            print("Marginal distribution of c:", distr.marginalise("c").probabilities)
            print(distr.marginalise("c").variables)
            distr.marginalise("x1", "x0").plotProbs()
            
            print("H(c, x0, x1):", distr.entropy())
            
            print("MI(x0; c):", distr.mutualInformation("x0", "c"))
            print("H(x0, x1):", distr.sumOverVar("c").entropy())
            print("H(c, x0):", distr.marginalise("c", "x0").entropy())
            
            # Test conditioning
            variables = ["Color", "y", "x"]
            ranges = [["Red", "Blue"], ["0", "1", "2"], ["0", "1"]]
            joint_probs = [
                [ # Red
                    [4/25, 1/25],
                    [2/25, 2/25], 
                    [1/25, 2/25]
                    ], 
                [ # Blue
                    [8/25, 1/25],
                    [2/25, 1/25], 
                    [0/25, 1/25]]
                           ]
            distr = JointDistribution(variables, ranges, joint_probs)
            print(distr.probabilities)
            print("\n\n")
            print(distr.conditionOnVars("Color").probabilities) # Works for 2-3 vars, didn't test for higher nr
            distr.plotEntropy()
    
    if False:
        # Test Variable
        print("\n == Test with Variable type ==")
        from enum import Enum
        
        class Color(Enum):
            BLUE = 1
            RED = 2
        
        c = Variable("c", VariableType.STATEVAR, QualType.QUALITATIVE, OrdinalType.NON_ORDINAL, Color)
        x1 = Variable("x1", VariableType.STATEVAR, QualType.QUALITATIVE, OrdinalType.ORDINAL, minVal = 5, maxVal = 15)
        x2 = Variable("x2", VariableType.STATEVAR, QualType.QUALITATIVE, OrdinalType.ORDINAL)
    
        variables = [c, x1, x2]
        
        ranges = [list(Color), ["0", "1", "2"], ["0", "1"]]
        
        joint_probs = [
            [ # Red
                [4/25, 1/25],
                [2/25, 2/25], 
                [1/25, 2/25]
                ], 
            [ # Blue
                [8/25, 1/25],
                [2/25, 1/25], 
                [0/25, 1/25]]
                       ]
        
        distr = JointDistribution([c, x1, x2], ranges, joint_probs)
    
        print("variables:", distr.variables)
        print("ranges:", distr.ranges)
        print("probabilities:\n", distr.probabilities)
        distr.plotEntropy()
        print("conditioned:", distr.conditionOnVars(c).probabilities)
        print("marginalised:", distr.marginalise(x1, x2).probabilities)
        print("MI(x1;x2) =", distr.mutualInformation(x1, x2))
    
    
    