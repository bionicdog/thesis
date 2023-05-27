# -*- coding: utf-8 -*-
"""
Created on Mon Jan 16 15:38:48 2023

@author: Nick
"""

# https://stackabuse.com/kernel-density-estimation-in-python-using-scikit-learn/

import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import KernelDensity
from sklearn.model_selection import GridSearchCV
# from scipy.stats import entropy
import matplotlib.pyplot as plt
import seaborn as sns
from rsLibrary.StatisticsUtils import plotKDE2D, plotEntropy2D, generateRandData, entropyBase2, JointDistribution
entropy = entropyBase2


def generative_model_4D(X):
    """
    Returns class label y from input vector (size 4) X
    Independent from first variable x[0]
    """
    def generate(X):
        if X[1]>50:
            return 3
        elif X[2]>X[3]:
            return 2
        else:
            return 1
    if type(X[0])==int:
        return generate(X)
    else:
        return [generate(entry) for entry in X]
    

def generative_model_2D(X):
    """
    Returns class label y from input vector (size 4) X
    Independent from first variable x[0]
    """
    def generate(X):
        if X[0]>75:
            return 3
        elif X[1]>2*X[0]:
            return 2
        else:
            return 1
    if type(X[0])==int:
        return generate(X)
    else:
        return [generate(entry) for entry in X]
        
# def generate_rand_data(nr_samples, N_dim=4, seed=2023):
#     # Fix the seed to reproduce the results
#     np.random.seed(seed)
#     x = np.random.uniform(0, 100, size=(nr_samples, N_dim))
#     return x

def entropy_new(probs):   
    return -sum([prob*np.log(prob) for prob in probs])

class QKernelDensity(KernelDensity):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
    
if __name__ == "__main__":
    print(entropy([0.33, 0.33, 0.33]))
    print(entropy_new([0.33, 0.33, 0.33]))
    # print(generative_model([70, 60, 50, 40]))
    # plotProbability(['A', 'B', 'C'], [0.33, 0.33, 0.33])
    
    
    # Generate raw data and corresponding true classes from generative model
    N_dim=2
    X = generateRandData(10000, N_dim=N_dim)
    y = generative_model_2D(X)
    
    plot_chars = {1:'r.', 2:'b.', 3:'g.', 4:'y.'}
    if N_dim==2:
        plt.title("2D plot of the data (3 classes)")
        for label in [1,2,3,4]:
            data = [point for point, l in zip(X, y)\
                    if l==label]
            a = [p[0] for p in data]
            b = [p[1] for p in data]
            plt.plot(a, b, plot_chars[label])
        plt.show()
        
        models = {} # Dict for the models
        for i in range(1, 4):
            # KDE for one class
            models[i]=KernelDensity(kernel='gaussian', bandwidth=5.0)
            X_train_1 = [x for x, l in zip(X, y) if l==i]
            models[i].fit(X_train_1)     
            
        for i in range(1, 4):
            # Plot KDE for each class separately
            plotKDE2D(models[i], i)
            
        nbins = 30
        # Construct full grid
        X_grid = np.mgrid[X[:, 0].min():X[:, 0].max():nbins*1j, X[:, 1].min():X[:, 1].max():nbins*1j]
        
        # Probabilities for each class, given X_0 and X_1
        probs = [np.exp(models[i].score_samples(np.stack((X_grid[0].flatten(),X_grid[1].flatten()),axis=-1))) for i in range(1, 4)]
        probs = [np.exp(models[i].score_samples(np.stack([x.flatten() for x in X_grid],axis=-1))) for i in range(1, 4)]
        probs = np.asarray(probs).reshape([3, nbins, nbins])
        
        # Plot entropy for class variable
        plotEntropy2D(probs)
        
        # Test conditioning on one single value
        condition_val = 20
        X_0 = np.arange(0, 100, 0.1)
        X_1 = np.full(X_0.shape, condition_val)
        condProb = np.exp(models[1].score_samples(np.stack((X_0, X_1),axis=-1)))
        plt.title(f"P(X0|T=1, X1={condition_val})")
        plt.plot(X_0, condProb)
        plt.show() # P(X0|T=1, X1=20) ?
        
        # Probability for each class: P(T=t)
        print(len(y))
        probs_T = []
        for i in range(1, 4):
            count = len([j for j in y if i==j])
            freq = count/len(y)
            probs_T.append(freq)
        print("probs_T:", probs_T)
        H_T = entropyBase2(probs_T)
        print("P(T)", probs_T)
        print("H(T) =", H_T)
        
        # nr_per_class = [len([i for i in y if i==j]) for j in range(1, 4)]
        # print("nr per class", sum(nr_per_class))
        
        X_0 = np.linspace(0, 100, 30)
        X_1 = np.linspace(0, 100, 30)
        # Joint probability of T, X0, X1
        probs_T_X0_X1 = np.asarray([probs[i].reshape((30, 30)) for i in range(3)])
        print("shape:", probs_T_X0_X1.shape)
        print("probs:", probs_T_X0_X1)
        # Marginalise over X1
        probs_T_X0 = np.sum(probs_T_X0_X1, axis=2)
        print(probs_T_X0.shape)
        for i in range(3):
            plt.xlabel('X_0')
            plt.title(f"P(X0|T={i+1})")
            p = probs_T_X0[i]/np.sum(probs_T_X0[i])
            plt.plot(X_0, p)
            plt.show()
            print(f"sum{i}", np.sum(p))
            
        # Marginalise over X0
        probs_T_X1 = np.sum(probs_T_X0_X1, axis=1)
        for i in range(3):
            plt.xlabel('X_1')
            plt.title(f"P(X1|T={i+1})")
            p = probs_T_X1[i]/np.sum(probs_T_X1[i])
            plt.plot(X_1, p)
            plt.show()
            
        # Marginalise over X1 and X0
        probs_T = np.sum(probs_T_X0, axis=1)
        print("probs_T:", probs_T)
            
        # Marginalise over T and X1
        probs_X0_T = np.sum(probs_T_X0_X1, axis=2)
        probs_X0_T /= np.sum(probs_X0_T, axis=0)
        probs_X0 = np.sum(probs_X0_T, axis=0) # np.sum(probs_T_X0_X1, axis=2)
        probs_X0 /= np.sum(probs_X0)
        plt.title("P(X0)")
        plt.plot(X_0, probs_X0) # Should be uniform
        plt.show()
        
        # Marginalise over T and X0
        probs_X1_T = np.sum(probs_T_X0_X1, axis=1)
        probs_X1_T /= np.sum(probs_X1_T, axis=0)
        probs_X1 = np.sum(probs_X1_T, axis=0)
        probs_X1 /= np.sum(probs_X1)
        plt.title("P(X1)")
        plt.plot(X_1, probs_X1) # Should be uniform
        plt.show()
        
        # Probabilities at a random point in X0-X1 state space
        # idx_0 = 25
        # idx_1 = 10
        # X_0_sample = round(X_0[idx_0], 2)
        # X_1_sample = round(X_1[idx_1], 2)
        # print(f"P(T|X0={X_0_sample}, X1={X_1_sample}):", probs_T_X0_X1[:, idx_0, idx_1])
        # print(f"H(T|X0={X_0_sample}, X1={X_1_sample}):", entropy(probs_T_X0_X1[:, idx_0, idx_1]))
            
        # print(entropy([for x in ]))
        # Conditional entropy of T given X0 H(T|X0): what uncertainty remains, after X0 is known?
        
        # print(probs_T_X0)
        prob_T_given_X_0 = []
        print(prob_T_given_X_0)
        for idx_0 in range(len(X_0)):
            X_0_sample = X_0[idx_0]
            p = probs_T_X0[:, idx_0]
            p /= sum(p)
            prob_T_given_X_0.append(p)
            print(f"P(T|X0={X_0_sample}):", probs_T_X0[:, idx_0])
        prob_T_given_X_0=np.asarray(prob_T_given_X_0)
        
        # print(probs_X0[5], prob_T_given_X_0[5])
        
        H_test = [entropy(s) for s in prob_T_given_X_0]
        plt.title("H(T|X0=x)")
        plt.xlabel("X0")
        plt.ylabel("H")
        plt.plot(X_0, H_test)
        plt.show()
        
        H_T_given_X0 = sum([probs_X0[idx]*entropy(prob_T_given_X_0[idx]) for idx, x in enumerate(X_0)])
        print("H(T|X0)", H_T_given_X0)
        
        #Mutual information between T and X0
        MI_T_X0 = H_T - H_T_given_X0
        print(f"I(T;X0) = {MI_T_X0}")
        
        # Conditional entropy of T given X1 H(T|X1): what uncertainty remains, after X1 is known?
        
        # print(probs_T_X1)
        prob_T_given_X_1 = []
        print(prob_T_given_X_1)
        for idx_1 in range(len(X_1)):
            X_1_sample = X_1[idx_1]
            p = probs_T_X1[:, idx_1]
            p /= sum(p)
            prob_T_given_X_1.append(p)
            print(f"P(T|X1={X_1_sample}):", probs_T_X1[:, idx_1])
        prob_T_given_X_1=np.asarray(prob_T_given_X_1)
        
        # print(probs_X1[5], prob_T_given_X_1[5])
        
        H_test = [entropy(s) for s in prob_T_given_X_1]
        plt.title("H(T|X1=x)")
        plt.xlabel("H")
        plt.ylabel("X1")
        plt.plot(H_test, X_1)
        plt.show()
        
        H_T_given_X1 = sum([probs_X1[idx]*entropy(prob_T_given_X_1[idx]) for idx, x in enumerate(X_1)])
        print("H(T|X1) =", H_T_given_X1)
        
        # Mutual information between T and X0
        MI_T_X1 = H_T - H_T_given_X1
        print(f"I(T;X1) = {MI_T_X1}")
        
        # H(T|X0, X1) = 
        
        # Joint entropy H(T, X0, X1)
        print(probs_T_X0_X1.shape)
        print((probs_T_X0_X1*np.log2(probs_T_X0_X1)).shape)
        # probs_T_X0_X1 = probs_T_X0_X1/np.sum(probs_T_X0_X1, axis=0) # Normalise to sum=1 for each T (should this be done at concatenation of KDEs?)
        norm_probs_T_X0_X1 = probs_T_X0_X1/np.sum(probs_T_X0_X1)
        # H_T_X0_X1 = -np.sum(np.sum(np.sum(norm_probs_T_X0_X1*np.log2(norm_probs_T_X0_X1), axis=2), axis=1), axis=0)
        H_T_X0_X1 = 0
        # Order of sums doesn't matter
        for idx_1 in range(len(X_1)):
            sum_0 = 0
            for idx_0 in range(len(X_0)):
                sum_1 = 0
                for idx_t in range(3):
                    sum_1 += norm_probs_T_X0_X1[idx_t, idx_0, idx_1]*np.log2(norm_probs_T_X0_X1[idx_t, idx_0, idx_1])
                sum_0 += sum_1
            H_T_X0_X1 += -sum_0
        print("H(T) =", H_T, "H(X0) =", entropy(probs_X0), "H(X1) =", entropy(probs_X1))
        print("H(T, X0, X1) =", H_T_X0_X1) 
        print("Verify =", -np.sum(np.sum(np.sum(norm_probs_T_X0_X1*np.log2(norm_probs_T_X0_X1), axis=2), axis=1), axis=0))
        # H(T, X0, X1) should be higher than max(H(T), H(X0), H(X1)) and less than sum(H(T), H(X0), H(X1))
        
        # Relationships between Joint and Conditional entropies
        norm_probs_T_X1 = np.sum(norm_probs_T_X0_X1, axis=1)
        H_T_X1 = -np.sum(np.sum(norm_probs_T_X1*np.log2(norm_probs_T_X1), axis=1), axis=0)
        print(H_T_X1)
        H_X1 = entropy(probs_X1)
        
        H_T_given_X1_verif = H_T_X1 - H_X1
        print("H(T|X1) from joint entropy:", H_T_given_X1_verif, "Direct conditional entropy H(T|X1):", H_T_given_X1) # Works!
        
        # With JointDistribution from StatisticUtils
        print("\nVerify if JointDistribution class gives same results:")
        dat = [list([str(yi)])+list(xi) for xi, yi in zip(X, y)]
        distr = JointDistribution.createJointDistrFromData(
            ["T", "X0", "X1"], dat, [["1", "2", "3"], [0, 100], [0, 100]],
            nr_bins=30, bw=5)
        distr.plotProbs()
        
        print("H(T, X0, X1) =", distr.entropy())
        print("H(T) =", round(distr.marginalise("T").entropy(), 2), end='  ')
        print("H(X0) =", round(distr.marginalise("X0").entropy(), 2), end='  ')
        print("H(X1) =", round(distr.marginalise("X1").entropy(), 2))
        print("I(T;X0) =", round(distr.mutualInformation("T", "X0"), 2))
        print("I(T;X1) =", round(distr.mutualInformation("T", "X1"), 2))
        
        
    else:
        # Test to show 4D data (in 3D)
        sns.set(style = "darkgrid")
        ax = plt.axes(projection='3d')
        ax.set_xlabel("var1")
        ax.set_ylabel("var2")
        ax.set_zlabel("var3")
        plt.title("3D plot")
        for label in [1, 2, 3, 4]:
            data = [point for point, l in zip(X, y)\
                    if l==label]
            a = [p[1] for p in data]
            b = [p[2] for p in data]
            c = [p[3] for p in data]
            ax.scatter(a, b, c, plot_chars[label])
        plt.show()
    
    
    
    