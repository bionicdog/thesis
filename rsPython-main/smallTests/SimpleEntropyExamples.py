# -*- coding: utf-8 -*-
"""
Created on Tue Jan 24 10:33:25 2023

@author: Nick
"""

from rsLibrary.StatisticsUtils import entropyBase2, conditionalEntropy, mutualInformation

if __name__ == "__main__":
    print("Entropy example:")
    # There are 3 types of letters in a bowl: A, B and C
    # The bowl contains 50 letters in total: 10 letters A, 25 letters B and 15 letters C
    # The variable that represents the letter type is called T
    probs_T = [10/50, 25/50, 15/50]
    print(f"""
          Probability of A: {probs_T[0]}, 
          probability of B: {probs_T[1]}, 
          probability of C: {probs_T[2]}
          """)
    # The entropy of T is an uncertainty measure: H(T)
    ent_T = entropyBase2(probs_T)
    print("Entropy:", ent_T)
    # Each letter is either red or blue
    # There are 30 blue letters, and 20 red letters
    # The variable that represents letter color is called X
    probs_X = [30/50, 20/50]
    print(f"""
          Probability of blue: {probs_X[0]}, 
          probability of red: {probs_X[1]}, 
          """)
    # 5 blue letters A
    p_A_blue = 5/30
    # 24 blue letters B
    p_B_blue = 24/30
    # 1 blue letter C
    p_C_blue = 1/30
    # 5 red letters A
    p_A_red = 5/20
    # 1 red letter B
    p_B_red = 1/20
    # 14 red letters C
    p_C_red = 14/20
    
    probs_T_given_X = [[p_A_blue, p_B_blue, p_C_blue], [p_A_red, p_B_red, p_C_red]]
    # If we know the color, how much uncertainty remains over the type of letter?
    # => Conditional entropy H(T|X)
    print("Conditional entropy after conditioning on color:",
          conditionalEntropy(probs_T_given_X, probs_X))
    # How much is the uncertainty over the letters reduced after color is known?
    # Mutual information
    print("Mutual information:", mutualInformation(probs_T, probs_T_given_X, probs_X))
    
    
    