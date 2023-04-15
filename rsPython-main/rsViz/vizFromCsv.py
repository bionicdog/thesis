import pandas as pd
import matplotlib.pylab as plt

# create dataframes
frame = pd.read_csv('results.csv')
robotFrame = frame[['t', 'ex', 'ev', 'ea', 'sX', 'sV', 'sA', 'tch', 'mX']]
groundTruthFrame = frame[['t', 'x', 'v', 'a']]

# create dicts from dataframes
rd = robotFrame.to_dict()
gtd = groundTruthFrame.to_dict()

valuesDict = {}
for var, val in rd.items():
    valuesDict[var] = list(rd[var].values())
groundTruthDict = {}
for var, val in gtd.items():
    groundTruthDict[var] = list(gtd[var].values())


# visualize data
fig, axs = plt.subplots(nrows=3, ncols=3)
i = 1
variables = list(valuesDict.keys())
states_variables = list(groundTruthDict.keys())
for axi in axs:
    for axj in axi:
        if i<len(variables):
            axj.grid()
            axj.set_title(variables[i])
            
            #axj.plot(groundTruthDict['t'], groundTruthDict[states_variables[i]], label='Truth')
            #axj.plot(noCorruptionDict['t'], noCorruptionDict[variables[i]], label='Base')
            #axj.plot(valuesDict['t'], valuesDict[variables[i]], label='Robot Model')
            axj.plot(robotFrame['t'], robotFrame[variables[i]], label='Robot Model')
            
            #ticks = axj.set_xticks(valuesDict['t'])
            axj.legend()
            i+=1
i = 1
for axi in axs:
    for axj in axi:
        if i<len(states_variables):
            axj.plot(groundTruthFrame['t'], groundTruthFrame[states_variables[i]], label='Truth')
            axj.legend()
            i+=1


#frame.plot(x= 't',subplots=True, layout=(4, 4), figsize=(10,6), sharex=False)