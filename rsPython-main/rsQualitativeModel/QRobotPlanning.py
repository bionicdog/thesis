# -*- coding: utf-8 -*-
"""
Created on Thu Mar 23 15:12:26 2023

@author: Marco
"""

import sys
if '..' not in sys.path:
    sys.path.append('..') # rsPython folder (specify all our libraries relative to that folder) # repository of Robotic Sensing Lab's Python code

from pgmpy.base import DAG

"""
pgm_params = shape=None, origin=None, grid_unit=2.0, node_unit=1.0, observed_style='shaded', alternate_style='inner',
line_width=1.0, node_ec='k', node_fc='w', plate_fc='w', directed=True, aspect=1.0, label_params=None, dpi=None

node_pos={'a': (0, 0), 'b': (1, 0), 'c': (2, 0), 'd': (1, 1)}

edge_params={('a', 'b'): {'label': 2}}
node_params={'a': {'shape': 'rectangle'}}
"""
"""
categories:
    main
    previous
    delta
"""
def getMainNodesInOrder(dag):
    mainEdgeArr = []
    for edge in dag.edges:
        if 'd' not in edge[0] and '-1' not in edge[0] and 'd' not in edge[1] and '-1' not in edge[1]:
            mainEdgeArr.append(edge)
    mainDAG = DAG(mainEdgeArr)
    
    mainRoot = mainDAG.get_roots()[0]
    nodeSort = [mainRoot]
    leftNode = mainRoot
    for i in range(len(mainDAG.edges())):
        for edge in mainDAG.edges():
            if edge[0] == leftNode:
                nodeSort.append(edge[1])
                leftNode = edge[1]
                break
    return nodeSort

if __name__== "__main__":
    fontsize = 20
    headWidth = 0.1
    nodeSize = 0.6
    
    G = DAG([
         ("ma-1", "ma"),
         ("ma", "dma"),
         ("ma-1", "dma"),
         
         ("ma", "a"),
         ("a-1", "a"),
         ("a-1", "da"),
         ("a", "da"),
         
         ("a", "v"),
         ("v-1", "v"),
         ("v-1", "dv"),
         ("v", "dv"),
         
         ("v", "x"),
         ("x-1", "x"),
         ("x-1", "dx"),
         ("x", "dx"),
    ])
    
    ''' category counter'''
    ammCat = 3
    ammMain = 0
    ammPrevious = 0
    ammDelta = 0
    for node in G.nodes:
        if 'd' in node:
            ammDelta += 1
        elif '-1' in node:
            ammPrevious += 1
        else:
            ammMain += 1
        
    nodeSort = getMainNodesInOrder(G)
    
    nPosDict = {}     
    nParDict = {}
    rows = len(nodeSort)*2-1
    collumns = ammCat -1
    i = 0
    for node in G.nodes():
        nParDict[node] = {'fontsize': fontsize}
        i = i+1

    locRow = rows
    locColl = collumns
    for mainnode in nodeSort:
        nPosDict[mainnode] = (locColl , locRow)
        for node in G.nodes():
            if node == mainnode + '-1':
                nPosDict[node] = (locColl-2 , locRow)
            elif node == 'd' + mainnode:
                nPosDict[node] = (locColl-1 , locRow-1)
        locRow -= 2

    eParDict = {}
    for u, v in G.edges():
        eParDict[(u,v)] = {'plot_params': {'head_width' : headWidth}}
        
    da = G.to_daft(node_pos=nPosDict,
                   pgm_params={'node_unit': nodeSize},
                   edge_params=eParDict,
                   node_params=nParDict)

    da.render()
    