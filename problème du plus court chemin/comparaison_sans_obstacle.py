import sys
sys.path.append('C:\Users\user\Desktop\project')

from algorithms_test import * 
import matplotlib.pyplot as plt 
import numpy as np 
from random import randint 

source = (0,0) 
time = [[],[],[]]    
dimension = [] 


for x in range(2,200): # nombre de tests 
    dest = (randint(x//2,x),randint(x//2,x)) # genere une destination aleatoire
    G = Graph(noeuds=create_vertices(x,x))
    dimension += [x**2]
    c1 = G.greedy_algo(source,dest)
    c2 = G.A_star(source,dest) 
    c3 = G.dijkstra(source,dest) 
    time[0] += [c1[2]]
    time[1] += [c2[2]] 
    time[2] += [c3[2]] 

    
plt.plot(dimension,time[0],'b',label='greedy_algo')
plt.plot(dimension,time[1],'r',label='A star')
plt.plot(dimension,time[2],'g',label='dijkstra')
 
plt.legend(loc='best') 
plt.title('comparaison des algorithmes : sans obstacle ')
plt.xlabel('nombre de noeuds/sommets')
plt.ylabel('temps en s') 

plt.show() 






































































































