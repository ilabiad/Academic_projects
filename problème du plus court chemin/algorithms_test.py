from collections import namedtuple
import time 

# we'll use infinity as a default distance to nodes.
inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
    return Edge(start, end, cost)

def G_cost(point,dest):
    c=0
    if dest[0] > point[0]:
        c+=dest[0]-point[0]
    else:
        c+=point[0]-dest[0]
    if dest[1] > point[1]:
        c+= dest[1]-point[1]
    else:
        c+= point[1]-dest[1]
    return c 
        
def calc_distance(point, dest):
    return ((dest[0]-point[0])**2 + (dest[1]-point[1])**2)**0.5
    
def minimum(tab,key,dest):
    m = tab[0] 
    for elt in tab[1:] : 
        if key(elt) < key(m) :
            m = elt 
        elif key(elt) == key(m):
            if G_cost(elt,dest) < G_cost(m,dest):
                m = elt 
    return m 

def get_neighbours(point,extremum):
    neighbours = [] 
    if point[0] < extremum[0] :
        neighbours += [((point[0]+1,point[1]),1)]
    if point[1] < extremum[1]:
        neighbours +=[((point[0],point[1]+1),1)]
    if len(neighbours)==2:
        neighbours += [((point[0]+1,point[1]+1),1.4)]
    
    return neighbours 

class Graph:
    def __init__(self, edges=[],  noeuds=[], obstacles=[]):
        self.noeuds = [ noeud for noeud in noeuds if noeud not in obstacles] 
        self.edges = [make_edge(*edge) for edge in edges if edge.start not in obstacles and edge.end not in obstacles ]
        self.obstacles = obstacles 
        
    @property
    def vertices(self):
        return set(
            # this piece of magic turns ([1,2], [3,4]) into [1, 2, 3, 4]
            # the set above makes it's elements unique.
            sum(    
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )
        
    def get_node_pairs(self, n1, n2, both_ends=True):
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))
    
    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices }
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def A_star(self, source, dest):
        begin = time.time() 

        distances = {vertex: inf for vertex in self.noeuds}
        previous_vertices ={source:None}
        distances[source] = 0
        openset = [source]
        extr = self.noeuds[-1] 
        while openset:

            f = lambda vertex: distances[vertex]+G_cost(vertex,dest)
            current_vertex = minimum(openset, f, dest)
            
            neighbours = get_neighbours(current_vertex,extr)
            for neighbour, cost in neighbours:
                alternative_route = distances[current_vertex] + cost 
                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    openset.append(neighbour) 
                    previous_vertices[neighbour] = current_vertex

            openset.remove(current_vertex)
            if current_vertex == dest :
                break 
    
        path, current_vertex = [], dest
        while previous_vertices[current_vertex] is not None:
            path = [current_vertex] + path 
            current_vertex = previous_vertices[current_vertex]
        if path:
            path = [current_vertex] + path
        end = time.time() 
        return path, distances[dest], end-begin
    
    
    def greedy_algo(self, source, dest):
        begin = time.time() 
        vertices = self.noeuds 

        previous_vertices = {source:None}
        costs = {source:0} 
        openset = [source] 
        distances = {vertex: calc_distance(vertex,dest) for vertex in openset}
        extr = self.noeuds[-1] 
        while openset:
            current_vertex = min(openset, key = lambda vertex: distances[vertex]) 
            
            neighbours = get_neighbours(current_vertex,extr) 
            
            for neighbour, cost in neighbours: 
                costs[neighbour] = costs[current_vertex] + cost 
                openset.append(neighbour)
                distances[neighbour] =  calc_distance(neighbour,dest)
                previous_vertices[neighbour] = current_vertex
    
            if current_vertex == dest :
                break 
            openset.remove(current_vertex)

        path, current_vertex = [], dest
        while previous_vertices[current_vertex] is not None:
            path = [current_vertex] + path 
            current_vertex = previous_vertices[current_vertex]
        if path:
            path = [current_vertex] + path
        end = time.time()
        return path, costs[dest], end-begin
    
    
    def dijkstra(self, source, dest):
        begin = time.time()

        distances = {vertex: inf for vertex in self.noeuds}
        previous_vertices = {source: None }
        distances[source] = 0
        openset = [source]
        extr = self.noeuds[-1] 
        while openset:
            current_vertex = min(
                openset, key=lambda vertex: distances[vertex])
            
            neighbours = get_neighbours(current_vertex,extr) 
            for neighbour, cost in neighbours:
                alternative_route = distances[current_vertex] + cost
                if alternative_route < distances[neighbour]:

                    distances[neighbour] = alternative_route
                    openset.append(neighbour) 
                    previous_vertices[neighbour] = current_vertex

            openset.remove(current_vertex)
            if current_vertex == dest :
                break

        path, current_vertex = [], dest
        while previous_vertices[current_vertex] is not None:
            path=[current_vertex]+path
            current_vertex = previous_vertices[current_vertex]
        if path:
            path=[current_vertex] + path 
        end = time.time() 
        return path, distances[dest], end-begin
        
def create_edges(a,b):
    ''' create edges for x*y vertices, each vertice has 3 edges 1 to the right ,1 up and 1 to up-right '''
    edges=[] 
    for x in xrange(a+1):
        for y in xrange(b+1):
            edges+=[make_edge((x,y),(x+1,y)),
            make_edge((x,y),(x,y+1)),
            make_edge((x,y),(x+1,y+1),1.4)]  
    return edges 

def create_obstacles(point1,point2):
    ''' create a list containing points representing a segment that point1 and point2 defines '''
    obstacles=[] 
    if point1[0] == point2[0]:
        for i in xrange(point1[1],point2[1]+1):
            obstacles+=[(point1[0],i)] 
    else: 
        a = (point2[1]-point1[1])/float(point2[0]-point1[0])
        b = -point1[0]*a + point1[1] 
        for x in xrange(point1[0],point2[0]+1):
            obstacles+=[(x,a*x+b)] 
    return obstacles
 
def create_vertices(a,b):
    vertices = []
    for x in xrange(a+1):
        for y in xrange(b+1):
            vertices += [(x,y)] 
    return vertices 
        

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
    