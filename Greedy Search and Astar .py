import heapq
    
#SEARCH ALGORITHMS BFS, DFS, GREEDYSEARCG, UCS, A*    

# Grafo do exemplo discutido em aula
G0 = {
    'S': ['d', 'e', 'p'],
    'a': [],
    'b': ['a'],
    'c': ['a'],
    'd': ['b', 'c', 'e'],
    'e': ['h', 'r'],
    'f': ['c', 'g'],
    'g': [],
    'h': ['p', 'q'],
    'p': ['q'],
    'q': [],
    'r': ['f']
}
# Versão do Grafo G0 ponderado
G1 = {
    'S': {'d': 3, 'e': 9, 'p': 1},
    'a': {},
    'b': {'a': 2},
    'c': {'a': 2},
    'd': {'b': 1, 'c': 8, 'e': 2},
    'e': {'h': 8, 'r': 2},
    'f': {'c': 3, 'g': 2},
    'g': {},
    'h': {'p': 4, 'q': 4},
    'p': {'q': 15},
    'q': {},
    'r': {'f': 1}
}

heuristic_lsb = {'Arad': 366, 'Bucharest': 0, "Craiova": 160, "Drobeta": 242,
    "Eforie": 161, "Fagaras": 176, "Giurgiu": 77, "Hirsova": 151,
    "Iasi": 226, "Lugoj": 244, "Mehadia": 241, "Neamt": 234,
    "Oradea": 380, "Pitesti": 100, "Rimnicu Vilcea": 193,
    "Sibiu": 253,
    "Timisoara": 329, "Urziceni": 80, "Vaslui": 199,
    "Zerind":374
}

# Graph of Romania
romenia = {
    "Arad": {"Zerind": 75, "Timisoara": 118, "Sibiu": 140},
    "Zerind": {"Oradea": 71, "Arad": 75},
    "Timisoara": {"Lugoj": 111, "Arad": 118},
    "Lugoj": {"Timisoara": 111, "Mehadia": 70},
    "Mehadia": {"Lugoj": 70, "Drobeta": 75},
    "Drobeta": {"Mehadia": 75, "Craiova": 120},
    "Craiova": {"Rimnicu Vilcea": 146, "Pitesti": 138, "Drobeta": 120,},
    "Rimnicu Vilcea": {"Sibiu": 80, "Pitesti": 97, "Craiova": 146,},
    "Sibiu": {"Rimnicu Vilcea": 80, "Oradea": 151, "Fagaras": 99, "Arad":
    140,},
    "Oradea": {"Zerind": 71, "Sibiu": 151},
    "Fagaras": {"Sibiu": 99, "Bucharest": 211},
    "Pitesti": {"Rimnicu Vilcea": 97, "Craiova": 138, "Bucharest": 101},
    "Bucharest": {"Urziceni": 85, "Pitesti": 101, "Giurgiu": 90,
    "Fagaras": 211},
    "Giurgiu": {"Bucharest": 90},
    "Urziceni": {"Vaslui": 142, "Hirsova": 98, "Bucharest": 85,},
    "Hirsova": {"Urziceni": 98, "Eforie": 86},
    "Eforie": {"Hirsova": 86},
    "Vaslui": {"Urziceni": 142, "Iasi": 92},
    "Iasi": {"Vaslui": 92, "Neamt": 87},
    "Neamt": {"Iasi": 87}
}


def DFS(adj_list, start, goal):
    visited = []

    # pilha com os vértices candidatos (da fronteira)
    fringe = [start]

    # variavel para armazenar o caminho completo
    path = [[start]]

    while fringe:
        current_node = fringe.pop()

        current_path = path.pop()
        
        if current_node not in visited:
            visited.append(current_node)

            if current_node == goal:
                return current_path
            else:
                for neighbor in sorted(adj_list[current_node], reverse = True):
                    if neighbor not in visited:
                        fringe.append(neighbor)
                        # adiciona um novo caminho, composto pelo atual + vértice adjacente
                        path.append(current_path + [neighbor])
    return None

def BFS(adj_list, start, goal):
    visited = []

    # pilha com os vértices candidatos (da fronteira)
    fringe = [start]

    # variavel para armazenar o caminho completo
    path = [[start]]

    while fringe:
        current_node = fringe.pop(0)

        current_path = path.pop(0)
        
        if current_node not in visited:
            visited.append(current_node)

            if current_node == goal:
                return current_path
            else:
                for neighbor in sorted(adj_list[current_node]):
                    if neighbor not in visited:
                        fringe.append(neighbor)
                        # adiciona um novo caminho, composto pelo atual + vértice adjacente
                        path.append(current_path + [neighbor])
    return None

def UCS(adj_list, start, goal):
    visited = []

    # pilha com os vértices candidatos (da fronteira)
    fringe = []
    heapq.heappush(fringe, (0, start))
    # variavel para armazenar o caminho completo
    path = []
    heapq.heappush(path, (0, [start]))

    while fringe:
        item = heapq.heappop(fringe)
        current_cost = item[0]
        current_node = item[1]

        current_path = heapq.heappop(path)[1]
        
        if current_node not in visited:
            visited.append(current_node)

            if current_node == goal:
                return current_path, current_cost

            else:
                for neighbor, cost in adj_list[current_node].items():
                    if neighbor not in visited:
                        heapq.heappush(fringe, (current_cost+cost, neighbor))
                        heapq.heappush(path, [current_cost + cost, current_path + [neighbor]])
    return None

def GreedySearch(graph, heuristic, start, goal):
    visited = []
    fringe = []
    
    heapq.heappush(fringe, (heuristic[start], start))
    
    path = []
    heapq.heappush(path, (heuristic[start], 0, [start]))
    
    while fringe:
        current_node = heapq.heappop(fringe)[1]
        
        item = heapq.heappop(path)
        current_cost = item[1]
        current_path = item[2]
        
        if current_node == goal:
            return current_path, current_cost
        
        for neighbour, cost in graph[current_node].items():
            if neighbour not in visited:
                heapq.heappush(fringe, (heuristic[neighbour], neighbour))
                heapq.heappush(path, (heuristic[neighbour], current_cost + cost, current_path + [neighbour]))
    return None

def AStar(graph, heuristic, start, goal):
    visited = []
    fringe = []
    
    heapq.heappush(fringe, (heuristic[start], start))
    
    path = []
    heapq.heappush(path, (heuristic[start], 0, [start]))
    
    while fringe:
        current_node = heapq.heappop(fringe)[1]
        
        item = heapq.heappop(path)
        current_cost = item[1]
        current_path = item[2]
        
        if current_node == goal:
            return current_path, current_cost
        
        for neighbour, cost in graph[current_node].items():
            if neighbour not in visited:
                heapq.heappush(fringe, (heuristic[neighbour] + cost, neighbour))
                heapq.heappush(path, (heuristic[neighbour] + cost, current_cost + cost, current_path + [neighbour]))
    return None

print("DFS: ", DFS(G0, 'S', 'g'))
print("UCS: ", UCS(romenia, 'Arad', 'Bucharest'))
print("GreedySearch: ", GreedySearch(romenia, heuristic_lsb, 'Arad', 'Bucharest'))
print("AStar: ", AStar(romenia, heuristic_lsb, 'Arad', 'Bucharest'))
    