import numpy as np

def unravelElements(elements):
    if isinstance(elements, int):
        unraveled_elem = [elements]
        pass
    elif any(isinstance(el, list) for el in elements):
        unraveled_elem = list()
        for el in elements:
            temp = list(range(el[0], el[1]+1)) # +1 # todo cannot add+1 here?
            for item in temp:
                unraveled_elem.append(item) if item not in unraveled_elem else unraveled_elem
    elif isinstance(elements, list):
        unraveled_elem = list()
        temp = list(range(elements[0], elements[1]+1)) # +1
        for item in temp:
            unraveled_elem.append(item)

    return unraveled_elem


def listOfListFLOATtoINT(listOfList):
    # transform every element to INT
    for i in range(len(listOfList)):
        if isinstance(listOfList[i], list):
            for j in range(len(listOfList[i])):
                listOfList[i][j] = int(listOfList[i][j])
        else:
            listOfList[i] = int(listOfList[i])

    return listOfList

def checkNodes(nodes, nodes_self=None):

    if hasattr(nodes, "__iter__") and not isinstance(nodes, str):
        if nodes_self is None:
            pass
        else:
            nodes = [node for node in nodes if node in nodes_self]
    elif isinstance(nodes, int):
        if nodes_self is None:
            nodes = [nodes]
        else:
            nodes = [nodes] if nodes in nodes_self else []
    else:
        raise Exception('Type {} is not supported to specify nodes.'.format(type(nodes)))

    # todo ADD WARNING if node (or nodes) are NOT present in own nodes.
    #  (basically, when setting bounds for some node, it is left out because the var/fun does not exist in that node
    return nodes

def checkValueEntry(val):
    if isinstance(val, (int, float)):
        val = np.array([val])
    else:
        val = np.array(val).flatten()

    return val

def ravelElements(list_values):
    list_ranges = list()
    first = list_values[0]
    if len(list_values) == 1:
        list_ranges.append([first, list_values[0]])
    else:
        for i in range(1, len(list_values)):
            if list_values[i] - list_values[i - 1] > 1:
                last = list_values[i-1]
                list_ranges.append([first, last])
                first = list_values[i]

            if i ==len(list_values)-1:
                last = list_values[i]
                list_ranges.append([first, last])

    return list_ranges

if __name__ == '__main__':
    penis = [[1, 5], [3, 9], [12, 18]]
    unr_elem = unravelElements(penis)
    print(unr_elem)

    another = [20]
    print(ravelElements(another))