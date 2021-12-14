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
            checked_nodes = nodes
        else:
            checked_nodes = [node for node in nodes if node in nodes_self]
    elif isinstance(nodes, (int, np.integer)):
        if nodes_self is None:
            checked_nodes = [nodes]
        else:
            checked_nodes = [nodes] if nodes in nodes_self else []
    else:
        raise Exception('Type {} is not supported to specify nodes.'.format(type(nodes)))

    if checked_nodes is None:
        raise Exception(f'Invalid nodes inserted: {nodes}')
    # todo ADD WARNING if node (or nodes) are NOT present in own nodes.
    #  (basically, when setting bounds for some node, it is left out because the var/fun does not exist in that node
    return checked_nodes

def checkValueEntry(val):
    if isinstance(val, (int, float)):
        val = np.array([val])
    else:
        val = np.array(val)

        # if single value, flatten
        # note: dont flatten matrix of values!
        multiple_vals = val.ndim == 2 and val.shape[1] != 1
        if not multiple_vals:
            val = val.flatten()

    return val

if __name__ == '__main__':
    penis = [[1, 5], [3, 9], [12, 18]]
    print(unravelElements(penis))