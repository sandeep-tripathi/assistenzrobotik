import PyKDL

def list_to_jntArray(list):
    size = len(list)
    arr = PyKDL.JntArray(size)
    for i in xrange(0, size):
        arr[i] = list[i]

    return arr


def jntArray_to_list(array):
    size = max(array.columns(), array.rows())
    ret = []
    for i in xrange(0, size):
        ret.append(array[i])

    return ret