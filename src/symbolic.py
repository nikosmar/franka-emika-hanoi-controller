import random
import copy


class MyDict(dict):
    def __getitem__(self, key):
        key = tuple([tuple(x) for x in key])
        return super().__getitem__(key)

    def __setitem__(self, key, value):
        key = tuple([tuple(x) for x in key])
        return super().__setitem__(key, value)


def bfs(start_node, end_node):
    # map to keep parents of nodes during search
    path = MyDict()
    if start_node == end_node:  # sanity check
        return path
    path[start_node._value] = None  # the starting node has no parent
    # we implement the graph-search version of DFS: we keep a frontier and a visited list
    frontier = [start_node]  # we insert the starting node to the frontier
    explored = []

    while len(frontier) > 0:  # if our frontier becomes empty, we have failed!
        node = frontier.pop(0)  # get and remove the first inserted node to the frontier (FIFO)

        if node not in explored:  # if node not visited
            explored.append(node)  # put it in the visited list

        # Get the children of this node
        children = node._children
        if len(children) == 0 and node._child_creator:
            children = node._child_creator(node)

        for child in children:  # foreach child
            # if the child is not visited nor in the frontier
            if child not in explored and child not in frontier:
                path[child._value] = node  # we update the parent of this child
                if child == end_node:  # if the child is the target, we exit the search
                    return path
                frontier.append(child)  # if not, we add the child to the frontier and continue

    assert False  # we should never reach here

    return path


def solution_from_path(path, goal_state):
    p = []
    curr = goal_state
    while curr:
        p.append(curr)
        curr = path[curr._value]
    p.reverse()
    return p


def solution_cost(sol):
    total_cost = 0.
    for s in sol:
        total_cost += s.cost()

    return total_cost


class TreeNode:
    # Constructor
    def __init__(self, value, cost=0., child_creator=None):
        # Variables
        self._value = value  # value to store
        self._cost = cost  # cost of moving from parent
        self._children = []  # list of pointers to children
        self._child_creator = child_creator

    # Add child and return reference to it
    def add_child(self, value, cost=0.):
        self._children.append(TreeNode(value, cost))

        return self._children[-1]

    # overrides == operator
    def __eq__(self, other):
        return isinstance(other, TreeNode) and other._value == self._value

    def num_children(self):
        return len(self._children)

    def child(self, index):
        # assert that query index is inside the required range
        assert index >= 0 and index < len(self._children)
        return self._children[index]

    def cost(self):
        return self._cost

    # Get height of the (sub-)tree
    def height(self):
        # if we have no children, the sub-tree has zero height
        if len(self._children) == 0:
            return 0

        # get the maximum sub-tree height of the children's sub-trees
        max_height = 0
        for child in self._children:
            h = child.height()
            if h > max_height:
                max_height = h

        # return max_height + 1, because the leaf nodes will return 0
        return max_height + 1

    # Helper function to print one level of the sub-tree
    def _print_level(self, level):
        # if we are at level zero, print the value of the node
        if level == 0:
            print(self._value, end=" ")  # note the 'end = " "' to avoid printing a new line
        elif level > 0:
            # otherwise, iterate over children with level-1
            for child in self._children:
                child._print_level(level - 1)

    # Print sub-tree level-by-level
    def print_level_order(self):
        h = self.height()
        for i in range(h + 1):
            self._print_level(i)
            print(' ')  # print new-line after each level (to separate them visually)


def valid_state(state):
    if len(state) < 3:
        return False
    for i in range(3):
        sz = len(state[i])
        for j in range(sz-1):
            if state[i][j] <= state[i][j+1]:
                return False

    return True


def action_from_states(from_state, to_state):
    from_id = -1
    to_id = -1
    for i in range(3):
        psz = len(from_state[i])
        tsz = len(to_state[i])

        if psz > tsz:
            from_id = i
        elif psz < tsz:
            to_id = i

    return [from_state[from_id][-1], to_id]


# This is the function that creates children for a given node
def hanoi_creator(node):
    children = []

    vals = node._value

    if not valid_state(vals):  # sanity check
        return children

    for i in range(3):
        if len(vals[i]) == 0:
            continue
        block_to_move = vals[i][-1]
        for j in range(3):
            if i == j:
                continue
            block_on_top = -1
            if len(vals[j]) > 0:
                block_on_top = vals[j][-1]
            if block_on_top == -1 or block_to_move < block_on_top:
                new_vals = copy.deepcopy(vals)
                new_vals[i].pop(-1)
                new_vals[j].append(block_to_move)
                children.append(TreeNode(new_vals, 1., hanoi_creator))  # add child to the list

    random.shuffle(children)
    return children


def next_move(state):
    start = TreeNode(copy.copy(state), child_creator=hanoi_creator)

    final_state = [[2, 1, 0], [], []]
    target = TreeNode(final_state, 1.)

    path = bfs(start, target)  # BFS
    sol = solution_from_path(path, target)

    # for s in sol:
    #     print(s._value)

    return action_from_states(sol[0]._value, sol[1]._value)


# print(next_move([[], [2, 1, 0], []]))
# print(next_move([[0], [2,1], []]))
# print(next_move([[], [2,1], [0]]))
