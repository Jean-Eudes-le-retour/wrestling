class FiniteStateMachine:
    def __init__(self, states, initial_state, actions=None):
        self.states = states
        self.current_state = initial_state
        self.actions = actions

    def transition_to(self, state):
        if state not in self.states:
            raise ValueError("Invalid state: {}".format(state))
        self.current_state = state
    
    def execute_action(self):
        action = self.actions[self.current_state]
        action()