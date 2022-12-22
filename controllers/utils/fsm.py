class FiniteStateMachine:
    def __init__(self, states, initial_state):
        self.states = states
        self.current_state = initial_state

    def transition_to(self, state):
        if state not in self.states:
            raise ValueError("Invalid state: {}".format(state))
        self.current_state = state
