"""Dynamic programming over sequences of sampled configurations."""

import numpy as np


def apply_cost_function(data, function):
    return [function(data[index - 1], data[index]) for index in range(1, len(data))]


def calculate_value_function(transition_costs):
    state_dimensions = [cost.shape[0] for cost in transition_costs]
    state_dimensions.append(transition_costs[-1].shape[1])
    values = [np.zeros(dimension) for dimension in state_dimensions]
    actions = [np.zeros(dimension, dtype=int) for dimension in state_dimensions]
    for index in range(len(state_dimensions) - 2, -1, -1):
        rhs = transition_costs[index] + values[index + 1]
        values[index] = np.min(rhs, axis=1)
        actions[index] = np.argmin(rhs, axis=1)
    return actions, values


def calculate_value_function_with_state_cost(transition_costs, state_costs):
    assert len(transition_costs) == len(state_costs) - 1
    state_dimensions = [cost.shape[0] for cost in transition_costs]
    state_dimensions.append(transition_costs[-1].shape[1])
    values = [np.array(cost, copy=True) for cost in state_costs]
    actions = [np.zeros(dimension, dtype=int) for dimension in state_dimensions]
    for index in range(len(state_dimensions) - 2, -1, -1):
        rhs = transition_costs[index] + values[index + 1]
        values[index] += np.min(rhs, axis=1)
        actions[index] = np.argmin(rhs, axis=1)
    return actions, values


def extract_shortest_path(data, actions, values):
    path = []
    next_index = np.argmin(values[0])
    for stage in range(len(data)):
        path.append(data[stage][next_index])
        next_index = actions[stage][next_index]
    return path


def shortest_path(data, cost_function):
    actions, values = calculate_value_function(apply_cost_function(data, cost_function))
    return {
        "success": True,
        "path": extract_shortest_path(data, actions, values),
        "length": np.min(values[0]),
    }


def shortest_path_with_state_cost(states, state_costs, cost_function):
    actions, values = calculate_value_function_with_state_cost(
        apply_cost_function(states, cost_function), state_costs
    )
    return {
        "success": True,
        "path": extract_shortest_path(states, actions, values),
        "length": np.min(values[0]),
    }
