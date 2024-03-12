// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import java.util.Optional;

/**
 * A simple state machine that runs state interfaces and switches between them for more complex
 * behavior
 */
public class StateMachine<Inputs> {
  private State<Inputs> state;

  /**
   * Constructs a new StateMachine with a given initial state
   *
   * @param initialState The initial state for the state machine
   */
  public StateMachine(State<Inputs> initialState) {
    setState(initialState);
  }

  /**
   * Runs the state machine with common inputs for the states to use
   *
   * @param inputs The inputs for the states to read from
   */
  public void run(Inputs inputs) {
    final Optional<State<Inputs>> nextState = state.run(inputs);
    nextState.ifPresent(this::setState);
  }

  /**
   * Sets and initializes the state of the state machine
   *
   * @param state The state to set
   */
  public void setState(State<Inputs> state) {
    this.state = state;
    this.state.initialize();
  }

  /**
   * Gets the current state of the state machine
   *
   * @return The current state
   */
  public State<Inputs> getState() {
    return this.state;
  }

  /**
   * Returns an optional of the state of the state machine
   *
   * @param <T> The state to check. Must be a valid state.
   * @param desired The class of the state to check
   * @return A full optional of the state machine's state if the state machine is in that state, and
   *     empty otherwise
   */
  @SuppressWarnings("unchecked")
  public <T> Optional<T> query(Class<T> desired) {
    if (isInState(desired)) {
      return Optional.of((T) this.state);
    }
    return Optional.empty();
  }

  /**
   * Checks if the state machine is in a state
   *
   * @param <T> The state to check. Must be a valid state.
   * @param state The class of the state to check
   * @return If the state machine is running that state class
   */
  public <T> boolean isInState(Class<T> state) {
    return getState().getClass().isAssignableFrom(state);
  }

  /**
   * Checks if the state machine is in a state
   *
   * @param state The state to check. Will check for an exact state object instead of a type.
   * @return If the state machine is running that specific state
   */
  public boolean isInState(State<Inputs> state) {
    return getState().equals(state);
  }

  public static interface State<Inputs> {
    /** Initializes the state when the state machine selects it */
    public default void initialize() {}

    /**
     * Runs this state and gets the next action of the state machine from it
     *
     * @param inputs The inputs to the state machine
     * @return The next state of the state machine. If empty, will keep using this state without
     *     changing it.
     */
    public default Optional<State<Inputs>> run(Inputs inputs) {
      return Optional.empty();
    }

    /** Finishes the state when the state machine deselects it */
    public default void finish() {}
  }
}
