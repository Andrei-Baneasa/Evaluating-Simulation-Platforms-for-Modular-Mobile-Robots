import tensorflow as tf
import numpy as np

class NNController:
    def __init__(self, input_size, output_size, hidden_sizes=None):
        self.input_size = input_size
        self.output_size = output_size
        self.hidden_sizes = hidden_sizes if hidden_sizes else [32, 16]
        self._build_model()

    def _build_model(self):
        model = tf.keras.Sequential()
        model.add(tf.keras.layers.InputLayer(input_shape=(self.input_size,)))
        for size in self.hidden_sizes:
            model.add(tf.keras.layers.Dense(size, activation='relu'))
        model.add(tf.keras.layers.Dense(self.output_size, activation='linear'))
        model.compile(optimizer='adam', loss='mse')
        self.model = model

    def get_command(self, state, goal):
        """
        state: current robot state (e.g., position, velocity)
        goal: target position/path point
        returns: command for path following (e.g., velocity, steering)
        """
        input_vec = np.concatenate([state, goal])
        input_vec = input_vec.reshape(1, -1)
        command = self.model.predict(input_vec, verbose=0)[0]
        return command

    def train(self, x, y, epochs=1):
        self.model.fit(x, y, epochs=epochs, verbose=0)

    def reset(self):
        self._build_model()