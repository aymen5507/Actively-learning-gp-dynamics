# Actively Learning Gaussian Process Dynamics

## Overview

This repository contains an implementation of a method for actively learning Gaussian process (GP) dynamics. The method aims to efficiently learn the dynamics of a system by actively selecting state-action pairs for observation.

### Description

The process can be summarized as follows:

1. **Initialization:** Generate an initial set of state-action pairs (denoted as Z) from the initial state x0 and randomly generated control inputs that satisfy constraints.

2. **Model Fitting:** Use the true model to calculate outcomes for each state-action pair, introducing Gaussian noise to simulate observed values. Fit an initial GP model using the MATLAB function `fitrgp`.

3. **Receding Horizon Update:** Utilize a receding horizon approach to update the training dataset. Solve for a set of control inputs, use the first control input to retrain the model, and iterate the process multiple times until the desired size of the training dataset is reached.

4. **Evaluation:** Evaluate the results by creating 1000 random state-action pairs. Employ the GP model to predict their outcomes, calculate the mean squared error between the predicted values and the true values computed using the true system, using the predefined `immse` function.

## Usage

### Getting Started

To use this implementation, follow these steps:

1. Clone this repository to your local machine:

    ```bash
    git clone https://github.com/aymen5507/actively-learning-gp-dynamics.git
    ```

2. Open MATLAB and navigate to the cloned repository.

3. Run the necessary scripts to execute the actively learning Gaussian process dynamics method.

4. Customize parameters and settings as needed for your specific application.

## Requirements

- MATLAB R2019b or later

## Acknowledgements

- This work is based on the paper "Actively Learning Gaussian Process Dynamics".

## Contact

For questions or feedback, please contact me.
