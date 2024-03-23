Paper "Actively learning gaussian process dynamics"

The initial set of state-action pairs, denoted as Z, is generated from the initial
state x0 and a set of randomly generated control inputs that satisfy the con-
straints.

Subsequently, the true model is employed to calculate the outcome for
each state-action pair, with an additional Gaussian noise introduced to simulate
observed values. In the next step, the predefined Matlab function fitrgp was
utilized to fit the initial GP model. 

 In this work, we finally chose the receding horizon fashion to update
the training dataset due to its accuracy and robustness. So, once we solve for a
set of control inputs, we utilize the first control input and retrain the model. We
iterate the informative control generation process multiple times until the desired
size of the training data set is reached. For the evaluation of the results, we have
created 1000 random state-action pairs. The GP model is employed to predict
their outcomes, and the mean squared error between the predicted values and the
true values, computed using the true system, is calculated with predefined immse
function.
